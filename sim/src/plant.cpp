#include "plant.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace sim {

namespace {

double read(const YAML::Node& root, const std::string& name) {
  if (!root[name]) {
    throw std::runtime_error("sim config: missing " + name);
  }
  return root[name].as<double>();
}

Eigen::Vector3d read_vector3(const YAML::Node& root, const std::string& name) {
  if (!root[name] || root[name].size() != 3) {
    throw std::runtime_error("sim config: " + name + " must have 3 entries");
  }
  return Eigen::Vector3d(root[name][0].as<double>(), root[name][1].as<double>(),
                         root[name][2].as<double>());
}

}  // namespace

SimConfig load_sim_config(const std::string& yaml_path) {
  const YAML::Node root = YAML::LoadFile(yaml_path);
  SimConfig c;
  c.physics_rate = read(root, "physics_rate");
  c.imu_rate = read(root, "imu_rate");
  c.flow_rate = read(root, "flow_rate");
  c.range_rate = read(root, "range_rate");
  c.control_rate = read(root, "control_rate");
  c.command_latency = read(root, "command_latency");
  c.attitude_gain = read(root, "attitude_gain");
  c.rate_gain = read(root, "rate_gain");
  c.gyro_noise = read(root, "gyro_noise");
  c.gyro_bias = read_vector3(root, "gyro_bias");
  c.accel_noise = read(root, "accel_noise");
  c.accel_bias = read_vector3(root, "accel_bias");
  c.range_noise = read(root, "range_noise");
  c.flow_noise = read(root, "flow_noise");
  c.mass_factor = read(root, "mass_factor");
  c.thrust_factor = read(root, "thrust_factor");
  c.random_seed = static_cast<int>(read(root, "random_seed"));
  return c;
}

Quadrotor::Quadrotor(const cf_model::Params& params, const SimConfig& config)
    : params_(params), config_(config), rng_(config.random_seed) {
  true_mass_ = params_.mass * config_.mass_factor;
  max_motor_thrust_ = config_.thrust_factor * cf_model::thrust_per_motor_from_cmd(params_, params_.thrust_cmd_max);

  // X configuration. Motor i at body position (x_i, y_i), spin direction s_i.
  // Order: front-right, back-right, back-left, front-left.
  const double d = params_.motor_to_motor_diagonal / 2.0 / std::sqrt(2.0);
  const double x[4] = {d, -d, -d, d};
  const double y[4] = {-d, -d, d, d};
  const double spin[4] = {-1.0, 1.0, -1.0, 1.0};
  const double torque_per_thrust = params_.torque_coefficient / params_.thrust_coefficient;  // m
  for (int i = 0; i < 4; ++i) {
    allocation_(0, i) = 1.0;                       // total thrust
    allocation_(1, i) = y[i];                      // roll torque  = sum y_i f_i
    allocation_(2, i) = -x[i];                     // pitch torque = -sum x_i f_i
    allocation_(3, i) = spin[i] * torque_per_thrust;  // yaw torque
  }
  allocation_inv_ = allocation_.inverse();
}

void Quadrotor::set_position(const Eigen::Vector3d& position) {
  state_.position = position;
  state_.on_ground = position.z() <= 0.0;
}

Eigen::Vector4d Quadrotor::inner_controller() const {
  // Firmware behaviour: zero thrust setpoint means motors off.
  if (setpoint_.thrust_cmd <= 0.0) {
    return Eigen::Vector4d::Zero();
  }
  const Eigen::Vector3d euler = cf_model::euler_zyx(state_.attitude.toRotationMatrix());

  // Angle loop -> body rate setpoint, then rate loop -> angular acceleration.
  Eigen::Vector3d rate_setpoint;
  rate_setpoint.x() = config_.attitude_gain * (setpoint_.roll - euler.x());
  rate_setpoint.y() = config_.attitude_gain * (setpoint_.pitch - euler.y());
  rate_setpoint.z() = setpoint_.yaw_rate;
  const Eigen::Vector3d angular_acceleration =
      config_.rate_gain * (rate_setpoint - state_.body_rate);
  const Eigen::Vector3d torque = params_.inertia * angular_acceleration;

  // Base thrust from the thrust command (firmware: same base for all motors).
  const double total_thrust =
      4.0 * config_.thrust_factor * cf_model::thrust_per_motor_from_cmd(params_, setpoint_.thrust_cmd);

  Eigen::Vector4d wrench(total_thrust, torque.x(), torque.y(), torque.z());
  Eigen::Vector4d motor_cmd = allocation_inv_ * wrench;
  for (int i = 0; i < 4; ++i) {
    motor_cmd(i) = std::clamp(motor_cmd(i), 0.0, max_motor_thrust_);
  }
  return motor_cmd;
}

void Quadrotor::step(double dt) {
  // Motors: first-order lag towards the controller's command.
  const Eigen::Vector4d motor_cmd = inner_controller();
  state_.motor_thrust += (motor_cmd - state_.motor_thrust) * (dt / params_.motor_time_constant);

  const Eigen::Vector4d wrench = allocation_ * state_.motor_thrust;
  const double thrust = wrench(0);
  const Eigen::Vector3d torque = wrench.segment<3>(1);
  const Eigen::Matrix3d R = state_.attitude.toRotationMatrix();

  // Translational dynamics.
  const Eigen::Vector3d gravity(0.0, 0.0, -params_.gravity);
  Eigen::Vector3d accel = R * Eigen::Vector3d(0.0, 0.0, thrust / true_mass_) + gravity;

  // Ground contact: the floor pushes back while the drone rests on it.
  if (state_.on_ground && accel.z() <= 0.0) {
    state_.velocity.setZero();
    state_.body_rate.setZero();
    last_specific_force_body_ = R.transpose() * Eigen::Vector3d(0.0, 0.0, params_.gravity);
    return;
  }
  state_.on_ground = false;
  last_specific_force_body_ = Eigen::Vector3d(0.0, 0.0, thrust / true_mass_);

  state_.position += state_.velocity * dt + 0.5 * accel * dt * dt;
  state_.velocity += accel * dt;
  if (state_.position.z() <= 0.0) {  // touchdown
    state_.position.z() = 0.0;
    state_.velocity.setZero();
    state_.body_rate.setZero();
    state_.on_ground = true;
  }

  // Rotational dynamics: J w_dot = tau - w x (J w).
  const Eigen::Vector3d& w = state_.body_rate;
  const Eigen::Vector3d w_dot =
      params_.inertia.inverse() * (torque - w.cross(params_.inertia * w));
  state_.body_rate += w_dot * dt;

  // Attitude: q_dot = 0.5 * q * (0, w).
  const Eigen::Quaterniond omega(0.0, w.x(), w.y(), w.z());
  Eigen::Quaterniond q_dot = state_.attitude * omega;
  state_.attitude.coeffs() += 0.5 * q_dot.coeffs() * dt;
  state_.attitude.normalize();
}

Eigen::Vector3d Quadrotor::measure_gyro() {
  Eigen::Vector3d gyro = state_.body_rate + config_.gyro_bias;
  for (int i = 0; i < 3; ++i) gyro(i) += config_.gyro_noise * unit_normal_(rng_);
  return gyro;
}

Eigen::Vector3d Quadrotor::measure_accel() {
  Eigen::Vector3d accel = last_specific_force_body_ + config_.accel_bias;
  for (int i = 0; i < 3; ++i) accel(i) += config_.accel_noise * unit_normal_(rng_);
  return accel;
}

double Quadrotor::measure_range() {
  const Eigen::Matrix3d R = state_.attitude.toRotationMatrix();
  return state_.position.z() / R(2, 2) + config_.range_noise * unit_normal_(rng_);
}

Eigen::Vector2d Quadrotor::measure_flow(double flow_dt) {
  // Pixel motion seen by the downward camera (firmware model, mm_flow.c).
  const Eigen::Matrix3d R = state_.attitude.toRotationMatrix();
  const Eigen::Vector3d v_body = R.transpose() * state_.velocity;
  const double z = std::max(state_.position.z(), 0.01);
  const double scale = flow_dt * params_.flow_npix / params_.flow_thetapix;
  Eigen::Vector2d flow;
  flow.x() = scale * (v_body.x() * R(2, 2) / z - state_.body_rate.y());
  flow.y() = scale * (v_body.y() * R(2, 2) / z + state_.body_rate.x());
  flow.x() += config_.flow_noise * unit_normal_(rng_);
  flow.y() += config_.flow_noise * unit_normal_(rng_);
  return flow;
}

}  // namespace sim
