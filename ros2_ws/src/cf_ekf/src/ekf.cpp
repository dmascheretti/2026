#include "cf_ekf/ekf.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace cf_ekf {

namespace {

constexpr double kJacobianStep = 1e-6;

double read(const YAML::Node& root, const std::string& name) {
  if (!root[name]) {
    throw std::runtime_error("ekf config: missing " + name);
  }
  return root[name].as<double>();
}

}  // namespace

EkfConfig load_ekf_config(const std::string& yaml_path) {
  const YAML::Node root = YAML::LoadFile(yaml_path);
  EkfConfig c;
  c.accel_noise = read(root, "accel_noise");
  c.gyro_noise = read(root, "gyro_noise");
  c.position_noise = read(root, "position_noise");
  c.range_noise = read(root, "range_noise");
  c.flow_noise = read(root, "flow_noise");
  c.range_max = read(root, "range_max");
  c.flow_min_height = read(root, "flow_min_height");
  c.outlier_threshold = read(root, "outlier_threshold");
  c.initial_position_std = read(root, "initial_position_std");
  c.initial_velocity_std = read(root, "initial_velocity_std");
  c.initial_attitude_std = read(root, "initial_attitude_std");
  return c;
}

Ekf::Ekf(const cf_model::Params& params, const EkfConfig& config)
    : params_(params), config_(config) {
  reset(StateVector::Zero());
}

void Ekf::reset(const StateVector& x0) {
  x_ = x0;
  P_.setZero();
  for (int i = PX; i <= PZ; ++i) {
    P_(i, i) = config_.initial_position_std * config_.initial_position_std;
  }
  for (int i = VX; i <= VZ; ++i) {
    P_(i, i) = config_.initial_velocity_std * config_.initial_velocity_std;
  }
  for (int i = ROLL; i <= YAW; ++i) {
    P_(i, i) = config_.initial_attitude_std * config_.initial_attitude_std;
  }
}

// ----------------------------------------------------------------- predict

StateVector Ekf::propagate(const StateVector& x, const ImuSample& imu, double dt) const {
  const double roll = x(ROLL);
  const double pitch = x(PITCH);
  const double yaw = x(YAW);
  const Eigen::Matrix3d R = cf_model::rotation_zyx(roll, pitch, yaw);

  // Acceleration in world frame: rotate specific force, remove gravity.
  const Eigen::Vector3d accel_world =
      R * imu.accel - Eigen::Vector3d(0.0, 0.0, params_.gravity);

  // Euler angle rates from body rates (ZYX convention).
  const double p = imu.gyro.x();
  const double q = imu.gyro.y();
  const double r = imu.gyro.z();
  const double roll_rate =
      p + std::sin(roll) * std::tan(pitch) * q + std::cos(roll) * std::tan(pitch) * r;
  const double pitch_rate = std::cos(roll) * q - std::sin(roll) * r;
  const double yaw_rate = (std::sin(roll) * q + std::cos(roll) * r) / std::cos(pitch);

  StateVector x_next = x;
  x_next.segment<3>(PX) += x.segment<3>(VX) * dt + 0.5 * accel_world * dt * dt;
  x_next.segment<3>(VX) += accel_world * dt;
  x_next(ROLL) += roll_rate * dt;
  x_next(PITCH) += pitch_rate * dt;
  x_next(YAW) += yaw_rate * dt;
  x_next(YAW) = std::atan2(std::sin(x_next(YAW)), std::cos(x_next(YAW)));
  return x_next;
}

void Ekf::predict(const ImuSample& imu, double dt) {
  // Numeric Jacobian F = d(propagate)/dx, one column per state.
  StateMatrix F;
  for (int i = 0; i < kNx; ++i) {
    StateVector x_plus = x_;
    StateVector x_minus = x_;
    x_plus(i) += kJacobianStep;
    x_minus(i) -= kJacobianStep;
    StateVector diff = propagate(x_plus, imu, dt) - propagate(x_minus, imu, dt);
    diff(YAW) = std::atan2(std::sin(diff(YAW)), std::cos(diff(YAW)));
    F.col(i) = diff / (2.0 * kJacobianStep);
  }

  // Process noise: IMU noise integrated over dt (random walk on v and angles).
  StateMatrix Q = StateMatrix::Zero();
  const double pos_var = config_.position_noise * config_.position_noise * dt;
  const double vel_var = config_.accel_noise * config_.accel_noise * dt;
  const double att_var = config_.gyro_noise * config_.gyro_noise * dt;
  for (int i = PX; i <= PZ; ++i) Q(i, i) = pos_var;
  for (int i = VX; i <= VZ; ++i) Q(i, i) = vel_var;
  for (int i = ROLL; i <= YAW; ++i) Q(i, i) = att_var;

  x_ = propagate(x_, imu, dt);
  P_ = F * P_ * F.transpose() + Q;
  P_ = 0.5 * (P_ + P_.transpose());  // keep symmetric against rounding
}

// ------------------------------------------------------------ measurements

double Ekf::predict_range(const StateVector& x) const {
  // ToF points along body -z. Over a flat floor the measured distance is
  // height / cos(tilt), and cos(tilt) = R(2,2) = cos(roll) * cos(pitch).
  const double r22 = std::cos(x(ROLL)) * std::cos(x(PITCH));
  return x(PZ) / r22;
}

Eigen::Vector2d Ekf::predict_flow(const StateVector& x, const FlowSample& flow) const {
  // Same model as the firmware (mm_flow.c): pixel motion = translation
  // seen from height z, minus the apparent motion from rotation.
  const Eigen::Matrix3d R = cf_model::rotation_zyx(x(ROLL), x(PITCH), x(YAW));
  const Eigen::Vector3d v_body = R.transpose() * x.segment<3>(VX);
  const double r22 = R(2, 2);
  const double z = std::max(x(PZ), config_.flow_min_height);
  const double scale = flow.dt * params_.flow_npix / params_.flow_thetapix;
  const double nx = scale * (v_body.x() * r22 / z - flow.gyro.y());
  const double ny = scale * (v_body.y() * r22 / z + flow.gyro.x());
  return Eigen::Vector2d(nx, ny);
}

template <int M, typename MeasurementFunction>
UpdateResult Ekf::update(const Eigen::Matrix<double, M, 1>& z,
                         const Eigen::Matrix<double, M, M>& R,
                         MeasurementFunction h) {
  // Numeric Jacobian H = dh/dx.
  Eigen::Matrix<double, M, kNx> H;
  for (int i = 0; i < kNx; ++i) {
    StateVector x_plus = x_;
    StateVector x_minus = x_;
    x_plus(i) += kJacobianStep;
    x_minus(i) -= kJacobianStep;
    H.col(i) = (h(x_plus) - h(x_minus)) / (2.0 * kJacobianStep);
  }

  const Eigen::Matrix<double, M, 1> innovation = z - h(x_);
  const Eigen::Matrix<double, M, M> S = H * P_ * H.transpose() + R;
  const Eigen::Matrix<double, M, M> S_inv = S.inverse();

  UpdateResult result;
  result.mahalanobis_sq = innovation.dot(S_inv * innovation);
  if (result.mahalanobis_sq > config_.outlier_threshold) {
    return result;  // rejected as outlier
  }

  const Eigen::Matrix<double, kNx, M> K = P_ * H.transpose() * S_inv;
  x_ += K * innovation;
  x_(YAW) = std::atan2(std::sin(x_(YAW)), std::cos(x_(YAW)));
  // Joseph form: keeps P symmetric positive definite.
  const StateMatrix I_KH = StateMatrix::Identity() - K * H;
  P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
  result.accepted = true;
  return result;
}

UpdateResult Ekf::update_range(double range) {
  if (range <= 0.0 || range > config_.range_max) {
    return UpdateResult{};
  }
  Eigen::Matrix<double, 1, 1> z;
  z << range;
  Eigen::Matrix<double, 1, 1> R;
  R << config_.range_noise * config_.range_noise;
  return update<1>(z, R, [this](const StateVector& x) {
    Eigen::Matrix<double, 1, 1> y;
    y << predict_range(x);
    return y;
  });
}

UpdateResult Ekf::update_flow(const FlowSample& flow) {
  if (x_(PZ) < config_.flow_min_height || flow.dt <= 0.0) {
    return UpdateResult{};
  }
  const Eigen::Vector2d z(flow.dpixel_x, flow.dpixel_y);
  const Eigen::Matrix2d R =
      Eigen::Matrix2d::Identity() * config_.flow_noise * config_.flow_noise;
  return update<2>(z, R, [this, &flow](const StateVector& x) {
    return Eigen::Vector2d(predict_flow(x, flow));
  });
}

}  // namespace cf_ekf
