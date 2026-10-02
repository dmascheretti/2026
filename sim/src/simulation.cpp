#include "simulation.hpp"

#include <cmath>
#include <stdexcept>

namespace sim {

namespace {

// Number of physics steps between two events at the given rate.
int steps_per_event(double physics_rate, double rate) {
  const double ratio = physics_rate / rate;
  const int steps = static_cast<int>(std::lround(ratio));
  if (std::abs(ratio - steps) > 1e-9) {
    throw std::runtime_error("all rates must divide physics_rate");
  }
  return steps;
}

}  // namespace

Simulation::Simulation(const cf_model::Params& params, const SimConfig& sim_config,
                       const cf_ekf::EkfConfig& ekf_config,
                       const cf_bringup::SafetyConfig& safety_config,
                       const Eigen::Vector3d& initial_position)
    : params_(params),
      config_(sim_config),
      quad_(params, sim_config),
      ekf_(params, ekf_config),
      mpc_(params),
      supervisor_(safety_config) {
  if (std::abs(1.0 / config_.control_rate - mpc_.sample_time()) > 1e-9) {
    throw std::runtime_error("control_rate does not match the MPC sample time");
  }
  dt_ = 1.0 / config_.physics_rate;
  imu_every_ = steps_per_event(config_.physics_rate, config_.imu_rate);
  flow_every_ = steps_per_event(config_.physics_rate, config_.flow_rate);
  range_every_ = steps_per_event(config_.physics_rate, config_.range_rate);
  control_every_ = steps_per_event(config_.physics_rate, config_.control_rate);

  quad_.set_position(initial_position);
  reset_estimator();
  reference_ = [initial_position](double) {
    cf_bringup::Waypoint wp;
    wp.position = initial_position;
    return wp;
  };
}

void Simulation::arm() {
  supervisor_.reset();
  mpc_.reset_disturbance();
  kill_ = false;
  armed_ = true;
}

void Simulation::reset_estimator() {
  const TrueState& s = quad_.state();
  const Eigen::Vector3d euler = cf_model::euler_zyx(s.attitude.toRotationMatrix());
  cf_ekf::StateVector x0 = cf_ekf::StateVector::Zero();
  x0.segment<3>(cf_ekf::PX) = s.position;
  x0(cf_ekf::YAW) = euler.z();
  ekf_.reset(x0);
}

bool Simulation::step() {
  const double t = time();

  // ---- sensors -> EKF ----
  if (step_count_ % imu_every_ == 0) {
    cf_ekf::ImuSample imu;
    imu.gyro = quad_.measure_gyro();
    imu.accel = quad_.measure_accel();
    last_gyro_ = imu.gyro;
    ekf_.predict(imu, imu_every_ * dt_);
  }
  if (step_count_ % range_every_ == 0) {
    ekf_.update_range(quad_.measure_range());
  }
  if (step_count_ % flow_every_ == 0) {
    cf_ekf::FlowSample flow;
    flow.dt = flow_every_ * dt_;
    const Eigen::Vector2d pixels = quad_.measure_flow(flow.dt);
    flow.dpixel_x = pixels.x();
    flow.dpixel_y = pixels.y();
    flow.gyro = last_gyro_;
    ekf_.update_flow(flow);
  }

  // ---- MPC + supervisor ----
  const bool control_ran = (step_count_ % control_every_ == 0);
  if (control_ran) {
    control_step(t);
  }

  // ---- radio -> drone ----
  while (!radio_.empty() && radio_.front().release_time <= t + 1e-12) {
    quad_.set_setpoint(radio_.front().setpoint);
    radio_.pop_front();
  }

  quad_.step(dt_);
  ++step_count_;
  return control_ran;
}

void Simulation::control_step(double t) {
  const TrueState& truth = quad_.state();
  const Eigen::Vector3d true_euler = cf_model::euler_zyx(truth.attitude.toRotationMatrix());

  cf_mpc::VehicleState estimate;
  if (use_ekf_) {
    const cf_ekf::StateVector& x = ekf_.state();
    estimate.position = x.segment<3>(cf_ekf::PX);
    estimate.velocity = x.segment<3>(cf_ekf::VX);
    estimate.roll = x(cf_ekf::ROLL);
    estimate.pitch = x(cf_ekf::PITCH);
    estimate.yaw = x(cf_ekf::YAW);
  } else {
    estimate.position = truth.position;
    estimate.velocity = truth.velocity;
    estimate.roll = true_euler.x();
    estimate.pitch = true_euler.y();
    estimate.yaw = true_euler.z();
  }

  const int horizon = mpc_.horizon_steps();
  cf_mpc::Reference reference;
  for (int k = 0; k <= horizon; ++k) {
    const cf_bringup::Waypoint wp = reference_(t + k * mpc_.sample_time());
    reference.positions.push_back(wp.position);
    reference.velocities.push_back(Eigen::Vector3d::Zero());
  }
  const cf_bringup::Waypoint current_wp = reference_(t);
  reference.yaw = current_wp.yaw;

  const cf_mpc::MpcOutput mpc_out = mpc_.compute(estimate, reference);

  // Not armed: nothing is forwarded, zero-thrust setpoints (motors off).
  cf_bringup::SupervisorOutput sup_out;
  sup_out.mode = cf_bringup::Mode::kStopped;
  if (armed_) {
    cf_bringup::SupervisorInput sup_in;
    sup_in.now = t;
    sup_in.position = estimate.position;
    sup_in.estimate_stamp = t;
    sup_in.command.roll = mpc_out.command.roll;
    sup_in.command.pitch = mpc_out.command.pitch;
    sup_in.command.yaw_rate = mpc_out.command.yaw_rate;
    sup_in.command.thrust_cmd = mpc_out.command.thrust_cmd;
    sup_in.command_stamp = t;
    sup_in.solver_ok = mpc_out.ok;
    sup_in.kill_switch = kill_;
    sup_out = supervisor_.step(sup_in);
  }

  PendingSetpoint pending;
  pending.release_time = t + config_.command_latency;
  pending.setpoint.roll = sup_out.command.roll;
  pending.setpoint.pitch = sup_out.command.pitch;
  pending.setpoint.yaw_rate = sup_out.command.yaw_rate;
  pending.setpoint.thrust_cmd = sup_out.command.thrust_cmd;
  radio_.push_back(pending);

  sample_.t = t;
  sample_.true_position = truth.position;
  sample_.true_velocity = truth.velocity;
  sample_.true_euler = true_euler;
  sample_.estimate = estimate;
  sample_.reference = current_wp;
  sample_.sent = sup_out.command;
  sample_.armed = armed_;
  sample_.stopped = (sup_out.mode == cf_bringup::Mode::kStopped);
  sample_.stop_reason = sup_out.stop_reason;
  sample_.solver_ok = mpc_out.ok;
  sample_.solve_time = mpc_out.solve_time;
  sample_.disturbance = mpc_out.disturbance;
}

}  // namespace sim
