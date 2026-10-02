#include "cf_mpc/mpc_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "acados_c/ocp_nlp_interface.h"
#include "acados_solver_cf_mpc.h"
#include "cf_mpc_constants.h"

namespace cf_mpc {

static_assert(CF_MPC_NX == kNx, "generated solver has a different state size");
static_assert(CF_MPC_NU == kNu, "generated solver has a different input size");

// ---------------------------------------------------------------- AcadosMpc

AcadosMpc::AcadosMpc() {
  capsule_ = cf_mpc_acados_create_capsule();
  if (cf_mpc_acados_create(capsule_) != 0) {
    throw std::runtime_error("acados: could not create the cf_mpc solver");
  }
}

AcadosMpc::~AcadosMpc() {
  cf_mpc_acados_free(capsule_);
  cf_mpc_acados_free_capsule(capsule_);
}

int AcadosMpc::horizon_steps() const { return CF_MPC_N; }

double AcadosMpc::sample_time() const { return cf_mpc_constants::kSampleTime; }

int AcadosMpc::solve(const Eigen::Matrix<double, kNx, 1>& x0,
                     const std::vector<Eigen::Matrix<double, kNx, 1>>& x_ref,
                     Eigen::Matrix<double, kNu, 1>& u0, double& solve_time) {
  const int n = CF_MPC_N;
  if (static_cast<int>(x_ref.size()) != n + 1) {
    throw std::invalid_argument("MPC reference must have horizon_steps + 1 points");
  }
  ocp_nlp_config* config = cf_mpc_acados_get_nlp_config(capsule_);
  ocp_nlp_dims* dims = cf_mpc_acados_get_nlp_dims(capsule_);
  ocp_nlp_in* in = cf_mpc_acados_get_nlp_in(capsule_);
  ocp_nlp_out* out = cf_mpc_acados_get_nlp_out(capsule_);

  // Initial state constraint: x_0 = x0 (lower bound = upper bound).
  double x0_array[kNx];
  for (int i = 0; i < kNx; ++i) {
    x0_array[i] = x0(i);
  }
  ocp_nlp_constraints_model_set(config, dims, in, out, 0, "lbx", x0_array);
  ocp_nlp_constraints_model_set(config, dims, in, out, 0, "ubx", x0_array);

  // Stage references y_ref = [x_ref; u_ref] with u_ref = 0 (hover input).
  for (int k = 0; k < n; ++k) {
    double yref[kNx + kNu];
    for (int i = 0; i < kNx; ++i) {
      yref[i] = x_ref[k](i);
    }
    for (int i = 0; i < kNu; ++i) {
      yref[kNx + i] = 0.0;
    }
    ocp_nlp_cost_model_set(config, dims, in, k, "yref", yref);
  }
  double yref_terminal[kNx];
  for (int i = 0; i < kNx; ++i) {
    yref_terminal[i] = x_ref[n](i);
  }
  ocp_nlp_cost_model_set(config, dims, in, n, "yref", yref_terminal);

  const int status = cf_mpc_acados_solve(capsule_);

  double u0_array[kNu];
  ocp_nlp_out_get(config, dims, out, 0, "u", u0_array);
  for (int i = 0; i < kNu; ++i) {
    u0(i) = u0_array[i];
  }
  ocp_nlp_get(cf_mpc_acados_get_nlp_solver(capsule_), "time_tot", &solve_time);
  return status;
}

// ------------------------------------------------------------ MpcController

MpcController::MpcController(const cf_model::Params& params) : params_(params) {}

MpcOutput MpcController::compute(const VehicleState& state, const Reference& reference) {
  const int n = mpc_.horizon_steps();
  if (static_cast<int>(reference.positions.size()) != n + 1 ||
      static_cast<int>(reference.velocities.size()) != n + 1) {
    throw std::invalid_argument("MPC reference must have horizon_steps + 1 points");
  }
  const double yaw = state.yaw;
  update_disturbance(state);

  // Current state in the yaw frame.
  Eigen::Matrix<double, kNx, 1> x0;
  x0.segment<3>(0) = world_to_yaw_frame(state.position, yaw);
  x0.segment<3>(3) = world_to_yaw_frame(state.velocity, yaw);
  x0(6) = state.roll;
  x0(7) = state.pitch;

  // Reference in the yaw frame. Roll/pitch reference is 0 (level).
  std::vector<Eigen::Matrix<double, kNx, 1>> x_ref(n + 1);
  for (int k = 0; k <= n; ++k) {
    x_ref[k].setZero();
    x_ref[k].segment<3>(0) = world_to_yaw_frame(reference.positions[k], yaw);
    x_ref[k].segment<3>(3) = world_to_yaw_frame(reference.velocities[k], yaw);
  }

  Eigen::Matrix<double, kNu, 1> u0;
  MpcOutput output;
  output.solver_status = mpc_.solve(x0, x_ref, u0, output.solve_time);
  output.ok = (output.solver_status == 0);

  const double hover_thrust = params_.mass * params_.gravity;
  AttitudeCommand& cmd = output.command;
  if (output.ok) {
    cmd.roll = u0(0);
    cmd.pitch = u0(1);
    // Cancel the estimated disturbance, stay inside the MPC thrust bounds.
    cmd.thrust = std::clamp(hover_thrust + u0(2) - disturbance_,
                            hover_thrust + cf_mpc_constants::kThrustDeltaMin,
                            hover_thrust + cf_mpc_constants::kThrustDeltaMax);
  } else {
    // Solver failed: level attitude, hover thrust. The safety supervisor
    // decides what happens next.
    cmd.roll = 0.0;
    cmd.pitch = 0.0;
    cmd.thrust = hover_thrust;
  }

  // Yaw: proportional control on the wrapped yaw error, saturated.
  const double yaw_error = wrap_angle(reference.yaw - yaw);
  cmd.yaw_rate = std::clamp(cf_mpc_constants::kYawGain * yaw_error,
                            -cf_mpc_constants::kMaxYawRate,
                            cf_mpc_constants::kMaxYawRate);

  cmd.thrust_cmd = cf_model::cmd_from_thrust_per_motor(params_, cmd.thrust / 4.0);
  output.disturbance = disturbance_;

  previous_thrust_ = cmd.thrust;
  previous_tilt_cos_ = std::cos(state.roll) * std::cos(state.pitch);
  previous_vz_ = state.velocity.z();
  have_previous_ = true;
  return output;
}

void MpcController::reset_disturbance() {
  disturbance_ = 0.0;
  have_previous_ = false;
}

void MpcController::update_disturbance(const VehicleState& state) {
  // Off near the floor: there the ground pushes back and would be
  // mistaken for a disturbance.
  if (!observer_enabled_ || !have_previous_ ||
      state.position.z() < cf_mpc_constants::kObserverMinHeight) {
    return;
  }
  const double dt = cf_mpc_constants::kSampleTime;
  const double m = params_.mass;
  // Vertical force needed to explain the measured change of vz ...
  const double measured_force = m * (state.velocity.z() - previous_vz_) / dt;
  // ... minus the force our model predicts from the thrust we sent.
  const double model_force = previous_thrust_ * previous_tilt_cos_ - m * params_.gravity;
  const double raw_disturbance = measured_force - model_force;
  // First-order low-pass: the raw value is very noisy (it differentiates vz).
  const double alpha = dt / cf_mpc_constants::kDisturbanceTimeConstant;
  disturbance_ += alpha * (raw_disturbance - disturbance_);
  const double limit = cf_mpc_constants::kMaxDisturbanceRatio * m * params_.gravity;
  disturbance_ = std::clamp(disturbance_, -limit, limit);
}

// ------------------------------------------------------------------ helpers

Reference make_hold_reference(const Eigen::Vector3d& position, double yaw,
                              int horizon_steps) {
  Reference reference;
  reference.positions.assign(horizon_steps + 1, position);
  reference.velocities.assign(horizon_steps + 1, Eigen::Vector3d::Zero());
  reference.yaw = yaw;
  return reference;
}

double wrap_angle(double angle) {
  double wrapped = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (wrapped <= 0.0) {
    wrapped += 2.0 * M_PI;
  }
  return wrapped - M_PI;
}

Eigen::Vector3d world_to_yaw_frame(const Eigen::Vector3d& v, double yaw) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return Eigen::Vector3d(c * v.x() + s * v.y(), -s * v.x() + c * v.y(), v.z());
}

}  // namespace cf_mpc
