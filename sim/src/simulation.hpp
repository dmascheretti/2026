// One complete simulated system: drone + sensors + EKF + MPC + safety
// supervisor + radio latency, advanced one physics step at a time.
//
// Used by the batch simulator (closed_loop_sim) and by the interactive
// game (sim/game), so both run exactly the same code and timing.
#pragma once

#include <deque>
#include <functional>
#include <string>

#include <Eigen/Dense>

#include "cf_bringup/safety_supervisor.hpp"
#include "cf_bringup/scenario.hpp"
#include "cf_ekf/ekf.hpp"
#include "cf_model/params.hpp"
#include "cf_mpc/mpc_controller.hpp"
#include "plant.hpp"

namespace sim {

// Everything that happened at one control step (50 Hz), for logging and display.
struct ControlSample {
  double t = 0.0;                                         // s
  Eigen::Vector3d true_position = Eigen::Vector3d::Zero();  // m
  Eigen::Vector3d true_velocity = Eigen::Vector3d::Zero();  // m/s
  Eigen::Vector3d true_euler = Eigen::Vector3d::Zero();     // rad [roll, pitch, yaw]
  cf_mpc::VehicleState estimate;                          // what the controller used
  cf_bringup::Waypoint reference;                         // current reference
  cf_bringup::Command sent;                               // after the supervisor
  bool armed = false;
  bool stopped = false;
  std::string stop_reason;
  bool solver_ok = true;
  double solve_time = 0.0;                                // s
  double disturbance = 0.0;                               // N, observer estimate
};

class Simulation {
 public:
  // Reference waypoint as a function of time (the MPC previews the horizon).
  using ReferenceFunction = std::function<cf_bringup::Waypoint(double t)>;

  Simulation(const cf_model::Params& params, const SimConfig& sim_config,
             const cf_ekf::EkfConfig& ekf_config, const cf_bringup::SafetyConfig& safety_config,
             const Eigen::Vector3d& initial_position);

  void set_reference(ReferenceFunction reference) { reference_ = std::move(reference); }
  void set_use_ekf(bool use_ekf) { use_ekf_ = use_ekf; }
  void set_disturbance_observer(bool enabled) { mpc_.set_disturbance_observer(enabled); }

  // Like the ROS safety/arm service: reset the supervisor and start forwarding.
  void arm();
  // Like the kill key: latched stop at the next control step.
  void kill() { kill_ = true; }
  // Like the ROS ekf/reset service: estimate := true pose (drone on the floor).
  void reset_estimator();

  // Advances one physics step. Returns true if a control step ran
  // (then last_sample() is new).
  bool step();

  double time() const { return step_count_ * dt_; }
  double physics_dt() const { return dt_; }
  const ControlSample& last_sample() const { return sample_; }
  Quadrotor& drone() { return quad_; }
  const cf_ekf::Ekf& ekf() const { return ekf_; }
  const cf_mpc::MpcController& mpc() const { return mpc_; }

 private:
  void control_step(double t);

  cf_model::Params params_;
  SimConfig config_;
  Quadrotor quad_;
  cf_ekf::Ekf ekf_;
  cf_mpc::MpcController mpc_;
  cf_bringup::SafetySupervisor supervisor_;
  ReferenceFunction reference_;

  bool use_ekf_ = true;
  bool armed_ = false;
  bool kill_ = false;

  double dt_ = 0.0;
  int imu_every_ = 0;
  int flow_every_ = 0;
  int range_every_ = 0;
  int control_every_ = 0;
  long step_count_ = 0;

  struct PendingSetpoint {
    double release_time = 0.0;
    Setpoint setpoint;
  };
  std::deque<PendingSetpoint> radio_;
  Eigen::Vector3d last_gyro_ = Eigen::Vector3d::Zero();
  ControlSample sample_;
};

}  // namespace sim
