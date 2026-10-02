// Safety supervisor (no ROS in this file).
//
// Sits between the MPC and the radio. Every cycle it receives the latest
// estimate and MPC command (with their timestamps) and returns the command
// that is actually sent. It
//   - clamps tilt, yaw rate and thrust (independently of the MPC limits),
//   - stops the motors on kill switch, geofence violation, stale estimate
//     or command (watchdog), or repeated MPC solver failures.
// A stop is latched: once stopped, motors stay off until reset().
#pragma once

#include <string>

#include <Eigen/Dense>

namespace cf_bringup {

struct SafetyConfig {
  double max_tilt = 0.0;             // rad
  double max_yaw_rate = 0.0;         // rad/s
  double max_thrust_pwm = 0.0;       // -
  double watchdog_timeout = 0.0;     // s
  int max_solver_failures = 0;       // -
  Eigen::Vector3d geofence_min = Eigen::Vector3d::Zero();  // m
  Eigen::Vector3d geofence_max = Eigen::Vector3d::Zero();  // m
};

SafetyConfig load_safety_config(const std::string& yaml_path);

struct Command {
  double roll = 0.0;        // rad
  double pitch = 0.0;       // rad
  double yaw_rate = 0.0;    // rad/s
  double thrust_pwm = 0.0;  // per motor, 0 = motors off
};

struct SupervisorInput {
  double now = 0.0;                  // s
  Eigen::Vector3d position = Eigen::Vector3d::Zero();  // m, latest estimate
  double estimate_stamp = -1.0;      // s, time of latest estimate (<0: none)
  Command command;                   // latest MPC command
  double command_stamp = -1.0;       // s, time of latest command (<0: none)
  bool solver_ok = true;             // latest MPC solve succeeded
  bool kill_switch = false;          // operator pressed the kill key
};

enum class Mode { kActive, kStopped };

struct SupervisorOutput {
  Command command;          // what to send; thrust_pwm = 0 when stopped
  Mode mode = Mode::kActive;
  std::string stop_reason;  // empty while active
};

class SafetySupervisor {
 public:
  explicit SafetySupervisor(const SafetyConfig& config);

  SupervisorOutput step(const SupervisorInput& input);
  void reset();

  Mode mode() const { return mode_; }
  const std::string& stop_reason() const { return stop_reason_; }

 private:
  void stop(const std::string& reason);

  SafetyConfig config_;
  Mode mode_ = Mode::kActive;
  std::string stop_reason_;
  int consecutive_solver_failures_ = 0;
};

}  // namespace cf_bringup
