// A flight scenario: waypoints over time (sim/scenarios/*.yaml).
// Used by the simulator and by the mission node, so simulation and
// flight run the same scenario files.
#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>

namespace cf_bringup {

struct Waypoint {
  double time = 0.0;                                   // s since start
  Eigen::Vector3d position = Eigen::Vector3d::Zero();  // m, world
  double yaw = 0.0;                                    // rad
};

struct Scenario {
  double duration = 0.0;                                       // s
  Eigen::Vector3d initial_position = Eigen::Vector3d::Zero();  // m
  bool use_ekf = true;       // simulator only: false = controller gets the true state
  double kill_time = -1.0;   // simulator only: s, < 0 = never
  std::vector<Waypoint> waypoints;
};

Scenario load_scenario(const std::string& yaml_path);

// The waypoint active at time t (the last one whose time has passed).
const Waypoint& waypoint_at(const Scenario& scenario, double t);

}  // namespace cf_bringup
