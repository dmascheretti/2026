#include "cf_bringup/scenario.hpp"

#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace cf_bringup {

Scenario load_scenario(const std::string& yaml_path) {
  const YAML::Node root = YAML::LoadFile(yaml_path);
  Scenario s;
  s.duration = root["duration"].as<double>();
  const YAML::Node p0 = root["initial_position"];
  s.initial_position = Eigen::Vector3d(p0[0].as<double>(), p0[1].as<double>(), p0[2].as<double>());
  s.use_ekf = root["use_ekf"].as<bool>();
  if (root["kill_time"]) {
    s.kill_time = root["kill_time"].as<double>();
  }
  if (root["mass_factor"]) {
    s.mass_factor = root["mass_factor"].as<double>();
  }
  if (root["thrust_factor"]) {
    s.thrust_factor = root["thrust_factor"].as<double>();
  }
  if (root["disturbance_observer"]) {
    s.disturbance_observer = root["disturbance_observer"].as<bool>();
  }
  for (const YAML::Node& w : root["waypoints"]) {
    if (w.size() != 5) {
      throw std::runtime_error("scenario waypoint must be [time, x, y, z, yaw]");
    }
    Waypoint wp;
    wp.time = w[0].as<double>();
    wp.position = Eigen::Vector3d(w[1].as<double>(), w[2].as<double>(), w[3].as<double>());
    wp.yaw = w[4].as<double>();
    s.waypoints.push_back(wp);
  }
  if (s.waypoints.empty()) {
    throw std::runtime_error("scenario has no waypoints");
  }
  return s;
}

const Waypoint& waypoint_at(const Scenario& scenario, double t) {
  const Waypoint* active = &scenario.waypoints.front();
  for (const Waypoint& wp : scenario.waypoints) {
    if (wp.time <= t) {
      active = &wp;
    }
  }
  return *active;
}

}  // namespace cf_bringup
