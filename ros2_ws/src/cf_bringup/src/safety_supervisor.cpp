#include "cf_bringup/safety_supervisor.hpp"

#include <algorithm>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace cf_bringup {

namespace {

YAML::Node require(const YAML::Node& root, const std::string& name) {
  if (!root[name]) {
    throw std::runtime_error("safety config: missing " + name);
  }
  return root[name];
}

Eigen::Vector3d read_vector3(const YAML::Node& root, const std::string& name) {
  const YAML::Node node = require(root, name);
  if (node.size() != 3) {
    throw std::runtime_error("safety config: " + name + " must have 3 entries");
  }
  return Eigen::Vector3d(node[0].as<double>(), node[1].as<double>(), node[2].as<double>());
}

}  // namespace

SafetyConfig load_safety_config(const std::string& yaml_path) {
  const YAML::Node root = YAML::LoadFile(yaml_path);
  SafetyConfig c;
  c.max_tilt = require(root, "max_tilt").as<double>();
  c.max_yaw_rate = require(root, "max_yaw_rate").as<double>();
  c.max_thrust_cmd = require(root, "max_thrust_cmd").as<double>();
  c.watchdog_timeout = require(root, "watchdog_timeout").as<double>();
  c.max_solver_failures = require(root, "max_solver_failures").as<int>();
  c.geofence_min = read_vector3(root, "geofence_min");
  c.geofence_max = read_vector3(root, "geofence_max");
  return c;
}

SafetySupervisor::SafetySupervisor(const SafetyConfig& config) : config_(config) {}

void SafetySupervisor::reset() {
  mode_ = Mode::kActive;
  stop_reason_.clear();
  consecutive_solver_failures_ = 0;
}

void SafetySupervisor::stop(const std::string& reason) {
  if (mode_ == Mode::kActive) {  // keep the first reason
    mode_ = Mode::kStopped;
    stop_reason_ = reason;
  }
}

SupervisorOutput SafetySupervisor::step(const SupervisorInput& input) {
  // 1. Checks that latch a stop. Order = priority of the reported reason.
  if (input.kill_switch) {
    stop("kill switch");
  }
  if (input.estimate_stamp < 0.0 ||
      input.now - input.estimate_stamp > config_.watchdog_timeout) {
    stop("watchdog: no state estimate");
  }
  if (input.command_stamp < 0.0 ||
      input.now - input.command_stamp > config_.watchdog_timeout) {
    stop("watchdog: no command");
  }
  for (int i = 0; i < 3; ++i) {
    if (input.position(i) < config_.geofence_min(i) ||
        input.position(i) > config_.geofence_max(i)) {
      stop("geofence");
    }
  }
  if (input.solver_ok) {
    consecutive_solver_failures_ = 0;
  } else {
    ++consecutive_solver_failures_;
    if (consecutive_solver_failures_ >= config_.max_solver_failures) {
      stop("MPC solver failures");
    }
  }

  SupervisorOutput output;
  output.mode = mode_;
  output.stop_reason = stop_reason_;
  if (mode_ == Mode::kStopped) {
    output.command = Command{};  // all zero: motors off
    return output;
  }

  // 2. Saturation of the command that goes out.
  output.command.roll = std::clamp(input.command.roll, -config_.max_tilt, config_.max_tilt);
  output.command.pitch = std::clamp(input.command.pitch, -config_.max_tilt, config_.max_tilt);
  output.command.yaw_rate =
      std::clamp(input.command.yaw_rate, -config_.max_yaw_rate, config_.max_yaw_rate);
  output.command.thrust_cmd =
      std::clamp(input.command.thrust_cmd, 0.0, config_.max_thrust_cmd);
  return output;
}

}  // namespace cf_bringup
