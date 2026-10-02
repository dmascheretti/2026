#include "cf_model/params.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace cf_model {

namespace {

// Returns node[group][name]["value"], or throws with a clear message.
YAML::Node get_value(const YAML::Node& root, const std::string& group,
                     const std::string& name) {
  const YAML::Node entry = root[group][name];
  if (!entry || !entry["value"]) {
    throw std::runtime_error("params.yaml: missing " + group + "." + name);
  }
  return entry["value"];
}

double get_double(const YAML::Node& root, const std::string& group,
                  const std::string& name) {
  return get_value(root, group, name).as<double>();
}

}  // namespace

Params load_params(const std::string& yaml_path) {
  const YAML::Node root = YAML::LoadFile(yaml_path);
  Params p;
  p.gravity = get_double(root, "environment", "gravity");
  p.mass = get_double(root, "body", "mass_base") +
           get_double(root, "body", "mass_flow_deck");
  p.motor_to_motor_diagonal = get_double(root, "body", "motor_to_motor_diagonal");

  const YAML::Node inertia = get_value(root, "body", "inertia");
  if (inertia.size() != 9) {
    throw std::runtime_error("params.yaml: body.inertia must have 9 entries");
  }
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      p.inertia(row, col) = inertia[3 * row + col].as<double>();
    }
  }

  p.thrust_coefficient = get_double(root, "rotor", "thrust_coefficient");
  p.torque_coefficient = get_double(root, "rotor", "torque_coefficient");
  const std::string thrust_model = get_value(root, "rotor", "thrust_command_model").as<std::string>();
  if (thrust_model == "battery_compensated") {
    p.thrust_command_model = ThrustCommandModel::kBatteryCompensated;
  } else if (thrust_model == "pwm_polynomial") {
    p.thrust_command_model = ThrustCommandModel::kPwmPolynomial;
  } else {
    throw std::runtime_error("params.yaml: unknown rotor.thrust_command_model " + thrust_model);
  }
  p.thrust_cmd_max = get_double(root, "rotor", "thrust_cmd_max");
  p.max_thrust_per_motor = get_double(root, "rotor", "max_thrust_per_motor");
  p.min_thrust_per_motor = get_double(root, "rotor", "min_thrust_per_motor");
  const YAML::Node poly = get_value(root, "rotor", "thrust_pwm_poly");
  if (poly.size() != 3) {
    throw std::runtime_error("params.yaml: rotor.thrust_pwm_poly must have 3 entries");
  }
  p.thrust_pwm_c2 = poly[0].as<double>();
  p.thrust_pwm_c1 = poly[1].as<double>();
  p.thrust_pwm_c0 = poly[2].as<double>();
  p.motor_time_constant = get_double(root, "rotor", "motor_time_constant");
  p.attitude_time_constant = get_double(root, "inner_loop", "attitude_time_constant");
  p.flow_npix = get_double(root, "sensors", "flow_npix");
  p.flow_thetapix = get_double(root, "sensors", "flow_thetapix");
  return p;
}

double thrust_per_motor_from_cmd(const Params& p, double cmd) {
  if (p.thrust_command_model == ThrustCommandModel::kBatteryCompensated) {
    // Firmware motors.c: thrust = cmd / 65535 * THRUST_MAX, off below THRUST_MIN.
    const double thrust = cmd / p.thrust_cmd_max * p.max_thrust_per_motor;
    return thrust >= p.min_thrust_per_motor ? thrust : 0.0;
  }
  return p.thrust_pwm_c2 * cmd * cmd + p.thrust_pwm_c1 * cmd + p.thrust_pwm_c0;
}

double cmd_from_thrust_per_motor(const Params& p, double thrust) {
  double cmd = 0.0;
  if (p.thrust_command_model == ThrustCommandModel::kBatteryCompensated) {
    cmd = thrust / p.max_thrust_per_motor * p.thrust_cmd_max;
  } else {
    // Solve c2*cmd^2 + c1*cmd + (c0 - thrust) = 0, positive root.
    const double a = p.thrust_pwm_c2;
    const double b = p.thrust_pwm_c1;
    const double c = p.thrust_pwm_c0 - thrust;
    const double discriminant = b * b - 4.0 * a * c;
    if (discriminant < 0.0) {
      return 0.0;
    }
    cmd = (-b + std::sqrt(discriminant)) / (2.0 * a);
  }
  return std::clamp(cmd, 0.0, p.thrust_cmd_max);
}

double max_total_thrust(const Params& p) {
  return 4.0 * thrust_per_motor_from_cmd(p, p.thrust_cmd_max);
}

Eigen::Matrix3d rotation_zyx(double phi, double theta, double psi) {
  const Eigen::Matrix3d rz = Eigen::AngleAxisd(psi, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Matrix3d ry = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const Eigen::Matrix3d rx = Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitX()).toRotationMatrix();
  return rz * ry * rx;
}

}  // namespace cf_model
