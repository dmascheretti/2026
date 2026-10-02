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
  const YAML::Node poly = get_value(root, "rotor", "thrust_pwm_poly");
  if (poly.size() != 3) {
    throw std::runtime_error("params.yaml: rotor.thrust_pwm_poly must have 3 entries");
  }
  p.thrust_pwm_c2 = poly[0].as<double>();
  p.thrust_pwm_c1 = poly[1].as<double>();
  p.thrust_pwm_c0 = poly[2].as<double>();
  p.pwm_max = get_double(root, "rotor", "pwm_max");
  p.motor_time_constant = get_double(root, "rotor", "motor_time_constant");
  p.attitude_time_constant = get_double(root, "inner_loop", "attitude_time_constant");
  p.flow_npix = get_double(root, "sensors", "flow_npix");
  p.flow_thetapix = get_double(root, "sensors", "flow_thetapix");
  return p;
}

double thrust_per_motor_from_pwm(const Params& p, double pwm) {
  return p.thrust_pwm_c2 * pwm * pwm + p.thrust_pwm_c1 * pwm + p.thrust_pwm_c0;
}

double pwm_from_thrust_per_motor(const Params& p, double thrust) {
  // Solve c2*pwm^2 + c1*pwm + (c0 - thrust) = 0 and take the positive root.
  const double a = p.thrust_pwm_c2;
  const double b = p.thrust_pwm_c1;
  const double c = p.thrust_pwm_c0 - thrust;
  const double discriminant = b * b - 4.0 * a * c;
  if (discriminant < 0.0) {
    return 0.0;
  }
  const double pwm = (-b + std::sqrt(discriminant)) / (2.0 * a);
  return std::clamp(pwm, 0.0, p.pwm_max);
}

double max_total_thrust(const Params& p) {
  return 4.0 * thrust_per_motor_from_pwm(p, p.pwm_max);
}

Eigen::Matrix3d rotation_zyx(double phi, double theta, double psi) {
  const Eigen::Matrix3d rz = Eigen::AngleAxisd(psi, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Matrix3d ry = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const Eigen::Matrix3d rx = Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitX()).toRotationMatrix();
  return rz * ry * rx;
}

}  // namespace cf_model
