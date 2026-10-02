// Physical parameters of the Crazyflie, loaded from model/params.yaml.
//
// This struct is the C++ mirror of params.yaml. No value is hardcoded here:
// everything comes from the YAML file at runtime.
#pragma once

#include <string>

#include <Eigen/Dense>

namespace cf_model {

// How the firmware maps the 16-bit thrust command to thrust (params.yaml).
enum class ThrustCommandModel { kBatteryCompensated, kPwmPolynomial };

struct Params {
  double gravity = 0.0;                  // m/s^2
  double mass = 0.0;                     // kg (base + flow deck)
  double motor_to_motor_diagonal = 0.0;  // m
  Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();  // kg*m^2, body frame
  double thrust_coefficient = 0.0;       // N/rpm^2
  double torque_coefficient = 0.0;       // N*m/rpm^2
  ThrustCommandModel thrust_command_model = ThrustCommandModel::kBatteryCompensated;
  double thrust_cmd_max = 0.0;           // -   (65535)
  double max_thrust_per_motor = 0.0;     // N   (battery_compensated model)
  double min_thrust_per_motor = 0.0;     // N   (battery_compensated model)
  double thrust_pwm_c2 = 0.0;            // N   (pwm_polynomial model:
  double thrust_pwm_c1 = 0.0;            // N    F = c2*cmd^2 + c1*cmd + c0)
  double thrust_pwm_c0 = 0.0;            // N
  double motor_time_constant = 0.0;      // s
  double attitude_time_constant = 0.0;   // s
  double flow_npix = 0.0;                // -
  double flow_thetapix = 0.0;            // rad
};

// Reads params.yaml. Throws std::runtime_error if a parameter is missing.
Params load_params(const std::string& yaml_path);

// Thrust of ONE motor [N] for a firmware thrust command in [0, thrust_cmd_max].
double thrust_per_motor_from_cmd(const Params& p, double cmd);

// Inverse of the above: thrust command for a desired thrust of ONE motor [N].
// The result is clamped to [0, thrust_cmd_max].
double cmd_from_thrust_per_motor(const Params& p, double thrust);

// Total thrust of all 4 motors at full command [N].
double max_total_thrust(const Params& p);

// Rotation body -> world, ZYX Euler angles (yaw psi, pitch theta, roll phi).
Eigen::Matrix3d rotation_zyx(double phi, double theta, double psi);

// Inverse of rotation_zyx: [roll, pitch, yaw] of a body -> world rotation.
Eigen::Vector3d euler_zyx(const Eigen::Matrix3d& R);

}  // namespace cf_model
