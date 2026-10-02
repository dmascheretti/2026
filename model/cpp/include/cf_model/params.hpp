// Physical parameters of the Crazyflie, loaded from model/params.yaml.
//
// This struct is the C++ mirror of params.yaml. No value is hardcoded here:
// everything comes from the YAML file at runtime.
#pragma once

#include <string>

#include <Eigen/Dense>

namespace cf_model {

struct Params {
  double gravity = 0.0;                  // m/s^2
  double mass = 0.0;                     // kg (base + flow deck)
  double motor_to_motor_diagonal = 0.0;  // m
  Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();  // kg*m^2, body frame
  double thrust_coefficient = 0.0;       // N/rpm^2
  double torque_coefficient = 0.0;       // N*m/rpm^2
  double thrust_pwm_c2 = 0.0;            // N   (F = c2*pwm^2 + c1*pwm + c0)
  double thrust_pwm_c1 = 0.0;            // N
  double thrust_pwm_c0 = 0.0;            // N
  double pwm_max = 0.0;                  // -
  double motor_time_constant = 0.0;      // s
  double attitude_time_constant = 0.0;   // s
  double flow_npix = 0.0;                // -
  double flow_thetapix = 0.0;            // rad
};

// Reads params.yaml. Throws std::runtime_error if a parameter is missing.
Params load_params(const std::string& yaml_path);

// Thrust of ONE motor [N] for a motor command pwm in [0, pwm_max].
double thrust_per_motor_from_pwm(const Params& p, double pwm);

// Inverse of the above: motor command for a desired thrust of ONE motor [N].
// The result is clamped to [0, pwm_max].
double pwm_from_thrust_per_motor(const Params& p, double thrust);

// Total thrust of all 4 motors at full command [N].
double max_total_thrust(const Params& p);

// Rotation body -> world, ZYX Euler angles (yaw psi, pitch theta, roll phi).
Eigen::Matrix3d rotation_zyx(double phi, double theta, double psi);

}  // namespace cf_model
