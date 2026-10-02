// Simulated Crazyflie: rigid body, motors, a stand-in for the firmware
// attitude controller, and the sensors (IMU, ToF range, optical flow).
#pragma once

#include <random>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "cf_model/params.hpp"

namespace sim {

struct SimConfig {
  double physics_rate = 0.0;     // Hz
  double imu_rate = 0.0;         // Hz
  double flow_rate = 0.0;        // Hz
  double range_rate = 0.0;       // Hz
  double control_rate = 0.0;     // Hz
  double command_latency = 0.0;  // s
  double attitude_gain = 0.0;    // 1/s
  double rate_gain = 0.0;        // 1/s
  double gyro_noise = 0.0;       // rad/s
  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();   // rad/s
  double accel_noise = 0.0;      // m/s^2
  Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();  // m/s^2
  double range_noise = 0.0;      // m
  double flow_noise = 0.0;       // pixels
  double mass_factor = 1.0;      // -
  double thrust_factor = 1.0;    // -
  int random_seed = 0;
};

SimConfig load_sim_config(const std::string& yaml_path);

// Setpoint as sent over the radio (same content as cflib send_setpoint).
struct Setpoint {
  double roll = 0.0;        // rad
  double pitch = 0.0;       // rad
  double yaw_rate = 0.0;    // rad/s
  double thrust_cmd = 0.0;  // per motor, 0 = motors off
};

struct TrueState {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();         // m, world
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();         // m/s, world
  Eigen::Quaterniond attitude = Eigen::Quaterniond::Identity();  // body -> world
  Eigen::Vector3d body_rate = Eigen::Vector3d::Zero();        // rad/s, body
  Eigen::Vector4d motor_thrust = Eigen::Vector4d::Zero();     // N, per motor
  bool on_ground = true;
};

class Quadrotor {
 public:
  Quadrotor(const cf_model::Params& params, const SimConfig& config);

  void set_position(const Eigen::Vector3d& position);
  void set_setpoint(const Setpoint& setpoint) { setpoint_ = setpoint; }

  // Disturbances for experiments and the game: an external force on the
  // body (wind gust, push) and a different true mass (extra payload).
  void set_external_force(const Eigen::Vector3d& force_world) { external_force_ = force_world; }
  void set_mass_factor(double factor) { true_mass_ = params_.mass * factor; }

  // Advances the simulation by dt (inner controller + motors + rigid body).
  void step(double dt);

  const TrueState& state() const { return state_; }

  // Sensors (with noise and bias).
  Eigen::Vector3d measure_gyro();
  Eigen::Vector3d measure_accel();
  double measure_range();
  Eigen::Vector2d measure_flow(double flow_dt);

 private:
  Eigen::Vector4d inner_controller() const;

  cf_model::Params params_;  // what the controller believes
  SimConfig config_;
  double true_mass_ = 0.0;   // what the simulated drone actually weighs
  Eigen::Matrix4d allocation_ = Eigen::Matrix4d::Zero();      // motor thrusts -> [T, tau]
  Eigen::Matrix4d allocation_inv_ = Eigen::Matrix4d::Zero();
  double max_motor_thrust_ = 0.0;
  Setpoint setpoint_;
  TrueState state_;
  Eigen::Vector3d last_specific_force_body_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d external_force_ = Eigen::Vector3d::Zero();  // N, world
  std::mt19937 rng_;
  std::normal_distribution<double> unit_normal_{0.0, 1.0};
};

}  // namespace sim
