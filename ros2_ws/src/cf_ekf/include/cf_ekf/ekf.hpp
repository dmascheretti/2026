// Extended Kalman filter for the Crazyflie with Flow deck (no ROS here).
//
// State (world frame, z up):
//   x = [px, py, pz, vx, vy, vz, roll, pitch, yaw]       (9)
//
// Prediction uses the IMU as input (body-frame specific force and body
// rates). Corrections use the Flow deck: ToF range to the floor and optical
// flow pixel counts, with the same measurement model as the Crazyflie
// firmware (kalman_core/mm_flow.c, mm_tof.c). The floor is assumed flat
// at z = 0.
//
// Jacobians are computed numerically (central differences). This keeps the
// code short and easy to check; the cost (9 extra model evaluations per
// step) is negligible for 9 states.
#pragma once

#include <string>

#include <Eigen/Dense>

#include "cf_model/params.hpp"

namespace cf_ekf {

constexpr int kNx = 9;
using StateVector = Eigen::Matrix<double, kNx, 1>;
using StateMatrix = Eigen::Matrix<double, kNx, kNx>;

enum StateIndex { PX = 0, PY, PZ, VX, VY, VZ, ROLL, PITCH, YAW };

struct EkfConfig {
  double accel_noise = 0.0;           // m/s^2
  double gyro_noise = 0.0;            // rad/s
  double position_noise = 0.0;        // m/sqrt(s)
  double range_noise = 0.0;           // m
  double flow_noise = 0.0;            // pixels
  double range_max = 0.0;             // m
  double flow_min_height = 0.0;       // m
  double outlier_threshold = 0.0;     // -
  double initial_position_std = 0.0;  // m
  double initial_velocity_std = 0.0;  // m/s
  double initial_attitude_std = 0.0;  // rad
};

EkfConfig load_ekf_config(const std::string& yaml_path);

struct ImuSample {
  Eigen::Vector3d gyro = Eigen::Vector3d::Zero();   // rad/s, body
  Eigen::Vector3d accel = Eigen::Vector3d::Zero();  // m/s^2, body, specific force
};

struct FlowSample {
  double dt = 0.0;         // s, integration time of the flow sample
  double dpixel_x = 0.0;   // pixels
  double dpixel_y = 0.0;   // pixels
  Eigen::Vector3d gyro = Eigen::Vector3d::Zero();  // rad/s, body, at the same time
};

// Result of a measurement update (for logging and tests).
struct UpdateResult {
  bool accepted = false;
  double mahalanobis_sq = 0.0;
};

class Ekf {
 public:
  Ekf(const cf_model::Params& params, const EkfConfig& config);

  void reset(const StateVector& x0);

  void predict(const ImuSample& imu, double dt);
  UpdateResult update_range(double range);
  UpdateResult update_flow(const FlowSample& flow);

  const StateVector& state() const { return x_; }
  const StateMatrix& covariance() const { return P_; }

  // Model functions, public so they can be tested on their own.
  StateVector propagate(const StateVector& x, const ImuSample& imu, double dt) const;
  double predict_range(const StateVector& x) const;
  Eigen::Vector2d predict_flow(const StateVector& x, const FlowSample& flow) const;

 private:
  // Generic scalar / vector EKF update with numeric Jacobian H.
  template <int M, typename MeasurementFunction>
  UpdateResult update(const Eigen::Matrix<double, M, 1>& z,
                      const Eigen::Matrix<double, M, M>& R,
                      MeasurementFunction h);

  cf_model::Params params_;
  EkfConfig config_;
  StateVector x_ = StateVector::Zero();
  StateMatrix P_ = StateMatrix::Identity();
};

}  // namespace cf_ekf
