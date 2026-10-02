#include "cf_ekf/crazyflie_logs.hpp"

#include <cmath>
#include <stdexcept>

namespace cf_ekf {

ImuSample imu_from_log(const std::vector<float>& values, double gravity) {
  if (values.size() != 6) {
    throw std::invalid_argument("imu log block must have 6 values");
  }
  const double deg_to_rad = M_PI / 180.0;
  ImuSample imu;
  imu.accel = Eigen::Vector3d(values[0], values[1], values[2]) * gravity;
  imu.gyro = Eigen::Vector3d(values[3], values[4], values[5]) * deg_to_rad;
  return imu;
}

Eigen::Vector2d flow_pixels_from_log(const std::vector<float>& values) {
  if (values.size() != 3) {
    throw std::invalid_argument("flow log block must have 3 values");
  }
  const double delta_x = values[0];
  const double delta_y = values[1];
  return Eigen::Vector2d(-delta_y, -delta_x);
}

double range_from_log(const std::vector<float>& values) {
  if (values.size() != 3) {
    throw std::invalid_argument("flow log block must have 3 values");
  }
  return values[2] / 1000.0;
}

}  // namespace cf_ekf
