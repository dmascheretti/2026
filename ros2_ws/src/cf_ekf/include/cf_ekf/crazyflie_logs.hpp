// Conversion of Crazyflie log variables (as streamed by Crazyswarm2
// custom log topics) to the EKF's SI-unit samples.
//
// Log block "imu":  [acc.x, acc.y, acc.z, gyro.x, gyro.y, gyro.z]
//                   acc in g, gyro in deg/s, body frame (x fwd, y left, z up)
// Log block "flow": [motion.deltaX, motion.deltaY, range.zrange]
//                   raw flow counts, range in mm
//
// Flow axes: the firmware (deck/drivers/src/flowdeck_v1v2.c) uses
//   dpixelx = -motion.deltaY,  dpixely = -motion.deltaX
// because of how the sensor is mounted.
#pragma once

#include <vector>

#include "cf_ekf/ekf.hpp"

namespace cf_ekf {

ImuSample imu_from_log(const std::vector<float>& values, double gravity);

// Returns pixel motion (dpixelx, dpixely) in the EKF convention.
Eigen::Vector2d flow_pixels_from_log(const std::vector<float>& values);

// Range in m.
double range_from_log(const std::vector<float>& values);

}  // namespace cf_ekf
