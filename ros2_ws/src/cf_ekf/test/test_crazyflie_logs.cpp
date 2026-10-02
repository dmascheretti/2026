#include <gtest/gtest.h>

#include <cmath>

#include "cf_ekf/crazyflie_logs.hpp"

namespace {

TEST(CrazyflieLogs, ImuUnits) {
  const cf_ekf::ImuSample imu = cf_ekf::imu_from_log({0.0f, 0.0f, 1.0f, 90.0f, 0.0f, -180.0f}, 9.81);
  EXPECT_NEAR(imu.accel.z(), 9.81, 1e-6);
  EXPECT_NEAR(imu.gyro.x(), M_PI / 2.0, 1e-6);
  EXPECT_NEAR(imu.gyro.z(), -M_PI, 1e-6);
}

TEST(CrazyflieLogs, FlowAxesSwappedAndNegated) {
  const Eigen::Vector2d px = cf_ekf::flow_pixels_from_log({3.0f, -5.0f, 500.0f});
  EXPECT_DOUBLE_EQ(px.x(), 5.0);   // -deltaY
  EXPECT_DOUBLE_EQ(px.y(), -3.0);  // -deltaX
}

TEST(CrazyflieLogs, RangeInMeters) {
  EXPECT_DOUBLE_EQ(cf_ekf::range_from_log({0.0f, 0.0f, 750.0f}), 0.75);
}

TEST(CrazyflieLogs, WrongSizeThrows) {
  EXPECT_ANY_THROW(cf_ekf::imu_from_log({1.0f}, 9.81));
  EXPECT_ANY_THROW(cf_ekf::flow_pixels_from_log({1.0f}));
}

}  // namespace
