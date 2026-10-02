#include <gtest/gtest.h>

#include <cmath>

#include "cf_bringup/crazyflie_interface.hpp"

namespace {

TEST(CrazyflieInterface, UnitsAndSigns) {
  cf_bringup::Command c;
  c.roll = 0.1;        // rad, right side down
  c.pitch = 0.2;       // rad, nose down -> forward
  c.yaw_rate = 0.5;    // rad/s, counter-clockwise
  c.thrust_cmd = 40000.0;
  const cf_bringup::LegacyTwist t = cf_bringup::to_cmd_vel_legacy(c);
  EXPECT_NEAR(t.linear_y, 0.1 * 180.0 / M_PI, 1e-12);
  EXPECT_NEAR(t.linear_x, 0.2 * 180.0 / M_PI, 1e-12);   // Crazyswarm2 negates it again
  EXPECT_NEAR(t.angular_z, -0.5 * 180.0 / M_PI, 1e-12); // firmware negates yaw rate
  EXPECT_DOUBLE_EQ(t.linear_z, 40000.0);
}

TEST(CrazyflieInterface, ThrustClampedLikeCrazyswarm) {
  cf_bringup::Command c;
  c.thrust_cmd = 65000.0;
  EXPECT_DOUBLE_EQ(cf_bringup::to_cmd_vel_legacy(c).linear_z, 60000.0);
  c.thrust_cmd = -5.0;
  EXPECT_DOUBLE_EQ(cf_bringup::to_cmd_vel_legacy(c).linear_z, 0.0);
}

TEST(CrazyflieInterface, RoundTripThroughFirmwareChain) {
  cf_bringup::Command c;
  c.roll = -0.15;
  c.pitch = 0.07;
  c.yaw_rate = -0.3;
  c.thrust_cmd = 38000.0;
  const cf_bringup::Command back =
      cf_bringup::from_cmd_vel_legacy(cf_bringup::to_cmd_vel_legacy(c));
  EXPECT_NEAR(back.roll, c.roll, 1e-12);
  EXPECT_NEAR(back.pitch, c.pitch, 1e-12);
  EXPECT_NEAR(back.yaw_rate, c.yaw_rate, 1e-12);
  EXPECT_DOUBLE_EQ(back.thrust_cmd, c.thrust_cmd);
}

}  // namespace
