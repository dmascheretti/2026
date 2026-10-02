#include "cf_bringup/crazyflie_interface.hpp"

#include <algorithm>
#include <cmath>

namespace cf_bringup {

namespace {
constexpr double kRadToDeg = 180.0 / M_PI;
constexpr double kDegToRad = M_PI / 180.0;
constexpr double kCrazyswarmThrustLimit = 60000.0;  // clamp in crazyflie_server.cpp
}  // namespace

LegacyTwist to_cmd_vel_legacy(const Command& command) {
  LegacyTwist twist;
  twist.linear_y = command.roll * kRadToDeg;
  twist.linear_x = command.pitch * kRadToDeg;
  twist.angular_z = -command.yaw_rate * kRadToDeg;
  twist.linear_z = std::clamp(command.thrust_cmd, 0.0, kCrazyswarmThrustLimit);
  return twist;
}

Command from_cmd_vel_legacy(const LegacyTwist& twist) {
  // Step by step through Crazyswarm2 and the firmware (see header).
  const double sent_roll = twist.linear_y;          // deg
  const double sent_pitch = -twist.linear_x;        // deg, Crazyswarm2 negates
  const double sent_yawrate = twist.angular_z;      // deg/s
  const double sent_thrust = std::clamp(twist.linear_z, 0.0, kCrazyswarmThrustLimit);

  const double legacy_pitch_setpoint = sent_pitch;  // firmware compares with legacy pitch
  const double yaw_rate_setpoint = -sent_yawrate;   // firmware negates

  Command command;
  command.roll = sent_roll * kDegToRad;
  command.pitch = -legacy_pitch_setpoint * kDegToRad;  // legacy pitch = -theta
  command.yaw_rate = yaw_rate_setpoint * kDegToRad;
  command.thrust_cmd = sent_thrust;
  return command;
}

}  // namespace cf_bringup
