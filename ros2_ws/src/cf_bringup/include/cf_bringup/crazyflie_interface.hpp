// Conversion of our command (SI units, ZYX Euler, z up) to the Crazyswarm2
// "cmd_vel_legacy" topic (geometry_msgs/Twist). No ROS types here so the
// sign conventions can be unit-tested on their own.
//
// The chain, checked in the source code:
//   Crazyswarm2 crazyflie_server.cpp, cmd_vel_legacy_changed():
//     roll = linear.y, pitch = -linear.x, yawrate = angular.z,
//     thrust = clamp(linear.z, 0, 60000), all sent with sendSetpoint().
//   Firmware crtp_commander_rpyt.c:
//     attitude.roll = roll, attitude.pitch = pitch, attitudeRate.yaw = -yawrate
//     (degrees, degrees/s). Thrust is locked until one setpoint with thrust 0
//     has been received.
//   Firmware kalman_core.c: attitude.pitch = -pitch ("legacy" coordinates),
//     roll and yaw are not inverted.
// So for our pitch theta (positive = nose down):
//   firmware legacy pitch = -theta  =>  sent pitch = -theta  =>  linear.x = theta.
// And for our yaw rate r (positive = counter-clockwise from above):
//   firmware rate = -yawrate = r     =>  angular.z = -r.
#pragma once

#include "cf_bringup/safety_supervisor.hpp"

namespace cf_bringup {

struct LegacyTwist {
  double linear_x = 0.0;   // pitch, deg
  double linear_y = 0.0;   // roll, deg
  double linear_z = 0.0;   // thrust command, 0..60000
  double angular_z = 0.0;  // yaw rate, deg/s (sign flipped, see above)
};

LegacyTwist to_cmd_vel_legacy(const Command& command);

// Inverse mapping: what the firmware will try to fly for a given Twist.
// Used by the simulator bridge to play the role of Crazyswarm2 + firmware.
Command from_cmd_vel_legacy(const LegacyTwist& twist);

}  // namespace cf_bringup
