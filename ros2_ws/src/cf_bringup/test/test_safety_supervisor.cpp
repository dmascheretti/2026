#include <gtest/gtest.h>

#include "cf_bringup/safety_supervisor.hpp"

namespace {

const std::string kSafetyConfig = CF_SAFETY_CONFIG;  // set by CMake

class SupervisorTest : public ::testing::Test {
 protected:
  cf_bringup::SafetyConfig config_ = cf_bringup::load_safety_config(kSafetyConfig);
  cf_bringup::SafetySupervisor supervisor_{config_};

  // A healthy input: fresh estimate and command, inside the geofence.
  cf_bringup::SupervisorInput healthy(double now = 1.0) {
    cf_bringup::SupervisorInput in;
    in.now = now;
    in.position = Eigen::Vector3d(0.0, 0.0, 0.5);
    in.estimate_stamp = now - 0.01;
    in.command_stamp = now - 0.01;
    in.command.roll = 0.1;
    in.command.pitch = -0.1;
    in.command.yaw_rate = 0.2;
    in.command.thrust_cmd = 40000.0;
    return in;
  }
};

TEST_F(SupervisorTest, HealthyCommandPassesThrough) {
  const cf_bringup::SupervisorOutput out = supervisor_.step(healthy());
  EXPECT_EQ(out.mode, cf_bringup::Mode::kActive);
  EXPECT_DOUBLE_EQ(out.command.roll, 0.1);
  EXPECT_DOUBLE_EQ(out.command.pitch, -0.1);
  EXPECT_DOUBLE_EQ(out.command.yaw_rate, 0.2);
  EXPECT_DOUBLE_EQ(out.command.thrust_cmd, 40000.0);
}

TEST_F(SupervisorTest, SaturatesCommand) {
  cf_bringup::SupervisorInput in = healthy();
  in.command.roll = 2.0;
  in.command.pitch = -2.0;
  in.command.yaw_rate = 10.0;
  in.command.thrust_cmd = 70000.0;
  const cf_bringup::SupervisorOutput out = supervisor_.step(in);
  EXPECT_DOUBLE_EQ(out.command.roll, config_.max_tilt);
  EXPECT_DOUBLE_EQ(out.command.pitch, -config_.max_tilt);
  EXPECT_DOUBLE_EQ(out.command.yaw_rate, config_.max_yaw_rate);
  EXPECT_DOUBLE_EQ(out.command.thrust_cmd, config_.max_thrust_cmd);
}

TEST_F(SupervisorTest, KillSwitchStopsAndLatches) {
  cf_bringup::SupervisorInput in = healthy();
  in.kill_switch = true;
  cf_bringup::SupervisorOutput out = supervisor_.step(in);
  EXPECT_EQ(out.mode, cf_bringup::Mode::kStopped);
  EXPECT_EQ(out.stop_reason, "kill switch");
  EXPECT_DOUBLE_EQ(out.command.thrust_cmd, 0.0);
  // Releasing the key does not restart the motors.
  out = supervisor_.step(healthy(1.1));
  EXPECT_EQ(out.mode, cf_bringup::Mode::kStopped);
  EXPECT_DOUBLE_EQ(out.command.thrust_cmd, 0.0);
  // Only an explicit reset does.
  supervisor_.reset();
  out = supervisor_.step(healthy(1.2));
  EXPECT_EQ(out.mode, cf_bringup::Mode::kActive);
}

TEST_F(SupervisorTest, StaleEstimateStops) {
  cf_bringup::SupervisorInput in = healthy();
  in.estimate_stamp = in.now - 0.101;
  const cf_bringup::SupervisorOutput out = supervisor_.step(in);
  EXPECT_EQ(out.mode, cf_bringup::Mode::kStopped);
  EXPECT_EQ(out.stop_reason, "watchdog: no state estimate");
}

TEST_F(SupervisorTest, EstimateJustInsideTimeoutIsFine) {
  cf_bringup::SupervisorInput in = healthy();
  in.estimate_stamp = in.now - 0.099;
  EXPECT_EQ(supervisor_.step(in).mode, cf_bringup::Mode::kActive);
}

TEST_F(SupervisorTest, StaleCommandStops) {
  cf_bringup::SupervisorInput in = healthy();
  in.command_stamp = in.now - 0.2;
  EXPECT_EQ(supervisor_.step(in).stop_reason, "watchdog: no command");
}

TEST_F(SupervisorTest, NoDataYetStops) {
  cf_bringup::SupervisorInput in = healthy();
  in.estimate_stamp = -1.0;
  EXPECT_EQ(supervisor_.step(in).mode, cf_bringup::Mode::kStopped);
}

TEST_F(SupervisorTest, GeofenceStops) {
  cf_bringup::SupervisorInput in = healthy();
  in.position.x() = config_.geofence_max.x() + 0.01;
  EXPECT_EQ(supervisor_.step(in).stop_reason, "geofence");
}

TEST_F(SupervisorTest, RepeatedSolverFailuresStop) {
  for (int i = 0; i < config_.max_solver_failures - 1; ++i) {
    cf_bringup::SupervisorInput in = healthy(1.0 + 0.01 * i);
    in.solver_ok = false;
    EXPECT_EQ(supervisor_.step(in).mode, cf_bringup::Mode::kActive);
  }
  // One success resets the counter.
  supervisor_.step(healthy(2.0));
  for (int i = 0; i < config_.max_solver_failures; ++i) {
    cf_bringup::SupervisorInput in = healthy(3.0 + 0.01 * i);
    in.solver_ok = false;
    supervisor_.step(in);
  }
  EXPECT_EQ(supervisor_.mode(), cf_bringup::Mode::kStopped);
  EXPECT_EQ(supervisor_.stop_reason(), "MPC solver failures");
}

}  // namespace
