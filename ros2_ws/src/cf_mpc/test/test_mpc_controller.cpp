#include <gtest/gtest.h>

#include <cmath>

#include "cf_mpc/mpc_controller.hpp"
#include "cf_mpc_constants.h"

namespace {

const std::string kParamsFile = CF_PARAMS_FILE;  // set by CMake

class MpcControllerTest : public ::testing::Test {
 protected:
  cf_model::Params params_ = cf_model::load_params(kParamsFile);
  cf_mpc::MpcController controller_{params_};

  cf_mpc::Reference hold(double x, double y, double z, double yaw = 0.0) {
    return cf_mpc::make_hold_reference(Eigen::Vector3d(x, y, z), yaw,
                                       controller_.horizon_steps());
  }
};

TEST(Helpers, WrapAngle) {
  EXPECT_NEAR(cf_mpc::wrap_angle(0.0), 0.0, 1e-12);
  EXPECT_NEAR(cf_mpc::wrap_angle(3.0 * M_PI / 2.0), -M_PI / 2.0, 1e-12);
  EXPECT_NEAR(cf_mpc::wrap_angle(-3.0 * M_PI / 2.0), M_PI / 2.0, 1e-12);
  EXPECT_NEAR(cf_mpc::wrap_angle(M_PI), M_PI, 1e-12);
}

TEST(Helpers, WorldToYawFrame) {
  // Drone facing world +y: a world +x vector is on the drone's right (-y).
  const Eigen::Vector3d v = cf_mpc::world_to_yaw_frame(Eigen::Vector3d(1, 0, 2), M_PI / 2.0);
  EXPECT_NEAR(v.x(), 0.0, 1e-12);
  EXPECT_NEAR(v.y(), -1.0, 1e-12);
  EXPECT_NEAR(v.z(), 2.0, 1e-12);
}

TEST_F(MpcControllerTest, AtReferenceGivesHover) {
  cf_mpc::VehicleState state;
  state.position = Eigen::Vector3d(0.3, -0.2, 1.0);
  const cf_mpc::MpcOutput out = controller_.compute(state, hold(0.3, -0.2, 1.0));
  ASSERT_TRUE(out.ok);
  EXPECT_NEAR(out.command.roll, 0.0, 1e-8);
  EXPECT_NEAR(out.command.pitch, 0.0, 1e-8);
  EXPECT_NEAR(out.command.yaw_rate, 0.0, 1e-12);
  EXPECT_NEAR(out.command.thrust, params_.mass * params_.gravity, 1e-8);
  EXPECT_GT(out.command.thrust_cmd, 0.0);
  EXPECT_LT(out.command.thrust_cmd, params_.thrust_cmd_max);
}

TEST_F(MpcControllerTest, TargetAheadPitchesForward) {
  cf_mpc::VehicleState state;
  state.position = Eigen::Vector3d(0.0, 0.0, 1.0);
  const cf_mpc::MpcOutput out = controller_.compute(state, hold(0.5, 0.0, 1.0));
  ASSERT_TRUE(out.ok);
  EXPECT_GT(out.command.pitch, 0.01);  // ax = g * pitch > 0
  EXPECT_NEAR(out.command.roll, 0.0, 1e-8);
}

TEST_F(MpcControllerTest, TargetAboveIncreasesThrust) {
  cf_mpc::VehicleState state;
  state.position = Eigen::Vector3d(0.0, 0.0, 1.0);
  const cf_mpc::MpcOutput out = controller_.compute(state, hold(0.0, 0.0, 1.3));
  ASSERT_TRUE(out.ok);
  EXPECT_GT(out.command.thrust, params_.mass * params_.gravity);
}

TEST_F(MpcControllerTest, CommandIsRotatedWithYaw) {
  // Facing world +y, target at world +x: the drone must roll (to its right),
  // not pitch.
  cf_mpc::VehicleState state;
  state.position = Eigen::Vector3d(0.0, 0.0, 1.0);
  state.yaw = M_PI / 2.0;
  const cf_mpc::MpcOutput out = controller_.compute(state, hold(0.5, 0.0, 1.0, M_PI / 2.0));
  ASSERT_TRUE(out.ok);
  EXPECT_GT(out.command.roll, 0.01);
  EXPECT_NEAR(out.command.pitch, 0.0, 1e-8);
}

TEST_F(MpcControllerTest, LargeErrorRespectsBounds) {
  cf_mpc::VehicleState state;
  const cf_mpc::MpcOutput out = controller_.compute(state, hold(10.0, -10.0, 10.0, 3.0));
  ASSERT_TRUE(out.ok);
  const double eps = 1e-8;
  EXPECT_LE(std::abs(out.command.roll), cf_mpc_constants::kMaxTilt + eps);
  EXPECT_LE(std::abs(out.command.pitch), cf_mpc_constants::kMaxTilt + eps);
  EXPECT_LE(std::abs(out.command.yaw_rate), cf_mpc_constants::kMaxYawRate + eps);
  EXPECT_LE(out.command.thrust,
            params_.mass * params_.gravity + cf_mpc_constants::kThrustDeltaMax + eps);
}

TEST_F(MpcControllerTest, SolvesFastEnoughFor50Hz) {
  cf_mpc::VehicleState state;
  const cf_mpc::MpcOutput out = controller_.compute(state, hold(1.0, 1.0, 1.0));
  ASSERT_TRUE(out.ok);
  EXPECT_LT(out.solve_time, 0.005);  // 5 ms budget out of 20 ms
}

// Vertical-only closed loop with a drone 10 % heavier than the model.
// Without the observer this settles about 7 cm low; with it, at the
// reference.
TEST_F(MpcControllerTest, DisturbanceObserverRemovesAltitudeOffset) {
  const double true_mass = 1.1 * params_.mass;
  const double dt = controller_.sample_time();
  cf_mpc::VehicleState state;
  state.position.z() = 0.5;
  const cf_mpc::Reference reference = hold(0.0, 0.0, 0.5);
  cf_mpc::MpcOutput out;
  for (int k = 0; k < 500; ++k) {  // 10 s
    out = controller_.compute(state, reference);
    ASSERT_TRUE(out.ok);
    const double az = out.command.thrust / true_mass - params_.gravity;
    state.position.z() += state.velocity.z() * dt + 0.5 * az * dt * dt;
    state.velocity.z() += az * dt;
  }
  EXPECT_NEAR(state.position.z(), 0.5, 0.002);
  // The estimated disturbance is the missing weight (negative force).
  EXPECT_NEAR(out.disturbance, -0.1 * params_.mass * params_.gravity, 0.002);
}

TEST_F(MpcControllerTest, ObserverOffNearTheFloor) {
  cf_mpc::VehicleState state;  // z = 0, resting: no acceleration despite low thrust
  for (int k = 0; k < 100; ++k) {
    const cf_mpc::MpcOutput out = controller_.compute(state, hold(0.0, 0.0, 0.0));
    EXPECT_DOUBLE_EQ(out.disturbance, 0.0);
  }
}

TEST_F(MpcControllerTest, WrongReferenceLengthThrows) {
  cf_mpc::VehicleState state;
  cf_mpc::Reference bad = hold(0.0, 0.0, 1.0);
  bad.positions.pop_back();
  bad.velocities.pop_back();
  EXPECT_ANY_THROW(controller_.compute(state, bad));
}

}  // namespace
