#include <gtest/gtest.h>

#include "cf_model/params.hpp"

namespace {

const std::string kParamsFile = CF_PARAMS_FILE;  // set by CMake

TEST(Params, LoadsAllValues) {
  const cf_model::Params p = cf_model::load_params(kParamsFile);
  EXPECT_NEAR(p.gravity, 9.81, 1e-12);
  EXPECT_NEAR(p.mass, 0.027 + 0.0016, 1e-12);
  EXPECT_GT(p.inertia(2, 2), p.inertia(0, 0));  // yaw inertia is the largest
  EXPECT_DOUBLE_EQ(p.inertia(0, 1), p.inertia(1, 0));  // symmetric
  EXPECT_GT(p.attitude_time_constant, 0.0);
}

TEST(Params, MissingFileThrows) {
  EXPECT_ANY_THROW(cf_model::load_params("does_not_exist.yaml"));
}

TEST(Params, BatteryCompensatedThrustIsLinear) {
  cf_model::Params p = cf_model::load_params(kParamsFile);
  p.thrust_command_model = cf_model::ThrustCommandModel::kBatteryCompensated;
  EXPECT_NEAR(cf_model::thrust_per_motor_from_cmd(p, p.thrust_cmd_max), p.max_thrust_per_motor, 1e-12);
  EXPECT_NEAR(cf_model::thrust_per_motor_from_cmd(p, p.thrust_cmd_max / 2.0),
              p.max_thrust_per_motor / 2.0, 1e-12);
  // Below THRUST_MIN the firmware switches the motor off.
  EXPECT_DOUBLE_EQ(cf_model::thrust_per_motor_from_cmd(p, 1000.0), 0.0);
}

TEST(Params, ThrustCommandRoundTripBothModels) {
  cf_model::Params p = cf_model::load_params(kParamsFile);
  for (auto model : {cf_model::ThrustCommandModel::kBatteryCompensated,
                     cf_model::ThrustCommandModel::kPwmPolynomial}) {
    p.thrust_command_model = model;
    for (double cmd = 10000.0; cmd <= p.thrust_cmd_max; cmd += 5000.0) {
      const double thrust = cf_model::thrust_per_motor_from_cmd(p, cmd);
      EXPECT_NEAR(cf_model::cmd_from_thrust_per_motor(p, thrust), cmd, 1e-6);
    }
  }
}

TEST(Params, ThrustCommandIsClamped) {
  const cf_model::Params p = cf_model::load_params(kParamsFile);
  EXPECT_DOUBLE_EQ(cf_model::cmd_from_thrust_per_motor(p, 100.0), p.thrust_cmd_max);
  EXPECT_DOUBLE_EQ(cf_model::cmd_from_thrust_per_motor(p, -1.0), 0.0);
}

TEST(Params, HoverNeedsLessThanMaxThrust) {
  const cf_model::Params p = cf_model::load_params(kParamsFile);
  EXPECT_LT(p.mass * p.gravity, cf_model::max_total_thrust(p));
}

TEST(Rotation, SmallPitchTiltsThrustTowardsPositiveX) {
  // Body z-axis expressed in world: positive pitch -> thrust leans to +x.
  const Eigen::Vector3d z_body = cf_model::rotation_zyx(0.0, 0.1, 0.0).col(2);
  EXPECT_GT(z_body.x(), 0.0);
  // Positive roll -> thrust leans to -y.
  const Eigen::Vector3d z_body_roll = cf_model::rotation_zyx(0.1, 0.0, 0.0).col(2);
  EXPECT_LT(z_body_roll.y(), 0.0);
}

TEST(Rotation, EulerRoundTrip) {
  const Eigen::Vector3d angles(0.1, -0.2, 2.5);
  const Eigen::Vector3d back = cf_model::euler_zyx(cf_model::rotation_zyx(angles.x(), angles.y(), angles.z()));
  EXPECT_LT((back - angles).norm(), 1e-12);
}

}  // namespace
