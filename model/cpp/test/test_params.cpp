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

TEST(Params, PwmThrustRoundTrip) {
  const cf_model::Params p = cf_model::load_params(kParamsFile);
  for (double pwm = 1000.0; pwm <= p.pwm_max; pwm += 5000.0) {
    const double thrust = cf_model::thrust_per_motor_from_pwm(p, pwm);
    EXPECT_NEAR(cf_model::pwm_from_thrust_per_motor(p, thrust), pwm, 1e-6);
  }
}

TEST(Params, PwmIsClamped) {
  const cf_model::Params p = cf_model::load_params(kParamsFile);
  EXPECT_DOUBLE_EQ(cf_model::pwm_from_thrust_per_motor(p, 100.0), p.pwm_max);
  EXPECT_DOUBLE_EQ(cf_model::pwm_from_thrust_per_motor(p, -1.0), 0.0);
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

}  // namespace
