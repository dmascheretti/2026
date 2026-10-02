#include <gtest/gtest.h>

#include "cf_bringup/scenario.hpp"

namespace {

const std::string kStepsScenario = CF_STEPS_SCENARIO;  // set by CMake

TEST(Scenario, LoadsAndPicksActiveWaypoint) {
  const cf_bringup::Scenario s = cf_bringup::load_scenario(kStepsScenario);
  ASSERT_GE(s.waypoints.size(), 2u);
  EXPECT_DOUBLE_EQ(cf_bringup::waypoint_at(s, 0.0).position.z(), 0.5);
  EXPECT_DOUBLE_EQ(cf_bringup::waypoint_at(s, 3.99).position.x(), 0.0);
  EXPECT_DOUBLE_EQ(cf_bringup::waypoint_at(s, 4.0).position.x(), 0.5);
  EXPECT_DOUBLE_EQ(cf_bringup::waypoint_at(s, 1000.0).position.x(),
                   s.waypoints.back().position.x());
}

}  // namespace
