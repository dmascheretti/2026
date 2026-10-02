cmake_minimum_required(VERSION 3.16)
project(cf_bringup)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cf_msgs REQUIRED)
find_package(crazyflie_interfaces REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)

set(REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../..)

add_library(cf_safety_core
  src/safety_supervisor.cpp
  src/crazyflie_interface.cpp
  src/scenario.cpp)
target_include_directories(cf_safety_core PUBLIC include)
target_link_libraries(cf_safety_core PUBLIC Eigen3::Eigen yaml-cpp)

add_executable(safety_supervisor_node src/safety_supervisor_node.cpp)
target_link_libraries(safety_supervisor_node cf_safety_core)
ament_target_dependencies(safety_supervisor_node
  rclcpp geometry_msgs nav_msgs std_msgs std_srvs cf_msgs)

add_executable(kill_switch_node src/kill_switch_node.cpp)
ament_target_dependencies(kill_switch_node rclcpp std_msgs)

add_executable(mission_node src/mission_node.cpp)
target_link_libraries(mission_node cf_safety_core)
ament_target_dependencies(mission_node rclcpp geometry_msgs std_msgs)

# Simulator bridge: the simulated plant lives in repo/sim.
add_executable(sim_bridge_node
  src/sim_bridge_node.cpp
  ${REPO_ROOT}/sim/src/plant.cpp
  ${REPO_ROOT}/model/cpp/src/params.cpp)
target_include_directories(sim_bridge_node PRIVATE
  ${REPO_ROOT}/sim/src ${REPO_ROOT}/model/cpp/include)
target_link_libraries(sim_bridge_node cf_safety_core)
ament_target_dependencies(sim_bridge_node
  rclcpp geometry_msgs nav_msgs crazyflie_interfaces)

install(TARGETS safety_supervisor_node kill_switch_node mission_node sim_bridge_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
# Files that live outside this package, installed so launch files find them.
install(FILES ${REPO_ROOT}/model/params.yaml DESTINATION share/${PROJECT_NAME}/model)
install(FILES ${REPO_ROOT}/sim/config/sim.yaml DESTINATION share/${PROJECT_NAME}/sim)
install(DIRECTORY ${REPO_ROOT}/sim/scenarios DESTINATION share/${PROJECT_NAME}/sim)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_cf_bringup
    test/test_safety_supervisor.cpp
    test/test_crazyflie_interface.cpp
    test/test_scenario.cpp)
  target_compile_definitions(test_cf_bringup PRIVATE
    CF_SAFETY_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/safety.yaml"
    CF_STEPS_SCENARIO="${REPO_ROOT}/sim/scenarios/steps.yaml")
  target_link_libraries(test_cf_bringup cf_safety_core)
endif()

ament_package()
