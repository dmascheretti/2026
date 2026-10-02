cmake_minimum_required(VERSION 3.16)
project(cf_ekf)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(crazyflie_interfaces REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)

# The model library lives outside the ROS workspace (repo/model).
set(REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../..)

add_library(cf_ekf_core
  src/ekf.cpp
  src/crazyflie_logs.cpp
  ${REPO_ROOT}/model/cpp/src/params.cpp)
target_include_directories(cf_ekf_core PUBLIC include ${REPO_ROOT}/model/cpp/include)
target_link_libraries(cf_ekf_core PUBLIC Eigen3::Eigen yaml-cpp)
set_target_properties(cf_ekf_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(ekf_node src/ekf_node.cpp)
target_link_libraries(ekf_node cf_ekf_core)
ament_target_dependencies(ekf_node rclcpp nav_msgs std_srvs crazyflie_interfaces)

install(TARGETS ekf_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_cf_ekf test/test_ekf.cpp test/test_crazyflie_logs.cpp)
  target_compile_definitions(test_cf_ekf PRIVATE
    CF_PARAMS_FILE="${REPO_ROOT}/model/params.yaml"
    CF_EKF_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/ekf.yaml")
  target_link_libraries(test_cf_ekf cf_ekf_core)
endif()

ament_package()
