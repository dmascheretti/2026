# ROS-free build of all the math (model, MPC, EKF, safety), their unit
# tests and the closed-loop simulator. The ROS 2 nodes in ros2_ws/ compile
# the same source files; this build exists so the math can be built and
# tested quickly without ROS.
#
#   cmake -S . -B build && cmake --build build -j && ctest --test-dir build
cmake_minimum_required(VERSION 3.16)
project(cf_mpc_ekf CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)
find_package(GTest REQUIRED)
enable_testing()

set(CF_PARAMS_FILE ${CMAKE_SOURCE_DIR}/model/params.yaml)

# ---- model ----
add_library(cf_model model/cpp/src/params.cpp)
target_include_directories(cf_model PUBLIC model/cpp/include)
target_link_libraries(cf_model PUBLIC Eigen3::Eigen yaml-cpp)

add_executable(test_cf_model model/cpp/test/test_params.cpp)
target_compile_definitions(test_cf_model PRIVATE CF_PARAMS_FILE="${CF_PARAMS_FILE}")
target_link_libraries(test_cf_model cf_model GTest::gtest_main)
add_test(NAME test_cf_model COMMAND test_cf_model)
