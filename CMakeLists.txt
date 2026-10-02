# ROS-free build of all the math (model, MPC, EKF, safety), their unit
# tests and the closed-loop simulator. The ROS 2 nodes in ros2_ws/ compile
# the same source files; this build exists so the math can be built and
# tested quickly without ROS.
#
#   cmake -S . -B build && cmake --build build -j && ctest --test-dir build
cmake_minimum_required(VERSION 3.16)
project(cf_mpc_ekf CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
# Use RPATH instead of RUNPATH: RPATH also applies to the dependencies of
# libacados (hpipm, blasfeo), so no LD_LIBRARY_PATH is needed.
add_link_options(-Wl,--disable-new-dtags)

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

# ---- MPC (needs acados and the generated solver) ----
# Generate the solver first:  python3 mpc_codegen/generate_solver.py
set(ACADOS_INSTALL_DIR "$ENV{ACADOS_SOURCE_DIR}" CACHE PATH "acados install directory")
if(NOT ACADOS_INSTALL_DIR)
  set(ACADOS_INSTALL_DIR /opt/acados)
endif()
set(CF_MPC_GENERATED_DIR ${CMAKE_SOURCE_DIR}/mpc_codegen/c_generated_code)
if(NOT EXISTS ${CF_MPC_GENERATED_DIR}/libacados_ocp_solver_cf_mpc.so)
  message(FATAL_ERROR "MPC solver not generated. Run: python3 mpc_codegen/generate_solver.py")
endif()

add_library(cf_mpc_core ros2_ws/src/cf_mpc/src/mpc_controller.cpp)
target_include_directories(cf_mpc_core
  PUBLIC ros2_ws/src/cf_mpc/include
  PRIVATE ${CF_MPC_GENERATED_DIR}
          ${ACADOS_INSTALL_DIR}/include
          ${ACADOS_INSTALL_DIR}/include/blasfeo/include
          ${ACADOS_INSTALL_DIR}/include/hpipm/include)
target_link_libraries(cf_mpc_core
  PUBLIC cf_model
  PRIVATE ${CF_MPC_GENERATED_DIR}/libacados_ocp_solver_cf_mpc.so
          ${ACADOS_INSTALL_DIR}/lib/libacados.so
          ${ACADOS_INSTALL_DIR}/lib/libacados.so)
set_target_properties(cf_mpc_core PROPERTIES
  BUILD_RPATH "${CF_MPC_GENERATED_DIR};${ACADOS_INSTALL_DIR}/lib")

add_executable(test_cf_mpc ros2_ws/src/cf_mpc/test/test_mpc_controller.cpp)
target_include_directories(test_cf_mpc PRIVATE ${CF_MPC_GENERATED_DIR})
target_compile_definitions(test_cf_mpc PRIVATE CF_PARAMS_FILE="${CF_PARAMS_FILE}")
target_link_libraries(test_cf_mpc cf_mpc_core GTest::gtest_main)
set_target_properties(test_cf_mpc PROPERTIES
  BUILD_RPATH "${CF_MPC_GENERATED_DIR};${ACADOS_INSTALL_DIR}/lib")
add_test(NAME test_cf_mpc COMMAND test_cf_mpc)

# ---- EKF ----
add_library(cf_ekf_core ros2_ws/src/cf_ekf/src/ekf.cpp ros2_ws/src/cf_ekf/src/crazyflie_logs.cpp)
target_include_directories(cf_ekf_core PUBLIC ros2_ws/src/cf_ekf/include)
target_link_libraries(cf_ekf_core PUBLIC cf_model)

add_executable(test_cf_ekf ros2_ws/src/cf_ekf/test/test_ekf.cpp ros2_ws/src/cf_ekf/test/test_crazyflie_logs.cpp)
target_compile_definitions(test_cf_ekf PRIVATE
  CF_PARAMS_FILE="${CF_PARAMS_FILE}"
  CF_EKF_CONFIG="${CMAKE_SOURCE_DIR}/ros2_ws/src/cf_ekf/config/ekf.yaml")
target_link_libraries(test_cf_ekf cf_ekf_core GTest::gtest_main)
add_test(NAME test_cf_ekf COMMAND test_cf_ekf)

# ---- safety supervisor ----
add_library(cf_safety_core ros2_ws/src/cf_bringup/src/safety_supervisor.cpp
  ros2_ws/src/cf_bringup/src/crazyflie_interface.cpp)
target_include_directories(cf_safety_core PUBLIC ros2_ws/src/cf_bringup/include)
target_link_libraries(cf_safety_core PUBLIC Eigen3::Eigen yaml-cpp)

add_executable(test_cf_safety ros2_ws/src/cf_bringup/test/test_safety_supervisor.cpp
  ros2_ws/src/cf_bringup/test/test_crazyflie_interface.cpp)
target_compile_definitions(test_cf_safety PRIVATE
  CF_SAFETY_CONFIG="${CMAKE_SOURCE_DIR}/ros2_ws/src/cf_bringup/config/safety.yaml")
target_link_libraries(test_cf_safety cf_safety_core GTest::gtest_main)
add_test(NAME test_cf_safety COMMAND test_cf_safety)

# ---- closed-loop simulator ----
add_executable(closed_loop_sim sim/src/closed_loop_sim.cpp sim/src/plant.cpp)
target_compile_definitions(closed_loop_sim PRIVATE
  CF_PARAMS_FILE="${CF_PARAMS_FILE}"
  CF_SIM_CONFIG="${CMAKE_SOURCE_DIR}/sim/config/sim.yaml"
  CF_EKF_CONFIG="${CMAKE_SOURCE_DIR}/ros2_ws/src/cf_ekf/config/ekf.yaml"
  CF_SAFETY_CONFIG="${CMAKE_SOURCE_DIR}/ros2_ws/src/cf_bringup/config/safety.yaml")
target_link_libraries(closed_loop_sim cf_mpc_core cf_ekf_core cf_safety_core)
