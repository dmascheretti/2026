cmake_minimum_required(VERSION 3.16)
project(cf_mpc)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
# RPATH (not RUNPATH) so libacados finds hpipm/blasfeo without LD_LIBRARY_PATH.
add_link_options(-Wl,--disable-new-dtags)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cf_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)

set(REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../..)

# acados + generated solver (python3 mpc_codegen/generate_solver.py)
set(ACADOS_INSTALL_DIR "$ENV{ACADOS_SOURCE_DIR}" CACHE PATH "acados install directory")
if(NOT ACADOS_INSTALL_DIR)
  set(ACADOS_INSTALL_DIR /opt/acados)
endif()
set(CF_MPC_GENERATED_DIR ${REPO_ROOT}/mpc_codegen/c_generated_code)
if(NOT EXISTS ${CF_MPC_GENERATED_DIR}/libacados_ocp_solver_cf_mpc.so)
  message(FATAL_ERROR "MPC solver not generated. Run: python3 mpc_codegen/generate_solver.py")
endif()
set(CF_MPC_RPATH "${CF_MPC_GENERATED_DIR};${ACADOS_INSTALL_DIR}/lib")
# Lets the linker find libacados' own dependencies (hpipm, blasfeo).
add_link_options(-Wl,-rpath-link,${ACADOS_INSTALL_DIR}/lib)

add_library(cf_mpc_core
  src/mpc_controller.cpp
  ${REPO_ROOT}/model/cpp/src/params.cpp)
target_include_directories(cf_mpc_core
  PUBLIC include ${REPO_ROOT}/model/cpp/include ${CF_MPC_GENERATED_DIR}
  PRIVATE ${ACADOS_INSTALL_DIR}/include
          ${ACADOS_INSTALL_DIR}/include/blasfeo/include
          ${ACADOS_INSTALL_DIR}/include/hpipm/include)
target_link_libraries(cf_mpc_core
  PUBLIC Eigen3::Eigen yaml-cpp
         ${CF_MPC_GENERATED_DIR}/libacados_ocp_solver_cf_mpc.so
         ${ACADOS_INSTALL_DIR}/lib/libacados.so)

add_executable(mpc_node src/mpc_node.cpp)
target_link_libraries(mpc_node cf_mpc_core)
ament_target_dependencies(mpc_node rclcpp nav_msgs geometry_msgs std_msgs cf_msgs)
set_target_properties(mpc_node PROPERTIES BUILD_RPATH "${CF_MPC_RPATH}" INSTALL_RPATH "${CF_MPC_RPATH}")

install(TARGETS mpc_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_cf_mpc test/test_mpc_controller.cpp)
  target_compile_definitions(test_cf_mpc PRIVATE CF_PARAMS_FILE="${REPO_ROOT}/model/params.yaml")
  target_link_libraries(test_cf_mpc cf_mpc_core)
  set_target_properties(test_cf_mpc PROPERTIES BUILD_RPATH "${CF_MPC_RPATH}")
endif()

ament_package()
