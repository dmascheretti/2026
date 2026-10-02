#!/usr/bin/env bash
# Build the ROS 2 workspace. Needs ROS 2 Jazzy and Crazyswarm2 sourced, and
# the MPC solver generated (scripts/run_sim_all.sh does that).
#   source /opt/ros/jazzy/setup.bash
#   source ~/crazyswarm2_ws/install/setup.bash
#   ./scripts/build_ros.sh
set -euo pipefail
cd "$(dirname "$0")/../ros2_ws"
export ACADOS_SOURCE_DIR="${ACADOS_SOURCE_DIR:-$HOME/acados}"
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test
colcon test-result --verbose
