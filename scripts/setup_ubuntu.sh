#!/usr/bin/env bash
# One-time setup of the non-ROS toolchain on Ubuntu 24.04:
# C++ libraries, Python packages and acados (built from source).
#
#   ./scripts/setup_ubuntu.sh            # acados goes to ~/acados
#   ACADOS_SOURCE_DIR=/opt/acados ./scripts/setup_ubuntu.sh
#
# ROS 2 Jazzy and Crazyswarm2 are installed separately (see README.md).
set -euo pipefail

ACADOS_SOURCE_DIR="${ACADOS_SOURCE_DIR:-$HOME/acados}"
# acados version this project was developed and tested with.
ACADOS_COMMIT=6cbbf9986ae0e8b9032ae4f9994eda8d2c269029

echo "== C++ libraries (Eigen, GoogleTest, yaml-cpp) and build tools"
sudo apt-get update
sudo apt-get install -y build-essential cmake git libeigen3-dev libgtest-dev libyaml-cpp-dev
# Only for the interactive simulator (raylib needs X11 + OpenGL headers).
sudo apt-get install -y libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev libgl1-mesa-dev

echo "== Python packages"
python3 -m pip install --user numpy scipy matplotlib casadi pyyaml

echo "== acados -> $ACADOS_SOURCE_DIR"
if [ ! -d "$ACADOS_SOURCE_DIR" ]; then
  git clone https://github.com/acados/acados.git "$ACADOS_SOURCE_DIR"
fi
cd "$ACADOS_SOURCE_DIR"
git fetch origin
git checkout "$ACADOS_COMMIT"
git submodule update --init --recursive
mkdir -p build && cd build
cmake -DACADOS_WITH_QPOASES=ON ..
make install -j"$(nproc)"
python3 -m pip install --user "$ACADOS_SOURCE_DIR/interfaces/acados_template"

echo
echo "Done. Add to your ~/.bashrc:"
echo "  export ACADOS_SOURCE_DIR=$ACADOS_SOURCE_DIR"
echo "  export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:$ACADOS_SOURCE_DIR/lib"
