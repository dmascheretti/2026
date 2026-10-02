#!/usr/bin/env bash
# Build and start the interactive 3D simulator (the "game").
# Run from anywhere:  ./scripts/run_game.sh
# Needs a desktop session (it opens a window) and, once, the X11/OpenGL
# headers raylib builds against:
#   sudo apt-get install libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev libgl1-mesa-dev
set -euo pipefail
cd "$(dirname "$0")/.."

export ACADOS_SOURCE_DIR="${ACADOS_SOURCE_DIR:-$HOME/acados}"
if [ ! -f mpc_codegen/c_generated_code/libacados_ocp_solver_cf_mpc.so ]; then
  python3 mpc_codegen/generate_solver.py
fi
cmake -S . -B build -DCF_BUILD_GAME=ON
cmake --build build -j"$(nproc)" --target cf_sim_game
./build/cf_sim_game "$@"
