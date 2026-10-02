#!/usr/bin/env bash
# Generate the MPC solver, build and test everything (no ROS), run all
# simulation scenarios and make the figures. Run from the repo root:
#   ./scripts/run_sim_all.sh
set -euo pipefail
cd "$(dirname "$0")/.."

export ACADOS_SOURCE_DIR="${ACADOS_SOURCE_DIR:-$HOME/acados}"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:$ACADOS_SOURCE_DIR/lib"

echo "== parameters"
python3 model/check_params.py

echo "== MPC code generation"
python3 mpc_codegen/generate_solver.py

echo "== Python tests"
python3 -m unittest discover model
python3 -m unittest discover mpc_codegen

echo "== C++ build and tests"
cmake -S . -B build
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure

echo "== simulation scenarios"
mkdir -p sim/output
for scenario in hover steps kill_switch geofence gap_mass gap_mass_no_observer; do
  ./build/closed_loop_sim "sim/scenarios/$scenario.yaml" "sim/output/$scenario.csv"
done

echo "== figures and metrics"
python3 analysis/plot_sim.py sim/output/{hover,steps,kill_switch,geofence}.csv
python3 analysis/plot_gap.py sim/output/gap_mass_no_observer.csv sim/output/gap_mass.csv
