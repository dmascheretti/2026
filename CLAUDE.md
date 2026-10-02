# CLAUDE.md — Crazyflie MPC + EKF (sim → hardware)

## What this project is

Port a Linear MPC controller (originally developed in MATLAB/CasADi for a Skydio X2 quadrotor, see github.com/dmascheretti/Quadcopter-Dynamics-LQR-MPC) to a Crazyflie 2.1 nano-quadrotor, add a custom EKF for state estimation, and validate it first in simulation, then in real flight.

The final result must show: same control design, simulation → hardware, a quantified sim-to-real gap, and how it was closed.

## Architecture (cascaded)

- **Offboard (laptop, ROS 2, C++):** position/velocity MPC → outputs roll, pitch, yaw-rate, thrust setpoints at 50–100 Hz.
- **Onboard (Crazyflie firmware, unchanged at first):** stock attitude/rate controller tracks those setpoints.
- **Estimation:** custom EKF (C++) running offboard on logged/streamed IMU + Flow deck data, compared against the stock onboard estimator.
- **Link:** Crazyradio 2.0 via Crazyswarm2 / cflib.

## Stack

- C++17 for all runtime nodes (MPC, EKF). Python only for: acados code generation, offline analysis, plots.
- ROS 2 (Humble or Jazzy — use whatever Crazyswarm2 currently supports; check its docs, don't assume).
- acados (Python template → generated C solver, called from C++).
- Eigen for linear algebra.
- Crazyswarm2 (with its simulation backend) and cflib.

## Repo layout

```
/model        dynamics, parameters, linearization (Python + C++ header)
/mpc_codegen  acados OCP definition (Python) → generated C code
/ros2_ws/src
  /cf_mpc     C++ MPC node
  /cf_ekf     C++ EKF node
  /cf_bringup launch files, params YAML, safety supervisor
/sim          simulation scenarios + scripts
/analysis     log parsing, plots, metrics
/docs         design notes, results, figures
```

## Coding rules

- Readable, explicit code over clever abstractions. Explicit loops are fine. I must be able to explain every line orally.
- All physical parameters in ONE place (`/model/params.yaml`), never hardcoded elsewhere. Every parameter has unit + source (datasheet, paper, own identification).
- Every node has unit tests (gtest) for the math parts.
- Plots: matplotlib, dark theme, units on every axis, saved as PNG + PDF in `/docs/figures`.
- Don't modify the Crazyflie firmware unless a phase explicitly says so.
- Don't add dependencies without telling me why.
- Small commits, one topic each, clear messages.

## Safety (hardware phases — non-negotiable)

- Software kill switch on a single key + hardware fallback (stop sending setpoints → firmware cuts motors).
- Thrust and attitude saturation in the MPC AND in a separate safety supervisor node.
- Geofence: land/cut if outside a box.
- Comms watchdog: no estimate or setpoint for >100 ms → emergency stop.
- First flights in a net/cage, low altitude.
