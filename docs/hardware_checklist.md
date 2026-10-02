# Hardware checklist

Nothing in this repository has flown yet. Everything below is the plan
for the first hardware sessions, in order. Do not skip a step because the
previous one "looked fine". Each step checks something the simulation
cannot.

Setup assumed: Crazyflie 2.1 (or 2.1+) with Flow deck v2, Crazyradio 2.0,
Crazyswarm2 on ROS 2 Jazzy, a net or cage, a textured floor (flow needs
texture), charged batteries.

## 0. Before powering anything

- [ ] **Firmware thrust mapping.** Find out whether your firmware has
      `CONFIG_ENABLE_THRUST_BAT_COMPENSATED=y` (default for Crazyflie 2.x
      in recent releases). Set `rotor.thrust_command_model` in
      `model/params.yaml` accordingly, then regenerate the solver
      (`./scripts/run_sim_all.sh`) and rebuild.
- [ ] **Mass.** Weigh the drone with battery and Flow deck (exact flight
      configuration). Put the value in `params.yaml` (`status: own_id`).
- [ ] Set the URI in `ros2_ws/src/cf_bringup/config/crazyflies.yaml`.
- [ ] Geofence in `ros2_ws/src/cf_bringup/config/safety.yaml` fits inside
      the cage with at least 0.3 m margin (the geofence acts on the
      *estimate*, which drifts 0.1–0.3 m).

## 1. Bench, propellers OFF

Run `flight.launch.py`, do **not** arm.

- [ ] Topics `imu` and `flow` arrive at ~100 Hz:
      `ros2 topic hz /cf231/imu`.
- [ ] **IMU signs.** Drone flat: `acc.z ≈ +1` (g). Tilt right side down:
      `gyro.x` positive while rotating. Nose down: `gyro.y` positive.
      Turn counter-clockwise (from above): `gyro.z` positive.
- [ ] **EKF signs.** `ros2 topic echo /cf231/ekf/odom`. Tilt right side
      down: roll becomes positive. Nose down: pitch becomes positive.
      Lift the drone straight up: z follows the ToF.
- [ ] **Flow signs.** Hold the drone ~0.3 m above the floor and move it
      forward (+x): the EKF velocity vx must become positive; move left
      (+y): vy positive. If not, the flow axis mapping in
      `cf_ekf/crazyflie_logs.cpp` is wrong for your deck: fix it before
      anything else.
- [ ] Compare our EKF with the stock onboard estimate
      (`stock_velocity` log topic) while moving by hand.
- [ ] **Kill switch path.** Start `kill_switch_node` in its own terminal,
      call `safety/arm`, press SPACE: the supervisor must log `STOP: kill
      switch`, and `cmd_vel_legacy` must show `linear.z = 0`.
- [ ] **Watchdog.** Arm, then stop the EKF node (Ctrl-C): supervisor must
      stop within 100 ms ("watchdog: no state estimate").

## 2. Bench, propellers ON, drone held/tethered

- [ ] **Command signs**, one at a time, with a temporary small fixed
      setpoint (or by moving the reference slightly): a positive pitch
      command must push the drone forward (+x), a positive roll command
      to the right (−y). A wrong sign here means a fly-away.
- [ ] **Hover thrust.** Expected `thrust_cmd` at hover is ~38 000. Log it.
      If the drone needs much more or less, the thrust model or mass is
      wrong; fix `params.yaml` before free flight.

## 3. First free flights (cage, low altitude)

- [ ] Scenario `hover` but with the waypoint lowered to 0.3 m.
      Kill switch hand on the key.
- [ ] Record every flight: `ros2 bag record -a` plus
      `analysis/record_ros.py` for quick plots.
- [ ] After each flight: altitude offset (→ disturbance estimate in the
      `mpc/command.disturbance` field), tilt jitter, solve time, drift.
- [ ] Only then: `steps` scenario (reduce step sizes to fit the cage).

## 4. Identification flights

See [`identification.md`](identification.md). Replace every
`status: assumed` entry in `params.yaml` before trusting the simulator
for tuning.
