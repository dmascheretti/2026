# Design notes

How the pieces fit together, the conventions they share, and why the main
decisions were taken. Numbers quoted here come from `model/params.yaml`,
`mpc_codegen/mpc_tuning.yaml` and the simulation results in
[`results_sim.md`](results_sim.md).

## 1. Architecture

```
                        laptop (ROS 2 Jazzy, namespace /<name>)
 ┌──────────────────────────────────────────────────────────────────────────┐
 │  mission_node ──reference──▶ mpc_node ──mpc/command──▶ safety_supervisor │
 │                                 ▲                         │    ▲    ▲     │
 │                              ekf/odom ─────────────────────┘    │    │     │
 │                                 │                    kill ──────┘    │     │
 │  ekf_node ◀── imu, flow ────────┤               safety/arm (service) │     │
 └─────────────────────────────────┼────────────────────────────────────┼─────┘
                                   │                          cmd_vel_legacy
              Crazyswarm2 server (flight)  or  sim_bridge_node (simulation)
                                   │                                    │
                     Crazyflie: stock PID attitude controller + motors
```

| Node | Package | In | Out | Rate |
|---|---|---|---|---|
| `ekf_node` | cf_ekf | `imu`, `flow` (Crazyswarm2 log topics) | `ekf/odom` | 100 Hz (per IMU packet) |
| `mpc_node` | cf_mpc | `ekf/odom`, `reference`, `safety/active` | `mpc/command` | 50 Hz |
| `safety_supervisor_node` | cf_bringup | `ekf/odom`, `mpc/command`, `kill` | `cmd_vel_legacy`, `safety/active` | 100 Hz |
| `mission_node` | cf_bringup | `safety/active` | `reference` | 20 Hz |
| `kill_switch_node` | cf_bringup | keyboard | `kill` | on key press |
| `sim_bridge_node` | cf_bringup | `cmd_vel_legacy` | `imu`, `flow`, `ground_truth` | 1 kHz physics, 100 Hz logs |

Only the safety supervisor talks to the drone. The MPC never does.

The same math runs in three places, compiled from the same source files:
1. the ROS-free C++ simulator (`sim/`, fast and deterministic, used for design),
2. the ROS pipeline against `sim_bridge_node` (tests the real nodes, topics, units, timing),
3. the ROS pipeline against the real Crazyflie (`flight.launch.py`).

## 2. Conventions

- **World frame**: x forward at take-off, y left, z up. Floor at z = 0.
- **Body frame**: x forward, y left, z up (Crazyflie convention).
- **Attitude**: ZYX Euler angles, R = Rz(yaw) · Ry(pitch) · Rx(roll).
  Positive roll = right side down, so the drone accelerates towards −y.
  Positive pitch = nose down, so it accelerates towards +x.
- **Units**: SI everywhere inside our code (m, s, rad, N). Degrees, g and
  millimetres only appear at the Crazyflie interface, converted in two
  tested functions: `cf_bringup/crazyflie_interface.cpp` (commands) and
  `cf_ekf/crazyflie_logs.cpp` (sensor logs).
- **Thrust command**: the firmware's 16-bit "base thrust" per motor
  (0..65535), called `thrust_cmd`.

### Command path to the firmware (checked in the source code)

| Our command | `cmd_vel_legacy` field | Crazyswarm2 sends | Firmware uses |
|---|---|---|---|
| roll φ [rad] | `linear.y = φ` [deg] | roll = `linear.y` | attitude.roll = roll |
| pitch θ [rad] | `linear.x = θ` [deg] | pitch = −`linear.x` | legacy pitch = pitch, and legacy pitch = −θ |
| yaw rate r [rad/s] | `angular.z = −r` [deg/s] | yawrate = `angular.z` | attitudeRate.yaw = −yawrate |
| thrust_cmd | `linear.z` | clamped to [0, 60000] | thrust lock until one 0 is received |

Sources: Crazyswarm2 `crazyflie_server_cpp/src/crazyflie_server.cpp`
(`cmd_vel_legacy_changed`), firmware `src/modules/src/crtp_commander_rpyt.c`
and `src/modules/src/kalman_core/kalman_core.c` (legacy pitch sign).

## 3. Model (`model/`)

The MPC sits on top of the onboard attitude loop, so its model only has
translation and a first-order attitude response:

```
x = [px, py, pz, vx, vy, vz, roll, pitch]      u = [roll_cmd, pitch_cmd, thrust_delta]
dp/dt = v
dv/dt = R(roll, pitch, 0) [0, 0, (m g + thrust_delta)/m] − [0, 0, g]
d(roll)/dt  = (roll_cmd  − roll)  / tau
d(pitch)/dt = (pitch_cmd − pitch) / tau
```

Linearised at hover: `ax = g·pitch`, `ay = −g·roll`, `az = thrust_delta/m`.
The hand-written A, B are checked against the casadi Jacobian of the
nonlinear model (`model/test_cf_model.py`). Discretisation: exact
zero-order hold (matrix exponential) at 50 Hz.

**Thrust command → thrust.** Current Crazyflie 2.x firmware compensates
for battery voltage and maps the command linearly:
`F = cmd / 65535 · 0.12 N` per motor, motor off below 0.0128 N
(`src/drivers/src/motors.c`, `platform_defaults_cf2.h`). Older firmware
used a raw PWM curve (Förster 2015). `params.yaml → thrust_command_model`
selects one. Check which one your firmware uses before flying.
Expected hover command: about 38 000 with either model.

## 4. MPC (`mpc_codegen/`, `cf_mpc`)

- **OCP**: N = 25 steps (0.5 s), quadratic cost on state and input error,
  terminal weight = DARE solution (so with no active bound the MPC equals
  the infinite-horizon LQR; this is a unit test), input bounds:
  |roll|, |pitch| ≤ 0.35 rad, thrust between 0.3 × hover and 0.85 × max.
- **Solver**: acados, HPIPM, one SQP iteration (the problem is a QP).
  acados scales stage costs by the step length by default; that is
  switched off (`cost_scaling = 1`) because our cost is already a
  discrete sum. Solve time in simulation: 0.7–3 ms (budget 20 ms).
- **Yaw**: not in the MPC. Positions and velocities are rotated into the
  yaw frame (world rotated by the current yaw). With ZYX angles, roll and
  pitch are already defined in that frame, so the solver's roll/pitch go
  to the drone unchanged. Yaw itself: `yaw_rate = 2.0 · yaw_error`, ≤ 1 rad/s.
- **Tuning** (why the weights are what they are): the first tuning
  weighted roll/pitch states and used high position/velocity weights. That
  gives a feedback of −0.97 on the *estimated* roll, i.e. the MPC tries to
  make the onboard attitude loop faster. With the radio delay and EKF
  noise this produced a 3 Hz, ~0.1 rad oscillation in simulation. The
  current weights (no angle weight, equivalent gains 0.40 rad/m and
  0.37 rad/(m/s)) give 0.02 rad RMS tilt in hover and a 0.5 m step in ~2 s.
- **Offset-free control (vertical disturbance observer)**. The model
  assumes the mass and thrust map in `params.yaml` are exact. A 10 % error
  gives a ~6–7 cm steady altitude offset (no integral action in a linear
  MPC). The observer estimates the unmodelled vertical force

  ```
  d_raw = m · Δvz/Δt − (T_prev · cos(roll) · cos(pitch) − m g)
  d    ← d + (Δt / 0.5 s) · (d_raw − d)        (low-pass, |d| ≤ 0.3 m g)
  T_cmd = m g + thrust_delta_mpc − d
  ```

  It is off below 0.15 m (the floor pushes back) and reset on every arming.
  Result: offset < 1 cm for ±10 % mass or thrust errors.

## 5. EKF (`cf_ekf`)

- **State**: position, velocity (world), roll, pitch, yaw (9).
- **Prediction**: IMU as input. Velocity integrates `R·accel − g`,
  angles integrate the Euler-rate kinematics of the gyro. Process noise =
  IMU noise integrated over dt.
- **Corrections**: ToF range `z / (cos(roll) cos(pitch))`; optical flow with
  the firmware's model (`mm_flow.c`): pixels = dt·Npix/θpix ·
  (v_body · R22 / z − ω), using the latest gyro.
- **Jacobians**: numerical central differences. Short code, easy to check,
  9 extra model calls per step, negligible.
- **Robustness**: Mahalanobis outlier gate (χ² > 25 rejected); after 10
  consecutive rejections the measurement is accepted again (otherwise the
  filter never recovers once it is wrong, e.g. after a crash); Joseph-form
  covariance update.
- **Time**: on-board log timestamps, not arrival times, so radio jitter
  does not enter the prediction.
- **Fundamental limit**: with flow only, x/y position is dead reckoning.
  The xy error grows like a random walk. In simulation, flow noise alone
  explains all of it (removing every other noise source leaves it
  unchanged): 0.14–0.2 m after 8–16 s. This also holds for the stock
  onboard estimator. Altitude is absolute (ToF) and stays at 3 mm RMS.

## 6. Safety layers

| Layer | Where | What |
|---|---|---|
| Input bounds | MPC | tilt ≤ 0.35 rad, thrust ≤ 0.85 × max |
| Saturation | supervisor (independent limits) | tilt ≤ 0.40 rad, yaw rate ≤ 1.5 rad/s, thrust ≤ 60000 |
| Kill switch | `kill_switch_node` → supervisor | any key → motors off, latched |
| Geofence | supervisor | estimate outside box → motors off, latched |
| Watchdog | supervisor | estimate or command older than 100 ms → motors off |
| Solver failures | supervisor | 5 consecutive failures → motors off |
| Arming | supervisor | nothing is forwarded until `safety/arm`; zero-thrust setpoints before |
| Firmware | Crazyflie | no setpoint for 0.5 s → level; 2 s → motors off |
| Crazyswarm2 | server | thrust clamped to 60000 |

Every stop is latched: the motors stay off until someone calls
`safety/arm` again.

## 7. Simulator (`sim/`)

Modelled: 6-DOF rigid body with the identified inertia; X-configuration
mixer; first-order motors; ground contact; a stand-in for the firmware
attitude controller (cascaded P, tuned to a ~0.1 s first-order response);
IMU with noise and bias; ToF and flow with noise (the bridge also rounds
flow to integer counts like the sensor); radio latency (10 ms); mass and
thrust mismatch factors.

**Not** modelled, so expected sources of sim-to-real gap: the real
firmware PID attitude controller (gains, filters, its own estimator in the
loop); motor vibration on the IMU; aerodynamic drag and ground effect;
propeller differences; flow-sensor behaviour over real textures and its
real noise; radio packet loss; the log sampling (100 Hz log of a 100 Hz
flow sensor can duplicate or skip samples).

## 8. Findings from reading the upstream source

1. **Crazyswarm2's simulation backend does not implement `cmd_vel_legacy`**
   (`crazyflie_sim/crazyflie_server.py` logs "not yet implemented"). It
   cannot test an attitude-setpoint controller. That is why this project
   has its own simulator and `sim_bridge_node`.
2. **Thrust is battery-compensated** in current firmware (section 3).
3. **Thrust lock**: the firmware ignores thrust until one setpoint with
   thrust 0 arrives. The supervisor sends zero setpoints while disarmed,
   which releases it.
4. **Flow axes**: `dpixelx = −motion.deltaY`, `dpixely = −motion.deltaX`
   (`deck/drivers/src/flowdeck_v1v2.c`).
5. **Flow resolution**: with the firmware model (35 px over 0.717 rad), a
   drone at 0.5 m moving at 0.3 m/s sees ~0.3 counts per 10 ms sample.
   Integer counts are therefore a major noise source; in the ROS
   simulation quantisation increases the xy estimation error from 0.07 m
   to 0.15 m RMS over 10 s. Check this against real logs early.
