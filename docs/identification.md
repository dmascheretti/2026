# Identification plan

Which parameters are not yet measured, and how to measure each one with
the hardware we have (Crazyflie, Flow deck, scale, logs). The goal is to
turn every `status: assumed` and the most influential `literature` entries
in `model/params.yaml` into `status: own_id`, and then to quantify the
sim-to-real gap with the same metrics as [`results_sim.md`](results_sim.md).

## Parameters by priority

| Parameter | Status now | Why it matters | How to identify |
|---|---|---|---|
| `mass_base` + decks | datasheet | hover thrust, altitude offset | kitchen scale (0.1 g), flight configuration |
| `thrust_command_model` | literature | wrong model → wrong thrust | check firmware build config; confirm with hover thrust |
| `max_thrust_per_motor` | literature | thrust bounds, hover command | hover `thrust_cmd` × 4 must equal m·g (with the observer off, or read the converged `disturbance`) |
| `attitude_time_constant` | **assumed** (0.1 s) | MPC prediction model | small roll/pitch steps (±3°) in hover; fit a first-order lag from command to the logged stock attitude; check delay too |
| `motor_time_constant` | **assumed** (0.03 s) | simulator only | thrust steps on the bench, or fit from vertical acceleration after thrust steps |
| `inertia` | literature (Förster) | simulator only (the MPC does not use it) | keep literature unless the sim attitude loop disagrees with flight data |
| flow noise and scale (`flow_npix`, `flow_thetapix`) | literature (firmware) | EKF velocity accuracy | hold still: noise; move a known distance by hand at known height: scale |
| IMU noise and bias | – | EKF tuning | 60 s static log with motors off, then with motors at hover speed (vibration) |
| EKF noise settings (`ekf.yaml`) | tuned in sim | estimate quality | replay logged flights offline, compare with the stock estimate and, if available, Lighthouse/mocap |
| Sim stand-in attitude gains (`sim.yaml`) | assumed | simulator fidelity | match the identified attitude response |

## Sim-to-real gap: what to compare

For the same scenario, flown and simulated:

1. hover altitude offset (with observer off and on),
2. tilt RMS in hover,
3. 0.5 m step: rise time, overshoot, settling time,
4. estimation: our EKF vs stock onboard estimate (velocity, attitude),
5. hover thrust command,
6. MPC solve time on the flight laptop.

Then update the simulator with the identified parameters and re-run. The
gap that remains after identification is the part that needs a model
change (e.g. attitude-loop delay, drag) rather than a parameter change.

## Offline replay

`ekf_node` takes everything from topics, so a recorded bag can be
replayed (`ros2 bag play`) into a fresh `ekf_node` with different
`ekf.yaml` settings, without flying again.
