# sim

C++ closed-loop simulator: simulated Crazyflie (rigid body, motors, a
stand-in for the firmware attitude controller, IMU/ToF/flow sensors,
radio latency) with the real EKF, MPC and safety supervisor code in the
loop.

- `config/sim.yaml`: simulator settings (rates, noise, mismatch factors)
- `scenarios/*.yaml`: waypoints over time; also flown by `mission_node`
- `src/plant.*`: the simulated drone (also used by `sim_bridge_node`)
- `src/closed_loop_sim.cpp`: the loop, writes a CSV log

Crazyswarm2's own simulation backend does not support attitude setpoints
(`cmd_vel_legacy`), which is why this simulator exists. See docs/design.md.
