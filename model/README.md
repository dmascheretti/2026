# model

Quadrotor dynamics, physical parameters and linearization.

- `params.yaml` is the single source of truth for every physical parameter (value + unit + source).
- Python side feeds `mpc_codegen`; a C++ header is used by the ROS 2 nodes.
