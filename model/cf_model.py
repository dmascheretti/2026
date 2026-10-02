"""Crazyflie model used by the MPC.

The MPC sits on top of the onboard attitude controller, so it does not see
motors or body rates. Its prediction model is:

    state  x = [px, py, pz, vx, vy, vz, phi, theta]      (8)
    input  u = [phi_cmd, theta_cmd, thrust_delta]          (3)

    d(p)/dt     = v
    d(v)/dt     = R(phi, theta, psi=0) * [0, 0, T/m] - [0, 0, g]
    d(phi)/dt   = (phi_cmd   - phi)   / tau
    d(theta)/dt = (theta_cmd - theta) / tau

with total thrust T = m*g + thrust_delta, so u = 0 is hover.
Yaw is not part of the MPC: it is held by a separate P controller on yaw
rate, and the MPC works in a yaw-aligned frame (see cf_mpc).

Units: m, m/s, rad, N, s.
"""
from pathlib import Path

import casadi as ca
import numpy as np
import scipy.linalg
import yaml

PARAMS_FILE = Path(__file__).parent / "params.yaml"

NX = 8
NU = 3
STATE_NAMES = ["px", "py", "pz", "vx", "vy", "vz", "phi", "theta"]
INPUT_NAMES = ["phi_cmd", "theta_cmd", "thrust_delta"]


def load_params(path=PARAMS_FILE):
    """Return a flat dict {name: value} from params.yaml (units in the file)."""
    with open(path) as f:
        raw = yaml.safe_load(f)
    params = {}
    for group_name, group in raw.items():
        if group_name == "sources":
            continue
        for name, entry in group.items():
            params[name] = entry["value"]
    params["mass"] = params["mass_base"] + params["mass_flow_deck"]
    return params


def max_total_thrust(params):
    """Total thrust of 4 motors at full command [N]."""
    c2, c1, c0 = params["thrust_pwm_poly"]
    pwm = params["pwm_max"]
    return 4.0 * (c2 * pwm**2 + c1 * pwm + c0)


def nonlinear_dynamics_casadi(params):
    """Return (x, u, xdot) casadi symbols of the nonlinear prediction model."""
    m = params["mass"]
    g = params["gravity"]
    tau = params["attitude_time_constant"]

    x = ca.SX.sym("x", NX)
    u = ca.SX.sym("u", NU)
    vx, vy, vz = x[3], x[4], x[5]
    phi, theta = x[6], x[7]
    phi_cmd, theta_cmd, thrust_delta = u[0], u[1], u[2]

    thrust = m * g + thrust_delta
    # Third column of R = Rz(psi) Ry(theta) Rx(phi) with psi = 0.
    ax = thrust / m * (ca.cos(phi) * ca.sin(theta))
    ay = thrust / m * (-ca.sin(phi))
    az = thrust / m * (ca.cos(phi) * ca.cos(theta)) - g

    xdot = ca.vertcat(
        vx, vy, vz,
        ax, ay, az,
        (phi_cmd - phi) / tau,
        (theta_cmd - theta) / tau,
    )
    return x, u, xdot


def linear_model(params):
    """Continuous-time A, B of the model linearised at hover (x = 0, u = 0).

    Written out by hand so every entry can be read off directly.
    """
    m = params["mass"]
    g = params["gravity"]
    tau = params["attitude_time_constant"]

    A = np.zeros((NX, NX))
    B = np.zeros((NX, NU))
    # position derivative = velocity
    A[0, 3] = 1.0
    A[1, 4] = 1.0
    A[2, 5] = 1.0
    # small tilt: ax = g * theta, ay = -g * phi
    A[3, 7] = g
    A[4, 6] = -g
    # vertical: az = thrust_delta / m
    B[5, 2] = 1.0 / m
    # first-order attitude loop
    A[6, 6] = -1.0 / tau
    A[7, 7] = -1.0 / tau
    B[6, 0] = 1.0 / tau
    B[7, 1] = 1.0 / tau
    return A, B


def discretize(A, B, dt):
    """Exact zero-order-hold discretisation via the matrix exponential."""
    n = A.shape[0]
    m = B.shape[1]
    M = np.zeros((n + m, n + m))
    M[:n, :n] = A
    M[:n, n:] = B
    Md = scipy.linalg.expm(M * dt)
    Ad = Md[:n, :n]
    Bd = Md[:n, n:]
    return Ad, Bd


if __name__ == "__main__":
    p = load_params()
    A, B = linear_model(p)
    print("mass [kg]:", p["mass"])
    print("hover thrust [N]:", p["mass"] * p["gravity"])
    print("max thrust [N]:", max_total_thrust(p))
    np.set_printoptions(precision=4, suppress=True)
    print("A =\n", A)
    print("B =\n", B)
