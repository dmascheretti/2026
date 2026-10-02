"""Tests for cf_model.py. Run: python3 -m unittest discover model"""
import unittest

import casadi as ca
import numpy as np

import cf_model


class TestModel(unittest.TestCase):
    def setUp(self):
        self.p = cf_model.load_params()

    def test_hand_written_linearisation_matches_casadi_jacobian(self):
        x, u, xdot = cf_model.nonlinear_dynamics_casadi(self.p)
        jac = ca.Function("linearization", [x, u], [ca.jacobian(xdot, x), ca.jacobian(xdot, u)])
        A_ca, B_ca = jac(np.zeros(cf_model.NX), np.zeros(cf_model.NU))
        A, B = cf_model.linear_model(self.p)
        np.testing.assert_allclose(np.array(A_ca), A, atol=1e-12)
        np.testing.assert_allclose(np.array(B_ca), B, atol=1e-12)

    def test_hover_is_equilibrium(self):
        x, u, xdot = cf_model.nonlinear_dynamics_casadi(self.p)
        f = ca.Function("f", [x, u], [xdot])
        np.testing.assert_allclose(np.array(f(np.zeros(8), np.zeros(3))).ravel(), 0.0, atol=1e-12)

    def test_discretisation_matches_euler_for_small_dt(self):
        A, B = cf_model.linear_model(self.p)
        dt = 1e-5
        Ad, Bd = cf_model.discretize(A, B, dt)
        np.testing.assert_allclose(Ad, np.eye(cf_model.NX) + A * dt, atol=1e-8)
        np.testing.assert_allclose(Bd, B * dt, atol=1e-8)

    def test_discrete_double_integrator_in_z(self):
        # z-axis is a pure double integrator: pz(dt) = 0.5 * a * dt^2
        A, B = cf_model.linear_model(self.p)
        dt = 0.02
        Ad, Bd = cf_model.discretize(A, B, dt)
        a = 1.0 / self.p["mass"]
        self.assertAlmostEqual(Bd[2, 2], 0.5 * a * dt**2, places=12)
        self.assertAlmostEqual(Bd[5, 2], a * dt, places=12)

    def test_thrust_to_weight_is_plausible(self):
        tw = cf_model.max_total_thrust(self.p) / (self.p["mass"] * self.p["gravity"])
        self.assertGreater(tw, 1.5)
        self.assertLess(tw, 4.0)


if __name__ == "__main__":
    unittest.main()
