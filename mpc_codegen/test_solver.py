"""Checks of the generated MPC. Run after generate_solver.py:

    python3 -m unittest discover mpc_codegen
"""
import sys
import unittest
from pathlib import Path

import numpy as np
import scipy.linalg
from acados_template import AcadosOcpSolver

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "model"))
import cf_model  # noqa: E402
import generate_solver  # noqa: E402


class TestSolver(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.params = cf_model.load_params()
        cls.tuning = generate_solver.load_tuning()
        ocp, cls.Ad, cls.Bd, cls.P, cls.u_min, cls.u_max = generate_solver.build_ocp(
            cls.params, cls.tuning)
        cls.solver = AcadosOcpSolver(ocp, generate=False, build=False, verbose=False)

    def solve(self, x0):
        self.solver.set(0, "lbx", x0)
        self.solver.set(0, "ubx", x0)
        status = self.solver.solve()
        self.assertEqual(status, 0)
        return self.solver.get(0, "u")

    def test_hover_gives_zero_input(self):
        u = self.solve(np.zeros(cf_model.NX))
        np.testing.assert_allclose(u, 0.0, atol=1e-8)

    def test_small_error_matches_lqr(self):
        # With no active bound and terminal cost P = DARE solution,
        # the first MPC input equals the infinite-horizon LQR input.
        Q = np.diag(self.tuning["state_weights"])
        R = np.diag(self.tuning["input_weights"])
        K = np.linalg.solve(R + self.Bd.T @ self.P @ self.Bd, self.Bd.T @ self.P @ self.Ad)
        x0 = np.array([0.02, -0.01, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0])
        u = self.solve(x0)
        np.testing.assert_allclose(u, -K @ x0, atol=1e-6)
        self.assertTrue(np.allclose(Q, Q.T))

    def test_large_error_respects_bounds(self):
        x0 = np.array([5.0, -5.0, -3.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        u = self.solve(x0)
        self.assertTrue(np.all(u >= self.u_min - 1e-8))
        self.assertTrue(np.all(u <= self.u_max + 1e-8))
        # 5 m behind in x -> pitch forward at the limit; z too low -> max thrust
        self.assertAlmostEqual(u[1], -self.u_max[1], places=6)
        self.assertAlmostEqual(u[2], self.u_max[2], places=6)

    def test_closed_loop_linear_converges(self):
        x = np.array([0.5, -0.3, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0])
        for _ in range(250):  # 5 s
            u = self.solve(x)
            x = self.Ad @ x + self.Bd @ u
        self.assertLess(np.linalg.norm(x[:3]), 1e-3)


if __name__ == "__main__":
    unittest.main()
