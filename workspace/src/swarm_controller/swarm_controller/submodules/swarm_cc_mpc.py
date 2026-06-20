"""Kinematic CC MPC controller — adas-exact formulation.

Plant model is purely kinematic (no mass / friction): only acceleration lag
with a single time constant tau. Control input is `a_cmd` (acceleration
command); the published velocity command is integrated by the ROS node
(strategy "B2" — see docs).

State (3 dimensions):
    x = [v, a, j]

Control:
    u = a_cmd                     (acceleration command, [m/s^2])

Continuous-time dynamics (after Euler with step ts):
    v_next   = v + ts*a
    a_next   = (1 - ts/tau)*a + (ts/tau)*u            (1st-order lag)
    j_next   = -(1/tau)*a + (1/tau)*u                 (= (u-a)/tau, observed jerk)

The `j` row has NO dependency on previous `j` — adas keeps `j` in the state
vector solely so that jerk^2 can be put directly into the cost function.

Output for cost:
    y = C*x - Z   with Z = [v_ref, 0, 0]
    so y = [v - v_ref, a, j]                          (n_out = 3)

Reference shaping: y_ref(k+i) = Phi^i * y(k) — exponential decay of the
tracking error toward zero. v_ref is passed per call so a velocity smoother
can change it as the pacemaker moves through curvature.

Constraints:
    a_cmd in [a_min, a_max]                           (input bound)

Mirrors adas/src/cc_mpc.cpp: identical A, B, Z. Designed as the pacemaker
counterpart of SwarmAccController — same plant, no gap-keeping outputs.
"""

import numpy as np
from scipy.sparse import csc_matrix
import osqp


class SwarmCruiseController:
    """Kinematic CC MPC for the pacemaker (adas-style 3-state)."""

    n_in = 3   # state:  [v, a, j]
    n_out = 3  # output: [v - v_ref, a, j]

    def __init__(
        self,
        tau: float,
        ts: float, p: int, c: int, s: float,
        phi_vals, q_vals,
        u_limits,
    ):
        self.tau = float(tau)
        self.ts = float(ts)
        self.p = int(p)
        self.c = int(c)
        self.s = float(s)
        self.u_min, self.u_max = float(u_limits[0]), float(u_limits[1])

        n_in, n_out, p_, c_ = self.n_in, self.n_out, self.p, self.c

        phi = np.asarray(phi_vals, dtype=float)
        q   = np.asarray(q_vals,   dtype=float)
        assert phi.shape == (n_out,), f"phi_vals must have length {n_out}"
        assert q.shape == (n_out,), f"q_vals must have length {n_out}"
        assert c_ <= p_, "control horizon c must not exceed prediction horizon p"

        # output matrix C (3x3): picks [v, a, j] from state; setpoint
        # subtraction (Z = [v_ref, 0, 0]) happens at solve time so v_ref
        # can change between calls.
        self.C = np.eye(n_in)

        self.H = np.eye(n_in)
        self.F_mat = self.C @ self.H

        # constant dynamics
        self.A, self.B = self._build_dynamics()

        # stacked error-correction matrix over horizon
        self.F_hat = np.tile(self.F_mat, (p_, 1))

        # reference shaping FA: y_ref(k+i) = Phi^i * y(k)
        Phi = np.diag(phi)
        FA = np.zeros((p_ * n_out, n_out))
        Phi_pow = np.eye(n_out)
        for i in range(p_):
            Phi_pow = Phi_pow @ Phi
            FA[i * n_out:(i + 1) * n_out, :] = Phi_pow
        self.FA = FA

        # block-diagonal Q over horizon
        Qblock = np.diag(q)
        Qmat = np.zeros((p_ * n_out, p_ * n_out))
        for i in range(p_):
            Qmat[i * n_out:(i + 1) * n_out, i * n_out:(i + 1) * n_out] = Qblock
        self.Qmat = Qmat

        # state-prediction stacks A_hat, C_hat
        A_hat = np.zeros((p_ * n_in, n_in))
        C_hat = np.zeros((p_ * n_out, n_in))
        A_pow = np.eye(n_in)
        for i in range(p_):
            A_pow = A_pow @ self.A
            A_hat[i * n_in:(i + 1) * n_in, :] = A_pow
            C_hat[i * n_out:(i + 1) * n_out, :] = self.C @ A_pow
        self.A_hat = A_hat
        self.C_hat = C_hat

        # impulse response blocks C*A^k*B, k = 0..p-1
        CAkB = np.zeros((p_, n_out))
        A_pow = np.eye(n_in)
        for k in range(p_):
            CAkB[k] = self.C @ A_pow @ self.B
            A_pow = A_pow @ self.A
        # convolution matrix D_hat (p*n_out x c)
        D_hat = np.zeros((p_ * n_out, c_))
        for i in range(p_):
            for m in range(min(i + 1, c_)):
                k = i - m
                D_hat[i * n_out:(i + 1) * n_out, m] = CAkB[k]
        self.D_hat = D_hat

        # move-suppression Hessian H1 (penalty on Delta u)
        H1 = np.zeros((c_, c_))
        for i in range(c_):
            H1[i, i] = self.s if i == c_ - 1 else 2.0 * self.s
            if i > 0:
                H1[i, i - 1] = -self.s
            if i < c_ - 1:
                H1[i, i + 1] = -self.s
        self.H1 = H1

        # M2 (linear coupling u_prev -> u_0)
        self.M2 = np.zeros(c_)
        self.M2[0] = self.s

        # CONSTANT QP Hessian (input bounds only — no gap-safety row)
        Hqp = self.H1 + self.D_hat.T @ self.Qmat @ self.D_hat
        Hqp = 0.5 * (Hqp + Hqp.T)
        self.Hqp = Hqp

        # constraint matrix is constant (input bounds): one row per move
        self.A_const = np.eye(c_)

        # state predictor + warm start
        self._x_predicted = None
        self._u_prev = 0.0

        self._prob = osqp.OSQP()
        self._initialized = False

    def _build_dynamics(self):
        """Build A (3x3) and B (3,) — adas-exact Euler discretisation."""
        ts, tau = self.ts, self.tau
        A = np.array([
            #  v    a              j
            [ 1.0, ts,            0.0 ],   # v
            [ 0.0, 1.0 - ts/tau,  0.0 ],   # a       (1st-order to u)
            [ 0.0, -1.0/tau,      0.0 ],   # j       (= (u-a)/tau)
        ])
        B = np.array([0.0, ts/tau, 1.0/tau])
        return A, B

    def calculate_control(self, v_ref: float, v: float):
        """Solve one MPC step.

        Args:
            v_ref  — desired longitudinal speed [m/s]
            v      — measured longitudinal speed [m/s] (from odom)

        Returns:
            (a_cmd, y) — optimal acceleration command and current output
                         error vector [v - v_ref, a_pred, j_pred]
        """
        n_in, c_, p_, n_out = self.n_in, self.c, self.p, self.n_out

        if self._x_predicted is not None:
            a_est = float(self._x_predicted[1])
            j_est = float(self._x_predicted[2])
        else:
            a_est = 0.0
            j_est = 0.0

        x = np.array([v, a_est, j_est])
        Z = np.array([float(v_ref), 0.0, 0.0])
        Z_hat = np.tile(Z, p_)

        # offset-free коррекция ОТКЛЮЧЕНА (ex=0). Интегральное действие уже даёт B2-интегратор
        # ноды (v_cmd = ∫a_cmd), поэтому внутренний ex избыточен. При смене v_ref он накапливал
        # windup: a-состояние уходило в минус -> ложный +сдвиг v -> CC застревал ниже цели
        # (виден при curve_slowdown: после поворота v_ref=0.3, а робот держал ~0.22).
        ex = np.zeros(n_in)

        y  = self.C @ x - Z
        Yr = self.FA @ y
        F2 = self.C_hat @ x - Z_hat + self.F_hat @ ex
        b1 = Yr - F2

        g = -self.M2 * self._u_prev - self.D_hat.T @ self.Qmat @ b1

        u_lb = np.full(c_, self.u_min)
        u_ub = np.full(c_, self.u_max)

        if not self._initialized:
            P_sparse = csc_matrix(self.Hqp)
            A_sparse = csc_matrix(self.A_const)
            self._prob.setup(
                P=P_sparse, q=g,
                A=A_sparse, l=u_lb, u=u_ub,
                verbose=False, time_limit=self.ts * 0.8,
                warm_starting=False,
            )
            self._initialized = True
        else:
            self._prob.update(q=g, l=u_lb, u=u_ub)

        result = self._prob.solve()
        if result.info.status not in ('solved', 'solved_inaccurate'):
            # infeasible (should not happen with bounds-only QP) → coast
            u_opt = 0.0
            self._prob.update_settings(warm_starting=False)
        else:
            u_opt = float(result.x[0])
            self._prob.update_settings(warm_starting=True)

        u_opt = float(np.clip(u_opt, self.u_min, self.u_max))

        # advance predictor
        self._x_predicted = self.A @ x + self.B * u_opt + self.H @ ex
        self._u_prev = u_opt

        return u_opt, y.copy()

    def reset(self):
        """Reset state predictor + warm start."""
        self._x_predicted = None
        self._u_prev = 0.0
        if self._initialized:
            self._prob = osqp.OSQP()
            self._initialized = False
