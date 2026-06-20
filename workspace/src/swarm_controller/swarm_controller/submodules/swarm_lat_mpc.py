"""Lateral MPC controller for swarm follower (path-following + actuator lag).

Plant model — error-space against `/pacemaker/path`, with first-order angular
actuator lag:

    state:    x = [e, e_θ, w, w_cmd, e_int]            (n_in = 5)
              e      — signed perpendicular distance to path [m]
              e_θ    — heading error vs path tangent [rad]
              w      — own angular velocity [rad/s] (measured)
              w_cmd  — integrator on α_cmd, published as Twist.angular.z
              e_int  — integral of e (∫e dt) — offset-free под постоянной кривизной
    control:  u = α_cmd                                (angular acceleration cmd)

    ė       = v · e_θ              ← LTV: `v` enters via path-following geom
    ė_θ     = w − v · κ_ref         ← LTV + W disturbance from path curvature
    ẇ       = (w_cmd − w) / τ_w     ← actuator lag, matches simulator
    ẇ_cmd   = u                    ← integrator on a_cmd (smooth Twist.angular.z)
    ė_int   = e                    ← интеграл поперечной ошибки

Output for cost:
    y = [e, e_θ, α, e_int]                              (n_out = 4)

    Штраф q_int·e_int² заставляет средний e → 0: при ненулевом среднем e интеграл
    растёт, стоимость расходится, оптимизатор гонит e в ноль; постоянный «доворот»
    (bias w) при этом держит интегратор w_cmd — стационарной ошибки нет.

    α = (w_cmd − w) / τ_w   — physical angular acceleration, linear in state,
                              ZERO in steady state. Penalising α² gives clean
                              damping (no static offset).

Path inputs (rebuilt every cycle):
    v        — own longitudinal velocity (from odom or /long_cmd) [m/s]
    κ_ref    — path curvature at nearest point [1/m]
    Both feed into A matrix and W disturbance — this is LTV-MPC.

Constraints:
    α_cmd ∈ [α_min, α_max]                              (input bound)
    w_cmd ∈ [w_cmd_min, w_cmd_max]   — clipped on integrator state at the end
                                       of each cycle (anti-windup, no QP row)
"""

from __future__ import annotations

import numpy as np
from scipy.sparse import csc_matrix
import osqp


class SwarmLatController:
    """Path-following lateral MPC for a swarm follower.

    Each call to `calculate_control` rebuilds dynamics matrices around the
    current longitudinal velocity `v` and reference curvature `κ_ref` (LTV).
    Hessian is constant in v_nominal but updated as v changes — see
    `_rebuild_at_operating_point`.
    """

    n_in = 5    # state:  [e, e_θ, w, w_cmd, e_int]
    n_out = 4   # output: [e, e_θ, α, e_int]

    def __init__(
        self,
        tau_w: float,
        ts: float, p: int, c: int, s: float,
        phi_vals, q_vals,
        alpha_limits, w_cmd_limits,
        e_int_limit: float = 2.0,
    ):
        # plant
        self.tau_w = float(tau_w)
        # MPC tuning
        self.ts = float(ts)
        self.p = int(p)
        self.c = int(c)
        self.s = float(s)
        # constraints
        self.alpha_min, self.alpha_max = (
            float(alpha_limits[0]), float(alpha_limits[1]))
        self.w_cmd_min, self.w_cmd_max = (
            float(w_cmd_limits[0]), float(w_cmd_limits[1]))
        self.e_int_limit = float(e_int_limit)   # anti-windup на интеграле

        n_in, n_out, p_, c_ = self.n_in, self.n_out, self.p, self.c

        phi = np.asarray(phi_vals, dtype=float)
        q = np.asarray(q_vals, dtype=float)
        assert phi.shape == (n_out,), f"phi_vals must have length {n_out}"
        assert q.shape == (n_out,), f"q_vals must have length {n_out}"
        assert c_ <= p_, "control horizon c must not exceed prediction horizon p"

        # output matrix C (4×5):  y = C @ x   (no Z thanks to error-space)
        #  state cols:    e   e_θ  w           w_cmd     e_int
        C = np.zeros((n_out, n_in))
        C[0, 0] = 1.0                            # e
        C[1, 1] = 1.0                            # e_θ
        C[2, 2] = -1.0 / self.tau_w              # α = (w_cmd − w) / τ_w
        C[2, 3] =  1.0 / self.tau_w              #
        C[3, 4] = 1.0                            # e_int (∫e dt) — offset-free
        self.C = C

        # error-correction H = I, F_mat = C @ H
        self.H = np.eye(n_in)
        self.F_mat = C @ self.H

        # reference shaping FA: y_ref(k+i) = Phi^i · y_now
        Phi = np.diag(phi)
        FA = np.zeros((p_ * n_out, n_out))
        Phi_pow = np.eye(n_out)
        for i in range(p_):
            Phi_pow = Phi_pow @ Phi
            FA[i * n_out:(i + 1) * n_out, :] = Phi_pow
        self.FA = FA

        # block-diagonal Q over horizon
        self.Qmat = np.zeros((p_ * n_out, p_ * n_out))
        Qblock = np.diag(q)
        for i in range(p_):
            self.Qmat[i*n_out:(i+1)*n_out, i*n_out:(i+1)*n_out] = Qblock

        # move-suppression Hessian H1 (penalty on Δu = α_cmd_k − α_cmd_{k-1})
        H1 = np.zeros((c_, c_))
        for i in range(c_):
            H1[i, i] = self.s if i == c_ - 1 else 2.0 * self.s
            if i > 0:
                H1[i, i - 1] = -self.s
            if i < c_ - 1:
                H1[i, i + 1] = -self.s
        self.H1 = H1
        self.M2 = np.zeros(c_)
        self.M2[0] = self.s

        # state predictor + warm start (rebuilt by reset())
        self._x_predicted = None
        self._u_prev = 0.0

        # OSQP — must be re-setup whenever Hessian changes (i.e. when v changes
        # significantly, since v enters Qmat·D_hat·D_hat^T via D_hat).  We
        # cache (Hqp, A_const sparsity pattern) and re-setup on first cycle.
        self._prob = osqp.OSQP()
        self._initialized = False
        self._v_at_setup: float | None = None  # v used for the cached Hessian

    # ── dynamics rebuild (called every cycle) ──────────────────────────────

    def _build_dynamics(self, v: float):
        """Continuous → discrete (forward Euler) at operating point v.

        Returns A_d (n_in×n_in), B_d (n_in,), Wext_d (n_in,) function-of-κ.
        Wext_d already includes scaling by ts but NOT κ — caller multiplies.
        """
        ts, tau_w = self.ts, self.tau_w
        # state:           e     e_θ    w               w_cmd     e_int
        A_d = np.array([
            [1.0,          ts*v,  0.0,            0.0,      0.0],   # e
            [0.0,          1.0,   ts,             0.0,      0.0],   # e_θ
            [0.0,          0.0,   1.0 - ts/tau_w, ts/tau_w, 0.0],   # w
            [0.0,          0.0,   0.0,            1.0,      0.0],   # w_cmd
            [ts,           0.0,   0.0,            0.0,      1.0],   # e_int = e_int + ts·e
        ])
        # input drives w_cmd via integrator
        B_d = np.array([0.0, 0.0, 0.0, ts, 0.0])
        # Disturbance template: e_θ row gets (-v · κ_ref) per ts.  Caller
        # multiplies by κ_ref each cycle.  Layout matches A's row order.
        W_per_kappa_d = np.array([0.0, -ts * v, 0.0, 0.0, 0.0])
        return A_d, B_d, W_per_kappa_d

    def _rebuild_at_operating_point(self, v: float):
        """Recompute A_hat, C_hat, D_hat, Hessian for current v.

        For LTV-MPC: matrices depend on v, so we cannot do this once in
        __init__.  Cost: O(p · n_in²) work each cycle, ≪ 1 ms for our sizes.
        """
        n_in, n_out, p_, c_ = self.n_in, self.n_out, self.p, self.c
        A_d, B_d, _ = self._build_dynamics(v)
        self.A_d = A_d
        self.B_d = B_d

        # state-prediction stacks
        A_hat = np.zeros((p_ * n_in, n_in))
        C_hat = np.zeros((p_ * n_out, n_in))
        A_pow = np.eye(n_in)
        for i in range(p_):
            A_pow = A_pow @ A_d
            A_hat[i*n_in:(i+1)*n_in, :] = A_pow
            C_hat[i*n_out:(i+1)*n_out, :] = self.C @ A_pow
        self.A_hat = A_hat
        self.C_hat = C_hat

        # impulse response C·A^k·B
        CAkB = np.zeros((p_, n_out))
        A_pow = np.eye(n_in)
        for k in range(p_):
            CAkB[k] = self.C @ A_pow @ B_d
            A_pow = A_pow @ A_d
        # convolution matrix D_hat (p·n_out × c)
        D_hat = np.zeros((p_ * n_out, c_))
        for i in range(p_):
            for m in range(min(i + 1, c_)):
                k = i - m
                D_hat[i*n_out:(i+1)*n_out, m] = CAkB[k]
        self.D_hat = D_hat

        # F_hat (error-correction stack)
        self.F_hat = np.tile(self.F_mat, (p_, 1))

        # Hessian (depends on D_hat which depends on v)
        Hqp = self.H1 + D_hat.T @ self.Qmat @ D_hat
        Hqp = 0.5 * (Hqp + Hqp.T)
        self.Hqp = Hqp

    # ── controller ─────────────────────────────────────────────────────────

    def calculate_control(
        self,
        e: float, e_theta: float, w: float,
        v: float, kappa_profile=None,
    ):
        """Solve one MPC step.

        Args:
            e             — signed lateral distance to path [m]
            e_theta       — heading error vs path tangent [rad]
            w             — own angular velocity [rad/s] (from odom)
            v             — own longitudinal velocity [m/s] (LTV op-point)
            kappa_profile — path curvature predicted along the horizon:
                            None or scalar 0 ⇒ no feed-forward;
                            scalar ⇒ same κ held constant over horizon
                            (legacy behaviour);
                            array of length ≥ p ⇒ time-varying κ that lets
                            MPC anticipate S-curves and other κ
                            transitions inside the horizon.

        Returns:
            (w_cmd_published, alpha_cmd, y) where:
                w_cmd_published — integrator state, clipped to [w_cmd_min,
                                  w_cmd_max] (publish to Twist.angular.z)
                alpha_cmd       — optimal angular acceleration command
                y               — current output [e, e_θ, α]
        """
        n_in, c_, p_ = self.n_in, self.c, self.p

        # Normalise feed-forward input: None / scalar / array → length-p array.
        if kappa_profile is None:
            kp = np.zeros(p_)
        else:
            arr = np.asarray(kappa_profile, dtype=float).ravel()
            if arr.size == 0:
                kp = np.zeros(p_)
            elif arr.size == 1:
                kp = np.full(p_, float(arr[0]))
            elif arr.size >= p_:
                kp = arr[:p_]
            else:
                kp = np.concatenate([arr, np.full(p_ - arr.size, arr[-1])])

        # Rebuild matrices around current v (LTV).  Floor v to avoid losing
        # lateral controllability at v→0 (e_dot = v·e_θ goes to zero).  We
        # allow v=0 in the state but use a small floor for the LTV linearisation.
        v_op = max(v, 0.05)
        self._rebuild_at_operating_point(v_op)

        # внутренние состояния (не измеряются — берём из предиктора, ex на них = 0):
        # w_cmd (интегратор α) и e_int (∫e dt).
        if self._x_predicted is not None:
            w_cmd_est = float(self._x_predicted[3])
            e_int_est = float(self._x_predicted[4])
        else:
            w_cmd_est = 0.0
            e_int_est = 0.0

        x = np.array([e, e_theta, w, w_cmd_est, e_int_est])

        # state-error correction on measured states (e, e_θ, w)
        if self._x_predicted is not None:
            ex = x - self._x_predicted
        else:
            ex = np.zeros(n_in)

        y = self.C @ x
        Yr = self.FA @ y
        F2 = self.C_hat @ x + self.F_hat @ ex
        # Add path-curvature feed-forward into the predicted free response.
        # κ enters via W disturbance: ė_θ = w − v·κ.  Free response (w=0):
        #   e_θ[k+1] = e_θ[0] − ts·v·sum_{j=0..k} κ[j]
        # so at step (i+1) we subtract -ts·v·cum(kp[0..i]) from e_θ.
        # When kp varies within the horizon (S-curve), the cumulative
        # shift's slope changes accordingly and the optimal w_cmd swings
        # the right way *before* the curve actually arrives.
        # Higher-order propagation into the e row is dropped — small for
        # our v·ts·κ scales at p=20 (max contribution ≈ 0.04 m).
        if np.any(np.abs(kp) > 1e-9):
            cum_kappa = 0.0
            for i in range(p_):
                cum_kappa += float(kp[i])
                F2[i*self.n_out + 1] += -self.ts * v_op * cum_kappa
        b1 = Yr - F2

        g = -self.M2 * self._u_prev - self.D_hat.T @ self.Qmat @ b1

        # constraints: only input bound (anti-windup on w_cmd handled below)
        rows = [np.eye(c_)]
        lb_parts = [np.full(c_, self.alpha_min)]
        ub_parts = [np.full(c_, self.alpha_max)]
        A_const = np.vstack(rows)
        u_lb = np.concatenate(lb_parts)
        u_ub = np.concatenate(ub_parts)

        # OSQP: re-setup if v changed enough that Hessian needs refresh, OR
        # on the very first call.  For modest v changes (<10%) we skip
        # re-setup to keep warm-start, but still update q/l/u.
        v_changed = (
            self._v_at_setup is None
            or abs(v_op - self._v_at_setup) / max(self._v_at_setup, 0.05) > 0.10
        )
        if not self._initialized or v_changed:
            P_sparse = csc_matrix(self.Hqp)
            A_sparse = csc_matrix(A_const)
            self._prob = osqp.OSQP()
            self._prob.setup(
                P=P_sparse, q=g,
                A=A_sparse, l=u_lb, u=u_ub,
                verbose=False, time_limit=self.ts * 0.8,
                warm_starting=False,
            )
            self._initialized = True
            self._v_at_setup = v_op
        else:
            self._prob.update(q=g, l=u_lb, u=u_ub)

        result = self._prob.solve()
        if result.info.status not in ('solved', 'solved_inaccurate'):
            # Infeasible (rare for lat with only input bound).  Fallback: 0.
            alpha_opt = 0.0
            self._prob.update_settings(warm_starting=False)
        else:
            alpha_opt = float(result.x[0])
            self._prob.update_settings(warm_starting=True)

        alpha_opt = float(np.clip(alpha_opt, self.alpha_min, self.alpha_max))

        # advance state predictor (with κ disturbance on e_θ).  Predictor
        # advances exactly one ts, so use κ at the current step (kp[0]).
        x_next = self.A_d @ x + self.B_d * alpha_opt + self.H @ ex
        x_next[1] += -self.ts * v_op * float(kp[0])  # e_θ disturbance from path
        self._x_predicted = x_next
        self._u_prev = alpha_opt

        # anti-windup: clip integrator state w_cmd to the published range so
        # the predictor stays in sync with what the plant actually receives
        self._x_predicted[3] = float(np.clip(
            self._x_predicted[3], self.w_cmd_min, self.w_cmd_max))
        # anti-windup на интеграле поперечной ошибки
        self._x_predicted[4] = float(np.clip(
            self._x_predicted[4], -self.e_int_limit, self.e_int_limit))

        w_cmd_published = float(self._x_predicted[3])
        return w_cmd_published, alpha_opt, y.copy()

    def reset(self):
        """Reset state predictor (incl. integrator) and warm start."""
        self._x_predicted = None
        self._u_prev = 0.0
        if self._initialized:
            self._prob = osqp.OSQP()
            self._initialized = False
            self._v_at_setup = None
