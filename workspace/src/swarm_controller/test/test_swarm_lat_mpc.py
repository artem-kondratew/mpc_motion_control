"""Unit tests for SwarmLatController (lateral path-following MPC, Variant B).

Sanity checks on the LTV plant + QP solver. These tests do NOT use ROS;
they exercise the controller class with synthetic measurements only.
"""

import numpy as np
import pytest

from swarm_controller.submodules.swarm_lat_mpc import SwarmLatController


def _make_controller(**overrides):
    """Default config matches params_swarm_lat.yaml.

    Tuning notes:
      - s = 5 — lat plant has fewer states and natural damping through
        e <- v*e_theta <- w chain; less move suppression needed.
    """
    params = dict(
        tau_w=0.03,
        ts=0.05, p=20, c=10, s=5.0,
        phi_vals=[0.6, 0.95, 0.6],
        q_vals=[10.0, 1.0, 0.0],
        alpha_limits=(-2.0, 2.0),
        w_cmd_limits=(-1.5, 1.5),
    )
    params.update(overrides)
    return SwarmLatController(**params)


def test_construction():
    """Class can be constructed with default parameters."""
    ctrl = _make_controller()
    assert ctrl.n_in == 4
    assert ctrl.n_out == 3
    # Hessian is built lazily on first calculate_control call (LTV-MPC),
    # so just verify the move-suppression part is PSD.
    H = ctrl.H1
    assert H.shape == (ctrl.c, ctrl.c)
    assert np.allclose(H, H.T, atol=1e-10)
    eigs = np.linalg.eigvalsh(H)
    assert eigs.min() > -1e-8, f"H1 not PSD: min eig = {eigs.min()}"


def test_first_call_returns_valid():
    """First call returns sane w_cmd, α_cmd in bounds."""
    ctrl = _make_controller()
    w_cmd, alpha, y = ctrl.calculate_control(
        e=0.0, e_theta=0.0, w=0.0, v=0.4)
    assert ctrl.alpha_min <= alpha <= ctrl.alpha_max
    assert ctrl.w_cmd_min <= w_cmd <= ctrl.w_cmd_max
    assert y.shape == (3,)
    # at e=0, e_θ=0, w=0 (steady state) → all outputs ~0
    assert abs(y[0]) < 1e-6
    assert abs(y[1]) < 1e-6


def test_bounds_respected_under_step():
    """Under transient, α_cmd stays within configured bounds."""
    ctrl = _make_controller()
    for _ in range(50):
        w_cmd, alpha, _ = ctrl.calculate_control(
            e=0.5, e_theta=0.0, w=0.0, v=0.4)
        assert ctrl.alpha_min - 1e-9 <= alpha <= ctrl.alpha_max + 1e-9
        assert ctrl.w_cmd_min - 1e-9 <= w_cmd <= ctrl.w_cmd_max + 1e-9


def test_corrects_positive_lateral_error():
    """e > 0 (follower LEFT of path) → MPC commands w < 0 (turn RIGHT, REP-103).

    REP-103: x forward, y left, θ CCW from +x.  e > 0 means follower
    deviates to +y (left).  To return: head toward −y → decrease θ → w < 0.
    """
    ctrl = _make_controller()
    # warm up a few steps to let predictor settle
    for _ in range(5):
        w_cmd, alpha, _ = ctrl.calculate_control(
            e=0.5, e_theta=0.0, w=0.0, v=0.4)
    assert w_cmd < 0, f'at e>0, expected w_cmd<0, got {w_cmd}'


def test_corrects_negative_lateral_error():
    """e < 0 (follower RIGHT of path) → MPC commands w > 0 (turn LEFT)."""
    ctrl = _make_controller()
    for _ in range(5):
        w_cmd, alpha, _ = ctrl.calculate_control(
            e=-0.5, e_theta=0.0, w=0.0, v=0.4)
    assert w_cmd > 0, f'at e<0, expected w_cmd>0, got {w_cmd}'


def test_kappa_feedforward_on_curve():
    """Non-zero κ_ref produces non-zero w_cmd even with zero error.

    On a constantly curving path (κ_ref ≠ 0), the controller must spin at
    w_ref = v · κ_ref just to stay on the path.  Feed-forward via the W
    disturbance term should produce this without integral build-up.
    """
    ctrl = _make_controller()
    v = 0.4
    kappa = 0.5  # tight turn, R = 2 m
    expected_w = v * kappa  # = 0.2 rad/s
    # constant κ over horizon — same as legacy single-κ_ref behaviour
    profile = np.full(ctrl.p, kappa)
    for _ in range(20):  # let the controller converge
        w_cmd, alpha, _ = ctrl.calculate_control(
            e=0.0, e_theta=0.0, w=expected_w, v=v, kappa_profile=profile)
    # Should be in the right direction with right order of magnitude.
    # Sign check is the important property; magnitude depends on tuning.
    assert w_cmd > 0, f'positive κ expected w>0, got {w_cmd}'


def test_closed_loop_with_kinematic_plant():
    """Full closed-loop sim: at e=0.5, e_θ=0, plant should drive e→0.

    Plant matches MPC's internal model exactly (Variant B angular):
        ė       = v · sin(e_θ) ≈ v · e_θ
        ė_θ     = w − v · κ_ref
        ẇ       = (w_cmd − w) / τ_w
    """
    ctrl = _make_controller()
    tau_w = ctrl.tau_w
    ts = ctrl.ts
    v = 0.4

    e, e_theta, w = 0.5, 0.0, 0.0
    history = []
    for k in range(200):  # 10 s
        w_cmd, _, _ = ctrl.calculate_control(
            e=e, e_theta=e_theta, w=w, v=v)
        # advance plant
        e_dot = v * np.sin(e_theta)
        e_theta_dot = w  # κ_ref = 0
        w_dot = (w_cmd - w) / tau_w
        e += ts * e_dot
        e_theta += ts * e_theta_dot
        w += ts * w_dot
        history.append((e, e_theta, w, w_cmd))

    final_e = history[-1][0]
    final_e_theta = history[-1][1]
    assert abs(final_e) < 0.05, f'lat error did not converge: e={final_e}'
    assert abs(final_e_theta) < 0.10, \
        f'heading did not align: e_θ={final_e_theta}'


def test_reset_clears_predictor():
    """reset() should clear internal state."""
    ctrl = _make_controller()
    for _ in range(10):
        ctrl.calculate_control(
            e=0.3, e_theta=0.0, w=0.0, v=0.4)

    assert ctrl._x_predicted is not None
    ctrl.reset()
    assert ctrl._x_predicted is None
    assert ctrl._u_prev == 0.0
    assert ctrl._initialized is False


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
