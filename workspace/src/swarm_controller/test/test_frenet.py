"""Unit tests for windowed Frenet projection and curvature helpers."""

import numpy as np
import pytest

from swarm_controller.submodules.frenet import (
    compute_kappa_profile_along_horizon,
    frenet_project,
    menger_curvature,
)


def _figure8(n: int = 200, scale: float = 4.0):
    """Lemniscate (figure-8) sampled at n points — self-intersects at origin."""
    t = np.linspace(0, 2 * np.pi, n, endpoint=False)
    x = scale * np.sin(t)
    y = scale * np.sin(t) * np.cos(t)
    return np.column_stack([x, y])


def _circle(R: float = 2.0, n: int = 200):
    t = np.linspace(0, 2 * np.pi, n, endpoint=False)
    return np.column_stack([R * np.cos(t), R * np.sin(t)])


def _straight(length: float = 10.0, n: int = 50):
    return np.column_stack([np.linspace(0, length, n), np.zeros(n)])


def test_global_search_basic():
    """Without prev_idx, falls back to global brute-force search."""
    wp = _circle(2.0, n=200)
    px, py = wp[:, 0], wp[:, 1]
    # robot at angle 0 → nearest is index 0
    idx, e, e_th, kappa = frenet_project(px, py, 2.0, 0.0, np.pi / 2)
    assert idx == 0
    assert abs(kappa - 0.5) < 0.05  # 1/R
    assert abs(e) < 1e-6


def test_windowed_search_avoids_jump_at_intersection():
    """On a figure-8, near the crossing the global search may snap to the
    OTHER branch.  Windowed search around prev_idx must stay on the current
    branch."""
    wp = _figure8(n=400)  # tightly sampled for clean test
    px, py = wp[:, 0], wp[:, 1]

    # Walk a robot along the path, near the self-intersection
    # The figure 8 crosses itself at (0, 0). Pick a point on first branch
    # that is geometrically close to a point on the OTHER branch.
    # Index ≈ 0..200 is one loop, 200..400 is the other (sin·cos symmetry).
    branch1_idx = 50    # somewhere on first branch, near the crossing
    branch2_idx = 350   # symmetric on the other branch

    # Robot is between branches but slightly toward branch1 (prev_idx=branch1)
    rx = (px[branch1_idx] + px[branch2_idx]) / 2.0 + 0.01 * (px[branch1_idx] - px[branch2_idx])
    ry = (py[branch1_idx] + py[branch2_idx]) / 2.0 + 0.01 * (py[branch1_idx] - py[branch2_idx])

    # Global search would pick whichever vertex is closest in pure distance,
    # which can be on the wrong branch.  Windowed must stay near branch1_idx.
    idx_windowed, _, _, _ = frenet_project(
        px, py, rx, ry, 0.0,
        prev_idx=branch1_idx, search_window=20,
    )
    assert abs(idx_windowed - branch1_idx) <= 20, \
        f'windowed search escaped to idx={idx_windowed} from prev=branch1={branch1_idx}'


def test_cyclic_autodetection_circle():
    """Circle has equal endpoint distance ≈ spacing → auto-detected cyclic."""
    wp = _circle(2.0, n=200)
    px, py = wp[:, 0], wp[:, 1]
    # at idx near 0 with cyclic, search window wraps around
    idx, _, _, kappa = frenet_project(
        px, py, 2.0, 0.05, 0.0,
        prev_idx=199, search_window=5,
    )
    # robot is just CCW of idx=0, with prev=199 cyclic should let us reach 0
    assert idx in (0, 199, 1, 198, 2, 197, 3, 196, 4, 195)
    # curvature should still be ≈1/R = 0.5
    assert abs(abs(kappa) - 0.5) < 0.1


def test_open_path_no_wrap():
    """Open straight path: no wrap, end-of-path uses backward tangent."""
    wp = _straight(10.0, n=11)
    px, py = wp[:, 0], wp[:, 1]
    # robot near the END
    idx, e, e_th, kappa = frenet_project(
        px, py, 9.5, 0.0, 0.0,
        prev_idx=10, search_window=3, cyclic=False,
    )
    assert idx in (9, 10)
    assert abs(kappa) < 1e-6  # straight has zero curvature
    assert abs(e) < 1e-6


def test_search_window_bounded():
    """When prev_idx given, idx never moves outside window."""
    wp = _circle(2.0, n=200)
    px, py = wp[:, 0], wp[:, 1]
    # robot far away from path — without window would land somewhere random
    # but with prev_idx=10 and window=5, must land in [5, 15]
    idx, _, _, _ = frenet_project(
        px, py, 100.0, 100.0, 0.0,
        prev_idx=10, search_window=5,
    )
    # accept wrap (cyclic for circle): indices 5..15 OR 195..199 (wrap)
    assert idx in list(range(5, 16)) + list(range(195, 200)) + [0, 1, 2, 3, 4], \
        f'idx={idx} escaped the window'


# ── menger_curvature + compute_kappa_profile_along_horizon ────────────────

def test_menger_curvature_circle():
    """On a circle of radius R, every point should have κ ≈ 1/R."""
    R = 2.0
    wp = _circle(R, n=60)
    k = menger_curvature(wp, cyclic=True)
    # all points well-defined with cyclic wrap; constant κ on a circle
    assert np.std(k) < 0.05
    assert abs(np.mean(k) - 1.0 / R) < 0.05


def test_menger_curvature_straight_is_zero():
    """Straight line ⇒ κ = 0 everywhere."""
    wp = _straight(10.0, n=50)
    k = menger_curvature(wp, cyclic=False)
    assert np.allclose(k, 0.0, atol=1e-9)


def test_menger_curvature_open_endpoints_zero():
    """Open path: endpoints get κ=0 (no neighbours one side)."""
    wp = _circle(2.0, n=60)  # treat as open
    k = menger_curvature(wp, cyclic=False)
    assert k[0] == 0.0
    assert k[-1] == 0.0
    # interior is still ≈ 1/R
    assert abs(np.mean(k[1:-1]) - 0.5) < 0.05


def test_kappa_profile_constant_on_circle():
    """On a circle, κ profile across horizon is ≈ const = 1/R."""
    R = 2.0
    wp = _circle(R, n=60)
    kappa = menger_curvature(wp, cyclic=True)
    diffs = np.diff(wp, axis=0)
    mean_spacing = float(np.mean(np.linalg.norm(diffs, axis=1)))
    profile = compute_kappa_profile_along_horizon(
        idx_now=0, v=0.4, ts=0.05, horizon=20,
        kappa_arr=kappa, mean_spacing=mean_spacing, cyclic=True,
    )
    assert profile.shape == (20,)
    assert np.std(profile) < 0.05
    assert abs(np.mean(profile) - 1.0 / R) < 0.05


def test_kappa_profile_advances_through_spike():
    """Profile picks up a spike at a future idx, not idx_now."""
    kappa = np.zeros(50)
    kappa[10] = 1.0
    profile = compute_kappa_profile_along_horizon(
        idx_now=7, v=0.4, ts=0.05, horizon=20,
        kappa_arr=kappa, mean_spacing=0.1, cyclic=False,
    )
    assert profile[0] == 0.0                   # idx 7 has κ=0
    assert profile.max() == pytest.approx(1.0, abs=1e-6)


def test_kappa_profile_v_zero_holds_const():
    """v=0 ⇒ profile holds κ_now constant for all steps."""
    kappa = np.array([0.0, 0.5, -0.5, 1.0])
    profile = compute_kappa_profile_along_horizon(
        idx_now=2, v=0.0, ts=0.05, horizon=10,
        kappa_arr=kappa, mean_spacing=0.1, cyclic=True,
    )
    assert np.allclose(profile, -0.5)


def test_kappa_profile_cyclic_wraps():
    """Cyclic ⇒ near-end idx_now wraps to start of array."""
    kappa = np.zeros(20)
    kappa[1] = 1.0
    profile = compute_kappa_profile_along_horizon(
        idx_now=18, v=0.4, ts=0.05, horizon=20,
        kappa_arr=kappa, mean_spacing=0.1, cyclic=True,
    )
    assert profile.max() == pytest.approx(1.0, abs=1e-6)
