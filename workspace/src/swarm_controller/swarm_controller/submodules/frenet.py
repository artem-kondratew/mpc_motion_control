"""Frenet-frame helpers shared by lateral controllers.

Used by:
  - swarm_lat_mpc_node.py (subscribes to /pacemaker/path)
  - pacemaker_controller.py (uses its own self.waypoints)
"""

import math
from typing import Tuple

import numpy as np


def menger_curvature(wpts: np.ndarray, cyclic: bool = False) -> np.ndarray:
    """Signed Menger curvature at each waypoint.

    κ_i = 2 · signed_area(i-1, i, i+1) / (|AB|·|BC|·|AC|).  Sign follows
    REP-103 (positive ⇔ left turn).

    cyclic=False (default): endpoints get κ = 0.
    cyclic=True:  endpoints use wrap-around triplet, so κ is well-defined
                  everywhere on a closed loop.
    """
    N = len(wpts)
    kappa = np.zeros(N)
    if N < 3:
        return kappa
    for i in range(N):
        if cyclic:
            i_prev, i_next = (i - 1) % N, (i + 1) % N
        elif 0 < i < N - 1:
            i_prev, i_next = i - 1, i + 1
        else:
            continue
        ax, ay = wpts[i_prev]
        bx, by = wpts[i]
        cx, cy = wpts[i_next]
        area2 = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
        ab = math.hypot(bx - ax, by - ay)
        bc = math.hypot(cx - bx, cy - by)
        ac = math.hypot(cx - ax, cy - ay)
        denom = ab * bc * ac
        if denom > 1e-9:
            kappa[i] = 2.0 * area2 / denom
    return kappa


def compute_kappa_profile_along_horizon(
    idx_now: int,
    v: float,
    ts: float,
    horizon: int,
    kappa_arr: np.ndarray,
    mean_spacing: float,
    cyclic: bool = False,
) -> np.ndarray:
    """Profile of path curvature at the robot's predicted future locations.

    Used by lateral MPC for time-varying disturbance feed-forward: instead
    of holding κ constant over the horizon, the controller sees κ at each
    horizon step k = curvature where the robot is predicted to be after
    k·ts seconds, assuming constant speed v.

    This lets MPC anticipate κ sign-flips on S-curves: when κ crosses zero
    inside the horizon, the cumulative feed-forward shift on e_θ correctly
    bends, and the optimal w_cmd swings the right way *before* the curve
    actually arrives — instead of overshooting reactively.

    Linear interpolation in fractional-idx space avoids the discrete
    waypoint-stepping that otherwise excites MPC at frequency v/spacing.

    Args:
        idx_now       — frenet-projected idx at robot's current pose
        v             — own longitudinal velocity [m/s]
        ts            — MPC step time [s]
        horizon       — MPC prediction horizon p (number of steps)
        kappa_arr     — pre-computed Menger curvature, length N
        mean_spacing  — mean inter-waypoint distance, used to convert
                        arc-length (v·ts·k) into fractional idx-offset
        cyclic        — if True, idx wraps around N (closed paths)

    Returns:
        profile — array of length `horizon`, profile[k] = κ at predicted
                  robot location at horizon step k.
    """
    N = len(kappa_arr)
    profile = np.zeros(horizon)
    if N == 0 or horizon <= 0:
        return profile
    idx_now = int(idx_now)
    idx_now = max(0, min(idx_now, N - 1))
    if mean_spacing <= 1e-9 or v <= 0:
        profile[:] = kappa_arr[idx_now]
        return profile
    for k in range(horizon):
        frac_offset = v * ts * k / mean_spacing
        i_low = int(math.floor(frac_offset))
        frac = frac_offset - i_low
        idx_lo = idx_now + i_low
        idx_hi = idx_lo + 1
        if cyclic:
            idx_lo %= N
            idx_hi %= N
        else:
            idx_lo = max(0, min(idx_lo, N - 1))
            idx_hi = max(0, min(idx_hi, N - 1))
        profile[k] = (1.0 - frac) * kappa_arr[idx_lo] + frac * kappa_arr[idx_hi]
    return profile


def yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    """Yaw angle (REP-103, CCW from +x) from quaternion."""
    return float(math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    ))


def frenet_project(
    px: np.ndarray, py: np.ndarray,
    rx: float, ry: float, rtheta: float,
    *,
    prev_idx: int | None = None,
    search_window: int = 20,
    cyclic: bool | None = None,
) -> Tuple[int, float, float, float]:
    """Project robot pose onto a polyline (path).

    Args:
        px, py        — path waypoints in map frame
        rx, ry        — robot position in map frame [m]
        rtheta        — robot heading [rad], REP-103
        prev_idx      — last cycle's idx; when given, search is restricted to
                        a window around it.  None ⇒ global O(N) brute force.
        search_window — half-width of the candidate index window (default 20)
        cyclic        — treat the path as a closed loop (wrap-around in
                        search and curvature).  None ⇒ auto-detect by
                        comparing dist(path[0], path[-1]) with mean spacing.

    Why windowed search: for self-intersecting paths (lemniscate, figure-8)
    the global argmin can snap to the WRONG branch when the robot is near
    the crossing → idx jumps → κ flips → controller lurches.  Searching in
    a small forward/backward window of the last known idx prevents this.

    Returns:
        idx       — index of nearest segment endpoint
        e         — signed perpendicular distance (REP-103: + ⇔ left)
        e_theta   — heading error vs path tangent, wrapped to [-π, π]
        kappa_ref — signed Menger curvature at idx
    """
    N = len(px)
    if N < 2:
        return 0, 0.0, 0.0, 0.0

    # auto-detect cyclic: endpoints close relative to average spacing
    if cyclic is None and N >= 3:
        diffs = np.diff(np.column_stack([px, py]), axis=0)
        mean_spacing = np.linalg.norm(diffs, axis=1).mean()
        cyclic = bool(math.hypot(px[0] - px[-1], py[0] - py[-1])
                      < 2.0 * mean_spacing)
    elif cyclic is None:
        cyclic = False

    # candidate indices: windowed around prev_idx, or all
    if prev_idx is not None and 0 <= prev_idx < N:
        offsets = np.arange(-search_window, search_window + 1)
        if cyclic:
            candidates = (prev_idx + offsets) % N
        else:
            cs = prev_idx + offsets
            candidates = cs[(cs >= 0) & (cs < N)]
    else:
        candidates = np.arange(N)
    cdx = px[candidates] - rx
    cdy = py[candidates] - ry
    cd2 = cdx * cdx + cdy * cdy
    idx = int(candidates[int(np.argmin(cd2))])

    # tangent: forward diff at idx; wrap if cyclic, else backward at end
    if idx + 1 < N:
        tx, ty = px[idx + 1] - px[idx], py[idx + 1] - py[idx]
    elif cyclic and N >= 2:
        tx, ty = px[0] - px[idx], py[0] - py[idx]
    else:
        tx, ty = px[idx] - px[idx - 1], py[idx] - py[idx - 1]
    tnorm = math.hypot(tx, ty)
    if tnorm < 1e-9:
        path_theta = rtheta  # degenerate; use robot heading
    else:
        tx, ty = tx / tnorm, ty / tnorm
        path_theta = math.atan2(ty, tx)

    # signed lateral distance: cross product (tangent × (robot - path))_z
    e_signed = tx * (ry - py[idx]) - ty * (rx - px[idx])

    # heading error wrapped to [-π, π]
    e_theta = math.atan2(
        math.sin(rtheta - path_theta),
        math.cos(rtheta - path_theta),
    )

    # curvature via 3-point finite difference (Menger curvature, signed)
    # cyclic → use wrap-around neighbours at endpoints
    triplet = None
    if cyclic and N >= 3:
        triplet = ((idx - 1) % N, idx, (idx + 1) % N)
    elif 0 < idx < N - 1:
        triplet = (idx - 1, idx, idx + 1)
    if triplet is not None:
        i_a, i_b, i_c = triplet
        ax, ay = px[i_a], py[i_a]
        bx, by = px[i_b], py[i_b]
        cx, cy = px[i_c], py[i_c]
        area2 = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
        ab = math.hypot(bx - ax, by - ay)
        bc = math.hypot(cx - bx, cy - by)
        ac = math.hypot(cx - ax, cy - ay)
        denom = ab * bc * ac
        kappa_ref = 0.0 if denom < 1e-9 else 2.0 * area2 / denom
    else:
        kappa_ref = 0.0

    return idx, float(e_signed), float(e_theta), float(kappa_ref)
