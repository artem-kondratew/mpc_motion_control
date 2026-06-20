#!/usr/bin/env python3

import os
from pathlib import Path as FilePath

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import TransformException

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path

from .submodules.frenet import (
    compute_kappa_profile_along_horizon,
    frenet_project,
    menger_curvature,
)
from .submodules.swarm import Agent
from .submodules.swarm_cc_mpc import SwarmCruiseController
from .submodules.swarm_lat_mpc import SwarmLatController


def _resolve_trajectory_path(path_str: str) -> FilePath:
    """Resolve a trajectory yaml path.

    Absolute paths are used as-is. Relative paths and bare filenames are
    resolved against `swarm_controller/config/` — the package's config
    directory and where `trajectory_drawer` saves by default.

    The lookup follows the symlink chain `install → build → src` for a
    known existing file, so trajectories saved by `trajectory_drawer`
    into the source tree are visible immediately, without needing a
    fresh `colcon build`.
    """
    p = FilePath(path_str).expanduser()
    if p.is_absolute():
        return p
    share_config = FilePath(
        get_package_share_directory('swarm_controller')) / 'config'
    # Resolve to source config dir via realpath of a known stable file.
    for known in ('params_pacemaker.yaml', 'params_simulator.yaml',
                  'logging_topics.yaml'):
        anchor = share_config / known
        if anchor.exists():
            return FilePath(os.path.realpath(anchor)).parent / p
    return share_config / p  # fallback (no known anchor)


def _load_trajectory_file(path_str: str):
    """Resolve `trajectory_file` and read trajectory + waypoints from it.

    Returns (trajectory, waypoints_x, waypoints_y, resolved_path) — any
    of the first three may be None if the file does not specify them.
    Raises FileNotFoundError if the path does not exist.
    """
    p = _resolve_trajectory_path(path_str)
    if not p.exists():
        raise FileNotFoundError(f'trajectory_file not found: {p}')
    with open(p) as f:
        data = yaml.safe_load(f) or {}
    for top_val in data.values():
        if isinstance(top_val, dict) and 'ros__parameters' in top_val:
            params = top_val['ros__parameters']
            return (
                params.get('trajectory'),
                params.get('waypoints_x'),
                params.get('waypoints_y'),
                p,
            )
    raise ValueError(
        f'{p}: no /**: ros__parameters block found')


class PacemakerController(Node):

    def __init__(self) -> None:
        super().__init__('pacemaker_controller')

        self.declare_parameters(namespace='', parameters=[
            ('cmd_vel_topic', ''),
            ('pacemaker_idx', 0),
            ('start', False),
            ('trajectory', 'straight'),
            ('linear_vel', 0.3),
            ('circle_linear_vel', 0.3),
            ('circle_radius', 2.0),
            ('waypoints_x', [0.0]),
            ('waypoints_y', [0.0]),
            ('lookahead_dist', 1.0),
            ('trajectory_file', ''),
            # Lateral controller for `lanelet` mode:
            #   'pure_pursuit' (default) — geometric, classic
            #   'mpc'                    — same SwarmLatController as
            #                              followers; needs lat MPC params
            ('lateral_controller', 'pure_pursuit'),
            # Lateral MPC parameters (used only when lateral_controller='mpc').
            # Defaults match params_swarm_lat.yaml — override at launch if
            # different tuning is needed for the leader vs followers.
            ('lat_tau_w', 0.03),
            ('lat_ts', 0.05),
            ('lat_p', 20),
            ('lat_c', 10),
            ('lat_s', 5.0),
            ('lat_phi_vals', [0.6, 0.95, 0.6]),
            ('lat_q_vals',   [10.0, 1.0, 0.0]),
            ('lat_alpha_min', -2.0),
            ('lat_alpha_max',  2.0),
            ('lat_w_cmd_min', -1.5),
            ('lat_w_cmd_max',  1.5),
            ('odom_topic', ''),
            # Longitudinal controller for pacemaker:
            #   'constant' (default) — open-loop linear_vel
            #   'mpc'                — kinematic CC MPC tracks v_ref = linear_vel
            ('longitudinal_controller', 'constant'),
            # Kinematic CC MPC parameters (adas-exact, plant = accel lag tau).
            # Output is 3-D: [v - v_ref, a, j]. Control is a_cmd; v_cmd is
            # integrated outside the controller (see callback).
            ('cc_tau', 0.15),                  # acceleration time constant [s]
            ('cc_ts', 0.05),
            ('cc_p', 40),
            ('cc_c', 30),
            ('cc_s', 20.0),
            ('cc_phi_vals', [0.6, 0.6, 0.6]),  # reference shaping per output
            ('cc_q_vals',   [10.0, 1.0, 0.0]), # [q_v_err, q_a, q_j]
            ('cc_a_min', -0.5),
            ('cc_a_max',  0.5),
            ('cc_v_cmd_min', 0.0),             # external integrator clip
            ('cc_v_cmd_max', 0.5),
        ])

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.pacemaker_idx = self.get_parameter('pacemaker_idx').value
        self.trajectory = self.get_parameter('trajectory').value
        self.linear_vel = self.get_parameter('linear_vel').value
        self.circle_linear_vel = self.get_parameter('circle_linear_vel').value
        self.circle_radius = self.get_parameter('circle_radius').value

        # If `trajectory_file` is set, load trajectory + waypoints from it and
        # override the values declared above.  This is the path written by
        # `trajectory_drawer` and the standard way to pick a custom route.
        traj_file_param = str(
            self.get_parameter('trajectory_file').value).strip()
        wx_override = wy_override = None
        if traj_file_param:
            traj, wx_override, wy_override, resolved = _load_trajectory_file(
                traj_file_param)
            if traj is not None:
                self.trajectory = traj
            self.get_logger().info(
                f'trajectory_file: {resolved} (trajectory={self.trajectory}, '
                f'{len(wx_override) if wx_override else 0} waypoints)')

        self.get_logger().info(f'cmd_vel_topic: {cmd_vel_topic}')
        self.get_logger().info(f'trajectory: {self.trajectory}')

        # lateral controller selection (only relevant in lanelet mode)
        self.lateral_controller = str(
            self.get_parameter('lateral_controller').value).strip()
        self.lat_mpc: SwarmLatController | None = None
        self._last_w_measured = 0.0  # latest yaw rate from /robot{idx}/odom
        self._last_v_measured = 0.0  # latest longitudinal v from same odom
        self._prev_path_idx: int | None = None  # for windowed Frenet projection

        # longitudinal controller: 'constant' (open-loop) or 'mpc' (kinematic CC).
        self.longitudinal_controller = str(
            self.get_parameter('longitudinal_controller').value).strip()
        self.cc_mpc: SwarmCruiseController | None = None
        # External v_cmd integrator state (kinematic MPC returns a_cmd; we
        # integrate to v_cmd here and clip to [cc_v_cmd_min, cc_v_cmd_max]).
        self._cc_v_cmd_published = 0.0
        self._cc_v_cmd_min = float(self.get_parameter('cc_v_cmd_min').value)
        self._cc_v_cmd_max = float(self.get_parameter('cc_v_cmd_max').value)
        if self.longitudinal_controller == 'mpc':
            self.cc_mpc = SwarmCruiseController(
                tau=float(self.get_parameter('cc_tau').value),
                ts=float(self.get_parameter('cc_ts').value),
                p=int(self.get_parameter('cc_p').value),
                c=int(self.get_parameter('cc_c').value),
                s=float(self.get_parameter('cc_s').value),
                phi_vals=list(self.get_parameter('cc_phi_vals').value),
                q_vals=list(self.get_parameter('cc_q_vals').value),
                u_limits=(
                    float(self.get_parameter('cc_a_min').value),
                    float(self.get_parameter('cc_a_max').value),
                ),
            )
            self.get_logger().info(
                f'Longitudinal controller: kinematic MPC '
                f'(tracking v_ref={self.linear_vel} m/s)')
        else:
            self.get_logger().info(
                f'Longitudinal controller: constant ({self.linear_vel} m/s)')

        # Subscribe to own odom if any MPC controller is active (lat OR long
        # need v / w from /robot{idx}/odom).
        if self.cc_mpc is not None or self.lateral_controller == 'mpc':
            odom_topic = str(self.get_parameter('odom_topic').value).strip()
            if not odom_topic:
                odom_topic = f'/robot{self.pacemaker_idx}/odom'
            self.create_subscription(
                Odometry, odom_topic, self._odom_cb, 10)
            self.get_logger().info(f'Subscribed to odom: {odom_topic}')

        if self.trajectory == 'lanelet':
            if wx_override is not None and wy_override is not None:
                wx = list(wx_override)
                wy = list(wy_override)
            else:
                wx = list(self.get_parameter('waypoints_x').value)
                wy = list(self.get_parameter('waypoints_y').value)
            self.waypoints = np.array(list(zip(wx, wy)), dtype=float)
            self.lookahead_dist = self.get_parameter('lookahead_dist').value
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            self.path_pub = self.create_publisher(Path, '/pacemaker/path', 10)
            # Trajectory is "anchored" on the first valid pose: shifted AND
            # rotated so that (a) the first waypoint coincides with the
            # pacemaker's current (x, y) and (b) the initial tangent
            # (direction from wp[0] to wp[1]) coincides with the pacemaker's
            # current heading. So a trajectory drawn around the origin
            # facing +x in `trajectory_drawer` automatically picks up
            # the pacemaker's spawn pose, no manual offsetting needed.
            self._waypoints_anchored = False
            # κ-profile inputs — precomputed once on first anchored cycle.
            # Curvature is rigid-invariant, so we could compute before the
            # anchor too, but cyclic-detection uses the actual published
            # geometry which is post-anchor.
            self._path_kappa: np.ndarray | None = None
            self._path_mean_spacing: float = 0.0
            self._path_cyclic: bool = False

            # ── lateral controller setup ────────────────────────────────────
            if self.lateral_controller == 'mpc':
                self.lat_mpc = SwarmLatController(
                    tau_w=float(self.get_parameter('lat_tau_w').value),
                    ts=float(self.get_parameter('lat_ts').value),
                    p=int(self.get_parameter('lat_p').value),
                    c=int(self.get_parameter('lat_c').value),
                    s=float(self.get_parameter('lat_s').value),
                    phi_vals=list(self.get_parameter('lat_phi_vals').value),
                    q_vals=list(self.get_parameter('lat_q_vals').value),
                    alpha_limits=(
                        float(self.get_parameter('lat_alpha_min').value),
                        float(self.get_parameter('lat_alpha_max').value),
                    ),
                    w_cmd_limits=(
                        float(self.get_parameter('lat_w_cmd_min').value),
                        float(self.get_parameter('lat_w_cmd_max').value),
                    ),
                )
                self.get_logger().info('Lateral controller: MPC')
            else:
                self.get_logger().info('Lateral controller: pure_pursuit')

            self.get_logger().info(
                f'Loaded {len(self.waypoints)} waypoints, '
                f'lookahead={self.lookahead_dist} m '
                f'(start will be anchored to pacemaker pose)'
            )
        else:
            self.get_logger().info(f'linear_vel: {self.linear_vel}')
            self.get_logger().info(f'circle_linear_vel: {self.circle_linear_vel}')
            self.get_logger().info(f'circle_radius: {self.circle_radius}')

        # Tracks `start` parameter for false->true edge detection (resets MPC
        # integrators so the published v_cmd does not jump when the kill-switch
        # is flipped after long idle).
        self._was_started = False

        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        # Path publisher always created; followers' `_path_cb` requires this
        # topic to be present even in `straight` mode (otherwise their
        # callback exits early on `self._path_x is None`).  In `straight`
        # mode we synthesise a long forward path along x-axis below.
        if not hasattr(self, 'path_pub'):
            self.path_pub = self.create_publisher(Path, '/pacemaker/path', 10)
        # Straight-mode synthetic path: long line along the +x axis in map
        # frame.  Assumes pacemaker init at (0, 0) heading 0; if your init
        # differs, override init_* in params_simulator.yaml accordingly.
        if self.trajectory == 'straight':
            n_pts = 100
            xs = np.linspace(-5.0, 50.0, n_pts)
            ys = np.zeros(n_pts)
            self.waypoints = np.column_stack([xs, ys])

        self.agent = Agent(self.pacemaker_idx)
        # Timer period matches MPC's `ts` so the integrator states (v_cmd in
        # CC MPC, w_cmd in lat MPC) advance at the rate the MPC expects.
        # Legacy pacemaker (pure-pursuit + constant v) ran at 10 Hz; that's
        # fine for those, but MPCs configured for ts=0.05 must be called at
        # 20 Hz or they double-step their internal integrator.
        if self.cc_mpc is not None or self.lat_mpc is not None:
            self.timer_period = float(self.get_parameter('cc_ts').value)
        else:
            self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.callback)
        self.get_logger().info(
            f'Callback period: {self.timer_period} s ({1/self.timer_period:.0f} Hz)')

    def _odom_cb(self, msg: Odometry) -> None:
        """Cache yaw rate (for lateral MPC) and longitudinal v (for CC MPC)."""
        self._last_w_measured = float(msg.twist.twist.angular.z)
        self._last_v_measured = float(msg.twist.twist.linear.x)

    def get_time(self):
        seconds, nanoseconds = self.get_clock().now().seconds_nanoseconds()
        return seconds + nanoseconds * 1e-9

    def _get_pose_from_tf(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'map',
                f'robot{self.pacemaker_idx}/base_footprint',
                rclpy.time.Time(),
            )
        except TransformException as e:
            self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=2.0)
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        theta = np.arctan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return float(t.x), float(t.y), float(theta)

    def _pure_pursuit(self, x, y, theta, v):
        pos = np.array([x, y])
        dists = np.linalg.norm(self.waypoints - pos, axis=1)
        closest_idx = int(np.argmin(dists))

        lookahead = None
        N = len(self.waypoints)
        for i in range(1, N + 1):
            idx = (closest_idx + i) % N
            if np.linalg.norm(self.waypoints[idx] - pos) >= self.lookahead_dist:
                lookahead = self.waypoints[idx]
                break

        if lookahead is None:
            lookahead = self.waypoints[closest_idx]

        dx = lookahead[0] - x
        dy = lookahead[1] - y
        alpha = np.arctan2(dy, dx) - theta
        alpha = np.arctan2(np.sin(alpha), np.cos(alpha))

        return 2.0 * v * np.sin(alpha) / self.lookahead_dist

    def _publish_path(self):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for wp in self.waypoints:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(wp[0])
            ps.pose.position.y = float(wp[1])
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.path_pub.publish(msg)

    def callback(self):
        t = self.get_time()

        # Re-read velocity targets so they can be changed at runtime via
        # `ros2 param set /pacemaker_controller linear_vel <value>` while
        # the experiment is running.
        self.linear_vel = float(self.get_parameter('linear_vel').value)
        self.circle_linear_vel = float(self.get_parameter('circle_linear_vel').value)

        # Detect `start` false -> true edge: reset MPC state so the integrator
        # does not jump out from whatever it accumulated while the kill-switch
        # was off. Without this, robot lurches with whatever v_cmd was reached.
        started = bool(self.get_parameter('start').value)
        if started and not self._was_started:
            if self.cc_mpc is not None:
                self.cc_mpc.reset()
                self._cc_v_cmd_published = 0.0
            if self.lat_mpc is not None:
                self.lat_mpc.reset()
            self.get_logger().info('start -> true: MPC state reset')
        self._was_started = started

        if self.trajectory == 'straight':
            self.agent.v_ref = self.linear_vel
            self.agent.w_ref = 0.0
            # Publish synthetic straight path so followers' lat-MPC has a
            # frenet reference (otherwise their callback bails on no path).
            self._publish_path()
        elif self.trajectory == 'circle':
            self.agent.v_ref = self.circle_linear_vel
            self.agent.w_ref = self.circle_linear_vel / self.circle_radius
        elif self.trajectory == 'lanelet':
            pose = self._get_pose_from_tf()
            if pose is None:
                return
            x, y, theta = pose
            if not self._waypoints_anchored:
                wp0 = self.waypoints[0].copy()
                # Trajectory's initial heading = direction of first segment.
                if len(self.waypoints) >= 2:
                    seg = self.waypoints[1] - self.waypoints[0]
                    traj_theta = float(np.arctan2(seg[1], seg[0]))
                else:
                    traj_theta = 0.0
                dtheta = theta - traj_theta
                c, s = float(np.cos(dtheta)), float(np.sin(dtheta))
                R_T = np.array([[c, s], [-s, c]])  # row-vector rotation
                # Translate to origin, rotate, then translate to pacemaker pose.
                relative = self.waypoints - wp0
                self.waypoints = relative @ R_T + np.array([x, y])
                self._waypoints_anchored = True
                self.get_logger().info(
                    f'Trajectory anchored: shift=({x - wp0[0]:+.2f}, '
                    f'{y - wp0[1]:+.2f}) m, rotate={np.degrees(dtheta):+.1f}° '
                    f'→ start=({self.waypoints[0, 0]:.2f}, '
                    f'{self.waypoints[0, 1]:.2f}, '
                    f'θ={np.degrees(theta):+.1f}°)'
                )
            # κ-profile inputs (precompute once after anchoring).
            if self._path_kappa is None:
                diffs = np.diff(self.waypoints, axis=0)
                self._path_mean_spacing = (
                    float(np.mean(np.linalg.norm(diffs, axis=1)))
                    if len(diffs) > 0 else 0.0
                )
                endpoint_dist = float(np.linalg.norm(
                    self.waypoints[0] - self.waypoints[-1]))
                self._path_cyclic = (
                    self._path_mean_spacing > 0.0
                    and endpoint_dist < 2.0 * self._path_mean_spacing
                )
                self._path_kappa = menger_curvature(
                    self.waypoints, cyclic=self._path_cyclic)
                self.get_logger().info(
                    f'κ array: |κ|_max={np.abs(self._path_kappa).max():.3f}, '
                    f'mean_spacing={self._path_mean_spacing:.3f} m, '
                    f'cyclic={self._path_cyclic}'
                )

            # Frenet projection (windowed around prev idx to avoid jumps on
            # self-intersecting paths).  Used by lat-MPC for κ-profile.
            px = self.waypoints[:, 0]
            py = self.waypoints[:, 1]
            idx_near, e, e_theta, kappa_ref = frenet_project(
                px, py, x, y, theta,
                prev_idx=self._prev_path_idx, search_window=20,
            )
            self._prev_path_idx = idx_near

            # ── longitudinal: constant or kinematic CC MPC ─────────────────
            if self.cc_mpc is not None:
                a_cmd, _y = self.cc_mpc.calculate_control(
                    v_ref=self.linear_vel, v=self._last_v_measured,
                )
                # external integrator: a_cmd → v_cmd, clipped to bounds
                self._cc_v_cmd_published = float(np.clip(
                    self._cc_v_cmd_published + self.cc_mpc.ts * a_cmd,
                    self._cc_v_cmd_min, self._cc_v_cmd_max,
                ))
                self.agent.v_ref = self._cc_v_cmd_published
            else:
                self.agent.v_ref = self.linear_vel

            # ── lateral: pure pursuit or lat MPC ───────────────────────────
            if self.lat_mpc is not None:
                # Use the COMMANDED v (not measured) for the LTV linearisation:
                # measured v lags by Coulomb stiction + force lag, leaving lat
                # MPC blind during startup transients. Commanded v leads.
                if self.cc_mpc is not None:
                    v_for_lat = self._cc_v_cmd_published
                else:
                    v_for_lat = self.linear_vel
                kappa_profile = compute_kappa_profile_along_horizon(
                    idx_now=idx_near, v=v_for_lat,
                    ts=self.lat_mpc.ts, horizon=self.lat_mpc.p,
                    kappa_arr=self._path_kappa,
                    mean_spacing=self._path_mean_spacing,
                    cyclic=self._path_cyclic,
                )
                w_cmd, _alpha, _y = self.lat_mpc.calculate_control(
                    e=e, e_theta=e_theta, w=self._last_w_measured,
                    v=v_for_lat, kappa_profile=kappa_profile,
                )
                self.agent.w_ref = w_cmd
            else:
                self.agent.w_ref = self._pure_pursuit(
                    x, y, theta, self.linear_vel)
            self._publish_path()
        else:
            self.get_logger().warn(
                f'unknown trajectory: {self.trajectory}',
                throttle_duration_sec=5.0,
            )
            self.agent.v_ref = 0.0
            self.agent.w_ref = 0.0

        twist = Twist()
        twist.linear.x = self.agent.v_ref
        twist.angular.z = self.agent.w_ref

        if self.get_parameter('start').value:
            self.cmd_vel_pub.publish(twist)

        self.get_logger().info(f't={t:.2f} v={twist.linear.x:.3f} w={twist.angular.z:.3f}')


def main(args=None):
    rclpy.init(args=args)
    node = PacemakerController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
