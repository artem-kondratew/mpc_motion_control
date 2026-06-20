#!/usr/bin/env python3
"""Lateral MPC node — удержание в полосе (path-following) на /pacemaker/path.

Цепочка (этап 2, лидер):
    /control/long_cmd (Twist)            ─┐  v = linear.x (команда CC) + выход linear.x
    /pacemaker/path   (Path)             ─┼─► swarm_lat_mpc_node ─► /cmd_vel
    /lio_sam/mapping/odometry (Odom)     ─┤      (глобальная поза x,y,θ)
    /odom (kobuki, Odom)                 ─┘      (измеренная угловая w)

Латеральный контроллер — чистая надстройка над продольным: берёт linear.x из
long_cmd как есть, переопределяет angular.z выходом lat MPC. Продольный (CC) не меняется.

РАЗВЯЗКА источников (turtlebot):
- поза (rx,ry,rtheta) — из `pose_topic` (lio_sam, глобальная, без дрейфа);
- угловая w — из `odom_topic` (kobuki, гироскоп);
- продольная v (рабочая точка LTV и выход) — из `long_cmd.linear.x`.

Диагностика (Float32/Float64 под /control) для графиков: e_lat, e_theta, w_cmd, w_meas.

Gating: нет своего `start` — публикует только при свежем long_cmd. Когда CC (его `start`)
перестаёт публиковать long_cmd, по telemetry_timeout выдаём нулевой Twist.
"""

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64

from .submodules.frenet import (
    compute_kappa_profile_along_horizon,
    frenet_project,
    menger_curvature,
    yaw_from_quat,
)
from .submodules.swarm_lat_mpc import SwarmLatController


class SwarmLatMpcNode(Node):

    def __init__(self) -> None:
        super().__init__('swarm_lat_mpc_node')

        self.declare_parameters(namespace='', parameters=[
            # topics
            ('long_cmd_topic',       '/control/long_cmd'),   # выход CC (v)
            ('cmd_vel_topic',        '/cmd_vel'),            # итоговая команда
            ('pacemaker_path_topic', '/pacemaker/path'),     # полоса
            ('pose_topic',           '/lio_sam/mapping/odometry'),  # глобальная поза (lio_sam)
            ('odom_topic',           '/odom'),               # измеренная w (kobuki)
            ('robot_id', -1),
            # plant
            ('tau_w', 0.03),
            # MPC tuning (n_out=3: [e, e_θ, α])
            ('ts', 0.05),
            ('p', 20),
            ('c', 10),
            ('s', 5.0),
            ('phi_vals', [0.6, 0.95, 0.6, 0.9]),
            ('q_vals',   [10.0, 1.0, 0.0, 2.0]),
            # constraints
            ('alpha_min', -2.0),
            ('alpha_max',  2.0),
            ('w_cmd_min', -1.5),
            ('w_cmd_max',  1.5),
            ('e_int_limit', 2.0),
            # curve slowdown: предел скорости по кривизне -> публикуем на /control/v_curve,
            # CC берёт min(v_ref, v_curve). Включается параметром curve_slowdown в cc_mpc.param.yaml.
            ('a_lat_max', 0.1),         # макс боковое ускорение [m/s²] — ГЛАВНЫЙ knob (меньше -> сильнее тормозит)
            ('v_min', 0.1),            # нижний предел скорости в повороте [m/s]
            ('curve_lookahead', 1.0),  # на сколько метров вперёд берём max|κ| (тормозить ДО входа в поворот)
            ('v_curve_cap', 1.0),      # верхний кап: на прямой κ≈0 -> v_curve=cap -> CC использует свой v_ref
            # safety
            ('telemetry_timeout', 0.5),
        ])

        self.long_cmd_topic       = self.get_parameter('long_cmd_topic').value
        self.cmd_vel_topic        = self.get_parameter('cmd_vel_topic').value
        self.pacemaker_path_topic = self.get_parameter('pacemaker_path_topic').value
        self.pose_topic           = self.get_parameter('pose_topic').value
        self.odom_topic           = self.get_parameter('odom_topic').value
        self.robot_id             = self.get_parameter('robot_id').value

        tau_w = self.get_parameter('tau_w').value
        ts = self.get_parameter('ts').value
        p  = self.get_parameter('p').value
        c  = self.get_parameter('c').value
        s  = self.get_parameter('s').value
        phi_vals = list(self.get_parameter('phi_vals').value)
        q_vals   = list(self.get_parameter('q_vals').value)

        alpha_min  = self.get_parameter('alpha_min').value
        alpha_max  = self.get_parameter('alpha_max').value
        w_cmd_min  = self.get_parameter('w_cmd_min').value
        w_cmd_max  = self.get_parameter('w_cmd_max').value
        e_int_limit = self.get_parameter('e_int_limit').value

        # curve slowdown
        self.a_lat_max = float(self.get_parameter('a_lat_max').value)
        self.v_min = float(self.get_parameter('v_min').value)
        self.curve_lookahead = float(self.get_parameter('curve_lookahead').value)
        self.v_curve_cap = float(self.get_parameter('v_curve_cap').value)

        self.telemetry_timeout = float(self.get_parameter('telemetry_timeout').value)
        self.ts = float(ts)

        self.get_logger().info(f'[swarm_lat_mpc] lateral / lane-keeping (этап 2)')
        self.get_logger().info(f'  long_cmd_topic: {self.long_cmd_topic}')
        self.get_logger().info(f'  cmd_vel_topic:  {self.cmd_vel_topic}')
        self.get_logger().info(f'  path_topic:     {self.pacemaker_path_topic}')
        self.get_logger().info(f'  pose_topic:     {self.pose_topic} (lio_sam)')
        self.get_logger().info(f'  odom_topic:     {self.odom_topic} (kobuki w)')
        self.get_logger().info(f'  plant: tau_w={tau_w}; MPC: ts={ts}, p={p}, c={c}, s={s}')
        self.get_logger().info(f'  q_vals={q_vals}, phi_vals={phi_vals}')
        self.get_logger().info(
            f'  α_cmd in [{alpha_min}, {alpha_max}], w_cmd in [{w_cmd_min}, {w_cmd_max}]')
        self.get_logger().info(
            f'  v_curve(кривизна): a_lat_max={self.a_lat_max}, v_min={self.v_min}, '
            f'lookahead={self.curve_lookahead}м, cap={self.v_curve_cap}')

        self.controller = SwarmLatController(
            tau_w=tau_w,
            ts=ts, p=p, c=c, s=s,
            phi_vals=phi_vals, q_vals=q_vals,
            alpha_limits=(alpha_min, alpha_max),
            w_cmd_limits=(w_cmd_min, w_cmd_max),
            e_int_limit=e_int_limit,
        )

        # state
        self.last_pose: Odometry | None = None    # глобальная поза (lio_sam)
        self.last_odom: Odometry | None = None    # для w (kobuki)
        self.last_long_cmd: Twist | None = None
        self.last_long_cmd_stamp = None
        self._long_cmd_was_fresh = False
        self._prev_path_idx: int | None = None
        self.last_path: Path | None = None
        self._path_x: np.ndarray | None = None
        self._path_y: np.ndarray | None = None
        self._path_kappa: np.ndarray | None = None
        self._path_mean_spacing: float = 0.0
        self._path_cyclic: bool = False

        self._dbg_period = max(1, int(round(1.0 / ts)))
        self._dbg_counter = 0

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        # диагностика для графиков (относительные имена -> под /control)
        self.elat_pub   = self.create_publisher(Float64, 'e_lat',   10)
        self.etheta_pub = self.create_publisher(Float64, 'e_theta', 10)
        self.wcmd_pub   = self.create_publisher(Float64, 'w_cmd',   10)
        self.wmeas_pub  = self.create_publisher(Float64, 'w_meas',  10)
        # предел скорости по кривизне (lookahead) -> CC берёт min(v_ref, v_curve)
        self.vcurve_pub = self.create_publisher(Float64, 'v_curve', 10)

        # lio_sam публикует odometry с BEST_EFFORT QoS — подписка должна совпадать
        be_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(Twist, self.long_cmd_topic, self._long_cmd_cb, 10)
        self.create_subscription(Path, self.pacemaker_path_topic, self._path_cb, 10)
        self.create_subscription(Odometry, self.pose_topic, self._pose_cb, be_qos)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 10)

        self.create_timer(ts, self._control_step)
        self.get_logger().info('[swarm_lat_mpc] ready, waiting for inputs...')

    # ── callbacks ────────────────────────────────────────────────────────────

    def _long_cmd_cb(self, msg: Twist) -> None:
        self.last_long_cmd = msg
        self.last_long_cmd_stamp = self.get_clock().now()

    def _pose_cb(self, msg: Odometry) -> None:
        self.last_pose = msg

    def _odom_cb(self, msg: Odometry) -> None:
        self.last_odom = msg

    def _path_cb(self, msg: Path) -> None:
        if not msg.poses:
            return
        new_x = np.array([p.pose.position.x for p in msg.poses], dtype=float)
        new_y = np.array([p.pose.position.y for p in msg.poses], dtype=float)
        if self._path_x is None or len(new_x) != len(self._path_x):
            self._prev_path_idx = None
        geom_changed = (
            self._path_x is None
            or len(new_x) != len(self._path_x)
            or not (np.array_equal(new_x, self._path_x)
                    and np.array_equal(new_y, self._path_y))
        )
        self._path_x, self._path_y = new_x, new_y
        if geom_changed:
            wpts = np.column_stack([new_x, new_y])
            diffs = np.diff(wpts, axis=0)
            self._path_mean_spacing = (
                float(np.mean(np.linalg.norm(diffs, axis=1))) if len(diffs) > 0 else 0.0)
            endpoint_dist = float(np.hypot(new_x[0] - new_x[-1], new_y[0] - new_y[-1]))
            self._path_cyclic = (
                self._path_mean_spacing > 0.0
                and endpoint_dist < 2.0 * self._path_mean_spacing)
            self._path_kappa = menger_curvature(wpts, cyclic=self._path_cyclic)
        self.last_path = msg

    # ── control loop ─────────────────────────────────────────────────────────

    def _publish_diag(self, e: float, e_theta: float, w_cmd: float, w_meas: float) -> None:
        self.elat_pub.publish(Float64(data=float(e)))
        self.etheta_pub.publish(Float64(data=float(e_theta)))
        self.wcmd_pub.publish(Float64(data=float(w_cmd)))
        self.wmeas_pub.publish(Float64(data=float(w_meas)))

    def _control_step(self) -> None:
        # preflight: нужна глобальная поза, путь и свежий long_cmd
        if self.last_pose is None:
            return
        if self._path_x is None or self._path_y is None:
            self.get_logger().warn('[swarm_lat_mpc] no path yet, idle',
                                   throttle_duration_sec=2.0)
            return
        if self.last_long_cmd is None or self.last_long_cmd_stamp is None:
            return

        age = (self.get_clock().now() - self.last_long_cmd_stamp).nanoseconds * 1e-9
        if age > self.telemetry_timeout:
            self.get_logger().warn(f'[swarm_lat_mpc] long_cmd stale ({age:.2f}s), stopping',
                                   throttle_duration_sec=1.0)
            self._stop_robot()
            self.controller.reset()
            self._long_cmd_was_fresh = False
            return

        if not self._long_cmd_was_fresh:
            self.controller.reset()
            self._long_cmd_was_fresh = True
            self.get_logger().info('[swarm_lat_mpc] long_cmd fresh, controller reset')

        # ── inputs ──────────────────────────────────────────────────────────
        pose = self.last_pose
        rx = float(pose.pose.pose.position.x)
        ry = float(pose.pose.pose.position.y)
        q = pose.pose.pose.orientation
        rtheta = yaw_from_quat(q.x, q.y, q.z, q.w)

        v = float(self.last_long_cmd.linear.x)             # рабочая точка / выход
        w = (float(self.last_odom.twist.twist.angular.z)   # измеренная угловая (kobuki)
             if self.last_odom is not None else 0.0)

        idx, e, e_theta, kappa_ref = frenet_project(
            self._path_x, self._path_y, rx, ry, rtheta,
            prev_idx=self._prev_path_idx, search_window=20)
        self._prev_path_idx = idx

        # --- предел скорости по кривизне (lookahead): v_curve = sqrt(a_lat_max / max|κ| впереди) ---
        # публикуем всегда; CC применяет min(v_ref, v_curve) при curve_slowdown=true. На прямой -> cap.
        if self._path_kappa is not None and len(self._path_kappa):
            n = len(self._path_kappa)
            n_ahead = max(1, int(round(self.curve_lookahead / max(self._path_mean_spacing, 1e-3))))
            if self._path_cyclic:
                look = np.array([self._path_kappa[(idx + j) % n] for j in range(n_ahead)])
            else:
                look = self._path_kappa[idx:min(idx + n_ahead, n)]
            kmax = float(np.max(np.abs(look))) if len(look) else 0.0
            v_curve = np.sqrt(self.a_lat_max / kmax) if kmax > 1e-4 else self.v_curve_cap
            v_curve = float(np.clip(v_curve, self.v_min, self.v_curve_cap))
            self.vcurve_pub.publish(Float64(data=v_curve))

        kappa_profile = compute_kappa_profile_along_horizon(
            idx_now=idx, v=v,
            ts=self.controller.ts, horizon=self.controller.p,
            kappa_arr=self._path_kappa,
            mean_spacing=self._path_mean_spacing,
            cyclic=self._path_cyclic)

        w_cmd, alpha_cmd, y = self.controller.calculate_control(
            e=e, e_theta=e_theta, w=w, v=v, kappa_profile=kappa_profile)

        out = Twist()
        out.linear.x = v
        out.angular.z = float(w_cmd)
        self.cmd_pub.publish(out)

        self._publish_diag(e, e_theta, w_cmd, w)

        self._dbg_counter += 1
        if self._dbg_counter >= self._dbg_period:
            self._dbg_counter = 0
            self.get_logger().info(
                f'idx={idx:3d}  e={e:+.3f} e_θ={e_theta:+.3f} κ={kappa_ref:+.3f}  '
                f'v={v:.2f} w={w:+.3f}  α={alpha_cmd:+.3f} w_cmd={w_cmd:+.3f}')

    def _stop_robot(self) -> None:
        self.cmd_pub.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SwarmLatMpcNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
