#!/usr/bin/env python3
"""Kinematic ACC MPC follower node (adas-style).

Subscribes to Telemetry, runs longitudinal kinematic MPC on a fixed timer,
publishes Twist. Acceleration command from MPC is integrated in-node into
v_cmd (strategy "B2") — this gives a smooth published velocity, analogous to
how the sliding-mode controller integrates `agent.v_ref`.
"""

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from swarm_msgs.msg import Telemetry

from .submodules.swarm_acc_mpc import SwarmAccController


class SwarmAccMpcNode(Node):

    def __init__(self) -> None:
        super().__init__('swarm_acc_mpc_node')

        self.declare_parameters(namespace='', parameters=[
            # topics / id
            ('telemetry_topic', '/swarm_controller/telemetry'),
            ('cmd_vel_topic',   '/cmd_vel'),
            ('robot_id', -1),
            # plant (kinematic)
            ('tau', 0.2),
            # gap policy
            ('d0', 0.5),
            ('th', 0.0),
            # явный интеграл по зазору (убирает droop MPC) с анти-windup:
            # v_int += ki_gap·(dx−dx_ref)·ts, ограничен ±gap_int_max, замораживается
            # при насыщении скорости (упор в кривую/v_max) — не копит и не залипает.
            ('ki_gap', 0.5),
            ('gap_int_max', 0.3),
            # MPC tuning
            ('ts', 0.05),
            ('p', 20),
            ('c', 10),
            ('s', 3.0),
            # adas-style: 4 outputs [y_gap, v_rel, a, j]
            ('phi_vals', [0.6, 0.95, 0.6, 0.6]),
            ('q_vals',   [10.0, 1.0, 1.0, 1.0]),
            # constraints
            ('a_min', -0.5),
            ('a_max',  0.5),
            ('v_cmd_min', 0.0),
            ('v_cmd_max', 0.5),
            ('gap_safe',  0.2),
            # angular
            ('kp_theta', 0.3),
            # curve slowdown: кап v_cmd по пределу кривизны от lat-ноды (как у CC).
            # На повороте ведомый не разгоняется закрывать зазор, пока он на дуге.
            ('curve_slowdown', True),
            ('v_curve_topic', 'v_curve'),     # относительный -> /<vehicle_id>/control/v_curve
            ('v_curve_timeout', 0.5),
            # headroom ведомого на кривой: кап = v_curve · scale. >1 даёт право
            # обогнать лидера и ЗАКРЫТЬ зазор на вираже (лидер со своим тугим
            # a_lat_max не трогается). На установившемся зазоре ведомый и так едет
            # со скоростью лидера, кап не бьёт; scale работает только в догоне.
            ('v_curve_scale', 1.0),
            # safety
            ('start', False),
            ('telemetry_timeout', 0.5),
        ])

        self.telemetry_topic = self.get_parameter('telemetry_topic').value
        self.cmd_vel_topic   = self.get_parameter('cmd_vel_topic').value
        self.robot_id        = self.get_parameter('robot_id').value

        tau = self.get_parameter('tau').value
        d0 = self.get_parameter('d0').value
        th = self.get_parameter('th').value
        self.d0 = float(d0)   # для диагностики: целевой зазор = d0 + th·v
        self.th = float(th)
        self.ki_gap = float(self.get_parameter('ki_gap').value)
        self.gap_int_max = float(self.get_parameter('gap_int_max').value)
        self._gap_int = 0.0       # явный интеграл по зазору (анти-windup)
        self._v_cmd_base = 0.0    # B2-аккумулятор a_cmd (отдельно от интеграла)

        ts = self.get_parameter('ts').value
        p  = self.get_parameter('p').value
        c  = self.get_parameter('c').value
        s  = self.get_parameter('s').value
        phi_vals = list(self.get_parameter('phi_vals').value)
        q_vals   = list(self.get_parameter('q_vals').value)

        self.a_min = self.get_parameter('a_min').value
        self.a_max = self.get_parameter('a_max').value
        self.v_cmd_min = self.get_parameter('v_cmd_min').value
        self.v_cmd_max = self.get_parameter('v_cmd_max').value
        gap_safe = self.get_parameter('gap_safe').value

        self.kp_theta = float(self.get_parameter('kp_theta').value)
        self.telemetry_timeout = float(self.get_parameter('telemetry_timeout').value)
        self.ts = float(ts)

        # curve slowdown (кап v_cmd по v_curve от lat-ноды)
        self.curve_slowdown = bool(self.get_parameter('curve_slowdown').value)
        self.v_curve_topic = self.get_parameter('v_curve_topic').value
        self.v_curve_timeout = float(self.get_parameter('v_curve_timeout').value)
        self.v_curve_scale = float(self.get_parameter('v_curve_scale').value)
        self._v_curve = None
        self._v_curve_stamp = None

        self.get_logger().info(f'[swarm_acc_mpc] robot_id={self.robot_id}')
        self.get_logger().info(f'  telemetry_topic: {self.telemetry_topic}')
        self.get_logger().info(f'  cmd_vel_topic:   {self.cmd_vel_topic}')
        self.get_logger().info(f'  plant: tau={tau}  (kinematic, adas-style)')
        self.get_logger().info(f'  gap policy: d0={d0}, th={th}')
        self.get_logger().info(f'  MPC: ts={ts}, p={p}, c={c}, s={s}')
        self.get_logger().info(f'  q_vals={q_vals}, phi_vals={phi_vals}')
        self.get_logger().info(
            f'  a_cmd in [{self.a_min}, {self.a_max}], v_cmd in [{self.v_cmd_min}, {self.v_cmd_max}]')
        self.get_logger().info(f'  gap_safe={gap_safe} (hard constraint)')

        self.controller = SwarmAccController(
            tau=tau,
            d0=d0, th=th,
            ts=ts, p=p, c=c, s=s,
            phi_vals=phi_vals, q_vals=q_vals,
            u_limits=(self.a_min, self.a_max),
            gap_safe=gap_safe,
        )

        self.last_telemetry = None
        self.last_telemetry_stamp = None
        self._was_started = False

        # B2 integrator: smooth v_cmd by integrating a_cmd in-node
        self._v_cmd_published = 0.0

        self._dbg_period = max(1, int(round(1.0 / ts)))
        self._dbg_counter = 0

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        # диагностика для графиков (относительные имена -> под /<id>/control/)
        self.dx_pub     = self.create_publisher(Float64, 'dx',       10)  # зазор до лидера [м]
        self.dxref_pub  = self.create_publisher(Float64, 'dx_ref',   10)  # цель = d0 + th·v
        self.dxerr_pub  = self.create_publisher(Float64, 'dx_error', 10)  # dx - dx_ref
        self.vrel_pub   = self.create_publisher(Float64, 'v_rel',    10)  # скорость относительно лидера
        self.vcmd_pub   = self.create_publisher(Float64, 'v_cmd',    10)  # команда скорости
        self.vmeas_pub  = self.create_publisher(Float64, 'v_meas',   10)  # своя измеренная v
        self.create_subscription(
            Telemetry, self.telemetry_topic, self._telemetry_cb, 10)
        self.create_subscription(
            Float64, self.v_curve_topic, self._v_curve_cb, 10)

        self.create_timer(ts, self._control_step)
        self.get_logger().info('[swarm_acc_mpc] ready, waiting for telemetry...')

    def _telemetry_cb(self, msg: Telemetry) -> None:
        self.last_telemetry = msg
        self.last_telemetry_stamp = self.get_clock().now()

    def _v_curve_cb(self, msg: Float64) -> None:
        self._v_curve = float(msg.data)
        self._v_curve_stamp = self.get_clock().now()

    def _apply_curve_cap(self, v_cmd: float) -> float:
        """Кап v_cmd по пределу кривизны: min(v_cmd, v_curve·scale) при свежем v_curve.
        scale>1 даёт ведомому headroom обогнать лидера и закрыть зазор на вираже."""
        if (self.curve_slowdown and self._v_curve is not None
                and self._v_curve_stamp is not None):
            age = (self.get_clock().now() - self._v_curve_stamp).nanoseconds * 1e-9
            if age <= self.v_curve_timeout:
                return min(v_cmd, self._v_curve * self.v_curve_scale)
        return v_cmd

    def _control_step(self) -> None:
        started = bool(self.get_parameter('start').value)

        if started and not self._was_started:
            self.controller.reset()
            # init B2 from current measured v if telemetry available; интеграл в 0
            self._v_cmd_base = float(self.last_telemetry.v) if self.last_telemetry is not None else 0.0
            self._gap_int = 0.0
            self.get_logger().info('[swarm_acc_mpc] start → true, controller reset')
        self._was_started = started

        if not started:
            return

        if self.last_telemetry_stamp is None:
            return

        age = (self.get_clock().now() - self.last_telemetry_stamp).nanoseconds * 1e-9
        if age > self.telemetry_timeout:
            self.get_logger().warn(
                f'[swarm_acc_mpc] telemetry stale ({age:.2f}s > {self.telemetry_timeout}s), stopping',
                throttle_duration_sec=1.0,
            )
            self._stop_robot()
            self.controller.reset()
            self._v_cmd_base = 0.0
            self._gap_int = 0.0
            return

        msg = self.last_telemetry
        if not msg.is_valid:
            self._stop_robot()
            self.controller.reset()
            self._v_cmd_base = 0.0
            self._gap_int = 0.0
            return

        # ── inputs ──────────────────────────────────────────────────────────
        dx_vec = np.array([msg.peer_x - msg.x, msg.peer_y - msg.y])
        dx = float(np.linalg.norm(dx_vec))
        v = float(msg.v)
        v_rel = float(msg.peer_v - msg.v)

        # ── MPC: returns acceleration command ───────────────────────────────
        a_cmd, y = self.controller.calculate_control(dx=dx, v=v, v_rel=v_rel)

        # ── B2 интегратор a_cmd → базовая скорость (пропорц./предиктивная часть) ──
        self._v_cmd_base += self.ts * a_cmd
        self._v_cmd_base = float(np.clip(
            self._v_cmd_base, self.v_cmd_min, self.v_cmd_max))

        # ── явный интеграл по зазору с анти-windup (conditional integration) ──
        # убирает droop MPC; замораживается, когда скорость насыщена (упор в кривую/
        # v_max) в сторону gap_err -> не копит на перелёте/виражах и не залипает.
        dx_ref = self.d0 + self.th * v
        gap_err = dx - dx_ref                       # >0 далеко (газ), <0 близко (тормоз)
        v_raw = self._v_cmd_base + self._gap_int
        v_pre = self._apply_curve_cap(float(np.clip(
            v_raw, self.v_cmd_min, self.v_cmd_max)))
        saturated = abs(v_raw - v_pre) > 1e-6
        if not (saturated and gap_err * (v_raw - v_pre) > 0.0):
            self._gap_int += self.ki_gap * gap_err * self.ts
            self._gap_int = float(np.clip(
                self._gap_int, -self.gap_int_max, self.gap_int_max))

        # ── итоговая команда: база + интеграл, ограничение + кап по кривизне ──
        self._v_cmd_published = self._apply_curve_cap(float(np.clip(
            self._v_cmd_base + self._gap_int, self.v_cmd_min, self.v_cmd_max)))

        # ── angular control: rotate toward peer ─────────────────────────────
        az_global = float(np.arctan2(dx_vec[1], dx_vec[0]))
        az_rel = az_global - float(msg.theta)
        az_rel = float(np.arctan2(np.sin(az_rel), np.cos(az_rel)))
        w_cmd = self.kp_theta * az_rel

        # ── publish ─────────────────────────────────────────────────────────
        twist = Twist()
        twist.linear.x = self._v_cmd_published
        twist.angular.z = w_cmd
        self.cmd_pub.publish(twist)

        # ── диагностика (Float64 под /<id>/control/) для Foxglove/графиков ──
        dx_ref = self.d0 + self.th * v          # целевой зазор при текущей скорости
        self.dx_pub.publish(Float64(data=dx))
        self.dxref_pub.publish(Float64(data=float(dx_ref)))
        self.dxerr_pub.publish(Float64(data=float(dx - dx_ref)))
        self.vrel_pub.publish(Float64(data=v_rel))
        self.vcmd_pub.publish(Float64(data=float(self._v_cmd_published)))
        self.vmeas_pub.publish(Float64(data=v))

        # throttled debug output
        self._dbg_counter += 1
        if self._dbg_counter >= self._dbg_period:
            self._dbg_counter = 0
            self.get_logger().info(
                f'dx={dx:.3f} v={v:.3f} v_rel={v_rel:+.3f}  '
                f'a_cmd={a_cmd:+.3f} v_base={self._v_cmd_base:.3f} gi={self._gap_int:+.3f} '
                f'v_cmd={self._v_cmd_published:.3f} w={w_cmd:+.3f}  y_gap={y[0]:+.3f}',
            )

    def _stop_robot(self) -> None:
        self.cmd_pub.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SwarmAccMpcNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
