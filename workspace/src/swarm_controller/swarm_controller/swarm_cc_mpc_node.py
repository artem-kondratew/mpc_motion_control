#!/usr/bin/env python3
"""Cruise Control (CC) MPC node — этап 1 тестов контроллеров.

Одиночный робот: держит целевую продольную скорость v_ref.
- v берётся из /odom (twist.linear.x, kobuki — энкодеры);
- MPC (SwarmCruiseController, adas-style 3-state [v,a,j]) выдаёт a_cmd;
- a_cmd интегрируется в v_cmd (стратегия B2) и публикуется в cmd_vel.

Целевая скорость v_ref — параметр (меняется на лету `ros2 param set ... v_ref X`).

Диагностические выходы (Float64) под /control — для графиков/ошибки трекинга:
  /control/v_ref    целевая скорость
  /control/v_meas   измеренная (из odom)
  /control/v_cmd    команда (то, что ушло в cmd_vel; 0 когда не едем)
  /control/v_error  ошибка = v_ref - v_meas

Kill-switch: параметр `start` (по умолчанию false).
Запуск: ros2 param set /control/swarm_cc_mpc_node start true
"""

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

from .submodules.swarm_cc_mpc import SwarmCruiseController


class SwarmCcMpcNode(Node):

    def __init__(self) -> None:
        super().__init__('swarm_cc_mpc_node')

        self.declare_parameters(namespace='', parameters=[
            # topics
            ('odom_topic',    '/odom'),
            ('cmd_vel_topic', '/cmd_vel'),
            # target
            ('v_ref', 0.3),
            # plant (kinematic)
            ('tau', 0.2),
            # MPC tuning (n_out = 3: [v - v_ref, a, j])
            ('ts', 0.05),
            ('p', 20),
            ('c', 10),
            ('s', 3.0),
            ('phi_vals', [0.6, 0.6, 0.6]),
            ('q_vals',   [10.0, 1.0, 1.0]),
            # constraints
            ('a_min', -0.5),
            ('a_max',  0.5),
            ('v_cmd_min', 0.0),
            ('v_cmd_max', 0.5),
            # curve slowdown: эфф. цель = min(v_ref, v_curve), где v_curve приходит от lat-ноды
            # (предел по кривизне). ТУМБЛЕР режима. Параметры профиля — в lat_mpc.param.yaml.
            ('curve_slowdown', True),
            ('v_curve_topic', 'v_curve'),   # относительный -> /<vehicle_id>/control/v_curve
            ('v_curve_timeout', 0.5),
            # safety
            ('start', False),
            ('odom_timeout', 0.5),
        ])

        self.odom_topic    = self.get_parameter('odom_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        tau = self.get_parameter('tau').value
        ts  = self.get_parameter('ts').value
        p   = self.get_parameter('p').value
        c   = self.get_parameter('c').value
        s   = self.get_parameter('s').value
        phi_vals = list(self.get_parameter('phi_vals').value)
        q_vals   = list(self.get_parameter('q_vals').value)

        self.a_min = self.get_parameter('a_min').value
        self.a_max = self.get_parameter('a_max').value
        self.v_cmd_min = self.get_parameter('v_cmd_min').value
        self.v_cmd_max = self.get_parameter('v_cmd_max').value
        self.odom_timeout = float(self.get_parameter('odom_timeout').value)
        self.ts = float(ts)

        # curve slowdown
        self.curve_slowdown = bool(self.get_parameter('curve_slowdown').value)
        self.v_curve_topic = self.get_parameter('v_curve_topic').value
        self.v_curve_timeout = float(self.get_parameter('v_curve_timeout').value)
        self._v_curve = None
        self._v_curve_stamp = None

        self.get_logger().info('[swarm_cc_mpc] cruise control (этап 1)')
        self.get_logger().info(f'  odom_topic:    {self.odom_topic}')
        self.get_logger().info(f'  cmd_vel_topic: {self.cmd_vel_topic}')
        self.get_logger().info(f'  v_ref={self.get_parameter("v_ref").value}  tau={tau}')
        self.get_logger().info(f'  curve_slowdown={self.curve_slowdown} (v_curve_topic={self.v_curve_topic})')
        self.get_logger().info(f'  MPC: ts={ts}, p={p}, c={c}, s={s}, q={q_vals}, phi={phi_vals}')
        self.get_logger().info(
            f'  a_cmd in [{self.a_min}, {self.a_max}], v_cmd in [{self.v_cmd_min}, {self.v_cmd_max}]')

        self.controller = SwarmCruiseController(
            tau=tau,
            ts=ts, p=p, c=c, s=s,
            phi_vals=phi_vals, q_vals=q_vals,
            u_limits=(self.a_min, self.a_max),
        )

        self._v_meas = None
        self._v_meas_stamp = None
        self._was_started = False
        self._v_cmd_published = 0.0   # B2 integrator

        self._dbg_period = max(1, int(round(1.0 / ts)))
        self._dbg_counter = 0

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        # диагностика для графиков (относительные имена -> под /control)
        self.vref_pub  = self.create_publisher(Float64, 'v_ref',   10)
        self.vmeas_pub = self.create_publisher(Float64, 'v_meas',  10)
        self.vcmd_pub  = self.create_publisher(Float64, 'v_cmd',   10)
        self.verr_pub  = self.create_publisher(Float64, 'v_error', 10)

        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 10)
        self.create_subscription(Float64, self.v_curve_topic, self._v_curve_cb, 10)
        self.create_timer(ts, self._control_step)
        self.get_logger().info('[swarm_cc_mpc] ready, waiting for odom; flip `start`=true to drive')

    def _odom_cb(self, msg: Odometry) -> None:
        self._v_meas = float(msg.twist.twist.linear.x)
        self._v_meas_stamp = self.get_clock().now()

    def _v_curve_cb(self, msg: Float64) -> None:
        self._v_curve = float(msg.data)
        self._v_curve_stamp = self.get_clock().now()

    def _effective_v_ref(self, v_ref: float) -> float:
        """v_ref с учётом замедления на кривизне: min(v_ref, v_curve) при свежем v_curve."""
        if (self.curve_slowdown and self._v_curve is not None
                and self._v_curve_stamp is not None):
            age = (self.get_clock().now() - self._v_curve_stamp).nanoseconds * 1e-9
            if age <= self.v_curve_timeout:
                return min(v_ref, self._v_curve)
        return v_ref

    def _publish_diag(self, v_ref: float, v_meas: float, v_cmd: float) -> None:
        self.vref_pub.publish(Float64(data=float(v_ref)))
        self.vmeas_pub.publish(Float64(data=float(v_meas)))
        self.vcmd_pub.publish(Float64(data=float(v_cmd)))
        self.verr_pub.publish(Float64(data=float(v_ref - v_meas)))

    def _control_step(self) -> None:
        started = bool(self.get_parameter('start').value)
        v_ref = float(self.get_parameter('v_ref').value)   # можно менять на лету
        v_ref = self._effective_v_ref(v_ref)               # замедление на кривизне: min(v_ref, v_curve)

        if started and not self._was_started:
            self.controller.reset()
            self._v_cmd_published = self._v_meas if self._v_meas is not None else 0.0
            self.get_logger().info('[swarm_cc_mpc] start → true, controller reset')
        self._was_started = started

        if self._v_meas is None:
            return                       # ещё нет odom

        fresh = (self.get_clock().now() - self._v_meas_stamp).nanoseconds * 1e-9 <= self.odom_timeout
        v = float(self._v_meas)

        # не управляем (kill-switch off): cmd_vel НЕ трогаем (чтобы не мешать teleop),
        # но диагностику публикуем — видно v_ref/v_meas/ошибку на графике.
        if not started:
            self._publish_diag(v_ref, v, 0.0)
            return

        if not fresh:
            self.get_logger().warn('[swarm_cc_mpc] odom stale, stopping',
                                   throttle_duration_sec=1.0)
            self._stop_robot()
            self.controller.reset()
            self._v_cmd_published = 0.0
            self._publish_diag(v_ref, v, 0.0)
            return

        a_cmd, y = self.controller.calculate_control(v_ref=v_ref, v=v)

        # B2: a_cmd → v_cmd, anti-windup на актюаторе
        self._v_cmd_published += self.ts * a_cmd
        self._v_cmd_published = float(np.clip(
            self._v_cmd_published, self.v_cmd_min, self.v_cmd_max))

        twist = Twist()
        twist.linear.x = self._v_cmd_published
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

        self._publish_diag(v_ref, v, self._v_cmd_published)

        self._dbg_counter += 1
        if self._dbg_counter >= self._dbg_period:
            self._dbg_counter = 0
            self.get_logger().info(
                f'v_ref={v_ref:.3f} v={v:.3f}  a_cmd={a_cmd:+.3f} v_cmd={self._v_cmd_published:.3f}  '
                f'err={v_ref - v:+.3f}')

    def _stop_robot(self) -> None:
        self.cmd_pub.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SwarmCcMpcNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
