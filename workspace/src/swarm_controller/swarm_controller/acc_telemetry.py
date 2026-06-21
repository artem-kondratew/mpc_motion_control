#!/usr/bin/env python3
"""acc_telemetry — строит Telemetry для ACC ведомого (bravo) из телеметрии лидера.

Замена peer_localization (лидар) на схему «изолированный DDS + UDP-мост»: позу,
скорость и полосу лидера присылает v2v_receiver зеркалом в alpha-фрейме. Этот узел:

  1) переносит позу лидера и его полосу из alpha-фрейма в bravo-фрейм статическим
     2D-оффсетом (peer_tf_{x,y,yaw} из .env — где старт alpha в системе bravo);
  2) переиздаёт полосу в bravo-фрейме -> /<id>/control/pacemaker/path (для латерали);
  3) проецирует лидера и себя на полосу -> s_leader, s_follower (arc-length);
  4) публикует Telemetry с ЗАЗОРОМ ВДОЛЬ ПОЛОСЫ, закодированным на 1-D ось:
        x=0, peer_x=gap, y=peer_y=theta=0, v=v_self, peer_v=v_leader.
     Тогда штатное ACC `dx=||peer−own||=gap`, `v_rel=peer_v−v`, а угловой член ACC
     обнуляется сам (peer на оси +x, theta=0) — править swarm_acc_mpc_node не нужно.

Почему arc-length, а не евклидово: зазор должен мериться ВДОЛЬ дороги, иначе
кривизна полосы протекает в измерение зазора и дерётся с замедлением на поворотах.
"""

import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from swarm_msgs.msg import Telemetry

from .submodules.frenet import (
    arclength_at,
    cumulative_arclength,
    frenet_project,
    yaw_from_quat,
)


class AccTelemetry(Node):

    def __init__(self) -> None:
        super().__init__('acc_telemetry')

        self.declare_parameters(namespace='', parameters=[
            ('vehicle_id', 'bravo'),
            ('peer_id', 'alpha'),
            # входы: зеркало лидера (alpha-фрейм) от v2v_receiver
            ('peer_pose_topic', '/alpha/lio_sam/mapping/odometry'),
            ('peer_vel_topic',  '/alpha/odom'),
            ('peer_path_topic', '/alpha/control/pacemaker/path'),
            # входы: своё (bravo-фрейм)
            ('self_pose_topic', '/bravo/lio_sam/mapping/odometry'),
            ('self_odom_topic', '/bravo/odom'),
            # выходы (относительные -> под /<id>/control)
            ('telemetry_topic', 'telemetry'),
            ('path_topic',      'pacemaker/path'),
            ('frame_id',        'bravo/lio_sam_odom'),
            # статический оффсет: поза старта peer (alpha) в системе self (bravo)
            ('peer_tf_x',   0.0),
            ('peer_tf_y',   0.0),
            ('peer_tf_yaw', 0.0),
            # темп публикации Telemetry и таймаут свежести входов
            ('rate', 20.0),
            ('input_timeout', 0.5),
        ])

        g = self.get_parameter
        self.vehicle_id = g('vehicle_id').value
        self.peer_id = g('peer_id').value
        self.frame_id = g('frame_id').value
        self.tx = float(g('peer_tf_x').value)
        self.ty = float(g('peer_tf_y').value)
        self.tyaw = float(g('peer_tf_yaw').value)
        self._c, self._s = math.cos(self.tyaw), math.sin(self.tyaw)
        self.input_timeout = float(g('input_timeout').value)
        rate = float(g('rate').value)

        # bravo-фрейм полоса (после переноса из alpha-фрейма)
        self._path_x = None
        self._path_y = None
        self._cumlen = None
        self._total_len = 0.0
        self._cyclic = False
        self._idx_leader = None
        self._idx_self = None

        self._peer_pose = None
        self._peer_vel = None
        self._self_pose = None
        self._self_odom = None
        self._stamp = {'peer_pose': None, 'peer_vel': None,
                       'self_pose': None, 'path': None}

        be_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.tele_pub = self.create_publisher(Telemetry, g('telemetry_topic').value, 10)
        self.path_pub = self.create_publisher(Path, g('path_topic').value, 10)

        # своя поза lio_sam — BEST_EFFORT (как издаёт lio_sam). Зеркало лидера от
        # receiver — RELIABLE, но best_effort-подписка совместима с обоими.
        self.create_subscription(Odometry, g('peer_pose_topic').value, self._peer_pose_cb, be_qos)
        self.create_subscription(Odometry, g('peer_vel_topic').value, self._peer_vel_cb, 10)
        self.create_subscription(Path, g('peer_path_topic').value, self._path_cb, 10)
        self.create_subscription(Odometry, g('self_pose_topic').value, self._self_pose_cb, be_qos)
        self.create_subscription(Odometry, g('self_odom_topic').value, self._self_odom_cb, 10)

        self.create_timer(1.0 / max(rate, 1e-3), self._tick)

        self.get_logger().info(
            f'[acc_telemetry] self={self.vehicle_id} peer={self.peer_id}')
        self.get_logger().info(
            f'  peer_tf (alpha origin in bravo frame): '
            f'x={self.tx:.3f} y={self.ty:.3f} yaw={self.tyaw:+.3f}')
        self.get_logger().info(
            f'  peer pose/vel/path: {g("peer_pose_topic").value} | '
            f'{g("peer_vel_topic").value} | {g("peer_path_topic").value}')
        self.get_logger().info('[acc_telemetry] ready, waiting for inputs...')

    # ── transform alpha-frame -> bravo-frame (2D rigid) ──────────────────────
    def _to_self_frame(self, x: float, y: float):
        return (self._c * x - self._s * y + self.tx,
                self._s * x + self._c * y + self.ty)

    # ── callbacks ────────────────────────────────────────────────────────────
    def _peer_pose_cb(self, msg: Odometry) -> None:
        self._peer_pose = msg
        self._stamp['peer_pose'] = self.get_clock().now()

    def _peer_vel_cb(self, msg: Odometry) -> None:
        self._peer_vel = msg
        self._stamp['peer_vel'] = self.get_clock().now()

    def _self_pose_cb(self, msg: Odometry) -> None:
        self._self_pose = msg
        self._stamp['self_pose'] = self.get_clock().now()

    def _self_odom_cb(self, msg: Odometry) -> None:
        self._self_odom = msg

    def _path_cb(self, msg: Path) -> None:
        if not msg.poses:
            return
        xs, ys = [], []
        for p in msg.poses:
            bx, by = self._to_self_frame(
                float(p.pose.position.x), float(p.pose.position.y))
            xs.append(bx)
            ys.append(by)
        self._path_x = np.asarray(xs, dtype=float)
        self._path_y = np.asarray(ys, dtype=float)
        self._cumlen = cumulative_arclength(self._path_x, self._path_y)
        endpoint = float(np.hypot(self._path_x[0] - self._path_x[-1],
                                  self._path_y[0] - self._path_y[-1]))
        mean_spacing = (float(self._cumlen[-1]) / max(len(self._path_x) - 1, 1)
                        if len(self._path_x) > 1 else 0.0)
        self._cyclic = mean_spacing > 0.0 and endpoint < 2.0 * mean_spacing
        self._total_len = float(self._cumlen[-1]) + (endpoint if self._cyclic else 0.0)
        self._idx_leader = None
        self._idx_self = None
        self._stamp['path'] = self.get_clock().now()
        self._republish_path(msg.header.stamp)

    def _republish_path(self, stamp) -> None:
        out = Path()
        out.header.stamp = stamp
        out.header.frame_id = self.frame_id
        for x, y in zip(self._path_x, self._path_y):
            ps = PoseStamped()
            ps.header = out.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            out.poses.append(ps)
        self.path_pub.publish(out)

    # ── arc-length projection ────────────────────────────────────────────────
    def _project(self, rx: float, ry: float, prev_idx):
        idx, _e, _eth, _k = frenet_project(
            self._path_x, self._path_y, rx, ry, 0.0,
            prev_idx=prev_idx, search_window=20, cyclic=self._cyclic)
        s = arclength_at(self._path_x, self._path_y, self._cumlen, idx, rx, ry)
        return idx, s

    def _fresh(self, key: str) -> bool:
        st = self._stamp[key]
        if st is None:
            return False
        return (self.get_clock().now() - st).nanoseconds * 1e-9 <= self.input_timeout

    # ── main loop ────────────────────────────────────────────────────────────
    def _tick(self) -> None:
        tele = Telemetry()
        tele.is_valid = False

        have_inputs = (
            self._path_x is not None and len(self._path_x) >= 2
            and self._peer_pose is not None and self._self_pose is not None
            and self._peer_vel is not None and self._self_odom is not None
            and self._fresh('peer_pose') and self._fresh('self_pose')
            and self._fresh('peer_vel') and self._fresh('path'))

        if not have_inputs:
            self.tele_pub.publish(tele)   # is_valid=False -> ACC остановит робота
            return

        # позы: лидер из alpha-фрейма -> bravo-фрейм; своя уже в bravo-фрейме
        plx, ply = self._to_self_frame(
            float(self._peer_pose.pose.pose.position.x),
            float(self._peer_pose.pose.pose.position.y))
        sx = float(self._self_pose.pose.pose.position.x)
        sy = float(self._self_pose.pose.pose.position.y)
        sq = self._self_pose.pose.pose.orientation
        stheta = yaw_from_quat(sq.x, sq.y, sq.z, sq.w)

        self._idx_leader, s_leader = self._project(plx, ply, self._idx_leader)
        self._idx_self, s_follower = self._project(sx, sy, self._idx_self)

        raw = s_leader - s_follower
        if self._cyclic and self._total_len > 1e-6:
            raw = raw % self._total_len
            if raw > 0.5 * self._total_len:        # лидер «позади» по петле = мы его обогнали
                raw -= self._total_len
        gap = max(0.0, raw)                         # < gap_safe (даже 0) -> ACC тормозит

        v_self = float(self._self_odom.twist.twist.linear.x)
        v_leader = float(self._peer_vel.twist.twist.linear.x)
        w_self = float(self._self_odom.twist.twist.angular.z)

        # 1-D кодировка: peer на оси +x, own в нуле -> dx=gap, угловой член ACC=0
        tele.x = 0.0
        tele.y = 0.0
        tele.theta = 0.0
        tele.v = v_self
        tele.w = w_self
        tele.peer_x = float(gap)
        tele.peer_y = 0.0
        tele.peer_v = v_leader
        tele.is_valid = True
        self.tele_pub.publish(tele)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AccTelemetry()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
