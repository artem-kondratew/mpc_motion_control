#!/usr/bin/env python3
"""lane_publisher — генератор опорной полосы (/pacemaker/path) для lateral-контроллера.

Режимы (`trajectory`):
- `straight` — прямая длиной `length` вдоль +x (шаг `spacing`);
- `circle`   — окружность радиуса `circle_radius` (старт в (0,0), касательная +x, влево);
- `lanelet`  — произвольные waypoints из `waypoints_x/y` или `trajectory_file` (yaml, формат pacemaker).

Полоса генерится в ЛОКАЛЬНЫХ координатах (старт в начале, вдоль +x), затем (если `anchor`)
переносится+поворачивается к текущей позе робота из `pose_topic` (lio_sam) и публикуется
как nav_msgs/Path в `frame_id` (по умолчанию lio_sam_odom) — в том же фрейме, что и поза,
чтобы lat-нода (Frenet) работала без TF.
"""

import os

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

from ament_index_python.packages import get_package_share_directory

from .submodules.frenet import yaw_from_quat


def _resolve(path: str) -> str:
    if path.startswith('~'):
        return os.path.expanduser(path)
    if os.path.isabs(path):
        return path
    return os.path.join(get_package_share_directory('swarm_controller'), 'config', path)


def _load_waypoints_file(path: str):
    with open(_resolve(path)) as f:
        data = yaml.safe_load(f)
    params = data['/**']['ros__parameters']
    return list(params['waypoints_x']), list(params['waypoints_y'])


class LanePublisher(Node):

    def __init__(self) -> None:
        super().__init__('lane_publisher')

        self.declare_parameters(namespace='', parameters=[
            ('trajectory',     'straight'),               # straight | circle | lanelet
            ('frame_id',       'lio_sam_odom'),
            ('path_topic',     '/pacemaker/path'),
            ('pose_topic',     '/lio_sam/mapping/odometry'),
            ('anchor',         True),
            ('publish_rate',   10.0),
            # straight / общий шаг
            ('length',         5.0),
            ('spacing',        0.2),
            # circle
            ('circle_radius',  2.0),
            # lanelet
            ('waypoints_x',    [0.0]),
            ('waypoints_y',    [0.0]),
            ('trajectory_file', ''),
        ])

        self.frame_id   = self.get_parameter('frame_id').value
        self.anchor     = bool(self.get_parameter('anchor').value)
        self.pose_topic = self.get_parameter('pose_topic').value
        traj            = self.get_parameter('trajectory').value
        rate            = float(self.get_parameter('publish_rate').value)

        self.base_wpts = self._generate(traj)     # (N,2) локальные координаты
        self.wpts = None if self.anchor else self.base_wpts
        self._anchored = not self.anchor

        self.last_pose = None
        # lio_sam публикует odometry с BEST_EFFORT QoS — подписка должна совпадать
        be_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, self.pose_topic, self._pose_cb, be_qos)
        self.path_pub = self.create_publisher(
            Path, self.get_parameter('path_topic').value, 10)
        self.create_timer(1.0 / max(rate, 1e-3), self._publish)

        self.get_logger().info(
            f'[lane_publisher] trajectory={traj}, frame={self.frame_id}, '
            f'anchor={self.anchor}, N={len(self.base_wpts)}')

    def _generate(self, traj: str) -> np.ndarray:
        spacing = float(self.get_parameter('spacing').value)
        if traj in ('straight', 'line'):
            length = float(self.get_parameter('length').value)
            s = np.arange(0.0, length + 1e-9, spacing)
            return np.column_stack([s, np.zeros_like(s)])
        if traj == 'circle':
            R = float(self.get_parameter('circle_radius').value)
            dphi = spacing / max(R, 1e-6)
            phi = np.arange(0.0, 2.0 * np.pi, dphi)
            x = R * np.sin(phi)
            y = R * (1.0 - np.cos(phi))          # старт (0,0), касательная +x, поворот влево
            return np.column_stack([x, y])
        if traj == 'lanelet':
            tf = self.get_parameter('trajectory_file').value
            if tf:
                wx, wy = _load_waypoints_file(tf)
            else:
                wx = list(self.get_parameter('waypoints_x').value)
                wy = list(self.get_parameter('waypoints_y').value)
            return np.column_stack([np.asarray(wx, float), np.asarray(wy, float)])
        raise ValueError(f'unknown trajectory mode: {traj}')

    def _pose_cb(self, msg: Odometry) -> None:
        self.last_pose = msg

    def _anchor_to(self, x: float, y: float, theta: float) -> None:
        wp = self.base_wpts
        wp0 = wp[0].copy()
        if len(wp) >= 2:
            seg = wp[1] - wp[0]
            traj_theta = float(np.arctan2(seg[1], seg[0]))
        else:
            traj_theta = 0.0
        dtheta = theta - traj_theta
        c, s = np.cos(dtheta), np.sin(dtheta)
        R_T = np.array([[c, s], [-s, c]])         # поворот row-векторов на dtheta
        self.wpts = (wp - wp0) @ R_T + np.array([x, y])
        self._anchored = True
        self.get_logger().info(
            f'[lane_publisher] anchored to ({x:.2f}, {y:.2f}, {theta:+.2f})')

    def _publish(self) -> None:
        if not self._anchored:
            if self.last_pose is None:
                return
            p = self.last_pose.pose.pose
            th = yaw_from_quat(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
            self._anchor_to(float(p.position.x), float(p.position.y), th)

        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        for wp in self.wpts:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(wp[0])
            ps.pose.position.y = float(wp[1])
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.path_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LanePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
