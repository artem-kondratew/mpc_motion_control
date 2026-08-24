#!/usr/bin/env python3
"""Планировщик: публикует опорную траекторию в /planning/trajectory.

Форма задаётся параметром trajectory (line | circle | lanelet) и может
меняться в рантайме — нода перегенерирует путь и опубликует новый:

    ros2 param set /planning/trajectory_planner trajectory circle
    ros2 param set /planning/trajectory_planner circle_radius 3.0

При anchor=true траектория привязывается к текущей позе робота (старт из
точки, где робот стоит). Смена формы на ходу привязывает новый путь к
позе на момент смены.
"""

import os

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from .submodules.frenet import yaw_from_quat


# формы, которые нода умеет строить; смена любого из этих параметров
# в рантайме приводит к перегенерации пути
SHAPE_PARAMS = ('trajectory', 'length', 'spacing', 'circle_radius',
                'waypoints_x', 'waypoints_y', 'trajectory_file')


def _resolve(path: str) -> str:
    if os.path.isabs(path):
        return path
    return os.path.join(
        get_package_share_directory('swarm_controller'), 'config', path)


def _load_waypoints_file(path: str):
    with open(_resolve(path)) as f:
        data = yaml.safe_load(f)
    params = data['/**']['ros__parameters']
    return list(params['waypoints_x']), list(params['waypoints_y'])


class TrajectoryPlanner(Node):

    def __init__(self) -> None:
        super().__init__('trajectory_planner')

        self.declare_parameters(namespace='', parameters=[
            ('trajectory',      'lanelet'),             # line | circle | lanelet
            ('frame_id',        'odom'),
            ('trajectory_topic', '/planning/trajectory'),
            ('pose_topic',      '/odom'),
            ('anchor',          True),
            ('publish_rate',    10.0),
            # line / общий шаг
            ('length',          5.0),
            ('spacing',         0.2),
            # circle
            ('circle_radius',   2.0),
            # lanelet
            ('waypoints_x',     [0.0]),
            ('waypoints_y',     [0.0]),
            ('trajectory_file', 'my_trajectory5.yaml'),
        ])

        self.frame_id = self.get_parameter('frame_id').value
        self.anchor = bool(self.get_parameter('anchor').value)
        rate = float(self.get_parameter('publish_rate').value)

        self.last_pose = None
        self.base_wpts = self._generate()
        self.wpts = None if self.anchor else self.base_wpts
        self._anchored = not self.anchor

        be_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            Odometry, self.get_parameter('pose_topic').value, self._pose_cb, be_qos)
        self.traj_pub = self.create_publisher(
            Path, self.get_parameter('trajectory_topic').value, 10)
        self.create_timer(1.0 / max(rate, 1e-3), self._publish)

        self.add_on_set_parameters_callback(self._on_set_params)

        self.get_logger().info(
            f'[trajectory_planner] {self.get_parameter("trajectory").value}, '
            f'frame={self.frame_id}, anchor={self.anchor}, N={len(self.base_wpts)}')

    # ── генерация ────────────────────────────────────────────────────────────

    def _generate(self, overrides: dict | None = None) -> np.ndarray:
        """Строит путь в локальных координатах.

        overrides — значения параметров, которые ещё не применены к ноде
        (приходят из колбэка смены параметров).
        """
        ov = overrides or {}

        def param(name):
            return ov[name] if name in ov else self.get_parameter(name).value

        traj = param('trajectory')
        spacing = float(param('spacing'))
        if traj in ('straight', 'line'):
            length = float(param('length'))
            s = np.arange(0.0, length + 1e-9, spacing)
            return np.column_stack([s, np.zeros_like(s)])
        if traj == 'circle':
            R = float(param('circle_radius'))
            dphi = spacing / max(R, 1e-6)
            phi = np.arange(0.0, 2.0 * np.pi, dphi)
            x = R * np.sin(phi)
            y = R * (1.0 - np.cos(phi))      # старт (0,0), касательная +x, поворот влево
            return np.column_stack([x, y])
        if traj == 'lanelet':
            tf = param('trajectory_file')
            if tf:
                wx, wy = _load_waypoints_file(tf)
            else:
                wx = list(param('waypoints_x'))
                wy = list(param('waypoints_y'))
            return np.column_stack([np.asarray(wx, float), np.asarray(wy, float)])
        raise ValueError(f'unknown trajectory mode: {traj}')

    # ── смена траектории в рантайме ──────────────────────────────────────────

    def _on_set_params(self, params) -> SetParametersResult:
        if not any(p.name in SHAPE_PARAMS for p in params):
            return SetParametersResult(successful=True)
        # колбэк вызывается ДО применения — новые значения передаём явно
        overrides = {p.name: p.value for p in params}
        try:
            self.base_wpts = self._generate(overrides)
        except (ValueError, FileNotFoundError, KeyError, OSError) as exc:
            self.get_logger().error(f'[trajectory_planner] {exc}')
            return SetParametersResult(successful=False, reason=str(exc))
        # перепривязываем к текущей позе: новый путь начинается там, где робот сейчас
        self._anchored = not self.anchor
        if not self.anchor:
            self.wpts = self.base_wpts
        shape = overrides.get('trajectory', self.get_parameter('trajectory').value)
        self.get_logger().info(
            f'[trajectory_planner] новая траектория: {shape}, N={len(self.base_wpts)}')
        return SetParametersResult(successful=True)

    # ── публикация ───────────────────────────────────────────────────────────

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
        R_T = np.array([[c, s], [-s, c]])     # поворот row-векторов на dtheta
        self.wpts = (wp - wp0) @ R_T + np.array([x, y])
        self._anchored = True
        self.get_logger().info(
            f'[trajectory_planner] привязка к ({x:.2f}, {y:.2f}, {theta:+.2f})')

    def _publish(self) -> None:
        if not self._anchored:
            if self.last_pose is None:
                return
            p = self.last_pose.pose.pose
            th = yaw_from_quat(p.orientation.x, p.orientation.y,
                               p.orientation.z, p.orientation.w)
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
        self.traj_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectoryPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
