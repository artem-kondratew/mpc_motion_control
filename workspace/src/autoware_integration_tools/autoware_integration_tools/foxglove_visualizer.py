
#!/usr/bin/env python3

from typing import Tuple

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

from autoware_planning_msgs.msg import Trajectory
from autoware_vehicle_msgs.msg import ControlModeReport

from .submodules.robot_modes import RobotModes


class FoxgloveVisualizer(Node):
    """Node that create visualizations for Foxglove"""
    
    def __init__(self):
        super().__init__('foxglove_visualizer')

        self.declare_parameter('robot_length', 1.0)
        self.declare_parameter('robot_width', 1.0)
        self.declare_parameter('robot_height', 1.0)
        self.declare_parameter('trajectory_color', [79., 88., 212., 1.0])
        self.declare_parameter('trajectory_scale', 1.2)

        self.l = self.get_parameter('robot_length').value
        self.w = self.get_parameter('robot_width').value
        self.h = self.get_parameter('robot_height').value
        self.trajectory_color = self.get_parameter('trajectory_color').value
        self.trajectory_scale = self.get_parameter('trajectory_scale').value

        self.get_logger().info(f'robot_length = {self.l}')
        self.get_logger().info(f'robot_width = {self.w}')
        self.get_logger().info(f'trajectory_color = {self.trajectory_color}')
        self.get_logger().info(f'trajectory_scale = {self.trajectory_scale}')

        self.trajectory_width = self.w * self.trajectory_scale

        self.robot_points = np.array([
            [ self.l,  self.w,  self.h],
            [ self.l, -self.w,  self.h],
            [-self.l, -self.w,  self.h],
            [-self.l,  self.w,  self.h],
            [ self.l,  self.w, 0.0],
            [ self.l, -self.w, 0.0],
            [-self.l, -self.w, 0.0],
            [-self.l,  self.w, 0.0],
        ])
        
        reliable_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        self.trajectory_sub = self.create_subscription(Trajectory, 'trajectory_input', self.trajectory_callback, reliable_qos)

        self.odometry_sub = self.create_subscription(Odometry, 'odometry_input', self.robot_box_callback, reliable_qos)

        # self.control_mode_sub = self.create_subscription(ControlModeReport, 'control_mode_input', self.control_mode_callback, reliable_qos)

        # self.crab_mode_sub = self.create_subscription(Bool, 'crab_mode_input', self.crab_mode_callback, reliable_qos)
        
        self.trajectory_pub = self.create_publisher(MarkerArray, 'trajectory_output', reliable_qos)

        self.robot_box_pub = self.create_publisher(Marker, 'robot_box_output', reliable_qos)

        self.robot_mode_pub = self.create_publisher(String, 'robot_mode_output', reliable_qos)

        self.robot_mode_timer = self.create_timer(0.1, self.pub_robot_mode_callback)

        self.robot_z = None

        self.robot_modes = RobotModes()

        self.aw_modes2triggers_map = {
            ControlModeReport.NO_COMMAND: 'set_emergency_stop',
            ControlModeReport.AUTONOMOUS: 'set_autonomous',
            ControlModeReport.MANUAL: 'set_manual',
        }

    def get_borders(self, p : Point) -> Tuple[Point, Point]:
        position = p.pose.position
        q = p.pose.orientation

        _, _, yaw = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')

        nx = -np.sin(yaw)
        ny = np.cos(yaw)
        
        left = Point(
            x=position.x + nx * self.trajectory_width / 2.0,
            y=position.y + ny * self.trajectory_width / 2.0,
            z=self.robot_z
        )

        right = Point(
            x=position.x - nx * self.trajectory_width / 2.0,
            y=position.y - ny * self.trajectory_width / 2.0,
            z=self.robot_z
        )

        return left, right
    
    def trajectory_callback(self, msg: Trajectory) -> None:
        if self.robot_z is None:
            self.get_logger().warn('no lidar odometry yet')
            return

        if len(msg.points) < 2:
            self.get_logger().warn('less than 2 points in trajectory')
            return

        marker_array = MarkerArray()

        marker = Marker()
        marker.header = msg.header
        marker.ns = "trajectory_ribbon"
        marker.id = 0
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.color.r = self.trajectory_color[0] / 255.0
        marker.color.g = self.trajectory_color[1] / 255.0
        marker.color.b = self.trajectory_color[2] / 255.0
        marker.color.a = self.trajectory_color[3]

        for i in range(len(msg.points) - 1):
            p0 = msg.points[i]
            p1 = msg.points[i+1]

            p0_left, p0_right = self.get_borders(p0)
            p1_left, p1_right = self.get_borders(p1)

            marker.points.append(p0_left)
            marker.points.append(p0_right)
            marker.points.append(p1_left)

            marker.points.append(p1_left)
            marker.points.append(p0_right)
            marker.points.append(p1_right)

            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0

        marker_array.markers.append(marker)

        self.trajectory_pub.publish(marker_array)

    def trigger(self, trigger: str) -> None:
        if not self.robot_modes.may_trigger(trigger):
            return
        
        prev_state = self.robot_modes.state
        self.robot_modes.trigger(trigger)
        self.get_logger().info(f'mode switched from "{prev_state}" to "{self.robot_modes.state}"')

    def control_mode_callback(self, msg: ControlModeReport) -> None:
        if not msg.mode in self.aw_modes2triggers_map.keys():
            self.get_logger().error(f'aw mode "{msg.mode}" is not supported')
            return
        
        trigger = self.aw_modes2triggers_map[msg.mode]
        self.trigger(trigger)

    def crab_mode_callback(self, msg: Bool) -> None:
        if msg.data:
            trigger = 'enable_crab'
        else:
            trigger = 'disable_crab'

        self.trigger(trigger)

    def robot_box_callback(self, msg: Odometry) -> None:
        marker = Marker()

        marker.header = msg.header
        marker.ns = "robot_box"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        position = msg.pose.pose.position
        self.robot_z = position.z
        position.z += self.h / 2.0

        marker.pose.position = position
        marker.pose.orientation = msg.pose.pose.orientation

        marker.scale.x = self.l
        marker.scale.y = self.w
        marker.scale.z = self.h

        color = self.robot_modes.color

        marker.color.r = color[0] / 255.
        marker.color.g = color[1] / 255.
        marker.color.b = color[2] / 255.
        marker.color.a = color[3]

        self.robot_box_pub.publish(marker)

    def pub_robot_mode_callback(self):
        msg = String()
        msg.data = self.robot_modes.state
        self.robot_mode_pub.publish(msg)
    
    def __del__(self):
        self.get_logger().info('Shutting down foxglove visualizer...')


def main(args=None):
    rclpy.init(args=args)
    
    node = FoxgloveVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()
