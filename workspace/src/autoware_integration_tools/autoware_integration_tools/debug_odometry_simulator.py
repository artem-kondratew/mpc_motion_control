#!/usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformBroadcaster

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import TransformStamped


class OdometrySimulatorNode(Node):
    
    def __init__(self):
        super().__init__('debug_odometry_simulator_node')
        
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odometry_frame', 'caterwil/livox_mid360_50/imu')
        self.declare_parameter('base_link_frame', 'base_link')
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('motion_type', 'circle')  # circle / rotation
        
        self.map_frame = self.get_parameter('map_frame').value
        self.odometry_frame = self.get_parameter('odometry_frame').value
        self.base_link_frame = self.get_parameter('base_link_frame').value
        self.motion_type = self.get_parameter('motion_type').value

        self.dt = self.get_parameter('dt').value
        
        default_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            'odom_output',
            default_qos
        )

        self.reference_twist_pub = self.create_publisher(
            TwistStamped,
            'reference_twist_output',
            default_qos
        )

        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tf_bl_lidar = None

        self.phi = 0.0
        self.phi_prev = None
        self.t_map_bl_prev = None

    def get_transforms(self):
        while not self.tf_buffer.can_transform(
            self.base_link_frame,
            self.odometry_frame,
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=1.0)
        ):
            self.get_logger().warn("Waiting for TF base_link -> livox...")

        self.tf_bl_lidar = self.tf_buffer.lookup_transform(
            self.base_link_frame,
            self.odometry_frame,
            rclpy.time.Time()
        )
    
    def timer_callback(self):
        if self.tf_bl_lidar is None:
            self.get_transforms()

        if self.motion_type == 'circle':
            r = 10.0
            x = r * np.cos(self.phi)
            y = r * np.sin(self.phi)
            yaw = self.phi + np.pi/2
        else:
            x = 0.0
            y = 0.0
            yaw = self.phi

        z = 0.141

        t_map_bl = np.array([x, y, z]).reshape(-1, 1)

        q_map_bl = Rotation.from_euler('xyz', [0, 0, yaw], degrees=False).as_quat()
        R_map_bl = Rotation.from_euler('xyz', [0, 0, yaw], degrees=False).as_matrix()

        tf_msg = TransformStamped()

        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.map_frame
        tf_msg.child_frame_id = self.base_link_frame

        tf_msg.transform.translation.x = t_map_bl[0, 0]
        tf_msg.transform.translation.y = t_map_bl[1, 0]
        tf_msg.transform.translation.z = t_map_bl[2, 0]

        tf_msg.transform.rotation.x = q_map_bl[0]
        tf_msg.transform.rotation.y = q_map_bl[1]
        tf_msg.transform.rotation.z = q_map_bl[2]
        tf_msg.transform.rotation.w = q_map_bl[3]

        self.tf_broadcaster.sendTransform(tf_msg)
        
        T_map_bl = np.eye(4)
        T_map_bl[:3, :3] = R_map_bl
        T_map_bl[:3, 3] = t_map_bl.reshape(-1)
        
        lx = self.tf_bl_lidar.transform.translation.x
        ly = self.tf_bl_lidar.transform.translation.y
        lz = self.tf_bl_lidar.transform.translation.z

        qx = self.tf_bl_lidar.transform.rotation.x
        qy = self.tf_bl_lidar.transform.rotation.y
        qz = self.tf_bl_lidar.transform.rotation.z
        qw = self.tf_bl_lidar.transform.rotation.w

        lX = np.array([lx, ly, lz]).reshape(-1, 1)

        lR = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()

        T_bl_lidar = np.eye(4)
        T_bl_lidar[:3, :3] = lR
        T_bl_lidar[:3, 3] = lX.reshape(-1)

        T_map_lidar = T_map_bl @ T_bl_lidar
        lidar_pose = T_map_lidar[:3, 3].reshape(-1, 1)
        lidar_R = T_map_lidar[:3, :3]
        lidar_q = Rotation.from_matrix(lidar_R).as_quat()

        odom = Odometry()

        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.map_frame
        odom.child_frame_id = self.odometry_frame
        odom.pose.pose.position.x = lidar_pose[0, 0]
        odom.pose.pose.position.y = lidar_pose[1, 0]
        odom.pose.pose.position.z = lidar_pose[2, 0]
        odom.pose.pose.orientation.x = lidar_q[0]
        odom.pose.pose.orientation.y = lidar_q[1]
        odom.pose.pose.orientation.z = lidar_q[2]
        odom.pose.pose.orientation.w = lidar_q[3]

        self.odom_pub.publish(odom)

        if self.t_map_bl_prev is not None or self.phi_prev is not None:
            wz = (self.phi - self.phi_prev) / self.dt
            w_map = np.array([0, 0, wz]).reshape(-1, 1)
            w_bl = R_map_bl.T @ w_map

            v_map_bl = (t_map_bl - self.t_map_bl_prev) / self.dt
            v_map_bl_local = R_map_bl.T @ v_map_bl

            msg = TwistStamped()

            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.base_link_frame
            msg.twist.linear.x = v_map_bl_local[0, 0]
            msg.twist.linear.y = v_map_bl_local[1, 0]
            msg.twist.linear.z = v_map_bl_local[2, 0]
            msg.twist.angular.x = 0.0
            msg.twist.angular.y = 0.0
            msg.twist.angular.z = w_bl[2, 0]

            self.reference_twist_pub.publish(msg)

        self.t_map_bl_prev = t_map_bl
        self.phi_prev = self.phi

        self.phi += 0.01 if self.motion_type == 'circle' else 0.1


def main(args=None):
    rclpy.init(args=args)
    
    node = OdometrySimulatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()
