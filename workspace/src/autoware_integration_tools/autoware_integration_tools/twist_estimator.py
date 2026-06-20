#!/usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from tf2_ros import Buffer, TransformListener

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped, TwistStamped


class TwistEstimatorNode(Node):

    def __init__(self):
        super().__init__('twist_estimator_node')

        self.declare_parameter('odometry_frame', 'caterwil/livox_mid360_50/imu')
        self.declare_parameter('base_link_frame', 'base_link')
        self.declare_parameter('filter_cutoff_frequency', 5.0)  # [Hz]
        self.declare_parameter('enable_filtering', True)
        self.declare_parameter('velocity_stddev_xx', 0.2)  # [m/s]
        self.declare_parameter('angular_velocity_stddev_zz', 0.1)  # [rad/s]
        self.declare_parameter('alpha_linear', 0.01)
        self.declare_parameter('alpha_angular', 0.1)
        self.declare_parameter('debug', False)
        
        self.odometry_frame = self.get_parameter('odometry_frame').value
        self.base_link_frame = self.get_parameter('base_link_frame').value
        
        self.velocity_stddev_xx = self.get_parameter('velocity_stddev_xx').value
        self.angular_velocity_stddev_zz = self.get_parameter('angular_velocity_stddev_zz').value
        self.filter_cutoff_freq = self.get_parameter('filter_cutoff_frequency').value
        self.enable_filtering = self.get_parameter('enable_filtering').value
        self.alpha_linear = self.get_parameter('alpha_linear').value
        self.alpha_angular = self.get_parameter('alpha_angular').value

        self.debug = self.get_parameter('debug').value
        
        # Filtered velocities in local frame
        self.v_bl_bl_filtered = np.zeros((3, 1))
        self.w_bl_filtered = np.zeros((3, 1))
        
        odom_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        
        twist_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom_input',
            self.odometry_callback,
            odom_qos
        )

        if self.debug:
            self.debug_ref_twist_sub = self.create_subscription(
            TwistStamped,
            'reference_twist_input',
            self.reference_twist_callback,
            twist_qos
        )
        
        self.twist_with_cov_pub = self.create_publisher(
            TwistWithCovarianceStamped,
            'twist_with_cov_output',
            twist_qos
        )

        self.twist_without_cov_pub = self.create_publisher(
            TwistStamped,
            'twist_without_cov',
            twist_qos
        )

        if self.debug:
            self.timer = self.create_timer(1.0, self.log_callback)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tf_bl_lidar = None

        self.R_map_lidar_prev = None
        self.ts_prev = None
        self.prev_position = None

        self.v_ref = None
        self.w_ref = None

        self.init_static_transforms = False

    def reference_twist_callback(self, msg: TwistStamped):
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        vz = msg.twist.linear.z

        self.v_ref = np.array([vx, vy, vz]).reshape(-1, 1)

        wx = msg.twist.angular.x
        wy = msg.twist.angular.y
        wz = msg.twist.angular.z

        self.w_ref = np.array([wx, wy, wz]).reshape(-1, 1)

    def log_callback(self):
        v = self.v_bl_bl_filtered.reshape(-1)
        w = self.w_bl_filtered.reshape(-1)
        self.get_logger().info(f'v = {v}, w = {w}')

        if self.v_ref is not None and self.w_ref is not None:
            v_ref = self.v_ref.reshape(-1)
            w_ref = self.w_ref.reshape(-1)
            linear_err = np.round(np.abs(v_ref - v), 3)
            angular_err = np.round(np.abs(w_ref - w), 3)
            self.get_logger().info(f'linear_error = {linear_err}, angular error= {angular_err}')

    def get_static_transforms(self, timestamp):
        while not self.tf_buffer.can_transform( 
            self.base_link_frame,
            self.odometry_frame,
            timestamp,
            timeout=rclpy.duration.Duration(seconds=1.0)
        ):
            self.get_logger().warn(f'Waiting for TF {self.base_link_frame} -> {self.odometry_frame}')

        self.tf_bl_lidar = self.tf_buffer.lookup_transform(
            self.base_link_frame,
            self.odometry_frame,
            timestamp
        )

        self.init_static_transforms = True

    def skew_symmetric(self, w):
        wx, wy, wz = w.reshape(-1)
        return np.array([
            [0, -wz,  wy],
            [wz,  0, -wx],
            [-wy, wx,  0],
        ])
    
    def update_filter_coefficients(self, dt):
        """Update low-pass filter coefficients based on time step."""
        if dt > 0.0:
            wc = 2.0 * np.pi * self.filter_cutoff_freq
            self.alpha_linear = dt / (dt + 1.0 / wc)
            self.alpha_angular = dt / (dt + 1.0 / wc)
            # Clamp to reasonable range
            self.alpha_linear = min(0.95, max(0.05, self.alpha_linear))
            self.alpha_angular = min(0.95, max(0.05, self.alpha_angular))

    def apply_low_pass_filter(self, new_value: np.ndarray, filtered_value: np.ndarray, alpha: float) -> np.ndarray:
        """Apply first-order low-pass filter."""
        if not self.enable_filtering:
            return new_value
        return alpha * new_value + (1.0 - alpha) * filtered_value
    
    def update_state(self, R_map_lidar, ts, current_position):
        self.R_map_lidar_prev = R_map_lidar
        self.ts_prev = ts
        self.prev_position = current_position

    def odometry_callback(self, msg: Odometry):
        if not self.init_static_transforms:
            self.get_static_transforms(msg.header.stamp)
            
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        current_position = msg.pose.pose.position

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        q_map_lidar = [qx, qy, qz, qw]

        R_map_lidar = Rotation.from_quat(q_map_lidar).as_matrix()

        if self.ts_prev is None:
            self.update_state(R_map_lidar, ts, current_position)
            return

        dt = ts - self.ts_prev

        if self.R_map_lidar_prev is None or dt <= 0 or self.prev_position is None:
            self.update_state(R_map_lidar, ts, current_position)
            return

        dR = R_map_lidar @ self.R_map_lidar_prev.T

        rotvec = Rotation.from_matrix(dR).as_rotvec().reshape(-1, 1)  # R_2 @ R_1^T = exp(S(w_map)dt)
        w_map = rotvec / dt

        self.update_filter_coefficients(dt)

        delta_x = current_position.x - self.prev_position.x
        delta_y = current_position.y - self.prev_position.y
        delta_z = current_position.z - self.prev_position.z

        v_map_lidar = np.array([delta_x, delta_y, delta_z]).reshape(-1, 1) / dt

        x = self.tf_bl_lidar.transform.translation.x
        y = self.tf_bl_lidar.transform.translation.y
        z = self.tf_bl_lidar.transform.translation.z

        t_bl_lidar = np.array([x, y, z]).reshape(-1, 1)

        qx = self.tf_bl_lidar.transform.rotation.x
        qy = self.tf_bl_lidar.transform.rotation.y
        qz = self.tf_bl_lidar.transform.rotation.z
        qw = self.tf_bl_lidar.transform.rotation.w

        q_bl_lidar = [qx, qy, qz, qw]

        R_bl_lidar = Rotation.from_quat(q_bl_lidar).as_matrix()

        R_map_bl = R_map_lidar @ R_bl_lidar.T

        t_map = R_map_bl @ t_bl_lidar  # t_bl_lidar vector in map frame

        v_map_bl = v_map_lidar - self.skew_symmetric(w_map) @ t_map
        v_bl_bl = R_map_bl.T @ v_map_bl
        w_bl = R_map_bl.T @ w_map

        v_bl_bl_filtered = self.apply_low_pass_filter(
            v_bl_bl, self.v_bl_bl_filtered, self.alpha_linear
        )
        w_bl_filtered = self.apply_low_pass_filter(
            w_bl, self.w_bl_filtered, self.alpha_angular
        )

        self.v_bl_bl_filtered = v_bl_bl_filtered
        self.w_bl_filtered = w_bl_filtered

        # Fill TwistWithCovarianceStamped
        twist_msg = TwistWithCovarianceStamped()

        twist_msg.header.stamp = msg.header.stamp
        twist_msg.header.frame_id = self.base_link_frame
        
        twist_msg.twist.twist.linear.x = v_bl_bl_filtered[0, 0]
        twist_msg.twist.twist.linear.y = v_bl_bl_filtered[1, 0]
        twist_msg.twist.twist.linear.z = v_bl_bl_filtered[2, 0]
        twist_msg.twist.twist.angular.x = w_bl_filtered[0, 0]
        twist_msg.twist.twist.angular.y = w_bl_filtered[1, 0]
        twist_msg.twist.twist.angular.z = w_bl_filtered[2, 0]
        
        # Set covariance matrix
        twist_msg.twist.covariance = [0.0] * 36
        twist_msg.twist.covariance[0 + 0 * 6] = self.velocity_stddev_xx ** 2
        twist_msg.twist.covariance[1 + 1 * 6] = 10000.0  # vy (high uncertainty)
        twist_msg.twist.covariance[2 + 2 * 6] = 10000.0  # vz
        twist_msg.twist.covariance[3 + 3 * 6] = 10000.0  # wx
        twist_msg.twist.covariance[4 + 4 * 6] = 10000.0  # wy
        twist_msg.twist.covariance[5 + 5 * 6] = self.angular_velocity_stddev_zz ** 2

        self.twist_with_cov_pub.publish(twist_msg)

        # Fill TwistStamped
        twist_msg = TwistStamped()

        twist_msg.header.stamp = msg.header.stamp
        twist_msg.header.frame_id = self.base_link_frame
        twist_msg.twist.linear.x = v_bl_bl_filtered[0, 0]
        twist_msg.twist.linear.y = v_bl_bl_filtered[1, 0]
        twist_msg.twist.linear.z = v_bl_bl_filtered[2, 0]
        twist_msg.twist.angular.x = w_bl_filtered[0, 0]
        twist_msg.twist.angular.y = w_bl_filtered[1, 0]
        twist_msg.twist.angular.z = w_bl_filtered[2, 0]

        self.twist_without_cov_pub.publish(twist_msg)

        self.update_state(R_map_lidar, ts, current_position)


def main(args=None):
    rclpy.init(args=args)
    
    node = TwistEstimatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
