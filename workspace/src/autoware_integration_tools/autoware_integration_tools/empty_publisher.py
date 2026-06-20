
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Message types
from autoware_perception_msgs.msg import PredictedObjects
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped


class EmptyMessagePublisher(Node):
    """Node that publishes empty messages to perception topics at 10Hz."""
    
    def __init__(self):
        super().__init__('empty_message_publisher')

        self.declare_parameter('use_empty_perception', False)

        self.use_empty_perception = self.get_parameter(
            'use_empty_perception'
        ).get_parameter_value().bool_value

        # Set up QoS profiles for different topics
        # For perception topics, we typically use best effort reliability
        best_effort_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Reliable QoS for topics that need guaranteed delivery
        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        
        if self.use_empty_perception:
            self.objects_pub = self.create_publisher(
                PredictedObjects,
                '/perception/object_recognition/objects',
                reliable_qos
            )

            self.pointcloud_pub = self.create_publisher(
                PointCloud2,
                '/perception/obstacle_segmentation/pointcloud',
                best_effort_qos
            )
        
        self.occupancy_grid_pub = self.create_publisher(
            OccupancyGrid,
            '/perception/occupancy_grid_map/map',
            reliable_qos
        )
        
        # New publishers for GNSS and vehicle velocity
        # self.gnss_pose_pub = self.create_publisher(
        #     PoseWithCovarianceStamped,
        #     '/sensing/gnss/pose_with_covariance',
        #     reliable_qos
        # )
        
        # self.vehicle_twist_pub = self.create_publisher(
        #     TwistWithCovarianceStamped,
        #     '/sensing/vehicle_velocity_converter/twist_with_covariance',
        #     reliable_qos
        # )
        
        # Create timer for 10Hz publishing (0.1 seconds)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info('Empty message publisher initialized. Publishing at 10Hz.')
        self.get_logger().info('Topics:')
        if self.use_empty_perception:
            self.get_logger().info('  - /perception/object_recognition/objects (PredictedObjects)')
            self.get_logger().info('  - /perception/obstacle_segmentation/pointcloud (PointCloud2)')
        self.get_logger().info('  - /perception/occupancy_grid_map/map (OccupancyGrid)')
        # self.get_logger().info('  - /sensing/gnss/pose_with_covariance (PoseWithCovarianceStamped)')
        # self.get_logger().info('  - /sensing/vehicle_velocity_converter/twist_with_covariance (TwistWithCovarianceStamped)')
    
    def timer_callback(self):
        """Timer callback that publishes empty messages to all topics."""
        
        current_time = self.get_clock().now().to_msg()

        if self.use_empty_perception:
            # Create and publish PredictedObjects message
            empty_objects_msg = PredictedObjects()
            empty_objects_msg.header.stamp = current_time
            empty_objects_msg.header.frame_id = 'map'
            self.objects_pub.publish(empty_objects_msg)
            
            # Create and publish PointCloud2 message
            empty_pointcloud_msg = PointCloud2()
            empty_pointcloud_msg.header.stamp = current_time
            empty_pointcloud_msg.header.frame_id = 'map'
            self.pointcloud_pub.publish(empty_pointcloud_msg)
        
        # Create and publish OccupancyGrid message
        empty_occupancy_grid_msg = OccupancyGrid()
        empty_occupancy_grid_msg.header.stamp = current_time
        empty_occupancy_grid_msg.header.frame_id = 'map'
        empty_occupancy_grid_msg.info.resolution = 0.0
        empty_occupancy_grid_msg.info.width = 0
        empty_occupancy_grid_msg.info.height = 0
        empty_occupancy_grid_msg.info.origin.position.x = 0.0
        empty_occupancy_grid_msg.info.origin.position.y = 0.0
        empty_occupancy_grid_msg.info.origin.position.z = 0.0
        empty_occupancy_grid_msg.info.origin.orientation.w = 1.0
        self.occupancy_grid_pub.publish(empty_occupancy_grid_msg)
        
        # Create and publish GNSS pose with covariance
        # gnss_pose_msg = PoseWithCovarianceStamped()
        # gnss_pose_msg.header.stamp = current_time
        # gnss_pose_msg.header.frame_id = 'map'
        # gnss_pose_msg.pose.pose.position.x = -19.3
        # gnss_pose_msg.pose.pose.position.y = 31.5
        # gnss_pose_msg.pose.pose.position.z = 0.0
        # gnss_pose_msg.pose.pose.orientation.x = 0.0
        # gnss_pose_msg.pose.pose.orientation.y = 0.0
        # gnss_pose_msg.pose.pose.orientation.z = 0.134491
        # gnss_pose_msg.pose.pose.orientation.w = 0.990915
        # gnss_pose_msg.pose.covariance = [
        #     0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
        #     0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
        #     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        #     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        #     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        #     0.0, 0.0, 0.0, 0.0, 0.0, 0.0685
        # ]
        # self.gnss_pose_pub.publish(gnss_pose_msg)
        
        # Create and publish vehicle twist with covariance
        # vehicle_twist_msg = TwistWithCovarianceStamped()
        # vehicle_twist_msg.header.stamp = current_time
        # vehicle_twist_msg.header.frame_id = 'base_link'
        # vehicle_twist_msg.twist.twist.linear.x = 0.0
        # vehicle_twist_msg.twist.twist.linear.y = 0.0
        # vehicle_twist_msg.twist.twist.linear.z = 0.0
        # vehicle_twist_msg.twist.twist.angular.x = 0.0
        # vehicle_twist_msg.twist.twist.angular.y = 0.0
        # vehicle_twist_msg.twist.twist.angular.z = 0.0
        # vehicle_twist_msg.twist.covariance = [
        #     0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
        #     0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
        #     0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
        #     0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
        #     0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
        #     0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        # ]
        # self.vehicle_twist_pub.publish(vehicle_twist_msg)
        
        # Optional: Log every 10th message to reduce console spam
        if hasattr(self, 'counter'):
            self.counter += 1
        else:
            self.counter = 0
            
        if self.counter % 10 == 0:
            self.get_logger().debug(f'Published all messages (count: {self.counter})')
    
    def __del__(self):
        self.get_logger().info('Shutting down empty message publisher...')


def main(args=None):
    rclpy.init(args=args)
    
    node = EmptyMessagePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()