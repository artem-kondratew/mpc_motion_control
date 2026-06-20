#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'autoware_integration_tools'

    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error)'
    )

    declare_odometry_frame = DeclareLaunchArgument(
        'odometry_frame',
        default_value='caterwil/livox_mid360_50/imu',
    )

    declare_base_link_frame = DeclareLaunchArgument(
        'base_link_frame',
        default_value='base_link',
    )

    declare_params_file = DeclareLaunchArgument(
        'odom_sim_params_file',
        default_value=os.path.join(
            get_package_share_directory(package_name),
            'config',
            'debug_twist_estimator.params.yaml',
        ),
    )

    odometry_frame = LaunchConfiguration('odometry_frame')
    base_link_frame = LaunchConfiguration('base_link_frame')
    params_file = LaunchConfiguration('odom_sim_params_file')

    rviz_file = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        'debug_twist_estimator.rviz'
    )

    debug_odometry_simulator_node = Node(
        package=package_name,
        executable='debug_odometry_simulator',
        name='debug_odometry_simulator_node',
        output='screen',
        parameters=[params_file],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        remappings=[
            ('odom_output', '/localization/lidar_odometry/state'),
            ('reference_twist_output', '/debug/odometry_simulator/reference_twist'),
        ]
    )
    
    twist_estimator_node = Node(
        package=package_name,
        executable='twist_estimator',
        name='twist_estimator_node',
        output='screen',
        parameters=[params_file, {'debug': True}],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        remappings=[
            ('odom_input', '/localization/lidar_odometry/state'),
            ('reference_twist_input', '/debug/odometry_simulator/reference_twist'),
            ('twist_with_cov_output', '/sensing/vehicle_velocity_converter/twist_with_covariance'),
            ('twist_without_cov', '/sensing/vehicle_velocity_converter/twist'),
        ],
    )

    static_tf_base_link_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_camera_init_node',
        arguments=[
            '--x',  '0.5',
            '--y',  '0.0',
            '--z',  '1.1',
            '--roll', '0.0',
            '--pitch', '0.611',
            '--yaw', '0.785',
            '--frame-id', base_link_frame,
            '--child-frame-id', odometry_frame,
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
    )
    
    ld = LaunchDescription()
    
    ld.add_action(declare_log_level)
    ld.add_action(declare_odometry_frame)
    ld.add_action(declare_base_link_frame)
    ld.add_action(declare_params_file)
    ld.add_action(debug_odometry_simulator_node)
    ld.add_action(twist_estimator_node)
    ld.add_action(static_tf_base_link_lidar)
    ld.add_action(rviz_node)
    
    return ld
