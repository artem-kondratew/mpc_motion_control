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

    declare_params_file = DeclareLaunchArgument(
        'twist_estimator_params_file',
        default_value=os.path.join(
            get_package_share_directory(package_name),
            'config',
            'twist_estimator.params.yaml',
        ),
    )

    params_file = LaunchConfiguration('twist_estimator_params_file')

    # namespacing (2+ роботов): frame_prefix ("robot1/") префиксует TF-фреймы,
    # topic_prefix ("/robot1") — абсолютные кросс-компонентные топики. Пусто = одиночный робот.
    declare_frame_prefix = DeclareLaunchArgument('frame_prefix', default_value='')
    declare_topic_prefix = DeclareLaunchArgument('topic_prefix', default_value='')
    frame_prefix = LaunchConfiguration('frame_prefix')
    topic_prefix = LaunchConfiguration('topic_prefix')

    twist_estimator_node = Node(
        package=package_name,
        executable='twist_estimator',
        name='twist_estimator_node',
        output='screen',
        parameters=[params_file, {
            'odometry_frame': [frame_prefix, 'lidar_50'],
            'base_link_frame': [frame_prefix, 'base_link'],
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        remappings=[
            ('odom_input', [topic_prefix, '/lio_sam/mapping/odometry']),
            ('twist_with_cov_output', [topic_prefix, '/sensing/vehicle_velocity_converter/twist_with_covariance']),
            ('twist_without_cov', [topic_prefix, '/sensing/vehicle_velocity_converter/twist']),
        ],
    )

    ld = LaunchDescription()

    ld.add_action(declare_log_level)
    ld.add_action(declare_params_file)
    ld.add_action(declare_frame_prefix)
    ld.add_action(declare_topic_prefix)
    ld.add_action(twist_estimator_node)

    return ld
