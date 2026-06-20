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
        description='Logging level'
    )

    declare_params_file = DeclareLaunchArgument(
        'foxglove_visualizer_params_file',
        default_value=os.path.join(
            get_package_share_directory(package_name),
            'config',
            'foxglove_visualizer.yaml'
        ),
    )

    foxglove_visualizer_node = Node(
        package=package_name,
        executable='foxglove_visualizer',
        name='foxglove_visualizer',
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[LaunchConfiguration('foxglove_visualizer_params_file')],
        remappings=[
                ('trajectory_input', '/planning/trajectory'),
                ('trajectory_output', '/planning/trajectory/foxglove'),

                ('odometry_input', '/localization/kinematic_state'),
                ('robot_box_output', '/localization/robot_box/foxglove'),

                ('control_mode_input', '/vehicle/status/control_mode'),
                ('crab_mode_input', '/caterwil/control/command/set_crab_kinematic_cmd'),
                ('robot_mode_output', '/foxglove/robot_mode')
        ]
    )
    
    ld = LaunchDescription()
    ld.add_action(declare_log_level)
    ld.add_action(declare_params_file)
    ld.add_action(foxglove_visualizer_node)
    
    return ld
