#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PythonExpression


def generate_launch_description():
    package_name = 'autoware_integration_tools'

    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level'
    )

    declare_launch_perception = DeclareLaunchArgument(
        'launch_perception',
        default_value='true',
        description='if false using empty PredictedObjects msgs and empty obstacle point cloud else using perception module'
    )

    use_empty_perception = PythonExpression([
        '"', LaunchConfiguration('launch_perception'), '" == "false"'
    ])

    empty_publisher_node = Node(
        package=package_name,
        executable='empty_publisher',
        name='empty_publisher_node',
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[{
            'use_empty_perception': use_empty_perception,
        }]
    )
    
    ld = LaunchDescription()
    ld.add_action(declare_log_level)
    ld.add_action(declare_launch_perception)
    ld.add_action(empty_publisher_node)
    
    return ld
