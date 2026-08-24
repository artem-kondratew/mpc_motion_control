"""Планировщик: опорная траектория -> /planning/trajectory.

Форму можно менять на ходу:
    ros2 param set /planning/trajectory_planner trajectory line
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


PKG = 'swarm_controller'


def generate_launch_description():
    trajectory = LaunchConfiguration('trajectory')
    trajectory_file = LaunchConfiguration('trajectory_file')
    trajectory_topic = LaunchConfiguration('trajectory_topic')
    pose_topic = LaunchConfiguration('pose_topic')
    frame_id = LaunchConfiguration('frame_id')

    args = [
        DeclareLaunchArgument('trajectory', default_value='lanelet',
                              description='line | circle | lanelet'),
        DeclareLaunchArgument('trajectory_file', default_value='my_trajectory5.yaml',
                              description='waypoints для trajectory:=lanelet'),
        DeclareLaunchArgument('trajectory_topic', default_value='/planning/trajectory'),
        DeclareLaunchArgument('pose_topic', default_value='/odom'),
        DeclareLaunchArgument('frame_id', default_value='odom'),
    ]

    planner = Node(
        package=PKG, executable='trajectory_planner', name='trajectory_planner',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory(PKG), 'config', 'lane.param.yaml'), {
            'trajectory': trajectory,
            'trajectory_file': trajectory_file,
            'trajectory_topic': trajectory_topic,
            'pose_topic': pose_topic,
            'frame_id': frame_id,
        }],
    )

    return LaunchDescription(args + [
        GroupAction(actions=[PushRosNamespace('planning'), planner]),
    ])
