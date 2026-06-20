import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('livox_drivers')
    default_params = os.path.join(package_share, 'config', 'rewrite_frames.yaml')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to YAML with input_topics, frame_names and output_topics',
        ),
        Node(
            package='livox_drivers',
            executable='frame_id_rewriter',
            name='frame_id_rewriter',
            output='screen',
            parameters=[params_file],
        ),
    ])
