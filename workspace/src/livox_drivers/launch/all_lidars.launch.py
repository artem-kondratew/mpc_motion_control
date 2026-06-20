import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition

from launch_ros.actions import Node


def generate_launch_description():

    package_share = get_package_share_directory('livox_drivers')

    user_config_path = LaunchConfiguration('user_config_path')
    rewrite_params_file = LaunchConfiguration('rewrite_params_file')
    use_rosbag = LaunchConfiguration('use_rosbag')
    # namespacing: префикс выходных фреймов (напр. "robot1/"); пустой = одиночный робот.
    frame_prefix = LaunchConfiguration('frame_prefix')

    livox_ros2_params = [
        {'xfer_format': 0},
        {'multi_topic': 1},
        {'data_src': 0},
        {'publish_freq': 10.0},
        {'output_data_type': 0},
        {'frame_id': 'livox_frame'},
        {'user_config_path': user_config_path},
        {'cmdline_input_bd_code': 'livox0000000001'},
    ]

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_mid360_publisher',
        output='screen',
        parameters=livox_ros2_params,
        condition=UnlessCondition(use_rosbag),
    )

    rewrite_frames = Node(
        package='livox_drivers',
        executable='rewrite_frames',
        name='rewrite_mid360_frames',
        output='screen',
        parameters=[rewrite_params_file, {'frame_prefix': frame_prefix}],
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_rosbag',
            default_value='false',
            description='If true, disable real sensors and use rosbag'
        ),

        DeclareLaunchArgument(
            'frame_prefix',
            default_value='',
            description='TF frame prefix for multi-robot (e.g. "robot1/"); "" = single robot'
        ),

        DeclareLaunchArgument(
            'user_config_path',
            default_value=os.path.join(package_share, 'config', 'config_mid360_all.json'),
            description='Path to Livox user config JSON'
        ),

        DeclareLaunchArgument(
            'rewrite_params_file',
            default_value=os.path.join(package_share, 'config', 'rewrite_frames.yaml'),
            description='Path to YAML with frame rewrite rules'
        ),

        livox_driver,
        rewrite_frames,
    ])