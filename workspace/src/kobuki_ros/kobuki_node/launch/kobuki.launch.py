import os

import ament_index_python.packages
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

import yaml


def generate_launch_description():

    share_dir = ament_index_python.packages.get_package_share_directory('kobuki_node')
    params_file = os.path.join(share_dir, 'config', 'kobuki_node_params.yaml')
    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['kobuki_ros_node']['ros__parameters']

    # namespacing: префикс TF-фреймов (напр. "robot1/"); пустой = одиночный робот.
    # Оверрайдим odom_frame/base_frame из yaml, чтобы совпасть с framePrefix остальных компонентов.
    frame_prefix = LaunchConfiguration('frame_prefix')
    frame_prefix_declare = DeclareLaunchArgument(
        'frame_prefix', default_value='',
        description='TF frame prefix for multi-robot (e.g. "robot1/"); "" = single robot')

    frame_overrides = {
        'odom_frame': [frame_prefix, params.get('odom_frame', 'odom')],
        'base_frame': [frame_prefix, params.get('base_frame', 'base_link')],
    }

    kobuki_ros_node = Node(package='kobuki_node',
                           executable='kobuki_ros_node',
                           output='screen',
                           parameters=[params, frame_overrides])

    # base_footprint/laser статики УБРАНЫ: их даёт kobuki URDF (robot_state_publisher в transforms).
    # Дублирование публиковало base_link->base_footprint против URDF -> цикл base_link<->base_footprint.

    wrapper = Node(
        package='cmd_vel_wrapper',
        executable='cmd_vel_wrapper',
        name='cmd_vel_wrapper',
    )

    return launch.LaunchDescription([frame_prefix_declare, kobuki_ros_node, wrapper])
