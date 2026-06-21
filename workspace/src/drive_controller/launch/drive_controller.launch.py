import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('drive_controller'), 'config', 'params.yaml')

    # namespacing: НЕ самоневмспейсимся — родитель (tvp_core_vehicle) делает
    # push-ros-namespace $(var vehicle_id), а топики в params относительные -> /<vehicle_id>/...
    # frame_prefix префиксует фреймы odom-сообщения (frame_id/child_frame_id).
    frame_prefix = LaunchConfiguration('frame_prefix')
    frame_prefix_arg = DeclareLaunchArgument(
        'frame_prefix', default_value='',
        description='TF frame prefix for multi-robot (e.g. "bravo/"); "" = single robot')

    drive_controller = Node(
        package='drive_controller',
        executable='drive_controller',
        name='drive_controller',
        output='screen',
        parameters=[params_file, {'frame_prefix': frame_prefix}],
    )

    return LaunchDescription([frame_prefix_arg, drive_controller])
