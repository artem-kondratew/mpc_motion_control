from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    package_share = get_package_share_directory('caterwil_sensor_kit_launch')
    livox_drivers_share = get_package_share_directory('livox_drivers')
    #insta_drivers_share = get_package_share_directory('insta360_ros_driver')

    declare_use_rosbag = DeclareLaunchArgument(
        'use_rosbag',
        default_value='false',
        description='true if rosbag is in use instead of sensor kit'
    )

    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                livox_drivers_share,
                'launch',
                'all_lidars.launch.py'
            )
        ),
        launch_arguments={
            'use_rosbag': LaunchConfiguration('use_rosbag'),
        }.items()
    )

    return LaunchDescription([
        declare_use_rosbag,
        livox_launch,
    ])
