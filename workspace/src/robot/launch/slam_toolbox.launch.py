import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace, SetRemap

from ament_index_python.packages import get_package_share_directory


robot_id = os.environ.get("ROBOT_ID", "unknown")

namespace = f"robot{robot_id}"

package_name = 'robot'

slam_toolobox_config = os.path.join(get_package_share_directory(package_name), 'config', 'slam_lidar_params.yaml')

slam_toolbox = GroupAction(
    actions = [
        PushRosNamespace(namespace),
        SetRemap('slam_toolbox', 'robot2_slam_toolbox'),
        #SetRemap('/scan', f'{namespace}/scan'),
        SetRemap('/map', f'map'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')]
            ),
            launch_arguments={'slam_params_file': slam_toolobox_config}.items()
        )
    ]
)


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(slam_toolbox)
    
    return ld
