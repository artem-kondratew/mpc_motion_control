import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


package_name = 'swarm_controller'
robot_id = 3


def generate_launch_description():
    pkg_dir = get_package_share_directory(package_name)

    controller_type_arg = DeclareLaunchArgument(
        'controller_type',
        default_value='sliding',
        description=(
            "Which follower controller to use: 'sliding' (sliding-mode), "
            "'mpc_kin' (kinematic ACC MPC only), 'mpc_full' "
            "(kinematic ACC MPC + lateral MPC)."
        ),
    )
    controller_type = LaunchConfiguration('controller_type')

    params_topics = os.path.join(pkg_dir, 'config', f'params{robot_id}.yaml')
    params_acc    = os.path.join(pkg_dir, 'config', 'params_swarm_acc.yaml')
    params_lat    = os.path.join(pkg_dir, 'config', 'params_swarm_lat.yaml')

    peer_localization = Node(
        package=package_name,
        executable='peer_localization',
        parameters=[params_topics],
        output='screen',
    )

    sliding_node = Node(
        package=package_name,
        executable='swarm_sliding_mode_node',
        parameters=[params_topics],
        condition=LaunchConfigurationEquals('controller_type', 'sliding'),
        output='screen',
    )

    kin_mpc_node = Node(
        package=package_name,
        executable='swarm_acc_mpc_node',
        parameters=[
            params_topics,
            params_acc,
            {
                'telemetry_topic': f'/swarm_controller/telemetry{robot_id}',
                'cmd_vel_topic':   f'/robot{robot_id}/cmd_vel',
                'robot_id':        robot_id,
            },
        ],
        condition=LaunchConfigurationEquals('controller_type', 'mpc_kin'),
        output='screen',
    )

    full_long_node = Node(
        package=package_name,
        executable='swarm_acc_mpc_node',
        parameters=[
            params_topics,
            params_acc,
            {
                'telemetry_topic': f'/swarm_controller/telemetry{robot_id}',
                'cmd_vel_topic':   f'/robot{robot_id}/long_cmd',  # ← redirect
                'robot_id':        robot_id,
            },
        ],
        condition=LaunchConfigurationEquals('controller_type', 'mpc_full'),
        output='screen',
    )
    full_lat_node = Node(
        package=package_name,
        executable='swarm_lat_mpc_node',
        parameters=[
            params_topics,
            params_lat,
            {
                'long_cmd_topic':       f'/robot{robot_id}/long_cmd',
                'cmd_vel_topic':        f'/robot{robot_id}/cmd_vel',
                'odom_topic':           f'/robot{robot_id}/odom',
                'pacemaker_path_topic': '/pacemaker/path',
                'robot_id':             robot_id,
            },
        ],
        condition=LaunchConfigurationEquals('controller_type', 'mpc_full'),
        output='screen',
    )

    return LaunchDescription([
        controller_type_arg,
        peer_localization,
        sliding_node,
        kin_mpc_node,
        full_long_node,
        full_lat_node,
    ])
