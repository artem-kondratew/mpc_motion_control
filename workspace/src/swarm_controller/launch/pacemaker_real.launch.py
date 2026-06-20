"""Pacemaker launch for a REAL robot.

Standalone — does NOT bring up the simulator. Assumes the robot is publishing
its own odometry on /robot{pacemaker_idx}/odom and TF (map -> base_footprint).

All parameters (including robot identity: `pacemaker_idx`, `cmd_vel_topic`,
`odom_topic`) live in config/params_pacemaker_real.yaml. Edit that file to
swap robots or topics — this launch file just loads it.

Args:
    trajectory_file : optional yaml with custom waypoints, overrides the
                      `trajectory_file` value in the config.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory('swarm_controller')
    main_yaml = os.path.join(pkg_dir, 'config', 'params_pacemaker_real.yaml')

    traj_file = LaunchConfiguration('trajectory_file').perform(context).strip()
    params = [main_yaml]
    if traj_file:
        params.append({'trajectory_file': traj_file})

    return [Node(
        package='swarm_controller',
        executable='pacemaker_controller',
        parameters=params,
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'trajectory_file',
            default_value='',
            description=(
                'Optional yaml that overrides waypoints_x/y (e.g. saved from '
                'trajectory_drawer). Empty => use whatever is in '
                'params_pacemaker_real.yaml.'
            ),
        ),
        OpaqueFunction(function=_launch_setup),
    ])
