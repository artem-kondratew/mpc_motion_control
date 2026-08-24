"""Контроллеры MPC для симулятора Gazebo (hackathon2026).

Отличия от стенда на реальном роботе:
  * топики без префикса робота — симулятор публикует /cmd_vel, /odom;
  * поза берётся из /odom (одометрия Gazebo точная), LIO-SAM не нужен;
  * фрейм траектории — odom.

Режимы (аргументы launch):
  lateral:=true|false        удержание в полосе поверх продольного контура
  longitudinal:=cc|acc       круиз-контроль | адаптивный (за лидером)
  trajectory:=line|circle|lanelet
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, PushRosNamespace

import os


PKG = 'swarm_controller'


def _config(name):
    return os.path.join(get_package_share_directory(PKG), 'config', name)


def generate_launch_description():
    lateral = LaunchConfiguration('lateral')
    longitudinal = LaunchConfiguration('longitudinal')
    trajectory = LaunchConfiguration('trajectory')
    trajectory_file = LaunchConfiguration('trajectory_file')
    odom_topic = LaunchConfiguration('odom_topic')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    frame_id = LaunchConfiguration('frame_id')
    peer_id = LaunchConfiguration('peer_id')

    is_cc = IfCondition(PythonExpression(["'", longitudinal, "' == 'cc'"]))
    is_acc = IfCondition(PythonExpression(["'", longitudinal, "' == 'acc'"]))

    # Продольный контур пишет сюда, боковой забирает отсюда и выдаёт итоговый cmd_vel.
    long_cmd = '/control/long_cmd'
    path_topic = '/control/pacemaker/path'

    args = [
        DeclareLaunchArgument('lateral', default_value='true',
                              description='удержание в полосе поверх продольного контура'),
        DeclareLaunchArgument('longitudinal', default_value='cc',
                              description='cc = свой v_ref | acc = зазор за лидером'),
        DeclareLaunchArgument('trajectory', default_value='circle',
                              description='форма полосы: line | circle | lanelet'),
        DeclareLaunchArgument('trajectory_file', default_value='my_trajectory5.yaml',
                              description='waypoints для trajectory:=lanelet'),
        DeclareLaunchArgument('odom_topic', default_value='/odom',
                              description='одометрия симулятора'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel',
                              description='команда скорости в симулятор'),
        DeclareLaunchArgument('frame_id', default_value='odom',
                              description='фрейм опорной траектории'),
        DeclareLaunchArgument('peer_id', default_value='leader',
                              description='префикс топиков лидера (для longitudinal:=acc)'),
    ]

    # ── lateral OFF: продольный MPC правит cmd_vel напрямую ──────────────────
    cc_direct = Node(
        package=PKG, executable='swarm_cc_mpc_node', name='swarm_cc_mpc_node',
        output='screen', condition=UnlessCondition(lateral),
        parameters=[_config('cc_mpc.param.yaml'), {
            'odom_topic': odom_topic,
            'cmd_vel_topic': cmd_vel_topic,
        }],
    )

    # ── lateral ON ───────────────────────────────────────────────────────────
    # продольный CC: свой целевой профиль скорости + источник полосы
    cc_group = GroupAction(condition=is_cc, actions=[
        Node(
            package=PKG, executable='swarm_cc_mpc_node', name='swarm_cc_mpc_node',
            output='screen',
            parameters=[_config('cc_mpc.param.yaml'), {
                'odom_topic': odom_topic,
                'cmd_vel_topic': long_cmd,
            }],
        ),
        Node(
            package=PKG, executable='lane_publisher', name='lane_publisher',
            output='screen',
            parameters=[_config('lane.param.yaml'), {
                'path_topic': path_topic,
                'pose_topic': odom_topic,      # поза из Gazebo, не из LIO-SAM
                'frame_id': frame_id,
                'trajectory': trajectory,
                'trajectory_file': trajectory_file,
            }],
        ),
    ])

    # продольный ACC: зазор за лидером (второй робот в симуляторе)
    acc_group = GroupAction(condition=is_acc, actions=[
        Node(
            package=PKG, executable='acc_telemetry', name='acc_telemetry',
            output='screen',
            parameters=[{
                'peer_id': peer_id,
                'peer_pose_topic': ['/', peer_id, '/odom'],
                'peer_vel_topic': ['/', peer_id, '/odom'],
                'peer_path_topic': ['/', peer_id, '/control/pacemaker/path'],
                'self_pose_topic': odom_topic,
                'self_odom_topic': odom_topic,
                'telemetry_topic': '/control/telemetry',
                'path_topic': path_topic,
                'frame_id': frame_id,
            }],
        ),
        Node(
            package=PKG, executable='swarm_acc_mpc_node', name='swarm_acc_mpc_node',
            output='screen',
            parameters=[_config('acc_mpc.param.yaml'), {
                'telemetry_topic': '/control/telemetry',
                'cmd_vel_topic': long_cmd,
                'v_curve_topic': '/control/v_curve',
            }],
        ),
    ])

    # боковой MPC: v от продольного + w от себя -> cmd_vel
    lat_node = Node(
        package=PKG, executable='swarm_lat_mpc_node', name='swarm_lat_mpc_node',
        output='screen', condition=IfCondition(lateral),
        parameters=[_config('lat_mpc.param.yaml'), {
            'long_cmd_topic': long_cmd,
            'cmd_vel_topic': cmd_vel_topic,
            'pacemaker_path_topic': path_topic,
            'pose_topic': odom_topic,
            'odom_topic': odom_topic,
        }],
    )

    lateral_group = GroupAction(condition=IfCondition(lateral),
                                actions=[cc_group, acc_group])

    return LaunchDescription(args + [
        GroupAction(actions=[
            PushRosNamespace('control'),
            cc_direct,
            lateral_group,
            lat_node,
        ]),
    ])
