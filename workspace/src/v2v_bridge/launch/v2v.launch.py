#!/usr/bin/env python3
"""Запуск UDP-моста телеметрии. role=leader -> sender, role=follower -> receiver.

leader_id — префикс топиков лидера (на лидере = свой VEHICLE_ID, на ведомом = PEER_ID).
peer_ip   — IP получателя (нужен только на лидере / sender).
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    role = LaunchConfiguration('role')
    leader_id = LaunchConfiguration('leader_id')
    peer_ip = LaunchConfiguration('peer_ip')
    channels_file = LaunchConfiguration('channels_file')

    default_cfg = os.path.join(
        get_package_share_directory('v2v_bridge'), 'config', 'channels.yaml')

    is_leader = IfCondition(PythonExpression(["'", role, "' == 'leader'"]))
    is_follower = IfCondition(PythonExpression(["'", role, "' == 'follower'"]))

    return LaunchDescription([
        DeclareLaunchArgument('role', default_value='follower',
                              description='leader (sender) | follower (receiver)'),
        DeclareLaunchArgument('leader_id', default_value='alpha',
                              description='префикс топиков лидера'),
        DeclareLaunchArgument('peer_ip', default_value='127.0.0.1',
                              description='IP получателя (для sender)'),
        DeclareLaunchArgument('channels_file', default_value=default_cfg),

        Node(
            package='v2v_bridge', executable='v2v_sender', name='v2v_sender',
            output='screen', condition=is_leader,
            parameters=[{
                'leader_id': leader_id,
                'peer_ip': peer_ip,
                'channels_file': channels_file,
            }],
        ),
        Node(
            package='v2v_bridge', executable='v2v_receiver', name='v2v_receiver',
            output='screen', condition=is_follower,
            parameters=[{
                'leader_id': leader_id,
                'channels_file': channels_file,
            }],
        ),
        # видимость лидера на карте ведомого: зеркало его позы -> TF
        # <leader>/lio_sam_odom -> <leader>  (ИМЕННО vehicle-фрейм лидера, напр. alpha),
        # подвешивается через статик-анкер peer_map_anchor_tf из сервиса transforms.
        Node(
            package='v2v_bridge', executable='odom_tf_broadcaster', name='peer_odom_tf',
            output='screen', condition=is_follower,
            parameters=[{
                'odom_topic': [TextSubstitution(text='/'), leader_id,
                               TextSubstitution(text='/lio_sam/mapping/odometry')],
                'child_frame': leader_id,   # публикуем голый фрейм лидера (alpha), не base_footprint
            }],
        ),
    ])
