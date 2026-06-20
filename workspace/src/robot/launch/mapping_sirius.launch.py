import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'fast_lio_launch'

    params_file = os.path.join(get_package_share_directory(package_name), 'config', 'mid360.yaml')
    rviz_file = os.path.join(get_package_share_directory(package_name), 'config', 'rviz_livox.rviz')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'fast_lio_sim_use_rviz',
        default_value='false'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('fast_lio_sim_use_rviz')

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fast_lio_node',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        output='screen',
        remappings=[('Odometry', 'caterwil/odometry'),],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        condition=IfCondition(use_rviz)
    )
    
    livox_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name), 'launch', 'msg_MID360.launch.py')]
        ),
    )
    
    static_tf_odom_camera_init_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_camera_init_node',
        arguments=[
            '--x',  '0.011',
            '--y',  '0.02329',
            '--z',  '0.23788',
            '--qx', '0.0',
            '--qy', '0.0',
            '--qz', '0.0',
            '--qw', '1.0',
            '--frame-id', 'caterwil/odom',
            '--child-frame-id', 'camera_init',
        ]
    )
    
    static_tf_body_imu_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_body_imu_node',
        arguments=[
            '--x',  '0.0',
            '--y',  '0.0',
            '--z',  '0.0',
            '--qx', '0.0',
            '--qy', '0.0',
            '--qz', '0.0',
            '--qw', '1.0',
            '--frame-id', 'body',
            '--child-frame-id', 'caterwil/imu',
        ]
    )
    
    static_tf_imu_lidar_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_imu_lidar_node',
        arguments=[
            '--x',  '-0.011',
            '--y',  '-0.02329',
            '--z',  '0.04412',
            '--qx', '0.0',
            '--qy', '0.0',
            '--qz', '0.0',
            '--qw', '1.0',
            '--frame-id', 'caterwil/imu',
            '--child-frame-id', 'caterwil/livox_mid360',
        ]
    )
    
    static_tf_imu_livox_frame_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_imu_livox_frame_node',
        arguments=[
            '--x',  '0.0',
            '--y',  '0.0',
            '--z',  '0.0',
            '--qx', '0.0',
            '--qy', '0.0',
            '--qz', '0.0',
            '--qw', '1.0',
            '--frame-id', 'caterwil/imu',
            '--child-frame-id', 'livox_frame',
        ]
    )
    
    static_tf_lidar_base_link_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_lidar_base_link_node',
        arguments=[
            '--x',  '0.0',
            '--y',  '0.0',
            '--z',  '-0.141',
            '--qx', '0.0',
            '--qy', '0.0',
            '--qz', '0.0',
            '--qw', '1.0',
            '--frame-id', 'caterwil/livox_mid360',
            '--child-frame-id', 'caterwil/BASE_LINK',
        ]
    )

    ld = LaunchDescription()
    
    ld.add_action(use_sim_time_arg)
    ld.add_action(use_rviz_arg)
    ld.add_action(fast_lio_node)
    ld.add_action(rviz_node)
    ld.add_action(livox_node)
    ld.add_action(static_tf_odom_camera_init_node)
    ld.add_action(static_tf_body_imu_node)
    ld.add_action(static_tf_imu_lidar_node)
    ld.add_action(static_tf_imu_livox_frame_node)
    ld.add_action(static_tf_lidar_base_link_node)

    return ld
