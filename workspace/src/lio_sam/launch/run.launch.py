import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():

    share_dir = get_package_share_directory('lio_sam')
    parameter_file = LaunchConfiguration('params_file')
    frame_prefix = LaunchConfiguration('frame_prefix')
    xacro_path = os.path.join(share_dir, 'config', 'robot.urdf.xacro')
    rviz_config_file = os.path.join(share_dir, 'config', 'rviz2.rviz')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            share_dir, 'config', 'params.yaml'),
        description='FPath to the ROS2 parameters file to use.')

    # namespacing: префикс TF-фреймов (напр. "robot1/"); пустой = одиночный робот.
    # Прокидывается в lio_sam-ноды как ros-param framePrefix (см. utility.hpp).
    frame_prefix_declare = DeclareLaunchArgument(
        'frame_prefix', default_value='',
        description='TF frame prefix for multi-robot (e.g. "robot1/"); "" = single robot')

    frame_overrides = {'framePrefix': frame_prefix}

    print("urdf_file_name : {}".format(xacro_path))

    rviz_declare = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='launch rviz2 (off for headless containers; виз. через Foxglove)')

    return LaunchDescription([
        params_declare,
        frame_prefix_declare,
        rviz_declare,
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            # mapFrame -> odometryFrame; префиксуем оба, чтобы совпасть с framePrefix в нодах
            arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0',
                       '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                       '--frame-id', [frame_prefix, 'lio_sam_map'],
                       '--child-frame-id', [frame_prefix, 'lio_sam_odom']],
            parameters=[parameter_file],
            output='screen'
            ),
        # robot_state_publisher УБРАН: URDF робота владеет компонент transforms.
        # Дублирование публиковало те же base_link/lidar_link/... и ломало дерево.
        Node(
            package='lio_sam',
            executable='lio_sam_imuPreintegration',
            # name='lio_sam_imuPreintegration',
            parameters=[parameter_file, frame_overrides],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_imageProjection',
            name='lio_sam_imageProjection',
            parameters=[parameter_file, frame_overrides],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_featureExtraction',
            name='lio_sam_featureExtraction',
            parameters=[parameter_file, frame_overrides],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_mapOptimization',
            name='lio_sam_mapOptimization',
            parameters=[parameter_file, frame_overrides],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz'))
        )
    ])
