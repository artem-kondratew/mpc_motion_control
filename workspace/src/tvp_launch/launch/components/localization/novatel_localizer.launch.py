from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    project_name = 'novatel_localizer'
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false'
    )

    parameters = [
        { "use_sim_time": use_sim_time },
        { "enable_tf_publish": True },
        { "path_min_distance_m": 1.0 },
        { "path_max_points": 100 }
    ]

    remappings=[
        ('in_map_projection_info', '/map/map_projector_info'),
        ('out_initial_pose', '/sensing/gnss/pose_with_covariance'),
        ('out_pose_covariance', '/localization/geo_pose_estimator/pose_with_covariance'),
        ('out_gnss_odometry', '/localization/geo_pose_estimator/kinematic_state'),
        ('out_path', '/localization/geo_pose_estimator/path'),
        ('in_best_position', '/sensing/peakpoints/bestpos'),
        ('in_heading2', '/sensing/peakpoints/heading2'),
        ('in_inspva', '/sensing/peakpoints/inspva'),
        ('in_inspvax', '/sensing/peakpoints/inspvax')
    ]

    return LaunchDescription([
        declare_use_sim_time_cmd,
        Node(
            package=project_name,
            executable=project_name,
            name=project_name,
            namespace=project_name,
            parameters=parameters,
            remappings=remappings
        )
    ])