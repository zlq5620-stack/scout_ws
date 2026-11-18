"""Handheld-RC localization on the exported FAST-LIO grid; no planner/controller."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory as pkg
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def include(package, filename, parameters):
    path = os.path.join(pkg(package), 'launch', filename)
    if not os.path.isfile(path):
        raise FileNotFoundError(path)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path), launch_arguments=parameters.items())


def generate_launch_description():
    defaults = {
        'map': str(Path.home() / 'scout_ws/maps/fastlio_nav_02.yaml'),
        'interface': 'can0', 'use_sim_time': 'false', 'rviz': 'true',
        'start_lidar': 'true', 'start_odometry': 'true',
        'lidar_x': '0.130', 'lidar_y': '0.0', 'lidar_z': '0.326',
        'lidar_pitch': '0.523599', 'lidar_yaw': '0.0',
    }
    config = LaunchConfiguration
    sim = ParameterValue(config('use_sim_time'), value_type=bool)
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()],
        Node(package='livox_ros_driver2', executable='livox_ros_driver2_node',
             name='livox_lidar_publisher', output='screen',
             condition=IfCondition(config('start_lidar')), parameters=[{
                 'use_sim_time': sim, 'xfer_format': 0, 'multi_topic': 0,
                 'data_src': 0, 'publish_freq': 10.0, 'output_data_type': 0,
                 'frame_id': 'livox_frame',
                 'user_config_path': os.path.join(pkg('livox_ros_driver2'), 'config', 'MID360_config.json'),
             }]),
        include('scout_description', 'scout_base_description.launch.py', {
            **{k: config(k) for k in ('lidar_x', 'lidar_y', 'lidar_z', 'lidar_pitch', 'lidar_yaw')},
            'use_sim_time': config('use_sim_time'),
        }),
        Node(package='scout_localization', executable='can_odometry', output='screen',
             condition=IfCondition(config('start_odometry')), parameters=[{
                 'use_sim_time': sim, 'interface': config('interface'),
                 'odom_frame': 'odom', 'base_frame': 'base_link', 'base_height': .079}]),
        Node(package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
             name='localization_cloud_to_scan', output='screen',
             remappings=[('cloud_in', '/livox/lidar'), ('scan', '/scan')],
             parameters=[os.path.join(pkg('scout_localization'), 'config', 'scan_localization.yaml'),
                         {'use_sim_time': sim}]),
        include('scout_localization', 'fastlio_grid_localization.launch.py', {
            'map': config('map'), 'use_sim_time': config('use_sim_time')}),
        Node(package='rviz2', executable='rviz2', name='rviz_localization', output='screen',
             condition=IfCondition(config('rviz')), arguments=[
                 '-d', os.path.join(pkg('scout_localization'), 'config', 'localization.rviz')]),
    ])
