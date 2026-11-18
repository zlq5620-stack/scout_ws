"""Localization + A* path preview only; no vehicle command/control nodes."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory as pkg
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include(package, filename, arguments):
    path = os.path.join(pkg(package), 'launch', filename)
    if not os.path.isfile(path):
        raise FileNotFoundError(path)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path), launch_arguments=arguments.items())


def generate_launch_description():
    defaults = {'map': str(Path.home() / 'scout_ws/maps/fastlio_nav_02.yaml'),
                'interface': 'can0', 'rviz': 'true', 'use_sim_time': 'false',
                'start_lidar': 'true', 'start_odometry': 'true',
                'clearance_radius': '0.47', 'lidar_x': '0.130', 'lidar_y': '0.0',
                'lidar_z': '0.326', 'lidar_pitch': '0.523599', 'lidar_yaw': '0.0'}
    config = LaunchConfiguration
    localization_args = {k: config(k) for k in defaults if k not in ('rviz', 'clearance_radius')}
    localization_args['rviz'] = 'false'
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()],
        include('scout_bringup', 'localization_system.launch.py', localization_args),
        include('scout_planning', 'global_planner.launch.py',
                {k: config(k) for k in ('rviz', 'use_sim_time', 'clearance_radius')}),
    ])
