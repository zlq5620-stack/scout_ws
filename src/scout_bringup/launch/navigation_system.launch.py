"""Compatibility alias for the explicit-enable autonomy system (disabled by default)."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hardware_enabled',default_value='false'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('scout_bringup'),'launch','autonomy_system.launch.py')),
            launch_arguments={'hardware_enabled':LaunchConfiguration('hardware_enabled')}.items()),
    ])
