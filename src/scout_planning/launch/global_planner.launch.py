"""Path preview against an existing /map and localization TF; never drives."""
import os
from ament_index_python.packages import get_package_share_directory as pkg
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = pkg('scout_planning')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('clearance_radius', default_value='0.47'),
        Node(package='scout_planning', executable='astar_node', name='scout_astar', output='screen',
             parameters=[os.path.join(share, 'config', 'planner.yaml'), {
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                 'clearance_radius': ParameterValue(LaunchConfiguration('clearance_radius'), value_type=float)}]),
        Node(package='rviz2', executable='rviz2', name='rviz_path_preview', output='screen',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', os.path.join(share, 'config', 'planning.rviz')]),
    ])
