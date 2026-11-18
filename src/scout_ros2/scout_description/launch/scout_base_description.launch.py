"""Publish the measured Scout Mini model and Mid-360 mounting transforms."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    defaults = {
        'publish_lidar_tf': 'true',
        'lidar_x': '0.130',
        'lidar_y': '0.0',
        'lidar_z': '0.326',
        'lidar_pitch': '0.523599',
        'lidar_yaw': '0.0',
    }
    model = PathJoinSubstitution([
        FindPackageShare('scout_description'), 'urdf', 'scout_mini.xacro'
    ])
    command = [FindExecutable(name='xacro'), ' ', model]
    for name in defaults:
        command.extend([' ', name, ':=', LaunchConfiguration(name)])
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        *[DeclareLaunchArgument(name, default_value=value)
          for name, value in defaults.items()],
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                'robot_description': ParameterValue(Command(command), value_type=str),
            }],
        ),
    ])
