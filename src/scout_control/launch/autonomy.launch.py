"""Add explicit-enable motion backend to an existing local planning stack."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hardware_enabled',default_value='false'),
        DeclareLaunchArgument('interface',default_value='can0'),
        Node(package='scout_control',executable='autonomy_controller',name='scout_autonomy_controller',output='screen',
             parameters=[os.path.join(get_package_share_directory('scout_control'),'config','autonomy.yaml'),{
                 'hardware_enabled':ParameterValue(LaunchConfiguration('hardware_enabled'),value_type=bool),
                 'interface':LaunchConfiguration('interface')}])])
