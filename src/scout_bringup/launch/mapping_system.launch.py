"""Mid-360 FAST-LIO2 mapping; drive with the handheld RC transmitter."""
import os
from ament_index_python.packages import get_package_share_directory as pkg
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def load(package, filename, parameters=None, directory='launch', condition=None):
    path = os.path.join(pkg(package), directory, filename)
    if not os.path.isfile(path):
        raise FileNotFoundError(f'Launch file not installed: {path}')
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
        launch_arguments=(parameters or {}).items(), condition=condition)


def generate_launch_description():
    defaults = {'x': '0.130', 'y': '0.0', 'z': '0.326',
                'lidar_pitch': '0.523599', 'lidar_yaw': '0.0',
                'use_sim_time': 'false', 'start_lidar': 'true'}
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()],
        load('livox_ros_driver2', 'msg_MID360_launch.py', directory='launch_ROS2',
             condition=IfCondition(LaunchConfiguration('start_lidar'))),
        load('scout_description', 'scout_base_description.launch.py', {
            'lidar_x': LaunchConfiguration('x'), 'lidar_y': LaunchConfiguration('y'),
            'lidar_z': LaunchConfiguration('z'), 'lidar_pitch': LaunchConfiguration('lidar_pitch'),
            'lidar_yaw': LaunchConfiguration('lidar_yaw'),
            'use_sim_time': LaunchConfiguration('use_sim_time')}),
        load('scout_mapping', 'slam.launch.py', {
            **{k: LaunchConfiguration(k) for k in ('x', 'y', 'z', 'lidar_pitch', 'lidar_yaw', 'use_sim_time')},
        }),
    ])
