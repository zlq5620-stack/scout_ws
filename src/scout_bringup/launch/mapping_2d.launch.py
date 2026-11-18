import os
from ament_index_python.packages import get_package_share_directory as pkg
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource as Source
from launch.substitutions import LaunchConfiguration

def load(package, filename, parameters=None, directory='launch'):
    path = os.path.join(pkg(package), directory, filename)
    if not os.path.isfile(path):
        raise FileNotFoundError(f'Launch file not installed: {path}')
    return IncludeLaunchDescription(Source(path),
        launch_arguments=(parameters or {}).items())

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('x',default_value='0.130'),
        DeclareLaunchArgument('y',default_value='0.00'),
        DeclareLaunchArgument('z',default_value='0.326'),
        load('scout_base','scout_base.launch.py',{'is_scout_mini':'true','is_omni_wheel':'false','port_name':'can0'}),
        load('livox_ros_driver2','rviz_MID360_launch.py', directory='launch_ROS2'),
        load('scout_description','scout_base_description.launch.py',{
            'lidar_x':LaunchConfiguration('x'),
            'lidar_y':LaunchConfiguration('y'),
            'lidar_z':LaunchConfiguration('z')}),
        load('scout_perception','scan.launch.py'),
        load('scout_mapping','slam_2d.launch.py')
    ])
