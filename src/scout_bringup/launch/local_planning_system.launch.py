"""Localization + global/local planning previews; no chassis command publisher."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory as pkg
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def load(package,filename,arguments):
    path=os.path.join(pkg(package),'launch',filename)
    if not os.path.isfile(path):raise FileNotFoundError(path)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path),launch_arguments=arguments.items())


def generate_launch_description():
    defaults={'map':str(Path.home()/'scout_ws/maps/fastlio_nav_02.yaml'),'interface':'can0',
              'execution_connected':'false','use_sim_time':'false','rviz':'true','start_lidar':'true','start_odometry':'true',
              'clearance_radius':'0.425','lidar_x':'0.130','lidar_y':'0.0','lidar_z':'0.326',
              'lidar_pitch':'0.523599','lidar_yaw':'0.0'}
    config=LaunchConfiguration
    planning={k:config(k) for k in defaults if k not in ('rviz','execution_connected')};planning['rviz']='false'
    return LaunchDescription([
        *[DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()],
        # Scope rviz=false to the background stack; do not overwrite the parent flag.
        GroupAction(actions=[load('scout_bringup','planning_system.launch.py',planning)], scoped=True),
        GroupAction(actions=[load('scout_avoidance','local_preview.launch.py',
                    {k:config(k) for k in ('rviz','use_sim_time','execution_connected')})], scoped=True),
    ])
