"""Localization, footprint planning, local checks, and opt-in CAN execution."""
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory as pkg


def include(package,name,args):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(pkg(package),'launch',name)),launch_arguments=args.items())


def generate_launch_description():
    config=LaunchConfiguration
    defaults={'hardware_enabled':'false','interface':'can0','rviz':'true',
              'map':str(Path.home()/'scout_ws/maps/fastlio_nav_02.yaml')}
    return LaunchDescription([
        *[DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()],
        GroupAction(actions=[include('scout_bringup','local_planning_system.launch.py',{
            **{k:config(k) for k in ('interface','rviz','map')},
            'execution_connected':config('hardware_enabled')})],scoped=True),
        GroupAction(actions=[include('scout_control','autonomy.launch.py',{
            k:config(k) for k in ('hardware_enabled','interface')})],scoped=True),
    ])
