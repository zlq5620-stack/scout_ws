import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    config=os.path.join(get_package_share_directory('scout_mapping'),'config','slam.yaml')
    upstream=os.path.join(get_package_share_directory('slam_toolbox'),'launch','online_async_launch.py')
    return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(upstream),
        launch_arguments={'slam_params_file':config,'use_sim_time':'false'}.items())])
