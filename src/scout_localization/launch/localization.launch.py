import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    config=os.path.join(get_package_share_directory('scout_localization'),'config','amcl.yaml')
    map_file=LaunchConfiguration('map')
    return LaunchDescription([
      DeclareLaunchArgument('map',default_value=os.path.expanduser('~/scout_ws/maps/scout_map.yaml')),
      Node(package='nav2_map_server',executable='map_server',name='map_server',output='screen',
           parameters=[{'yaml_filename':map_file,'use_sim_time':False}]),
      Node(package='nav2_amcl',executable='amcl',name='amcl',output='screen',parameters=[config]),
      Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',
           name='lifecycle_manager_localization',output='screen',
           parameters=[{'autostart':True,'node_names':['map_server','amcl'],'use_sim_time':False}])
    ])
