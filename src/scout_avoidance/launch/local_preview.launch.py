"""Add local obstacle/trajectory/velocity previews to an existing planning system."""
import os
from ament_index_python.packages import get_package_share_directory as pkg
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share=pkg('scout_avoidance')
    sim=ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool)
    parameters=[os.path.join(share,'config','local_preview.yaml'),{'use_sim_time':sim,'execution_connected':ParameterValue(LaunchConfiguration('execution_connected'),value_type=bool)}]
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('execution_connected',default_value='false'),
        Node(package='pointcloud_to_laserscan',executable='pointcloud_to_laserscan_node',
             name='local_obstacle_cloud_to_scan',output='screen',
             remappings=[('cloud_in','/livox/lidar'),('scan','/local_scan')],
             parameters=[os.path.join(share,'config','local_scan.yaml'),{'use_sim_time':sim,'execution_connected':ParameterValue(LaunchConfiguration('execution_connected'),value_type=bool)}]),
        Node(package='scout_avoidance',executable='dwa_preview',name='dwa_preview',output='screen',parameters=parameters),
        Node(package='scout_avoidance',executable='candidate_guard_preview',name='candidate_guard_preview',output='screen',parameters=parameters),
        Node(package='rviz2',executable='rviz2',name='rviz_local_preview',output='screen',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d',os.path.join(share,'config','local_preview.rviz')]),
    ])
