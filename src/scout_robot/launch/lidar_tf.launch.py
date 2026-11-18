from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Standalone alternative only: do not run together with the model lidar TF.
    # base_link at axle height 0.079 m; lidar ground height 0.405 m.
    xyz = [LaunchConfiguration(k) for k in ('x','y','z')]
    return LaunchDescription([
        DeclareLaunchArgument('x', default_value='0.130'),
        DeclareLaunchArgument('y', default_value='0.00'),
        DeclareLaunchArgument('z', default_value='0.326'),
        DeclareLaunchArgument('pitch', default_value='0.523599'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='base_to_livox_level', arguments=[
                '--x',xyz[0],'--y',xyz[1],'--z',xyz[2],
                '--roll','0','--pitch','0','--yaw','0',
                '--frame-id','base_link','--child-frame-id','livox_level']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='level_to_livox_frame', arguments=[
                '--x','0','--y','0','--z','0',
                '--roll','0','--pitch',LaunchConfiguration('pitch'),
                '--yaw',LaunchConfiguration('yaw'),
                '--frame-id','livox_level','--child-frame-id','livox_frame'])
    ])
