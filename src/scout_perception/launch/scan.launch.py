from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 雷达原点离地 0.405 m；选取离地约 0.105～0.705 m 的点用于二维建图。
    # 这是平地建图初值，不代表完整的低矮障碍物检测。
    return LaunchDescription([
        Node(package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
             name='cloud_to_scan', output='screen',
             remappings=[('cloud_in','/livox/lidar'),('scan','/scan')],
             parameters=[{
                'target_frame':'livox_level',
                'min_height':-0.30, 'max_height':0.30,
                'angle_min':-3.141592653589793,'angle_max':3.141592653589793,
                'angle_increment':0.017453293,
                'range_min':0.35,'range_max':12.0,
                'scan_time':0.10,'use_inf':True,'use_sim_time':False
             }])
    ])
