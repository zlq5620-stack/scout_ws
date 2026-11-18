"""FAST-LIO2 + measured Scout Mini mounting TF, without wheel odometry."""
import os
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def mounting_transforms(config, xyz, pitch, yaw, base_height):
    """T_base_imu = T_base_lidar * inverse(T_imu_lidar).

    Upstream initializes camera_init at the initial IMU pose (not gravity aligned).
    Starting on level ground, map aligns with the initial vehicle heading.
    """
    imu_lidar = np.eye(4)
    imu_lidar[:3, :3] = np.array(config['mapping']['extrinsic_R']).reshape(3, 3)
    imu_lidar[:3, 3] = config['mapping']['extrinsic_T']
    base_lidar = np.eye(4)
    base_lidar[:3, :3] = Rotation.from_euler('xyz', [0.0, pitch, yaw]).as_matrix()
    base_lidar[:3, 3] = xyz
    base_imu = base_lidar @ np.linalg.inv(imu_lidar)
    map_initial_base = np.eye(4)
    map_initial_base[2, 3] = base_height
    return map_initial_base @ base_imu, np.linalg.inv(base_imu)


def static_node(name, parent, child, transform):
    translation = transform[:3, 3]
    quat = Rotation.from_matrix(transform[:3, :3]).as_quat()
    args = ['--frame-id', parent, '--child-frame-id', child]
    for key, value in zip(('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'), [*translation, *quat]):
        args.extend(['--' + key, str(float(value))])
    return Node(package='tf2_ros', executable='static_transform_publisher',
                name=name, arguments=args, output='screen')


def launch_nodes(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    config_path = value('fastlio_config')
    with open(config_path) as stream:
        config = yaml.safe_load(stream)['/**']['ros__parameters']
    if config['mapping']['extrinsic_est_en']:
        raise ValueError('Fixed body-to-base TF requires mapping.extrinsic_est_en=false')
    map_path = Path(value('map_save_path')).expanduser().resolve()
    map_path.parent.mkdir(parents=True, exist_ok=True)
    map_initial_imu, imu_base = mounting_transforms(
        config, [float(value(k)) for k in ('x', 'y', 'z')],
        float(value('lidar_pitch')), float(value('lidar_yaw')), float(value('base_height')))
    return [
        static_node('map_to_fastlio_origin', 'map', 'camera_init', map_initial_imu),
        static_node('fastlio_body_to_base', 'body', 'base_link', imu_base),
        Node(package='fast_lio', executable='fastlio_mapping', name='fastlio_mapping',
             output='screen', parameters=[config_path, {
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                 'map_file_path': str(map_path),
             }]),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', os.path.join(get_package_share_directory('scout_mapping'), 'config', 'fastlio.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    defaults = {
        'x': '0.130', 'y': '0.0', 'z': '0.326',
        'lidar_pitch': '0.523599', 'lidar_yaw': '0.0', 'base_height': '0.079',
        'use_sim_time': 'false', 'rviz': 'true',
        'fastlio_config': os.path.join(get_package_share_directory('scout_mapping'), 'config', 'fastlio_mid360.yaml'),
        'map_save_path': str(Path.home() / 'scout_ws/maps/fastlio_map.pcd'),
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()],
        OpaqueFunction(function=launch_nodes),
    ])
