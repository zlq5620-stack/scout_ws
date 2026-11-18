"""Map server + AMCL only; consumes /scan and odom TF, never commands motion."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory as pkg
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def check_map(context):
    path = Path(LaunchConfiguration('map').perform(context)).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f'Map YAML not found: {path}')
    config = yaml.safe_load(path.read_text())
    image = Path(config['image'])
    if not image.is_absolute():
        image = path.parent / image
    if not image.is_file():
        raise FileNotFoundError(f'Map image not found: {image}')
    if not (float(config['resolution']) > 0 and 0 < float(config['free_thresh']) < float(config['occupied_thresh']) < 1):
        raise ValueError('Invalid map resolution or occupancy thresholds')
    return []


def generate_launch_description():
    config = str(Path(pkg('scout_localization')) / 'config/amcl_fastlio_grid.yaml')
    sim = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=str(Path.home() / 'scout_ws/maps/fastlio_nav_02.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=check_map),
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
             parameters=[{'yaml_filename': LaunchConfiguration('map'), 'use_sim_time': sim}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
             parameters=[config, {'use_sim_time': sim}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen', parameters=[{
                 'use_sim_time': sim, 'autostart': True, 'node_names': ['map_server', 'amcl']}]),
    ])
