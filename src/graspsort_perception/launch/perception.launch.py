"""GraspSort perception: object_detector_node (7.1) and object_localizer_node (7.2, D-12).

object_detector_node:   /camera/color/image_raw -> /detections, /detections/image, /metrics
object_localizer_node:  /detections + /camera/depth/image_raw -> /objects_3d (world frame)

Parameters come from config/perception.yaml (override the file with params_file:=...). The
localizer's table_height and world_frame are read from graspsort_gazebo's world_layout.yaml,
the single source of truth for the scene geometry (override with layout_file:=...).
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def layout_params(layout_file):
    """table_height and world_frame for the localizer from world_layout.yaml."""
    with open(layout_file, 'r', encoding='utf-8') as f:
        p = yaml.safe_load(f)['/**']['ros__parameters']
    return {'table_height': float(p['table']['z']), 'world_frame': p['world_frame']}


def nodes(context):
    params_file = LaunchConfiguration('params_file').perform(context)
    layout_file = LaunchConfiguration('layout_file').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    sim = {'use_sim_time': use_sim_time}
    return [
        Node(package='graspsort_perception', executable='object_detector_node',
             name='object_detector_node', output='screen', parameters=[params_file, sim]),
        Node(package='graspsort_perception', executable='object_localizer_node',
             name='object_localizer_node', output='screen',
             parameters=[params_file, layout_params(layout_file), sim]),
    ]


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory('graspsort_perception'), 'config',
                                  'perception.yaml')
    default_layout = os.path.join(get_package_share_directory('graspsort_gazebo'), 'config',
                                  'world_layout.yaml')
    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='Perception parameter file'),
        DeclareLaunchArgument('layout_file', default_value=default_layout,
                              description='World layout (table height, world frame)'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use the Gazebo /clock'),
        OpaqueFunction(function=nodes),
    ])
