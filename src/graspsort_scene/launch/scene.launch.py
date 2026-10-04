"""GraspSort scene_manager_node (architecture 7.3).

Fixed collision objects come from graspsort_gazebo's world_layout.yaml (single source of truth;
override with layout_file:=...), the tunables from config/scene.yaml (params_file:=...).
Needs move_group (graspsort_bringup sim.launch.py) and /objects_3d (graspsort_perception).
Freeze the scene with: ros2 service call /scene_manager/freeze std_srvs/srv/SetBool "{data: true}"
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory('graspsort_scene'), 'config',
                                  'scene.yaml')
    default_layout = os.path.join(get_package_share_directory('graspsort_gazebo'), 'config',
                                  'world_layout.yaml')
    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='scene_manager parameter file'),
        DeclareLaunchArgument('layout_file', default_value=default_layout,
                              description='World layout (table, pedestal, bins)'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use the Gazebo /clock'),
        Node(package='graspsort_scene', executable='scene_manager_node', name='scene_manager',
             output='screen',
             parameters=[LaunchConfiguration('layout_file'), LaunchConfiguration('params_file'),
                         {'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
