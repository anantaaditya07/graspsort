"""GraspSort Phase 4: one scripted pick-and-place (pick_place_test, architecture 7.5).

Needs the running sim with move_group (graspsort_bringup sim.launch.py), perception
(graspsort_perception perception.launch.py) and the scene manager (graspsort_scene scene.launch.py).
Bin geometry comes from graspsort_gazebo's world_layout.yaml (single source of truth), the
tunables from config/pick_place.yaml.

Example:
  ros2 launch graspsort_manipulation pick_place_test.launch.py target_class:=bottle
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, RegisterEventHandler,
                            SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory('graspsort_manipulation'), 'config',
                                  'pick_place.yaml')
    default_layout = os.path.join(get_package_share_directory('graspsort_gazebo'), 'config',
                                  'world_layout.yaml')
    kin_file = os.path.join(get_package_share_directory('graspsort_bringup'), 'config', 'moveit',
                            'kinematics.yaml')
    with open(kin_file, 'r', encoding='utf-8') as f:
        kinematics = {'robot_description_kinematics': yaml.safe_load(f)}
    node = Node(package='graspsort_manipulation', executable='pick_place_test',
                name='pick_place_test', output='screen',
                parameters=[LaunchConfiguration('layout_file'), LaunchConfiguration('params_file'),
                            kinematics,
                            {'use_sim_time': LaunchConfiguration('use_sim_time'),
                             'target_class': LaunchConfiguration('target_class')}])
    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        DeclareLaunchArgument('target_class', default_value='sports ball',
                              description='"sports ball" or "bottle"'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('layout_file', default_value=default_layout),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        node,
        RegisterEventHandler(OnProcessExit(target_action=node,
                                           on_exit=[EmitEvent(event=Shutdown())])),
    ])
