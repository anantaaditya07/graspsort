"""GraspSort Phase 5: sort_task_node, the SortObjects action server on /sort_objects (7.5).

Needs the running sim with move_group (graspsort_bringup sim.launch.py), perception
(graspsort_perception perception.launch.py) and the scene manager (graspsort_scene scene.launch.py).
Parameters: graspsort_gazebo's world_layout.yaml (bin geometry), config/pick_place.yaml
(pick-and-place tunables), config/sort_task.yaml (sort loop), graspsort_bringup's kinematics.yaml
(IK for the free moves).

Example:
  ros2 launch graspsort_manipulation sort_task.launch.py metrics_log_path:=/tmp/sort.jsonl
  ros2 action send_goal --feedback /sort_objects graspsort_msgs/action/SortObjects "{classes: []}"
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('graspsort_manipulation')
    default_pick = os.path.join(share, 'config', 'pick_place.yaml')
    default_sort = os.path.join(share, 'config', 'sort_task.yaml')
    default_layout = os.path.join(get_package_share_directory('graspsort_gazebo'), 'config',
                                  'world_layout.yaml')
    kin_file = os.path.join(get_package_share_directory('graspsort_bringup'), 'config', 'moveit',
                            'kinematics.yaml')
    with open(kin_file, 'r', encoding='utf-8') as f:
        kinematics = {'robot_description_kinematics': yaml.safe_load(f)}
    node = Node(package='graspsort_manipulation', executable='sort_task_node',
                name='sort_task_node', output='screen',
                parameters=[LaunchConfiguration('layout_file'),
                            LaunchConfiguration('pick_params_file'),
                            LaunchConfiguration('params_file'), kinematics,
                            {'use_sim_time': LaunchConfiguration('use_sim_time'),
                             'metrics_log_path': LaunchConfiguration('metrics_log_path')}])
    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        DeclareLaunchArgument('params_file', default_value=default_sort,
                              description='sort loop parameters'),
        DeclareLaunchArgument('pick_params_file', default_value=default_pick,
                              description='pick-and-place parameters'),
        DeclareLaunchArgument('layout_file', default_value=default_layout),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('metrics_log_path', default_value='',
                              description='append one JSON line per attempt ("" = off)'),
        node,
    ])
