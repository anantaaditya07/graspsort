"""GraspSort perception: object_detector_node (architecture 7.1).

Subscribes /camera/color/image_raw; publishes /detections, /detections/image and /metrics.
Parameters from config/perception.yaml (override the file with params_file:=...).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory('graspsort_perception'), 'config',
                                  'perception.yaml')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='Detector parameter file'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use the Gazebo /clock'),
        Node(package='graspsort_perception', executable='object_detector_node',
             name='object_detector_node', output='screen',
             parameters=[params_file, {'use_sim_time': use_sim_time}]),
    ])
