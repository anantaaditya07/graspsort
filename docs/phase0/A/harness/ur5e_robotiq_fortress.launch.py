# Phase 0 scratch launch - UR5e + Robotiq 2F-85 on Gazebo Fortress, server only. Set IGN_PARTITION for isolation.
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

HERE = os.path.dirname(os.path.realpath(__file__))


def generate_launch_description():
    xacro_file = os.path.join(HERE, 'ur5e_robotiq_fortress.urdf.xacro')
    ctrl_file = os.path.join(HERE, 'ur5e_robotiq_controllers.yaml')
    desc = Command([FindExecutable(name='xacro'), ' ', xacro_file, ' simulation_controllers:=', ctrl_file])
    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='both',
             parameters=[{'use_sim_time': True, 'robot_description': ParameterValue(desc, value_type=str)}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([FindPackageShare('ros_gz_sim'), '/launch/gz_sim.launch.py']),
            launch_arguments={'gz_args': ' -s -r -v 3 empty.sdf'}.items()),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-topic', 'robot_description', '-name', 'ur', '-allow_renaming', 'true']),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock']),
        *[Node(package='controller_manager', executable='spawner', arguments=[c, '-c', '/controller_manager'])
          for c in ('joint_state_broadcaster', 'joint_trajectory_controller', 'gripper_controller')],
    ])
