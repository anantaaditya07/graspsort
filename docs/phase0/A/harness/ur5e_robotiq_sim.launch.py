# Phase 0 scratch launch: UR5e + Robotiq 2F-85 in Gazebo Classic (headless), gazebo_ros2_control.
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
    xacro_file = os.environ.get('GS_XACRO', os.path.join(HERE, 'ur5e_robotiq.urdf.xacro'))
    ctrl_file = os.environ.get('GS_CONTROLLERS', os.path.join(HERE, 'ur5e_robotiq_controllers.yaml'))
    desc = ParameterValue(Command([FindExecutable(name='xacro'), ' ', xacro_file,
                                   ' simulation_controllers:=', ctrl_file]), value_type=str)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('gazebo_ros'), '/launch/gazebo.launch.py']),
        launch_arguments={'gui': 'false'}.items())
    spawners = [Node(package='controller_manager', executable='spawner',
                     arguments=[c, '-c', '/controller_manager'])
                for c in ('joint_state_broadcaster', 'joint_trajectory_controller', 'gripper_controller')]
    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='both',
             parameters=[{'use_sim_time': True, 'robot_description': desc}]),
        gazebo,
        Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
             arguments=['-entity', 'ur', '-topic', 'robot_description']),
        *spawners,
    ])
