"""Phase 0 / B harness: gzserver + UR5e (ur_description sim_gazebo) + JTC. No gzclient."""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler,
                            SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    world = LaunchConfiguration('world')
    controllers = LaunchConfiguration('controllers')
    description = ParameterValue(Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([FindPackageShare('ur_description'), 'urdf', 'ur.urdf.xacro']),
        ' ur_type:=ur5e name:=ur sim_gazebo:=true simulation_controllers:=', controllers]),
        value_type=str)

    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world],
        output='screen')
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               output='screen',
               parameters=[{'robot_description': description, 'use_sim_time': True}])
    spawn = Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
                 arguments=['-topic', 'robot_description', '-entity', 'ur', '-timeout', '120'])
    jsb = Node(package='controller_manager', executable='spawner', output='screen',
               arguments=['joint_state_broadcaster', '--controller-manager-timeout', '120'])
    jtc = Node(package='controller_manager', executable='spawner', output='screen',
               arguments=['joint_trajectory_controller', '--controller-manager-timeout', '120'])

    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        DeclareLaunchArgument('world'),
        DeclareLaunchArgument('controllers'),
        gzserver, rsp, spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb, jtc])),
    ])
