# GraspSort MoveIt 2 launch: move_group (OMPL default RRTConnect + Pilz pipeline, D-06) and an
# optional RViz with the MotionPlanning panel. Included by sim.launch.py; can also run alone
# against an already running sim (same robot_xacro / controllers_file / xacro_args).
# Parameter layout follows moveit_configs_utils (MoveIt 2.5.10) and ur_moveit_config 2.14.
import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

PKG = 'graspsort_bringup'
DEFAULT_ROBOT_XACRO = ('graspsort_gazebo', 'urdf', 'graspsort_robot.urdf.xacro')


def _load_yaml(path):
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def _resolve(context, arg, default_parts):
    """Value of launch arg `arg`, or <share of default_parts[0]>/<rest> when it is empty."""
    value = LaunchConfiguration(arg).perform(context)
    if value:
        return value
    share = FindPackageShare(default_parts[0]).perform(context)
    return os.path.join(share, *default_parts[1:])


def robot_description_command(robot_xacro, controllers_file, xacro_args):
    return ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', robot_xacro,
                 ' simulation_controllers:=', controllers_file, ' ', xacro_args]),
        value_type=str)


def moveit_parameters(moveit_dir, robot_description, use_sim_time, srdf_file):
    """Parameter dicts for move_group (and the RViz MotionPlanning plugin)."""
    with open(srdf_file, 'r', encoding='utf-8') as f:
        srdf = f.read()
    pipelines = ['ompl', 'pilz_industrial_motion_planner']
    planning = {
        'planning_pipelines': pipelines,
        'default_planning_pipeline': 'ompl',
    }
    for p in pipelines:
        planning[p] = _load_yaml(os.path.join(moveit_dir, p + '_planning.yaml'))
    # joint_limits.yaml and pilz_cartesian_limits.yaml both live under robot_description_planning
    limits = _load_yaml(os.path.join(moveit_dir, 'joint_limits.yaml'))
    limits.update(_load_yaml(os.path.join(moveit_dir, 'pilz_cartesian_limits.yaml')))
    return [
        {'robot_description': robot_description},
        {'robot_description_semantic': srdf},
        {'robot_description_kinematics': _load_yaml(os.path.join(moveit_dir, 'kinematics.yaml'))},
        {'robot_description_planning': limits},
        planning,
        _load_yaml(os.path.join(moveit_dir, 'moveit_controllers.yaml')),
        {
            'publish_robot_description_semantic': True,
            'publish_planning_scene': True,
            'publish_geometry_updates': True,
            'publish_state_updates': True,
            'publish_transforms_updates': True,
            'monitor_dynamics': False,
        },
        {'use_sim_time': use_sim_time},
    ]


def launch_setup(context):
    share = FindPackageShare(PKG).perform(context)
    moveit_dir = os.path.join(share, 'config', 'moveit')
    robot_xacro = _resolve(context, 'robot_xacro', DEFAULT_ROBOT_XACRO)
    controllers_file = _resolve(context, 'controllers_file', (PKG, 'config', 'controllers.yaml'))
    rviz_config = _resolve(context, 'rviz_config', (PKG, 'rviz', 'graspsort.rviz'))
    srdf_file = _resolve(context, 'srdf_file', (PKG, 'config', 'moveit', 'graspsort.srdf'))
    xacro_args = LaunchConfiguration('xacro_args').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'

    desc = robot_description_command(robot_xacro, controllers_file, xacro_args)
    params = moveit_parameters(moveit_dir, desc, use_sim_time, srdf_file)

    move_group = Node(
        package='moveit_ros_move_group', executable='move_group', output='screen',
        parameters=params)
    # RViz needs the description, SRDF, kinematics and planning pipelines for the MotionPlanning panel.
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='log',
        arguments=['-d', rviz_config],
        parameters=params[:5] + [{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')))
    return [move_group, rviz]


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        DeclareLaunchArgument('robot_xacro', default_value='',
                              description='Robot xacro; empty = graspsort_gazebo/urdf/'
                                          'graspsort_robot.urdf.xacro'),
        DeclareLaunchArgument('controllers_file', default_value='',
                              description='ros2_control yaml; empty = graspsort_bringup/config/'
                                          'controllers.yaml'),
        DeclareLaunchArgument('xacro_args', default_value='',
                              description='Extra xacro args, "name:=value name2:=value2"'),
        DeclareLaunchArgument('srdf_file', default_value='',
                              description='SRDF; empty = graspsort_bringup/config/moveit/'
                                          'graspsort.srdf'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false', description='Start RViz'),
        DeclareLaunchArgument('rviz_config', default_value='',
                              description='RViz config; empty = graspsort_bringup/rviz/'
                                          'graspsort.rviz'),
        OpaqueFunction(function=launch_setup),
    ])
