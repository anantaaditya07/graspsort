# GraspSort simulation bringup (Gazebo Classic 11, D-01): gzserver (+ optional gzclient), the robot
# (UR5e + custom gripper + fixed RGB-D camera, D-02/D-05) from graspsort_gazebo, ros2_control
# controllers, static TF world -> table / bin_<class> from world_layout.yaml, optional move_group
# and RViz.
#
# Adapted from ur_simulation_gazebo/launch/ur_sim_control.launch.py and ur_sim_moveit.launch.py,
# Universal_Robots_ROS2_Gazebo_Simulation, branch humble, SHA 34a041738bd8f736f6f85e8a1c73dd91396d7ee1
# (BSD-3-Clause). Per D-01 that repository is neither fetched nor a dependency.
#
# Headless example (Phase 1 verification):
#   ros2 launch graspsort_bringup sim.launch.py
#   ros2 launch graspsort_bringup sim.launch.py gui:=true rviz:=true
import os

import yaml
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

PKG = 'graspsort_bringup'
POSE_KEYS = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')


def _resolve(context, arg, default_parts):
    """Value of launch arg `arg`, or <share of default_parts[0]>/<rest> when it is empty."""
    value = LaunchConfiguration(arg).perform(context)
    if value:
        return value
    share = FindPackageShare(default_parts[0]).perform(context)
    return os.path.join(share, *default_parts[1:])


def _unwrap_ros_parameters(data):
    """Accept both a plain yaml and a ROS params file (<node or /**>: ros__parameters: ...)."""
    while isinstance(data, dict) and len(data) == 1:
        (key, value), = data.items()
        if key == 'ros__parameters' or (isinstance(value, dict) and 'ros__parameters' in value):
            data = value['ros__parameters'] if key != 'ros__parameters' else value
        else:
            break
    return data


def parse_pose(pose):
    """Pose as (x, y, z, roll, pitch, yaw) from a dict with those keys (missing = 0), a dict with
    position/xyz + rpy/orientation lists, or a list of 3 or 6 numbers."""
    if isinstance(pose, dict):
        if any(k in pose for k in POSE_KEYS):
            return tuple(float(pose.get(k, 0.0)) for k in POSE_KEYS)
        xyz = pose.get('position', pose.get('xyz', [0.0, 0.0, 0.0]))
        rpy = pose.get('rpy', pose.get('orientation', [0.0, 0.0, 0.0]))
        return tuple(float(v) for v in list(xyz) + list(rpy))
    values = [float(v) for v in pose]
    if len(values) == 3:
        values += [0.0, 0.0, 0.0]
    if len(values) != 6:
        raise ValueError(f'pose needs 3 or 6 values, got {pose}')
    return tuple(values)


def _entry_pose(entry):
    return parse_pose(entry['pose'] if isinstance(entry, dict) and 'pose' in entry else entry)


def layout_frames(layout):
    """[(child_frame, (x, y, z, roll, pitch, yaw))] for the table and every bin in the layout.
    Table: entry `table` (frame name from table.frame, default "table"). Bins: a mapping with
    `names: [...]` and one entry per name (graspsort_gazebo world_layout.yaml), a mapping
    name -> pose entry, or a list of {name, pose}. The frame name is the bin name."""
    layout = _unwrap_ros_parameters(layout)
    table = layout['table']
    frames = [(str(table.get('frame', 'table')) if isinstance(table, dict) else 'table',
               _entry_pose(table))]
    bins = layout['bins']
    if isinstance(bins, dict):
        names = bins.get('names') or [k for k, v in bins.items() if isinstance(v, (dict, list))]
        bins = [{'name': n, 'pose': bins[n]} for n in names]
    for b in bins:
        frames.append((str(b['name']), _entry_pose(b['pose'] if 'pose' in b else b)))
    return frames


def layout_xacro_args(layout):
    """Robot base and camera pose from the layout as robot xacro args (base_x.. base_yaw,
    camera_x.. camera_yaw), so world_layout.yaml stays the single source of truth."""
    layout = _unwrap_ros_parameters(layout)
    args = []
    for key, prefix, names in (('robot_base', 'base', ('x', 'y', 'z', 'yaw')),
                               ('camera', 'camera', POSE_KEYS)):
        entry = layout.get(key)
        if isinstance(entry, dict):
            args += [f'{prefix}_{n}:={float(entry[n])}' for n in names if n in entry]
    return ' '.join(args)


def static_tf_nodes(layout, use_sim_time):
    frames = layout_frames(layout)
    nodes = []
    for child, (x, y, z, roll, pitch, yaw) in frames:
        args = ['--x', str(x), '--y', str(y), '--z', str(z),
                '--roll', str(roll), '--pitch', str(pitch), '--yaw', str(yaw),
                '--frame-id', 'world', '--child-frame-id', child]
        nodes.append(Node(package='tf2_ros', executable='static_transform_publisher',
                          name=f'static_tf_{child}', output='log', arguments=args,
                          parameters=[{'use_sim_time': use_sim_time}]))
    return nodes


def launch_setup(context):
    share = FindPackageShare(PKG).perform(context)
    robot_xacro = _resolve(context, 'robot_xacro',
                           ('graspsort_gazebo', 'urdf', 'graspsort_robot.urdf.xacro'))
    world = _resolve(context, 'world', ('graspsort_gazebo', 'worlds', 'graspsort.world'))
    layout_file = _resolve(context, 'layout_file',
                           ('graspsort_gazebo', 'config', 'world_layout.yaml'))
    controllers_file = _resolve(context, 'controllers_file', (PKG, 'config', 'controllers.yaml'))
    with open(layout_file, 'r', encoding='utf-8') as f:
        layout = yaml.safe_load(f)
    xacro_args = LaunchConfiguration('xacro_args').perform(context)
    if LaunchConfiguration('layout_xacro_args').perform(context).lower() == 'true':
        xacro_args = (layout_xacro_args(layout) + ' ' + xacro_args).strip()
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    cm_timeout = LaunchConfiguration('controller_manager_timeout').perform(context)

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', robot_xacro,
                 ' simulation_controllers:=', controllers_file, ' ', xacro_args]),
        value_type=str)

    gazebo_launch = os.path.join(FindPackageShare('gazebo_ros').perform(context), 'launch')
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_launch, 'gzserver.launch.py')),
        launch_arguments={'world': world, 'verbose': LaunchConfiguration('verbose')}.items())
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_launch, 'gzclient.launch.py')),
        condition=IfCondition(LaunchConfiguration('gui')))

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='both',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}])
    spawn_robot = Node(
        package='gazebo_ros', executable='spawn_entity.py', name='spawn_ur', output='screen',
        arguments=['-entity', LaunchConfiguration('entity'), '-topic', 'robot_description',
                   '-timeout', LaunchConfiguration('spawn_timeout')])

    def spawner(name):
        return Node(package='controller_manager', executable='spawner', output='screen',
                    arguments=[name, '--controller-manager', '/controller_manager',
                               '--controller-manager-timeout', cm_timeout])

    jsb = spawner('joint_state_broadcaster')
    after_jsb = RegisterEventHandler(OnProcessExit(
        target_action=jsb,
        on_exit=[spawner('joint_trajectory_controller'), spawner('gripper_controller')]))

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'moveit.launch.py')),
        launch_arguments={'robot_xacro': robot_xacro, 'controllers_file': controllers_file,
                          'xacro_args': xacro_args,
                          'use_sim_time': LaunchConfiguration('use_sim_time'),
                          'rviz': LaunchConfiguration('rviz'),
                          'rviz_config': LaunchConfiguration('rviz_config')}.items(),
        condition=IfCondition(LaunchConfiguration('moveit')))

    return [gzserver, gzclient, robot_state_publisher, spawn_robot, jsb, after_jsb, moveit,
            *static_tf_nodes(layout, use_sim_time)]


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        DeclareLaunchArgument('gui', default_value='false', description='Start gzclient'),
        DeclareLaunchArgument('verbose', default_value='false', description='gzserver verbose'),
        DeclareLaunchArgument('world', default_value='',
                              description='World file; empty = graspsort_gazebo/worlds/'
                                          'graspsort.world'),
        DeclareLaunchArgument('robot_xacro', default_value='',
                              description='Robot xacro; empty = graspsort_gazebo/urdf/'
                                          'graspsort_robot.urdf.xacro'),
        DeclareLaunchArgument('xacro_args', default_value='',
                              description='Extra xacro args, "name:=value name2:=value2"'),
        DeclareLaunchArgument('controllers_file', default_value='',
                              description='ros2_control yaml; empty = graspsort_bringup/config/'
                                          'controllers.yaml'),
        DeclareLaunchArgument('layout_file', default_value='',
                              description='Static TF source; empty = graspsort_gazebo/config/'
                                          'world_layout.yaml'),
        DeclareLaunchArgument('layout_xacro_args', default_value='true',
                              description='Pass robot_base and camera pose from the layout file '
                                          'as xacro args (base_*, camera_*)'),
        DeclareLaunchArgument('entity', default_value='ur', description='Gazebo entity name'),
        DeclareLaunchArgument('spawn_timeout', default_value='60.0',
                              description='spawn_entity wait for /spawn_entity [s]'),
        DeclareLaunchArgument('controller_manager_timeout', default_value='60',
                              description='Spawner wait for the controller manager [s]'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('moveit', default_value='true', description='Start move_group'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Start RViz (needs moveit:=true)'),
        DeclareLaunchArgument('rviz_config', default_value=''),
        OpaqueFunction(function=launch_setup),
    ])
