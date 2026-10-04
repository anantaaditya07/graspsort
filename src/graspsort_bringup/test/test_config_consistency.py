"""Consistency checks for graspsort_bringup (no ROS graph needed).

- every joint in controllers.yaml and moveit_controllers.yaml exists in the robot URDF
- every SRDF group joint/link exists in the URDF; named states set every active group joint
  within the URDF limits; disabled-collision links exist
- MoveIt planning config: OMPL default RRTConnect, Pilz pipeline, acceleration limits
- every launch file under launch/ sets RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
- sim.launch.py disables the online Gazebo model database (D-11)
- the static TF layout parser in sim.launch.py (and the real world_layout.yaml if present)

Robot xacro under test: $GRASPSORT_ROBOT_XACRO if set, else graspsort_gazebo's
urdf/graspsort_robot.urdf.xacro (installed, then source tree), else the Phase 0 harness copy
docs/phase0/A/harness/ur5e_parallel.urdf.xacro (no camera links).
"""
import ast
import importlib.util
import os
import subprocess
import xml.etree.ElementTree as ET

import pytest
import yaml

SRC = os.environ.get('GRASPSORT_BRINGUP_SRC',
                     os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
REPO = os.path.dirname(os.path.dirname(SRC))
CONFIG = os.path.join(SRC, 'config')
MOVEIT = os.path.join(CONFIG, 'moveit')
LAUNCH = os.path.join(SRC, 'launch')
HARNESS_XACRO = os.path.join(REPO, 'docs', 'phase0', 'A', 'harness', 'ur5e_parallel.urdf.xacro')
ROBOT_REL = ('urdf', 'graspsort_robot.urdf.xacro')
LAYOUT_REL = ('config', 'world_layout.yaml')
ARM_GROUP = 'ur_manipulator'
REQUIRED_STATES = {'ur_manipulator': ['home', 'ready'], 'gripper': ['open', 'closed']}
CAMERA_LINKS = ('camera_link', 'camera_color_optical_frame')


def _gazebo_file(rel):
    """Path of a graspsort_gazebo file: installed share first, then the source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('graspsort_gazebo'), *rel)
        if os.path.exists(path):
            return path
    except Exception:  # package not installed / not on the ament index
        pass
    path = os.path.join(REPO, 'src', 'graspsort_gazebo', *rel)
    return path if os.path.exists(path) else None


def _robot_xacro():
    return (os.environ.get('GRASPSORT_ROBOT_XACRO') or _gazebo_file(ROBOT_REL) or HARNESS_XACRO)


def _load(path):
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


@pytest.fixture(scope='module')
def urdf():
    xacro_file = _robot_xacro()
    out = subprocess.run(
        ['xacro', xacro_file, 'simulation_controllers:=' + os.path.join(CONFIG, 'controllers.yaml')],
        check=True, capture_output=True, text=True).stdout
    root = ET.fromstring(out)
    joints = {}
    for j in root.findall('joint'):
        limit = j.find('limit')
        joints[j.get('name')] = {
            'type': j.get('type'),
            'parent': j.find('parent').get('link'),
            'child': j.find('child').get('link'),
            'mimic': j.find('mimic') is not None,
            'lower': float(limit.get('lower')) if limit is not None and limit.get('lower') else None,
            'upper': float(limit.get('upper')) if limit is not None and limit.get('upper') else None,
        }
    links = {link.get('name') for link in root.findall('link')}
    return {'file': xacro_file, 'joints': joints, 'links': links, 'xml': out}


@pytest.fixture(scope='module')
def srdf():
    return ET.parse(os.path.join(MOVEIT, 'graspsort.srdf')).getroot()


def _movable(urdf, name):
    j = urdf['joints'].get(name)
    return j is not None and j['type'] != 'fixed'


def _active(urdf, name):
    return _movable(urdf, name) and not urdf['joints'][name]['mimic']


def _chain_joints(urdf, base, tip):
    by_child = {j['child']: n for n, j in urdf['joints'].items()}
    chain, link = [], tip
    while link != base:
        assert link in by_child, f'no chain {base} -> {tip} in URDF (stuck at {link})'
        name = by_child[link]
        chain.append(name)
        link = urdf['joints'][name]['parent']
    return list(reversed(chain))


def _group_joints(urdf, srdf, group_name):
    group = next(g for g in srdf.iter('group') if g.get('name') == group_name)
    joints = [j.get('name') for j in group.findall('joint')]
    for c in group.findall('chain'):
        joints += _chain_joints(urdf, c.get('base_link'), c.get('tip_link'))
    for sub in group.findall('group'):
        joints += _group_joints(urdf, srdf, sub.get('name'))
    return joints


def test_robot_xacro_has_no_colon_space_comments(urdf):
    # D-06: gazebo_ros2_control 0.4 passes the URDF through the rcl YAML parser.
    for comment in ET.fromstring(urdf['xml']).iter(ET.Comment):
        assert ': ' not in (comment.text or ''), comment.text


def test_controller_joints_exist(urdf):
    c = _load(os.path.join(CONFIG, 'controllers.yaml'))
    jtc = c['joint_trajectory_controller']['ros__parameters']['joints']
    assert jtc == ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                   'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    for j in jtc + [c['gripper_controller']['ros__parameters']['joint']]:
        assert _active(urdf, j), f'{j} missing / fixed / mimic in {urdf["file"]}'
    names = set(c['controller_manager']['ros__parameters']) - {'update_rate'}
    assert names == {'joint_state_broadcaster', 'joint_trajectory_controller', 'gripper_controller'}


def test_moveit_controllers_match_ros2_control(urdf):
    c = _load(os.path.join(CONFIG, 'controllers.yaml'))
    m = _load(os.path.join(MOVEIT, 'moveit_controllers.yaml'))['moveit_simple_controller_manager']
    assert m['joint_trajectory_controller']['joints'] == \
        c['joint_trajectory_controller']['ros__parameters']['joints']
    assert m['joint_trajectory_controller']['action_ns'] == 'follow_joint_trajectory'
    assert m['gripper_controller']['joints'] == [c['gripper_controller']['ros__parameters']['joint']]
    assert m['gripper_controller']['action_ns'] == 'gripper_cmd'
    assert m['gripper_controller']['type'] == 'GripperCommand'


def test_srdf_groups_exist_in_urdf(urdf, srdf):
    groups = {g.get('name') for g in srdf.iter('group')}
    assert {'ur_manipulator', 'gripper'} <= groups
    for g in groups:
        joints = _group_joints(urdf, srdf, g)
        assert joints, g
        for j in joints:
            assert j in urdf['joints'], f'group {g}: joint {j} not in URDF'
    arm = [j for j in _group_joints(urdf, srdf, ARM_GROUP) if _active(urdf, j)]
    c = _load(os.path.join(CONFIG, 'controllers.yaml'))
    assert arm == c['joint_trajectory_controller']['ros__parameters']['joints']
    for ee in srdf.iter('end_effector'):
        assert ee.get('parent_link') in urdf['links']
        assert ee.get('group') in groups and ee.get('parent_group') in groups


def test_named_states_complete_and_within_limits(urdf, srdf):
    states = {}
    for gs in srdf.iter('group_state'):
        states.setdefault(gs.get('group'), {})[gs.get('name')] = {
            j.get('name'): float(j.get('value')) for j in gs.iter('joint')}
    for group, required in REQUIRED_STATES.items():
        for s in required:
            assert s in states.get(group, {}), f'missing named state {group}/{s}'
    for group, by_name in states.items():
        active = [j for j in _group_joints(urdf, srdf, group) if _active(urdf, j)]
        for name, values in by_name.items():
            assert sorted(values) == sorted(active), f'{group}/{name} joints {sorted(values)}'
            for j, v in values.items():
                lo, hi = urdf['joints'][j]['lower'], urdf['joints'][j]['upper']
                if lo is not None and hi is not None:
                    assert lo <= v <= hi, f'{group}/{name}: {j}={v} outside [{lo}, {hi}]'


def test_disabled_collision_links_exist(urdf, srdf):
    pairs = [(d.get('link1'), d.get('link2')) for d in srdf.iter('disable_collisions')]
    assert pairs
    has_camera = all(link in urdf['links'] for link in CAMERA_LINKS)
    for a, b in pairs:
        for link in (a, b):
            if not has_camera and link.startswith('camera_'):
                continue  # Phase 0 harness fallback has no camera
            assert link in urdf['links'], f'disable_collisions link {link} not in {urdf["file"]}'
    # every pair of links joined by a joint is disabled (adjacent), as the setup assistant does
    disabled = {frozenset(p) for p in pairs}
    geometric = {link.get('name') for link in ET.fromstring(urdf['xml']).findall('link')
                 if link.find('collision') is not None}
    for j in urdf['joints'].values():
        if j['parent'] in geometric and j['child'] in geometric:
            assert frozenset((j['parent'], j['child'])) in disabled, (j['parent'], j['child'])


def test_planning_pipelines():
    ompl = _load(os.path.join(MOVEIT, 'ompl_planning.yaml'))
    assert ompl[ARM_GROUP]['default_planner_config'] == 'RRTConnectkConfigDefault'
    assert 'RRTConnectkConfigDefault' in ompl['planner_configs']
    pilz = _load(os.path.join(MOVEIT, 'pilz_industrial_motion_planner_planning.yaml'))
    assert pilz['planning_plugin'] == 'pilz_industrial_motion_planner/CommandPlanner'
    cart = _load(os.path.join(MOVEIT, 'pilz_cartesian_limits.yaml'))['cartesian_limits']
    assert {'max_trans_vel', 'max_trans_acc', 'max_trans_dec', 'max_rot_vel'} <= set(cart)


def test_acceleration_limits_for_all_movable_joints(urdf):
    limits = _load(os.path.join(MOVEIT, 'joint_limits.yaml'))['joint_limits']
    for name in urdf['joints']:
        if _movable(urdf, name):
            assert limits.get(name, {}).get('has_acceleration_limits'), name
            assert limits[name]['max_acceleration'] > 0.0, name


def _sets_cyclonedds(path):
    tree = ast.parse(open(path, 'r', encoding='utf-8').read())
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and getattr(node.func, 'id', None) == 'SetEnvironmentVariable':
            args = [a.value for a in node.args if isinstance(a, ast.Constant)]
            if args == ['RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp']:
                return True
    return False


def test_every_launch_file_sets_cyclonedds():
    files = [f for f in os.listdir(LAUNCH) if f.endswith('.launch.py')]
    assert 'sim.launch.py' in files and 'moveit.launch.py' in files
    for f in files:
        assert _sets_cyclonedds(os.path.join(LAUNCH, f)), f



def test_sim_launch_disables_online_model_database():
    """D-11: gzserver must not download (unpatched) models that shadow models_external."""
    tree = ast.parse(open(os.path.join(LAUNCH, 'sim.launch.py'), 'r', encoding='utf-8').read())
    found = False
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and getattr(node.func, 'id', None) == 'SetEnvironmentVariable':
            args = [a.value for a in node.args if isinstance(a, ast.Constant)]
            found = found or args == ['GAZEBO_MODEL_DATABASE_URI', '']
    assert found

def _sim_launch_module():
    spec = importlib.util.spec_from_file_location('sim_launch', os.path.join(LAUNCH, 'sim.launch.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_layout_parser_formats():
    mod = _sim_launch_module()
    assert mod.parse_pose([1, 2, 3]) == (1.0, 2.0, 3.0, 0.0, 0.0, 0.0)
    assert mod.parse_pose({'x': 1, 'yaw': 0.5}) == (1.0, 0.0, 0.0, 0.0, 0.0, 0.5)
    assert mod.parse_pose({'position': [1, 2, 3], 'rpy': [0, 0, 1]}) == (1, 2, 3, 0, 0, 1)
    with pytest.raises(ValueError):
        mod.parse_pose([1, 2])
    as_list = {'table': {'pose': [0.5, 0, 0]},
               'bins': [{'name': 'bin_cup', 'pose': [0, 0.5, 0]}]}
    as_params = {'/**': {'ros__parameters': {'table': {'pose': [0.5, 0, 0]},
                                             'bins': {'bin_cup': {'pose': [0, 0.5, 0]}}}}}
    for layout in (as_list, as_params):
        assert [f for f, _ in mod.layout_frames(layout)] == ['table', 'bin_cup']


def test_real_layout_file_frames():
    path = _gazebo_file(LAYOUT_REL)
    if path is None:
        pytest.skip('graspsort_gazebo/config/world_layout.yaml not present yet')
    frames = dict(_sim_launch_module().layout_frames(_load(path)))
    assert {'table', 'bin_cup', 'bin_bottle', 'bin_ball'} <= set(frames)
