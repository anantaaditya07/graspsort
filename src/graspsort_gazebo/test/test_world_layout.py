"""Consistency checks for the GraspSort world.

config/world_layout.yaml is the single source of truth. These tests check that
worlds/graspsort.world and the default args of urdf/graspsort_robot.urdf.xacro agree with it,
that the layout satisfies the reach / spacing / camera-view constraints, and that the xacro
expands and contains the contract link and joint names (and no colon+space in comments, D-06).
"""

import itertools
import math
import os
import re
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

PKG = os.environ.get('GRASPSORT_GAZEBO_SRC',
                     os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
LAYOUT = os.path.join(PKG, 'config', 'world_layout.yaml')
WORLD = os.path.join(PKG, 'worlds', 'graspsort.world')
XACRO = os.path.join(PKG, 'urdf', 'graspsort_robot.urdf.xacro')

POSE_TOL = 1e-4          # m / rad, world file vs yaml
REACH_MIN = 0.30         # m, horizontal distance from the arm base axis
REACH_MAX = 0.70
MIN_OBJECT_SEPARATION = 0.08   # m, centre to centre
BIN_CLEARANCE = 0.05     # m, object centre to bin outer edge (object radius + margin)
IMAGE_MARGIN_PX = 20     # objects must project at least this far inside the image

BIN_NAMES = ['bin_cup', 'bin_bottle', 'bin_ball']
OBJECT_MODELS = {
    'ball_1': 'cricket_ball', 'ball_2': 'cricket_ball',
    'bottle_1': 'mustard_bottle', 'bottle_2': 'mustard_bottle',
    'cup_1': 'plastic_cup', 'cup_2': 'plastic_cup',
}
ARM_JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
              'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
CONTRACT_LINKS = ['world', 'base_link', 'tool0', 'gripper_base_link',
                  'gripper_left_finger_link', 'gripper_right_finger_link', 'gripper_tcp',
                  'camera_link', 'camera_color_optical_frame']
CONTRACT_JOINTS = ARM_JOINTS + ['gripper_left_finger_joint', 'gripper_right_finger_joint']


def floats(text):
    return [float(v) for v in text.split()]


@pytest.fixture(scope='module')
def layout():
    with open(LAYOUT) as f:
        return yaml.safe_load(f)['/**']['ros__parameters']


@pytest.fixture(scope='module')
def world():
    w = ET.parse(WORLD).getroot().find('world')
    models = {m.get('name'): m for m in w.findall('model')}
    includes = {i.findtext('name'): i for i in w.findall('include') if i.findtext('name')}
    return w, models, includes


@pytest.fixture(scope='module')
def urdf():
    out = subprocess.run(['xacro', XACRO, 'simulation_controllers:=/tmp/controllers.yaml'],
                         check=True, capture_output=True, text=True).stdout
    return out


def assert_pose(pose_text, x, y, z, yaw, what):
    p = floats(pose_text)
    assert len(p) == 6, what
    expected = [x, y, z, 0.0, 0.0, yaw]
    for got, exp in zip(p, expected):
        assert abs(got - exp) < POSE_TOL, f'{what} pose {p} != yaml {expected}'


# ---------------- world file vs yaml ----------------

def test_static_models_match_yaml(layout, world):
    _, models, _ = world
    t = layout['table']
    assert_pose(models['table'].findtext('pose'), t['x'], t['y'], t['z'], t['yaw'], 'table')
    p = layout['pedestal']
    assert_pose(models['pedestal'].findtext('pose'), p['x'], p['y'], p['z'], 0.0, 'pedestal')
    for name in BIN_NAMES:
        b = layout['bins'][name]
        assert models[name].find('static').text.strip() == 'true'
        assert_pose(models[name].findtext('pose'), b['x'], b['y'], b['z'], b['yaw'], name)


def test_static_sizes_match_yaml(layout, world):
    _, models, _ = world
    top = models['table'].find("link/collision[@name='top_c']")
    assert np.allclose(floats(top.findtext('geometry/box/size')), layout['table']['size'])
    assert abs(floats(top.findtext('pose'))[2] + layout['table']['size'][2] / 2) < POSE_TOL
    ped = models['pedestal'].find('link/collision')
    assert np.allclose(floats(ped.findtext('geometry/box/size')), layout['pedestal']['size'])
    size = layout['bins']['size']
    wt = layout['bins']['wall_thickness']
    for name in BIN_NAMES:
        link = models[name].find('link')
        wall = link.find("collision[@name='wall_px_c']")
        assert np.allclose(floats(wall.findtext('geometry/box/size')), [wt, size[1], size[2]])
        floor = link.find("collision[@name='floor_c']")
        assert np.allclose(floats(floor.findtext('geometry/box/size'))[:2], size[:2])


def test_objects_match_yaml(layout, world):
    _, _, includes = world
    objs = layout['objects']
    assert sorted(objs['names']) == sorted(OBJECT_MODELS)
    assert sorted(includes) == sorted(OBJECT_MODELS), 'world objects must be exactly the six'
    for name in objs['names']:
        o = objs[name]
        assert o['model'] == OBJECT_MODELS[name]
        assert includes[name].findtext('uri') == f'model://{o["model"]}'
        assert_pose(includes[name].findtext('pose'), o['x'], o['y'], o['z'], o['yaw'], name)


def test_world_plugins(world):
    w, _, _ = world
    plugins = {p.get('filename'): p for p in w.findall('plugin')}
    assert 'libgazebo_ros_state.so' in plugins
    attach = plugins['libgraspsort_attach.so']
    assert attach.findtext('joint_type') == 'fixed'
    assert attach.findtext('attach_service') == '/attach'
    assert attach.findtext('detach_service') == '/detach'
    physics = w.find('physics')
    assert physics.get('type') == 'ode'
    assert float(physics.findtext('max_step_size')) == pytest.approx(0.001)
    assert w.find("model[@name='ur']") is None, 'robot is spawned by bringup, not in the world'


# ---------------- layout constraints ----------------

def horizontal_distance(layout, x, y):
    b = layout['robot_base']
    return math.hypot(x - b['x'], y - b['y'])


def test_robot_base_on_pedestal_at_table_height(layout):
    b, p, t = layout['robot_base'], layout['pedestal'], layout['table']
    assert (b['x'], b['y'], b['z']) == (p['x'], p['y'], p['z'])
    assert abs(b['z'] - t['z']) < POSE_TOL
    # pedestal does not intersect the table
    assert p['x'] + p['size'][0] / 2 < t['x'] - t['size'][0] / 2


def test_objects_reachable_and_separated(layout):
    objs = layout['objects']
    t = layout['table']
    for name in objs['names']:
        o = objs[name]
        d = horizontal_distance(layout, o['x'], o['y'])
        assert REACH_MIN <= d <= REACH_MAX, f'{name} at {d:.3f} m'
        assert abs(o['z'] - t['z']) < POSE_TOL, f'{name} must stand on the table'
        assert abs(o['x'] - t['x']) < t['size'][0] / 2 and abs(o['y'] - t['y']) < t['size'][1] / 2
    for a, b in itertools.combinations(objs['names'], 2):
        d = math.hypot(objs[a]['x'] - objs[b]['x'], objs[a]['y'] - objs[b]['y'])
        assert d >= MIN_OBJECT_SEPARATION, f'{a}-{b} only {d:.3f} m apart'


def test_bins_reachable_on_table_and_clear_of_objects(layout):
    size = layout['bins']['size']
    t = layout['table']
    objs = layout['objects']
    for name in layout['bins']['names']:
        b = layout['bins'][name]
        d = horizontal_distance(layout, b['x'], b['y'])
        assert REACH_MIN <= d <= REACH_MAX, f'{name} at {d:.3f} m'
        assert abs(b['z'] - t['z']) < POSE_TOL
        assert abs(b['x'] - t['x']) + size[0] / 2 <= t['size'][0] / 2
        assert abs(b['y'] - t['y']) + size[1] / 2 <= t['size'][1] / 2
        for o in objs['names']:
            dx = abs(objs[o]['x'] - b['x']) - size[0] / 2
            dy = abs(objs[o]['y'] - b['y']) - size[1] / 2
            assert max(dx, dy) >= BIN_CLEARANCE, f'{o} too close to {name}'
    for a, b in itertools.combinations(layout['bins']['names'], 2):
        ba, bb = layout['bins'][a], layout['bins'][b]
        assert (abs(ba['x'] - bb['x']) >= size[0] or abs(ba['y'] - bb['y']) >= size[1]), \
            f'{a} overlaps {b}'


def rot_rpy(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def project(layout, point):
    """Pixel (u, v) of a world point, pinhole model as in gazebo_ros_camera."""
    c = layout['camera']
    r = rot_rpy(c['roll'], c['pitch'], c['yaw'])   # camera_link axes in world
    p = r.T @ (np.array(point) - np.array([c['x'], c['y'], c['z']]))
    assert p[0] > 0, 'point behind the camera'
    f = (c['width'] / 2) / math.tan(c['horizontal_fov'] / 2)
    u = c['width'] / 2 - f * p[1] / p[0]
    v = c['height'] / 2 - f * p[2] / p[0]
    return u, v


def test_camera_sees_object_area(layout):
    c = layout['camera']
    objs = layout['objects']
    for name in objs['names']:
        o = objs[name]
        for z in (o['z'], o['z'] + 0.10):   # base and a point 10 cm up
            u, v = project(layout, [o['x'], o['y'], z])
            assert IMAGE_MARGIN_PX <= u <= c['width'] - IMAGE_MARGIN_PX, f'{name} u={u:.0f}'
            assert IMAGE_MARGIN_PX <= v <= c['height'] - IMAGE_MARGIN_PX, f'{name} v={v:.0f}'
    # aim point projects to the image centre (pose consistent with the documented aim)
    u, v = project(layout, c['aim_point'])
    assert abs(u - c['width'] / 2) < 2 and abs(v - c['height'] / 2) < 2


def test_camera_pitch_and_distance_per_d05(layout):
    c = layout['camera']
    assert 45 <= math.degrees(c['pitch']) <= 55
    d = np.linalg.norm(np.array([c['x'], c['y'], c['z']]) - np.array(c['aim_point']))
    assert 0.7 <= d <= 1.0


# ---------------- robot xacro ----------------

def test_xacro_contains_contract_names(urdf):
    root = ET.fromstring(urdf)
    assert root.get('name') == 'ur'
    links = {lk.get('name') for lk in root.findall('link')}
    joints = {j.get('name'): j for j in root.findall('joint')}
    for name in CONTRACT_LINKS:
        assert name in links, name
    for name in CONTRACT_JOINTS:
        assert name in joints, name
    assert joints['gripper_left_finger_joint'].get('type') == 'prismatic'
    mimic = joints['gripper_right_finger_joint'].find('mimic')
    assert mimic is not None and mimic.get('joint') == 'gripper_left_finger_joint'
    assert joints['gripper_base_joint'].find('parent').get('link') == 'tool0'
    assert joints['camera_joint'].find('parent').get('link') == 'world'
    assert joints['camera_optical_joint'].find('child').get('link') == \
        'camera_color_optical_frame'
    rpy = floats(joints['camera_optical_joint'].find('origin').get('rpy'))
    assert np.allclose(rpy, [-math.pi / 2, 0, -math.pi / 2], atol=1e-6)
    plugins = [p.get('filename') for p in root.iter('plugin')]
    assert 'libgazebo_ros2_control.so' in plugins and 'libgazebo_ros_camera.so' in plugins
    params = root.find(".//plugin[@filename='libgazebo_ros2_control.so']/parameters")
    assert params.text == '/tmp/controllers.yaml'


def test_xacro_camera_topics_and_frame(urdf):
    root = ET.fromstring(urdf)
    sensor = root.find(".//gazebo[@reference='camera_link']/sensor")
    assert sensor.get('type') == 'depth'
    assert float(sensor.findtext('update_rate')) == pytest.approx(10.0)
    assert sensor.findtext('camera/image/width') == '640'
    assert sensor.findtext('camera/image/height') == '480'
    plugin = sensor.find('plugin')
    assert plugin.findtext('ros/namespace') == '/camera'
    assert plugin.findtext('camera_name') == 'color'
    assert plugin.findtext('frame_name') == 'camera_color_optical_frame'
    remaps = {r.text for r in plugin.findall('ros/remapping')}
    assert remaps == {'color/depth/image_raw:=depth/image_raw',
                      'color/depth/camera_info:=depth/camera_info',
                      'color/points:=depth/points'}


def test_xacro_defaults_match_yaml(layout, urdf):
    root = ET.fromstring(urdf)
    joints = {j.get('name'): j for j in root.findall('joint')}
    c = layout['camera']
    cam = joints['camera_joint'].find('origin')
    assert np.allclose(floats(cam.get('xyz')), [c['x'], c['y'], c['z']], atol=POSE_TOL)
    assert np.allclose(floats(cam.get('rpy')), [c['roll'], c['pitch'], c['yaw']], atol=POSE_TOL)
    sensor = root.find(".//gazebo[@reference='camera_link']/sensor")
    assert int(sensor.findtext('camera/image/width')) == c['width']
    assert int(sensor.findtext('camera/image/height')) == c['height']
    assert float(sensor.findtext('update_rate')) == pytest.approx(c['update_rate'])
    assert float(sensor.findtext('camera/horizontal_fov')) == pytest.approx(c['horizontal_fov'])
    b = layout['robot_base']
    base = joints['base_joint']
    assert base.find('parent').get('link') == 'world'
    assert base.find('child').get('link') == 'base_link'
    assert np.allclose(floats(base.find('origin').get('xyz')), [b['x'], b['y'], b['z']],
                       atol=POSE_TOL)
    assert np.allclose(floats(base.find('origin').get('rpy')), [0, 0, b['yaw']], atol=POSE_TOL)


COMMENT_RE = re.compile(r'<!--(.*?)-->', re.DOTALL)


def test_no_colon_space_in_comments(urdf):
    with open(XACRO) as f:
        source = f.read()
    for text, what in ((source, 'xacro source'), (urdf, 'expanded URDF')):
        bad = [c.strip()[:60] for c in COMMENT_RE.findall(text) if ': ' in c]
        assert not bad, f'colon+space in {what} comments (D-06): {bad}'
