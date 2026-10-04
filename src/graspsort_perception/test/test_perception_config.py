"""Config checks for graspsort_perception (no ROS graph needed).

- every launch file under launch/ sets RMW_IMPLEMENTATION=rmw_cyclonedds_cpp (D-07)
- perception.yaml's detector class_filter is exactly the D-04/D-13 COCO classes
- every detector class has a localizer shape; the localizer uses the D-12 wide band
- perception.launch.py reads table_height / world_frame from world_layout.yaml
"""
import ast
import importlib.util
import os

import yaml

SRC = os.environ.get('GRASPSORT_PERCEPTION_SRC',
                     os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
LAUNCH = os.path.join(SRC, 'launch')
CONFIG = os.path.join(SRC, 'config', 'perception.yaml')
D04_CLASSES = ['bottle', 'sports ball']  # D-04 amended by D-13 (no cup)


def _sets_cyclonedds(path):
    with open(path, 'r', encoding='utf-8') as f:
        tree = ast.parse(f.read())
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and getattr(node.func, 'id', None) == 'SetEnvironmentVariable':
            args = [a.value for a in node.args if isinstance(a, ast.Constant)]
            if args == ['RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp']:
                return True
    return False


def test_every_launch_file_sets_cyclonedds():
    files = [f for f in os.listdir(LAUNCH) if f.endswith('.launch.py')]
    assert 'perception.launch.py' in files
    for f in files:
        assert _sets_cyclonedds(os.path.join(LAUNCH, f)), f


def test_class_filter_is_d04():
    with open(CONFIG, 'r', encoding='utf-8') as f:
        params = yaml.safe_load(f)['object_detector_node']['ros__parameters']
    assert params['class_filter'] == D04_CLASSES


def test_node_default_class_filter_matches_yaml():
    src = os.path.join(SRC, 'src', 'object_detector_node.cpp')
    with open(src, 'r', encoding='utf-8') as f:
        text = f.read()
    assert '{"bottle", "sports ball"}' in text
    assert '"camera/color/image_raw"' in text


def _params(node):
    with open(CONFIG, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)[node]['ros__parameters']


def test_every_detector_class_has_a_localizer_shape():
    det = _params('object_detector_node')
    loc = _params('object_localizer_node')
    assert len(loc['shape_classes']) == len(loc['shape_types'])
    assert set(loc['shape_types']) <= {0, 1}
    assert set(det['class_filter']) <= set(loc['shape_classes'])


def test_localizer_uses_d12_wide_band():
    loc = _params('object_localizer_node')
    assert loc['footprint']['depth_band'] == 0.10
    assert loc['depth']['percentile'] == 0.20
    assert loc['depth']['central_fraction'] == 0.5


def test_launch_reads_table_height_from_world_layout():
    spec = importlib.util.spec_from_file_location(
        'perception_launch', os.path.join(LAUNCH, 'perception.launch.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    layout = os.path.join(SRC, '..', 'graspsort_gazebo', 'config', 'world_layout.yaml')
    assert mod.layout_params(layout) == {'table_height': 0.75, 'world_frame': 'world'}
