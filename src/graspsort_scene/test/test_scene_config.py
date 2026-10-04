"""Config checks for graspsort_scene (no ROS graph needed).

- every launch file under launch/ sets RMW_IMPLEMENTATION=rmw_cyclonedds_cpp (D-07)
- every parameter scene_manager_node declares is set by scene.yaml or world_layout.yaml
  (bins.<name>.x/y/z/yaw for every bin name), and scene.yaml has no unknown keys
- scene.yaml values equal the node's declared defaults
- the launch file names the node like the scene.yaml key and loads world_layout.yaml
- the node never declares the world file's object poses (scene from perception only)
"""
import ast
import os
import re

import yaml

SRC = os.environ.get('GRASPSORT_SCENE_SRC',
                     os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
LAUNCH = os.path.join(SRC, 'launch')
NODE = os.path.join(SRC, 'src', 'scene_manager_node.cpp')
LOGIC = os.path.join(SRC, 'include', 'graspsort_scene', 'scene_logic.hpp')
SCENE_YAML = os.path.join(SRC, 'config', 'scene.yaml')
LAYOUT = os.path.join(SRC, '..', 'graspsort_gazebo', 'config', 'world_layout.yaml')
NODE_NAME = 'scene_manager'

DECLARE = re.compile(
    r'declare_parameter<[^()"]*?>\(\s*(?:"([^"]+)"|p \+ "([^"]+)")\s*,\s*([^()]*?)\)', re.S)


def _read(path):
    with open(path, 'r', encoding='utf-8') as f:
        return f.read()


def _flatten(d, prefix=''):
    out = {}
    for k, v in d.items():
        key = prefix + str(k)
        if isinstance(v, dict):
            out.update(_flatten(v, key + '.'))
        else:
            out[key] = v
    return out


def _declared():
    """{name: default text} for literal names; per-bin suffixes under the key 'bins.*.'."""
    names = {}
    per_bin = set()
    for m in DECLARE.finditer(_read(NODE)):
        if m.group(1):
            names[m.group(1)] = m.group(3).strip()
        else:
            per_bin.add(m.group(2))
    return names, per_bin


def _scene_params():
    return _flatten(yaml.safe_load(_read(SCENE_YAML))[NODE_NAME]['ros__parameters'])


def _layout_params():
    return _flatten(yaml.safe_load(_read(LAYOUT))['/**']['ros__parameters'])


def _sets_cyclonedds(path):
    tree = ast.parse(_read(path))
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and getattr(node.func, 'id', None) == 'SetEnvironmentVariable':
            args = [a.value for a in node.args if isinstance(a, ast.Constant)]
            if args == ['RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp']:
                return True
    return False


def test_every_launch_file_sets_cyclonedds():
    files = [f for f in os.listdir(LAUNCH) if f.endswith('.launch.py')]
    assert 'scene.launch.py' in files
    for f in files:
        assert _sets_cyclonedds(os.path.join(LAUNCH, f)), f


def test_declared_parameters_are_found():
    names, per_bin = _declared()
    # Sanity: the regex sees the node's parameters.
    assert {'update_distance', 'remove_timeout', 'bins.names', 'table.size'} <= set(names)
    assert per_bin == {'x', 'y', 'z', 'yaw'}
    # ... and every declare_parameter call is parsed.
    assert len(names) + len(per_bin) == _read(NODE).count('declare_parameter<')


def test_every_declared_parameter_is_set_by_a_file():
    names, per_bin = _declared()
    scene = _scene_params()
    layout = _layout_params()
    missing = [n for n in names if n not in scene and n not in layout]
    assert not missing, missing
    for b in layout['bins.names']:
        for s in per_bin:
            assert f'bins.{b}.{s}' in layout, (b, s)


def test_scene_yaml_has_no_unknown_keys_and_no_layout_keys():
    names, _ = _declared()
    scene = _scene_params()
    layout = _layout_params()
    assert set(scene) <= set(names), set(scene) - set(names)
    assert not set(scene) & set(layout)


def _logic_defaults():
    text = _read(LOGIC)
    body = text[text.index('struct TrackerParams'):]
    body = body[:body.index('\n};')]
    return {m.group(1): float(m.group(2))
            for m in re.finditer(r'double (\w+)\{([-0-9.eE]+)\}', body)}


def test_scene_yaml_values_equal_node_defaults():
    names, _ = _declared()
    scene = _scene_params()
    logic = _logic_defaults()
    assert set(logic) == {'update_distance', 'update_yaw', 'remove_timeout'}
    for key, value in scene.items():
        default = names[key]
        if default.startswith('tp.'):
            assert value == logic[default[3:]], key
        elif default.startswith('"'):
            assert value == default.strip('"'), key
        else:
            assert value == ast.literal_eval(default), key


def test_never_declares_world_file_objects():
    names, _ = _declared()
    assert not [n for n in names if n.startswith('objects.') or n == 'objects'], names
    assert 'objects.names' in _layout_params()  # the layout has them; the node must not read them


def test_launch_loads_layout_and_names_the_node():
    text = _read(os.path.join(LAUNCH, 'scene.launch.py'))
    assert f"name='{NODE_NAME}'" in text
    assert "'world_layout.yaml'" in text and "'graspsort_gazebo'" in text
    assert "'scene.yaml'" in text
    assert re.search(r"Node\s*\(\s*package='graspsort_scene'", text)
    assert 'rclcpp::Node("scene_manager")' in _read(NODE)
