#!/usr/bin/env python3
"""Phase 4 evaluation harness for pick_place_test (EVAL ONLY: uses Gazebo ground truth).

For each trial:
  1. reset every object in world_layout.yaml (objects.names) to its layout pose with
     /gazebo/set_entity_state (zero twist),
  2. wait until perception has settled: /objects_3d holds `expect_objects` objects whose positions
     moved less than --settle-tol over --settle-time s (the scene manager's remove_timeout must
     also have passed so stale tracks are gone),
  3. run `pick_place_test` (ros2 run, its own process) for the class and parse its
     PICK_PLACE_RESULT line,
  4. wait until every object is at rest on /gazebo/model_states, then PASS only if exactly one
     model of the class rests in the class's bin: the object's centre (model pose + the model's
     link offset) is inside the bin's inner area in xy (shrunk by --xy-margin) and between the
     bin floor top and rim + the object's half height + --z-margin in z, and pick_place_test
     reported success.
After a failed trial the arm is sent to `ready` (graspsort_bringup move_named.py).

Prints a per-trial table (stage times, planning times, result). Exit 0 only if all trials pass.

Example (sim + perception + scene manager running):
  ros2 run graspsort_manipulation pick_place_trials.py --trials 5 --classes "sports ball" bottle
"""
import argparse
import json
import math
import os
import subprocess
import sys
import tempfile
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.msg import EntityState, ModelStates
from gazebo_msgs.srv import SetEntityState
from graspsort_msgs.msg import ObjectPoseArray
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

# Model name -> link origin offset above the model origin (model.sdf <link><pose>), and the
# class name of graspsort_perception (D-04).
MODEL_INFO = {
    'cricket_ball': {'link_z': 0.0375, 'half_height': 0.0375, 'class': 'sports ball'},
    'mustard_bottle': {'link_z': 0.09565, 'half_height': 0.09565, 'class': 'bottle'},
}
STAGE_COLUMNS = [('pregrasp', 'pre'), ('approach', 'app'), ('close_gripper', 'close'),
                 ('lift', 'lift'), ('move_above_bin', 'xfer'), ('lower', 'lower'),
                 ('release_gripper', 'rel'), ('retreat', 'retr'), ('ready', 'ready')]


def parse_args(argv):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument('--trials', type=int, default=5, help='trials per class')
    p.add_argument('--classes', nargs='+', default=['sports ball', 'bottle'])
    p.add_argument('--layout', default=os.path.join(
        get_package_share_directory('graspsort_gazebo'), 'config', 'world_layout.yaml'))
    p.add_argument('--params', default=os.path.join(
        get_package_share_directory('graspsort_manipulation'), 'config', 'pick_place.yaml'))
    p.add_argument('--expect-objects', type=int, default=None,
                   help='objects perception must report before a trial (default: layout count)')
    p.add_argument('--settle-time', type=float, default=3.0, help='[s] stable perception window')
    p.add_argument('--settle-tol', type=float, default=0.01, help='[m] max motion in the window')
    p.add_argument('--settle-timeout', type=float, default=60.0, help='[s]')
    p.add_argument('--run-timeout', type=float, default=240.0, help='[s] per pick_place_test')
    p.add_argument('--rest-speed', type=float, default=0.01, help='[m/s] at-rest speed')
    p.add_argument('--rest-time', type=float, default=1.0, help='[s] at rest before checking')
    p.add_argument('--rest-timeout', type=float, default=15.0, help='[s]')
    p.add_argument('--xy-margin', type=float, default=0.0, help='[m] inner-area shrink')
    p.add_argument('--z-margin', type=float, default=0.01, help='[m] above rim + half height')
    p.add_argument('--log-dir', default='', help='save each pick_place_test output here')
    p.add_argument('--bin-for', nargs=2, action='append', metavar=('CLASS', 'BIN'),
                   default=None, help='class -> bin (default: from --params)')
    return p.parse_args(argv)


def load_params(path):
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    key = 'pick_place_test' if 'pick_place_test' in data else '/**'
    return data[key]['ros__parameters']


def yaw_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def rotate(q, v):
    """Rotate vector v by quaternion q = (x, y, z, w)."""
    x, y, z, w = q
    vx, vy, vz = v
    # t = 2 q_vec x v; v' = v + w t + q_vec x t
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty), vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


class Harness(Node):
    def __init__(self):
        super().__init__('pick_place_trials')
        self.models = None
        self.objects = None
        self.create_subscription(ModelStates, '/gazebo/model_states', self._on_models, 10)
        self.create_subscription(ObjectPoseArray, '/objects_3d', self._on_objects, 10)
        self.set_state = self.create_client(SetEntityState, '/gazebo/set_entity_state')

    def _on_models(self, msg):
        self.models = msg

    def _on_objects(self, msg):
        self.objects = msg

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def reset_objects(self, layout):
        if not self.set_state.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('/gazebo/set_entity_state not available')
        objs = layout['objects']
        for name in objs['names']:
            o = objs[name]
            req = SetEntityState.Request()
            st = EntityState()
            st.name = name
            st.reference_frame = 'world'
            st.pose.position.x, st.pose.position.y = float(o['x']), float(o['y'])
            st.pose.position.z = float(o['z'])
            (st.pose.orientation.x, st.pose.orientation.y, st.pose.orientation.z,
             st.pose.orientation.w) = yaw_quat(float(o.get('yaw', 0.0)))
            req.state = st
            fut = self.set_state.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
            if not fut.done() or not fut.result().success:
                raise RuntimeError(f'set_entity_state {name} failed')

    def wait_perception(self, expect, settle_time, tol, timeout):
        """True once `expect` objects are perceived and none moved > tol for settle_time."""
        start = time.monotonic()
        window_start, ref = None, None
        while time.monotonic() - start < timeout:
            self.spin_for(0.1)
            msg = self.objects
            if msg is None or len(msg.objects) != expect:
                window_start, ref = None, None
                continue
            pos = {o.id: (o.pose.position.x, o.pose.position.y, o.pose.position.z)
                   for o in msg.objects}
            if ref is None or set(pos) != set(ref) or any(
                    math.dist(pos[i], ref[i]) > tol for i in pos):
                window_start, ref = time.monotonic(), pos
                continue
            if time.monotonic() - window_start >= settle_time:
                return True
        return False

    def model_centres(self, layout):
        """{name: (centre xyz, speed)} for the layout objects, from /gazebo/model_states."""
        msg = self.models
        out = {}
        objs = layout['objects']
        for name in objs['names']:
            if name not in msg.name:
                continue
            i = msg.name.index(name)
            p, q, tw = msg.pose[i], msg.pose[i].orientation, msg.twist[i].linear
            info = MODEL_INFO[objs[name]['model']]
            off = rotate((q.x, q.y, q.z, q.w), (0.0, 0.0, info['link_z']))
            c = (p.position.x + off[0], p.position.y + off[1], p.position.z + off[2])
            out[name] = (c, math.sqrt(tw.x ** 2 + tw.y ** 2 + tw.z ** 2))
        return out

    def wait_rest(self, layout, speed, rest_time, timeout):
        start = time.monotonic()
        since = None
        while time.monotonic() - start < timeout:
            self.spin_for(0.1)
            if self.models is None:
                continue
            moving = any(v > speed for _, v in self.model_centres(layout).values())
            if moving:
                since = None
            elif since is None:
                since = time.monotonic()
            elif time.monotonic() - since >= rest_time:
                return True
        return False


def bin_check(centre, half_height, layout, bin_name, xy_margin, z_margin):
    bins = layout['bins']
    b = bins[bin_name]
    sx, sy, sz = (float(v) for v in bins['size'])
    wall, floor = float(bins['wall_thickness']), float(bins['floor_thickness'])
    dx, dy = centre[0] - float(b['x']), centre[1] - float(b['y'])
    yaw = float(b.get('yaw', 0.0))
    lx = math.cos(yaw) * dx + math.sin(yaw) * dy
    ly = -math.sin(yaw) * dx + math.cos(yaw) * dy
    in_xy = abs(lx) <= sx / 2 - wall - xy_margin and abs(ly) <= sy / 2 - wall - xy_margin
    floor_top = float(b['z']) + floor
    rim = float(b['z']) + sz
    in_z = floor_top <= centre[2] <= rim + half_height + z_margin
    return in_xy and in_z, (lx, ly, centre[2])


def kinematics_params_file(log_dir):
    """Params file with robot_description_kinematics from graspsort_bringup (IK in the node)."""
    src = os.path.join(get_package_share_directory('graspsort_bringup'), 'config', 'moveit',
                       'kinematics.yaml')
    with open(src, 'r', encoding='utf-8') as f:
        kin = yaml.safe_load(f)
    if log_dir:
        path = os.path.join(log_dir, 'pick_place_kinematics.yaml')
    else:
        fd, path = tempfile.mkstemp(prefix='pick_place_kinematics_', suffix='.yaml')
        os.close(fd)
    with open(path, 'w', encoding='utf-8') as f:
        yaml.safe_dump({'/**': {'ros__parameters': {'robot_description_kinematics': kin}}}, f)
    return path


def run_pick(cls, args, log_path):
    cmd = ['ros2', 'run', 'graspsort_manipulation', 'pick_place_test', '--ros-args',
           '--params-file', args.layout, '--params-file', args.params,
           '--params-file', args.kinematics_file,
           '-p', f'target_class:={cls}', '-p', 'use_sim_time:=true']
    t0 = time.monotonic()
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=args.run_timeout)
        out, code = proc.stdout + proc.stderr, proc.returncode
    except subprocess.TimeoutExpired as e:
        out = (e.stdout or b'').decode() if isinstance(e.stdout, bytes) else (e.stdout or '')
        code = -1
    wall = time.monotonic() - t0
    if log_path:
        with open(log_path, 'w', encoding='utf-8') as f:
            f.write(out)
    result = None
    for line in out.splitlines():
        if line.startswith('PICK_PLACE_RESULT '):
            result = json.loads(line[len('PICK_PLACE_RESULT '):])
    return code, wall, result


def move_ready():
    subprocess.run(['ros2', 'run', 'graspsort_bringup', 'move_named.py', 'ready'],
                   capture_output=True, text=True, timeout=120)


def main(argv=None):
    args = parse_args(remove_ros_args(argv if argv is not None else sys.argv)[1:])
    with open(args.layout, 'r', encoding='utf-8') as f:
        layout = yaml.safe_load(f)['/**']['ros__parameters']
    params = load_params(args.params)
    bin_map = dict(zip(params['bin_classes'], params['bin_names']))
    if args.bin_for:
        bin_map.update(dict(args.bin_for))
    expect = args.expect_objects or len(layout['objects']['names'])
    if args.log_dir:
        os.makedirs(args.log_dir, exist_ok=True)
    args.kinematics_file = kinematics_params_file(args.log_dir)

    rclpy.init()
    h = Harness()
    rows = []
    trial = 0
    try:
        for cls in args.classes:
            for k in range(args.trials):
                trial += 1
                row = {'trial': trial, 'class': cls, 'model': '-', 'pass': False, 'why': ''}
                h.reset_objects(layout)
                h.spin_for(1.0)
                if not h.wait_perception(expect, args.settle_time, args.settle_tol,
                                         args.settle_timeout):
                    row['why'] = 'perception did not settle'
                    rows.append(row)
                    print_row(row)
                    continue
                log = (os.path.join(args.log_dir, f'trial{trial:02d}_{cls.replace(" ", "_")}.log')
                       if args.log_dir else '')
                code, wall, res = run_pick(cls, args, log)
                row['code'], row['wall'], row['res'] = code, wall, res
                h.wait_rest(layout, args.rest_speed, args.rest_time, args.rest_timeout)
                h.spin_for(0.3)
                bin_name = bin_map[cls]
                inside = []
                for name, (c, _) in h.model_centres(layout).items():
                    info = MODEL_INFO[layout['objects'][name]['model']]
                    if info['class'] != cls:
                        continue
                    ok, local = bin_check(c, info['half_height'], layout, bin_name,
                                          args.xy_margin, args.z_margin)
                    if ok:
                        inside.append((name, local))
                if inside:
                    row['model'] = inside[0][0]
                    row['local'] = inside[0][1]
                if code == 0 and res and res.get('success') and len(inside) == 1:
                    row['pass'] = True
                else:
                    stage = res.get('failed_stage') if res else '?'
                    row['why'] = (f'exit {code}, failed stage {stage}'
                                  f': {res.get("failure", "")[:120] if res else "no result"}; '
                                  f'{len(inside)} {cls} in {bin_name}; '
                                  f'recovery: {res.get("recovery", "") if res else "-"}')
                    move_ready()
                rows.append(row)
                print_row(row)
    finally:
        h.destroy_node()
        rclpy.shutdown()
    print_table(rows)
    return 0 if rows and all(r['pass'] for r in rows) else 1


def stage_times(res):
    st = {s['name']: s for s in res.get('stages', [])} if res else {}
    return st


def print_row(r):
    res = r.get('res')
    loc = r.get('local')
    where = f'({loc[0]:+.3f},{loc[1]:+.3f},{loc[2]:.3f})' if loc else ''
    print(f'trial {r["trial"]:2d} {r["class"]:<11} {r["model"]:<9} '
          f'{"PASS" if r["pass"] else "FAIL"} total {res["total_s"] if res else 0:.1f}s '
          f'{where} {r["why"]}', flush=True)


def print_table(rows):
    hdr = (f'{"#":>2} {"class":<11} {"model":<9} {"res":<4} {"total":>6} ' +
           ' '.join(f'{c:>5}' for _, c in STAGE_COLUMNS) + f' {"plan":>5}  bin-local centre')
    print('\nstage times [s]; plan = total planning time [s]')
    print(hdr)
    for r in rows:
        res = r.get('res')
        st = stage_times(res)
        cells = ' '.join(f'{st[s]["s"]:5.1f}' if s in st else f'{"-":>5}' for s, _ in STAGE_COLUMNS)
        plan = sum(s.get('plan_s', 0.0) for s in st.values())
        loc = r.get('local')
        where = f'({loc[0]:+.3f},{loc[1]:+.3f},{loc[2]:.3f})' if loc else r['why']
        print(f'{r["trial"]:2d} {r["class"]:<11} {r["model"]:<9} '
              f'{"PASS" if r["pass"] else "FAIL":<4} {res["total_s"] if res else 0:6.1f} {cells} '
              f'{plan:5.2f}  {where}')
    for cls in dict.fromkeys(r['class'] for r in rows):
        sel = [r for r in rows if r['class'] == cls]
        print(f'{cls}: {sum(r["pass"] for r in sel)}/{len(sel)} PASS')


if __name__ == '__main__':
    sys.exit(main())
