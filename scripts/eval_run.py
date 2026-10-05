#!/usr/bin/env python3
"""GraspSort Phase 6 evaluation runner (architecture 8, D-17). EVAL ONLY: uses Gazebo ground truth.

For every config (n_objects:min_gap) and trial:
  1. delete every object model left in Gazebo (the world-file objects ball_1, ball_2, bottle_1,
     bottle_2 and every model spawned by an earlier trial, names e<c>t<t>_...),
  2. sample a layout with seed = base_seed + 1000 * config_index + trial (config_index = position
     in the D-17 list 2:0.08 2:0.03 4:0.08 4:0.03): uniform x, y in the object area, bottle yaw
     uniform in [-pi, pi), rejection sampling on the footprint gap (centre distance minus both
     circumscribed footprint radii >= min_gap) and on the clearance to the bins and the table
     edge; fails loudly if no layout is found,
  3. wait until /objects_3d is empty (old tracks gone), spawn the models with /spawn_entity under
     unique names (D-06) at the table top with the sampled yaw, wait until they rest and until
     perception reports n stable objects,
  4. localization snapshot: per ground-truth object the nearest unused same-class /objects_3d
     estimate within --match-radius (3D error, xy, z, bottle yaw error mod 180),
  5. send ONE SortObjects goal (classes empty = all) with --goal-timeout (then cancel),
  6. wait until the objects rest, record final positions and the bin whose inner area holds each,
  7. append one JSON line to <out-dir>/trials.jsonl (flushed, fsync'd) and print one summary line.
After a goal that did not succeed the arm is sent to `ready` (graspsort_bringup move_named.py).
Trial numbers start at 0. --dry-run only samples and prints the layouts (no ROS needed).

Examples:
  scripts/eval_run.py --dry-run --trials 3
  scripts/eval_run.py --configs 2:0.08 4:0.03 --trials 10 --out-dir data/eval/run1
"""
import argparse
import hashlib
import json
import math
import os
import random
import re
import sys
import time

import yaml

REPO = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
D17_CONFIGS = ['2:0.08', '2:0.03', '4:0.08', '4:0.03']
WORLD_OBJECTS = ['ball_1', 'ball_2', 'bottle_1', 'bottle_2']
SPAWNED_RE = re.compile(r'^e\d+t\d+_')
# Model -> class (graspsort_perception names), short name, circumscribed footprint radius,
# link origin above the model origin (= half height, model.sdf <link><pose>).
MODELS = {
    'cricket_ball': {'class': 'sports ball', 'short': 'ball', 'radius': 0.0375,
                     'link_z': 0.0375, 'random_yaw': False},
    'mustard_bottle': {'class': 'bottle', 'short': 'bottle',
                       'radius': math.hypot(0.0958, 0.0582) / 2.0, 'link_z': 0.09565,
                       'random_yaw': True},
}
# Object mix per n_objects (D-17): equal numbers of balls and bottles.
MIX = ['cricket_ball', 'mustard_bottle']


def parse_args(argv):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument('--configs', nargs='+', default=D17_CONFIGS,
                   help='n_objects:min_gap[m] (default: the 4 D-17 configs)')
    p.add_argument('--trials', type=int, default=10, help='trials per config')
    p.add_argument('--start-trial', type=int, default=0, help='first trial number (resume)')
    p.add_argument('--base-seed', type=int, default=2026)
    p.add_argument('--out-dir', default='', help='trials.jsonl goes here (required unless dry run)')
    p.add_argument('--run-id', default='', help='default: basename of --out-dir')
    p.add_argument('--dry-run', action='store_true', help='only sample and print the layouts')
    p.add_argument('--layout', default=os.path.join(REPO, 'src', 'graspsort_gazebo', 'config',
                                                    'world_layout.yaml'))
    p.add_argument('--params', default=os.path.join(REPO, 'src', 'graspsort_manipulation',
                                                    'config', 'pick_place.yaml'),
                   help='pick_place.yaml (bin_classes, bin_names)')
    p.add_argument('--models-dir', default='',
                   help='dir with <model>/model.sdf (default: repo models_external, else share)')
    # Object area (centre of the model, world frame). Derived from world_layout.yaml: reachable
    # band 0.39..0.61 m from the base axis around the reference poses, camera aim (0.49, 0).
    p.add_argument('--x-min', type=float, default=0.35)
    p.add_argument('--x-max', type=float, default=0.62)
    p.add_argument('--y-min', type=float, default=-0.25)
    p.add_argument('--y-max', type=float, default=0.25)
    p.add_argument('--bin-clearance', type=float, default=0.03,
                   help='[m] min footprint distance to every bin outer wall')
    p.add_argument('--edge-clearance', type=float, default=0.03,
                   help='[m] min footprint distance to the table edge')
    p.add_argument('--max-tries', type=int, default=20000, help='rejection samples per layout')
    p.add_argument('--table-z', type=float, default=None, help='default: table.z of the layout')
    p.add_argument('--spawn-dz', type=float, default=0.0, help='[m] spawn above the table top')
    # Timing.
    p.add_argument('--clear-timeout', type=float, default=20.0,
                   help='[s] wait for /objects_3d to be empty after the delete')
    p.add_argument('--settle-time', type=float, default=3.0, help='[s] stable perception window')
    p.add_argument('--settle-tol', type=float, default=0.01, help='[m] max motion in the window')
    p.add_argument('--settle-timeout', type=float, default=60.0, help='[s]')
    p.add_argument('--goal-timeout', type=float, default=960.0, help='[s] then cancel')
    p.add_argument('--cancel-timeout', type=float, default=120.0, help='[s] result after cancel')
    p.add_argument('--idle-timeout', type=float, default=180.0,
                   help='[s] retry a rejected goal (node busy) this long')
    p.add_argument('--server-timeout', type=float, default=120.0, help='[s] wait for the server')
    p.add_argument('--service-timeout', type=float, default=30.0, help='[s] Gazebo services')
    p.add_argument('--rest-speed', type=float, default=0.01, help='[m/s] at-rest speed')
    p.add_argument('--rest-time', type=float, default=1.0, help='[s]')
    p.add_argument('--rest-timeout', type=float, default=15.0, help='[s]')
    p.add_argument('--match-radius', type=float, default=0.10, help='[m] localization match')
    p.add_argument('--xy-margin', type=float, default=0.0, help='[m] bin inner-area shrink')
    p.add_argument('--z-margin', type=float, default=0.01, help='[m] above rim + half height')
    p.add_argument('--no-ready', action='store_true', help='do not send the arm to ready')
    return p.parse_args(argv)


# ----------------------------------------------------------------------------- layout sampling


def parse_config(text):
    n, gap = text.split(':')
    n, gap = int(n), float(gap)
    if n <= 0 or n % len(MIX):
        raise ValueError(f'n_objects must be a positive multiple of {len(MIX)}: {text}')
    return n, gap


def config_index(text, position):
    """Index in the D-17 list (stable seeds when running a subset), else 100 + position."""
    n, gap = parse_config(text)
    for i, c in enumerate(D17_CONFIGS):
        if parse_config(c) == (n, gap):
            return i
    return 100 + position


def trial_seed(base_seed, cidx, trial):
    return base_seed + 1000 * cidx + trial


def object_specs(n, cidx, trial):
    """[(unique name, model)] for n objects: n/2 balls then n/2 bottles."""
    specs = []
    for model in MIX:
        for k in range(n // len(MIX)):
            specs.append((f'e{cidx}t{trial}_{MODELS[model]["short"]}_{k}', model))
    return specs


def rect_distance(px, py, cx, cy, yaw, hx, hy):
    """Distance from a point to a yawed rectangle (0 inside)."""
    dx, dy = px - cx, py - cy
    lx = math.cos(yaw) * dx + math.sin(yaw) * dy
    ly = -math.sin(yaw) * dx + math.cos(yaw) * dy
    ox, oy = max(abs(lx) - hx, 0.0), max(abs(ly) - hy, 0.0)
    return math.hypot(ox, oy)


def footprint_ok(x, y, r, layout, args):
    t = layout['table']
    tx, ty = float(t['x']), float(t['y'])
    hx, hy = float(t['size'][0]) / 2.0, float(t['size'][1]) / 2.0
    if abs(x - tx) > hx - r - args.edge_clearance or abs(y - ty) > hy - r - args.edge_clearance:
        return False
    bins = layout['bins']
    bx, by = float(bins['size'][0]) / 2.0, float(bins['size'][1]) / 2.0
    for b in bins['names']:
        bb = bins[b]
        d = rect_distance(x, y, float(bb['x']), float(bb['y']), float(bb.get('yaw', 0.0)), bx, by)
        if d - r < args.bin_clearance:
            return False
    return True


def gap(a, b):
    return (math.hypot(a['x'] - b['x'], a['y'] - b['y']) - MODELS[a['model']]['radius'] -
            MODELS[b['model']]['radius'])


def min_gap(objs):
    gaps = [gap(a, b) for i, a in enumerate(objs) for b in objs[i + 1:]]
    return min(gaps) if gaps else None


def sample_layout(specs, min_gap_m, seed, layout, args):
    """Rejection sampling; one random.Random(seed) stream, so a seed gives one layout."""
    rng = random.Random(seed)
    tries = 0
    while tries < args.max_tries:
        objs = []
        for name, model in specs:
            info = MODELS[model]
            placed = False
            while tries < args.max_tries:
                tries += 1
                x = rng.uniform(args.x_min, args.x_max)
                y = rng.uniform(args.y_min, args.y_max)
                yaw = rng.uniform(-math.pi, math.pi) if info['random_yaw'] else 0.0
                cand = {'name': name, 'model': model, 'x': x, 'y': y, 'yaw': yaw}
                if not footprint_ok(x, y, info['radius'], layout, args):
                    continue
                if any(gap(cand, o) < min_gap_m for o in objs):
                    continue
                objs.append(cand)
                placed = True
                break
            if not placed:
                break
        if len(objs) == len(specs):
            return objs, tries
    raise RuntimeError(f'cannot place {len(specs)} objects with gap >= {min_gap_m} m in '
                       f'{args.max_tries} samples (seed {seed}); widen the area or lower the gap')


def layout_digest(objs):
    text = ';'.join(f'{o["name"]},{o["x"]:.6f},{o["y"]:.6f},{o["yaw"]:.6f}' for o in objs)
    return hashlib.sha1(text.encode()).hexdigest()[:12]


def plan_trials(args, layout):
    plan = []
    for pos, text in enumerate(args.configs):
        n, g = parse_config(text)
        cidx = config_index(text, pos)
        for trial in range(args.start_trial, args.start_trial + args.trials):
            seed = trial_seed(args.base_seed, cidx, trial)
            objs, tries = sample_layout(object_specs(n, cidx, trial), g, seed, layout, args)
            plan.append({'config': {'n_objects': n, 'min_gap': g}, 'cidx': cidx, 'trial': trial,
                         'seed': seed, 'objects': objs, 'tries': tries})
    return plan


def print_plan(plan):
    for p in plan:
        mg = min_gap(p['objects'])
        print(f'config {p["config"]["n_objects"]}:{p["config"]["min_gap"]:.2f} trial {p["trial"]} '
              f'seed {p["seed"]} min_gap {mg:.4f} m samples {p["tries"]} digest '
              f'{layout_digest(p["objects"])}')
        for o in p['objects']:
            print(f'    {o["name"]:<16} {o["model"]:<15} x {o["x"]:.4f} y {o["y"]:+.4f} '
                  f'yaw {math.degrees(o["yaw"]):+7.2f} deg')


# ----------------------------------------------------------------------------- ROS part


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def yaw_err_mod180(a, b):
    d = math.degrees(a - b) % 180.0
    return min(d, 180.0 - d)


def localization(gt, estimates, radius):
    """gt: [(name, class, centre xyz, yaw)]; estimates: ObjectPose list. Greedy nearest match."""
    pairs = []
    for gi, (_, cls, c, _) in enumerate(gt):
        for ei, e in enumerate(estimates):
            if e.class_name != cls:
                continue
            p = e.pose.position
            d = math.dist(c, (p.x, p.y, p.z))
            if d <= radius:
                pairs.append((d, gi, ei))
    out = [{'matched': False, 'id': None, 'err_3d': None, 'err_xy': None, 'err_z': None,
            'yaw_err_deg': None} for _ in gt]
    used_g, used_e = set(), set()
    for d, gi, ei in sorted(pairs):
        if gi in used_g or ei in used_e:
            continue
        used_g.add(gi)
        used_e.add(ei)
        e = estimates[ei]
        p = e.pose.position
        c = gt[gi][2]
        out[gi] = {'matched': True, 'id': int(e.id), 'err_3d': d,
                   'err_xy': math.hypot(c[0] - p.x, c[1] - p.y), 'err_z': abs(c[2] - p.z),
                   'yaw_err_deg': (yaw_err_mod180(gt[gi][3], yaw_of(e.pose.orientation))
                                   if (gt[gi][1] == 'bottle' and e.shape == 1) else None)}
    return out, len(estimates) - len(used_e)


def run_live(args, layout, plan):
    import subprocess

    import rclpy
    from action_msgs.msg import GoalStatus
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    from gazebo_msgs.srv import DeleteEntity, SpawnEntity
    from graspsort_msgs.action import SortObjects
    from rclpy.action import ActionClient

    sys.path.insert(0, os.path.join(get_package_prefix('graspsort_manipulation'), 'lib',
                                    'graspsort_manipulation'))
    import pick_place_trials as ppt  # reuse the settle, rest and bin checks

    status_names = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
                    GoalStatus.STATUS_ABORTED: 'ABORTED', GoalStatus.STATUS_CANCELED: 'CANCELED'}
    params = ppt.load_params(args.params)
    bin_map = dict(zip(params['bin_classes'], params['bin_names']))
    models_dir = args.models_dir or os.path.join(REPO, 'src', 'graspsort_gazebo',
                                                 'models_external')
    if not os.path.isdir(models_dir):
        models_dir = os.path.join(get_package_share_directory('graspsort_gazebo'),
                                  'models_external')
    sdf = {}
    for m in MODELS:
        with open(os.path.join(models_dir, m, 'model.sdf'), 'r', encoding='utf-8') as f:
            sdf[m] = f.read()
    for m in MODELS:  # pick_place_trials.MODEL_INFO must agree with the footprint table here
        if abs(ppt.MODEL_INFO[m]['link_z'] - MODELS[m]['link_z']) > 1e-9:
            raise RuntimeError(f'link_z mismatch for {m}')
    table_z = float(layout['table']['z']) if args.table_z is None else args.table_z
    run_id = args.run_id or os.path.basename(os.path.normpath(args.out_dir))
    os.makedirs(args.out_dir, exist_ok=True)
    out_path = os.path.join(args.out_dir, 'trials.jsonl')

    class Runner(ppt.Harness):
        def __init__(self):
            super().__init__()
            self.spawn = self.create_client(SpawnEntity, '/spawn_entity')
            self.delete = self.create_client(DeleteEntity, '/delete_entity')
            self.client = ActionClient(self, SortObjects, '/sort_objects')

        def call(self, client, req, what):
            if not client.wait_for_service(timeout_sec=args.service_timeout):
                raise RuntimeError(f'{what}: service not available')
            fut = client.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=args.service_timeout)
            if not fut.done() or fut.result() is None:
                raise RuntimeError(f'{what}: no response')
            return fut.result()

        def wait_models(self):
            end = time.monotonic() + args.service_timeout
            while self.models is None and time.monotonic() < end:
                self.spin_for(0.1)
            if self.models is None:
                raise RuntimeError('/gazebo/model_states not received')

        def clear_objects(self):
            self.wait_models()
            stale = [n for n in self.models.name if n in WORLD_OBJECTS or SPAWNED_RE.match(n)]
            for n in stale:
                req = DeleteEntity.Request()
                req.name = n
                res = self.call(self.delete, req, f'delete {n}')
                if not res.success:
                    raise RuntimeError(f'delete {n} failed: {res.status_message}')
            if stale:
                self.models = None
                self.wait_models()
            left = [n for n in self.models.name if n in stale]
            if left:
                raise RuntimeError(f'still in Gazebo after delete: {left}')
            return stale

        def wait_objects_empty(self, timeout):
            end = time.monotonic() + timeout
            self.objects = None
            while time.monotonic() < end:
                self.spin_for(0.1)
                if self.objects is not None and len(self.objects.objects) == 0:
                    return True
            return False

        def spawn_objects(self, objs):
            names = set(self.models.name)
            for o in objs:
                if o['name'] in names:
                    raise RuntimeError(f'name {o["name"]} already in Gazebo (D-06 unique names)')
                req = SpawnEntity.Request()
                req.name = o['name']
                req.xml = sdf[o['model']]
                req.reference_frame = 'world'
                req.initial_pose.position.x = o['x']
                req.initial_pose.position.y = o['y']
                req.initial_pose.position.z = table_z + args.spawn_dz
                q = ppt.yaw_quat(o['yaw'])
                (req.initial_pose.orientation.x, req.initial_pose.orientation.y,
                 req.initial_pose.orientation.z, req.initial_pose.orientation.w) = q
                res = self.call(self.spawn, req, f'spawn {o["name"]}')
                if not res.success:
                    raise RuntimeError(f'spawn {o["name"]} failed: {res.status_message}')
            end = time.monotonic() + args.service_timeout
            while time.monotonic() < end:
                self.spin_for(0.1)
                if self.models and all(o['name'] in self.models.name for o in objs):
                    return
            raise RuntimeError('spawned models did not appear on /gazebo/model_states')

        def gt_yaws(self, names):
            msg = self.models
            return {n: yaw_of(msg.pose[msg.name.index(n)].orientation)
                    for n in names if n in msg.name}

        def sort(self, notes):
            if not self.client.wait_for_server(timeout_sec=args.server_timeout):
                raise RuntimeError('/sort_objects not available')
            fb = {'count': 0}

            def on_feedback(_msg):
                fb['count'] += 1

            idle_end = time.monotonic() + args.idle_timeout
            while True:
                fut = self.client.send_goal_async(SortObjects.Goal(),
                                                  feedback_callback=on_feedback)
                rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
                gh = fut.result() if fut.done() else None
                if gh is not None and gh.accepted:
                    break
                if time.monotonic() > idle_end:
                    return {'status': 'REJECTED', 'picked': 0, 'failed': 0,
                            'failure_reasons': [], 'duration': 0.0,
                            'feedback_count': 0}, None
                notes.append('goal rejected (node busy?), retrying')
                self.spin_for(5.0)
            t0 = time.monotonic()
            goal_id = bytes(gh.goal_id.uuid).hex()
            res_fut = gh.get_result_async()
            rclpy.spin_until_future_complete(self, res_fut, timeout_sec=args.goal_timeout)
            timed_out = not res_fut.done()
            if timed_out:
                notes.append(f'no result within {args.goal_timeout} s: cancel')
                gh.cancel_goal_async()
                rclpy.spin_until_future_complete(self, res_fut, timeout_sec=args.cancel_timeout)
                if not res_fut.done():
                    notes.append('no result after cancel either')
            duration = time.monotonic() - t0
            res = res_fut.result() if res_fut.done() else None
            action = {'status': 'TIMEOUT' if timed_out else
                      status_names.get(res.status, str(res.status)),
                      'picked': int(res.result.picked) if res else 0,
                      'failed': int(res.result.failed) if res else 0,
                      'failure_reasons': list(res.result.failure_reasons) if res else [],
                      'duration': duration, 'feedback_count': fb['count']}
            return action, goal_id

    rclpy.init()
    h = Runner()
    rows = []
    try:
        for p in plan:
            t_start = time.monotonic()
            notes = []
            objs = p['objects']
            cfg = p['config']
            print(f'\n=== config {cfg["n_objects"]}:{cfg["min_gap"]:.2f} trial {p["trial"]} '
                  f'seed {p["seed"]} min_gap {min_gap(objs):.4f} m ===', flush=True)
            deleted = h.clear_objects()
            if deleted:
                print(f'deleted {deleted}', flush=True)
            if not h.wait_objects_empty(args.clear_timeout):
                notes.append(f'/objects_3d not empty {args.clear_timeout} s after the delete')
            h.spawn_objects(objs)
            fake = {'objects': {'names': [o['name'] for o in objs],
                                **{o['name']: {'model': o['model']} for o in objs}}}
            h.spin_for(1.0)
            if not h.wait_rest(fake, args.rest_speed, args.rest_time, args.rest_timeout):
                notes.append('objects not at rest after spawn')
            if not h.wait_perception(len(objs), args.settle_time, args.settle_tol,
                                     args.settle_timeout):
                n_seen = len(h.objects.objects) if h.objects else 0
                notes.append(f'perception did not settle on {len(objs)} objects '
                             f'(last count {n_seen}); goal sent anyway')
            t_ready = time.monotonic()
            h.spin_for(0.3)
            centres = h.model_centres(fake)
            yaws = h.gt_yaws(fake['objects']['names'])
            gt = [(o['name'], MODELS[o['model']]['class'], centres[o['name']][0],
                   yaws[o['name']]) for o in objs]
            estimates = list(h.objects.objects) if h.objects else []
            loc, extra = localization(gt, estimates, args.match_radius)
            if extra:
                notes.append(f'{extra} unmatched /objects_3d estimates at the snapshot')

            action, goal_id = h.sort(notes)
            h.wait_rest(fake, args.rest_speed, args.rest_time, args.rest_timeout)
            h.spin_for(0.3)
            final = h.model_centres(fake)
            records = []
            for o, lo in zip(objs, loc):
                info = MODELS[o['model']]
                c = final[o['name']][0] if o['name'] in final else None
                in_bin = None
                if c is not None:
                    for b in layout['bins']['names']:
                        inside, _ = ppt.bin_check(c, info['link_z'], layout, b, args.xy_margin,
                                                  args.z_margin)
                        if inside:
                            in_bin = b
                else:
                    notes.append(f'{o["name"]} missing from /gazebo/model_states at the end')
                expected = bin_map[info['class']]
                records.append({
                    'name': o['name'], 'class': info['class'], 'model': o['model'],
                    'spawn': {'x': o['x'], 'y': o['y'], 'yaw': o['yaw']},
                    'localization': lo,
                    'final': ({'x': c[0], 'y': c[1], 'z': c[2]} if c else None),
                    'expected_bin': expected, 'final_bin': in_bin,
                    'correct': in_bin == expected})
            if action['status'] != 'SUCCEEDED' and not args.no_ready:
                r = subprocess.run(['ros2', 'run', 'graspsort_bringup', 'move_named.py', 'ready'],
                                   capture_output=True, text=True, timeout=180)
                notes.append(f'arm sent to ready (exit {r.returncode})')
            t_end = time.monotonic()
            row = {'run_id': run_id, 'config': cfg, 'trial': p['trial'], 'seed': p['seed'],
                   'min_gap_actual': min_gap(objs), 'layout_digest': layout_digest(objs),
                   'objects': records, 'action': action, 'goal_id': goal_id, 'notes': notes,
                   'timing': {'setup': t_ready - t_start, 'goal': action['duration'],
                              'total': t_end - t_start}}
            with open(out_path, 'a', encoding='utf-8') as f:
                f.write(json.dumps(row) + '\n')
                f.flush()
                os.fsync(f.fileno())
            rows.append(row)
            n_ok = sum(r['correct'] for r in records)
            n_loc = sum(r['localization']['matched'] for r in records)
            print(f'TRIAL {cfg["n_objects"]}:{cfg["min_gap"]:.2f} t{p["trial"]} seed {p["seed"]} '
                  f'{action["status"]} picked {action["picked"]} failed {action["failed"]} '
                  f'correct {n_ok}/{len(records)} localized {n_loc}/{len(records)} '
                  f'setup {t_ready - t_start:.1f} s goal {action["duration"]:.1f} s '
                  f'total {t_end - t_start:.1f} s'
                  f'{" notes: " + "; ".join(notes) if notes else ""}', flush=True)
    finally:
        h.destroy_node()
        rclpy.shutdown()
    total = sum(len(r['objects']) for r in rows)
    ok = sum(o['correct'] for r in rows for o in r['objects'])
    print(f'\n{len(rows)} trials, {ok}/{total} objects in the correct bin; {out_path}')
    return 0


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if '--ros-args' in argv:
        argv = argv[:argv.index('--ros-args')]
    args = parse_args(argv)
    with open(args.layout, 'r', encoding='utf-8') as f:
        layout = yaml.safe_load(f)['/**']['ros__parameters']
    plan = plan_trials(args, layout)  # samples everything first: fails before touching the sim
    if args.dry_run:
        print_plan(plan)
        return 0
    if not args.out_dir:
        print('--out-dir is required', file=sys.stderr)
        return 2
    return run_live(args, layout, plan)


if __name__ == '__main__':
    sys.exit(main())
