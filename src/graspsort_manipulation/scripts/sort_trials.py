#!/usr/bin/env python3
"""Phase 5 evaluation harness for sort_task_node (EVAL ONLY: uses Gazebo ground truth).

For each run:
  1. reset every object in world_layout.yaml (objects.names) to its layout pose with
     /gazebo/set_entity_state (zero twist); --move NAME X Y then puts an object elsewhere
     (e.g. out of reach, for a failure-path check),
  2. wait until perception has settled (same check as pick_place_trials.py),
  3. send ONE SortObjects goal on /sort_objects (--classes, default empty = all) and print every
     feedback change,
  4. wait until every object rests, then check on /gazebo/model_states that every object of a
     sorted class (not listed in --expect-left) rests in its class bin (pick_place_trials.py bin
     check) and that no other object is in a bin.
PASS only if the bin check holds and the result reports picked == --expect-picked (default: the
number of objects that must be sorted) and failed == --expect-failed (default 0).
With --metrics-log (the node's metrics_log_path) the per-attempt lines of the goal are printed.

Example (sim + perception + scene manager + sort_task.launch.py running):
  ros2 run graspsort_manipulation sort_trials.py --runs 3
"""
import argparse
import json
import os
import sys
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from graspsort_msgs.action import SortObjects
from rclpy.action import ActionClient
from rclpy.utilities import remove_ros_args

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import pick_place_trials as ppt  # noqa: E402  (reuse the reset, settle and bin checks)

STATUS = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED',
          GoalStatus.STATUS_CANCELED: 'CANCELED'}


def parse_args(argv):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument('--runs', type=int, default=1)
    p.add_argument('--classes', nargs='*', default=[], help='goal classes (default: all)')
    p.add_argument('--layout', default=os.path.join(
        get_package_share_directory('graspsort_gazebo'), 'config', 'world_layout.yaml'))
    p.add_argument('--params', default=os.path.join(
        get_package_share_directory('graspsort_manipulation'), 'config', 'pick_place.yaml'))
    p.add_argument('--move', nargs=3, action='append', metavar=('NAME', 'X', 'Y'), default=[],
                   help='after the reset, put this object at x, y (table height)')
    p.add_argument('--expect-left', nargs='*', default=[],
                   help='objects expected to stay out of the bins (e.g. moved out of reach)')
    p.add_argument('--expect-picked', type=int, default=None)
    p.add_argument('--expect-failed', type=int, default=0)
    p.add_argument('--expect-objects', type=int, default=None,
                   help='objects perception must report before the goal (default: layout count)')
    p.add_argument('--settle-time', type=float, default=3.0)
    p.add_argument('--settle-tol', type=float, default=0.01)
    p.add_argument('--settle-timeout', type=float, default=60.0)
    p.add_argument('--goal-timeout', type=float, default=900.0)
    p.add_argument('--rest-speed', type=float, default=0.01)
    p.add_argument('--rest-time', type=float, default=1.0)
    p.add_argument('--rest-timeout', type=float, default=15.0)
    p.add_argument('--xy-margin', type=float, default=0.0)
    p.add_argument('--z-margin', type=float, default=0.01)
    p.add_argument('--metrics-log', default='', help="sort_task_node's metrics_log_path")
    return p.parse_args(argv)


class SortHarness(ppt.Harness):
    def __init__(self):
        super().__init__()
        self.client = ActionClient(self, SortObjects, '/sort_objects')

    def move_object(self, layout, name, x, y):
        o = layout['objects'][name]
        moved = {'objects': {'names': [name],
                             name: dict(o, x=float(x), y=float(y))}}
        self.reset_objects(moved)

    def sort(self, classes, timeout):
        if not self.client.wait_for_server(timeout_sec=30.0):
            raise RuntimeError('/sort_objects not available')
        t0 = time.monotonic()
        events = []

        def on_feedback(msg):
            fb = msg.feedback
            line = f'  [{time.monotonic() - t0:6.1f} s] {fb.stage:<10} {fb.current_object}'
            events.append(line)
            print(line, flush=True)

        goal = SortObjects.Goal()
        goal.classes = list(classes)
        fut = self.client.send_goal_async(goal, feedback_callback=on_feedback)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            raise RuntimeError('goal rejected')
        goal_id = bytes(gh.goal_id.uuid).hex()
        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=timeout)
        if not res_fut.done():
            gh.cancel_goal_async()
            raise RuntimeError(f'no result within {timeout} s')
        res = res_fut.result()
        return goal_id, res.status, res.result, time.monotonic() - t0, events


def main(argv=None):
    args = parse_args(remove_ros_args(argv if argv is not None else sys.argv)[1:])
    with open(args.layout, 'r', encoding='utf-8') as f:
        layout = yaml.safe_load(f)['/**']['ros__parameters']
    params = ppt.load_params(args.params)
    bin_map = dict(zip(params['bin_classes'], params['bin_names']))
    names = layout['objects']['names']
    obj_class = {n: ppt.MODEL_INFO[layout['objects'][n]['model']]['class'] for n in names}
    classes = args.classes or sorted(set(obj_class.values()))
    must_sort = [n for n in names if obj_class[n] in classes and n not in args.expect_left]
    expect_picked = len(must_sort) if args.expect_picked is None else args.expect_picked
    expect = args.expect_objects or len(names)

    rclpy.init()
    h = SortHarness()
    rows = []
    try:
        for run in range(1, args.runs + 1):
            print(f'\n=== run {run}/{args.runs}: classes {args.classes or "[] (all)"} ===',
                  flush=True)
            h.reset_objects(layout)
            for name, x, y in args.move:
                h.move_object(layout, name, x, y)
                print(f'moved {name} to ({x}, {y})')
            h.spin_for(1.0)
            if not h.wait_perception(expect, args.settle_time, args.settle_tol,
                                     args.settle_timeout):
                rows.append({'run': run, 'pass': False, 'why': 'perception did not settle'})
                print('perception did not settle')
                continue
            goal_id, status, res, wall, _ = h.sort(args.classes, args.goal_timeout)
            print(f'result: status {STATUS.get(status, status)}, picked {res.picked}, failed '
                  f'{res.failed}, wall {wall:.1f} s')
            for r in res.failure_reasons:
                print(f'  failure: {r}')
            h.wait_rest(layout, args.rest_speed, args.rest_time, args.rest_timeout)
            h.spin_for(0.3)
            centres = h.model_centres(layout)
            ok_bins = True
            where = {}
            for name in names:
                c, _ = centres[name]
                info = ppt.MODEL_INFO[layout['objects'][name]['model']]
                in_bin = None
                for b in layout['bins']['names']:
                    inside, _ = ppt.bin_check(c, info['half_height'], layout, b, args.xy_margin,
                                              args.z_margin)
                    if inside:
                        in_bin = b
                where[name] = in_bin or f'table ({c[0]:.3f}, {c[1]:.3f}, {c[2]:.3f})'
                want = bin_map[obj_class[name]] if name in must_sort else None
                if in_bin != want:
                    ok_bins = False
                print(f'  {name:<9} {obj_class[name]:<11} -> {where[name]}'
                      f'{"" if in_bin == want else "  (expected " + str(want) + ")"}')
            metrics = []
            if args.metrics_log and os.path.exists(args.metrics_log):
                with open(args.metrics_log, 'r', encoding='utf-8') as f:
                    metrics = [json.loads(line) for line in f if goal_id in line]
            for m in metrics:
                print(f'  attempt: {m["object"]} ({m["class"]}) #{m["attempt"]} '
                      f'{"OK" if m["success"] else "FAIL " + m["category"] + " at " + m["stage"]}'
                      f' cycle {m["cycle_time"]:.1f} s planning {m["planning_time"]:.2f} s')
            ok = (status == GoalStatus.STATUS_SUCCEEDED and ok_bins and
                  res.picked == expect_picked and res.failed == args.expect_failed)
            rows.append({'run': run, 'pass': ok, 'picked': res.picked, 'failed': res.failed,
                         'wall': wall, 'bins_ok': ok_bins, 'metrics': metrics,
                         'reasons': list(res.failure_reasons)})
            bins_txt = 'ok' if ok_bins else 'WRONG'
            print(f'run {run}: {"PASS" if ok else "FAIL"} (picked {res.picked}/{expect_picked},'
                  f' failed {res.failed}/{args.expect_failed}, bins {bins_txt})', flush=True)
    finally:
        h.destroy_node()
        rclpy.shutdown()
    print('\nsummary')
    for r in rows:
        cyc = [m['cycle_time'] for m in r.get('metrics', []) if m['success']]
        extra = (f' cycle mean {sum(cyc) / len(cyc):.1f} s max {max(cyc):.1f} s' if cyc else '')
        print(f'run {r["run"]}: {"PASS" if r["pass"] else "FAIL"} picked {r.get("picked", "-")} '
              f'failed {r.get("failed", "-")} wall {r.get("wall", 0):.1f} s{extra} '
              f'{r.get("why", "")}')
    passed = sum(r['pass'] for r in rows)
    print(f'{passed}/{len(rows)} PASS')
    return 0 if rows and passed == len(rows) else 1


if __name__ == '__main__':
    sys.exit(main())
