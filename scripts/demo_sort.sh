#!/usr/bin/env bash
# GraspSort demo (Phase 7): with the stack already running in another terminal, wait for
# move_group, perception (/objects_3d) and /sort_objects, send ONE SortObjects goal, print the
# feedback and the per-object results.
#
# Usage: scripts/demo_sort.sh [class ...]        e.g. scripts/demo_sort.sh bottle
#        (no classes = sort all known classes)
# Env (optional):
#   GRASPSORT_SETUP      setup.bash to source (default: <repo>/install/setup.bash)
#   READY_TIMEOUT        s to wait for move_group and /sort_objects (default 300)
#   OBJECTS_TIMEOUT      s to wait for /objects_3d (default 120)
#   MIN_OBJECTS          objects /objects_3d must report before the goal (default 1)
#   GOAL_TIMEOUT         s to wait for the result, then cancel (default 900)
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SETUP="${GRASPSORT_SETUP:-${REPO}/install/setup.bash}"
READY_TIMEOUT="${READY_TIMEOUT:-300}"
OBJECTS_TIMEOUT="${OBJECTS_TIMEOUT:-120}"
MIN_OBJECTS="${MIN_OBJECTS:-1}"
GOAL_TIMEOUT="${GOAL_TIMEOUT:-900}"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

if [[ ! -f "${SETUP}" ]]; then
  echo "demo_sort: ${SETUP} not found (build the workspace first)" >&2
  exit 1
fi
set +u  # ROS setup scripts read unset variables
# shellcheck disable=SC1090
source "${SETUP}"
set -u

python3 - "${READY_TIMEOUT}" "${OBJECTS_TIMEOUT}" "${MIN_OBJECTS}" "${GOAL_TIMEOUT}" "$@" <<'EOF'
import re
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from graspsort_msgs.action import SortObjects
from graspsort_msgs.msg import ObjectPoseArray
from rclpy.action import ActionClient

ready_timeout, objects_timeout = float(sys.argv[1]), float(sys.argv[2])
min_objects, goal_timeout = int(sys.argv[3]), float(sys.argv[4])
classes = sys.argv[5:]
STATUS = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED',
          GoalStatus.STATUS_CANCELED: 'CANCELED'}

rclpy.init()
node = rclpy.create_node('demo_sort')


def fail(msg):
    print(f'demo_sort: {msg}', file=sys.stderr, flush=True)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(1)


# 1. move_group
print(f'demo_sort: waiting up to {ready_timeout:g} s for move_group', flush=True)
end = time.monotonic() + ready_timeout
while 'move_group' not in node.get_node_names():
    if time.monotonic() > end:
        fail('move_group not running')
    rclpy.spin_once(node, timeout_sec=0.5)

# 2. /sort_objects
client = ActionClient(node, SortObjects, '/sort_objects')
print(f'demo_sort: waiting up to {ready_timeout:g} s for /sort_objects', flush=True)
if not client.wait_for_server(timeout_sec=ready_timeout):
    fail('/sort_objects not available')

# 3. perception
latest = {'msg': None}
node.create_subscription(ObjectPoseArray, '/objects_3d', lambda m: latest.update(msg=m), 10)
print(f'demo_sort: waiting up to {objects_timeout:g} s for >= {min_objects} objects on '
      '/objects_3d', flush=True)
end = time.monotonic() + objects_timeout
while latest['msg'] is None or len(latest['msg'].objects) < min_objects:
    if time.monotonic() > end:
        n = 0 if latest['msg'] is None else len(latest['msg'].objects)
        fail(f'/objects_3d reports {n} objects')
    rclpy.spin_once(node, timeout_sec=0.5)
seen = sorted(f'object_{o.id} ({o.class_name})' for o in latest['msg'].objects)
print(f'demo_sort: perception sees {len(seen)}: {", ".join(seen)}', flush=True)

# 4. one goal
t0 = time.monotonic()
stages = {}  # current_object -> last stage reached


def on_feedback(msg):
    fb = msg.feedback
    if fb.current_object:
        stages[fb.current_object] = fb.stage
    print(f'  [{time.monotonic() - t0:6.1f} s] {fb.stage:<10} {fb.current_object}', flush=True)


goal = SortObjects.Goal()
goal.classes = classes
print(f'demo_sort: sending SortObjects goal, classes {classes or "[] (all)"}', flush=True)
fut = client.send_goal_async(goal, feedback_callback=on_feedback)
rclpy.spin_until_future_complete(node, fut, timeout_sec=ready_timeout)
gh = fut.result()
if gh is None or not gh.accepted:
    fail('goal rejected')
res_fut = gh.get_result_async()
rclpy.spin_until_future_complete(node, res_fut, timeout_sec=goal_timeout)
if not res_fut.done():
    gh.cancel_goal_async()
    fail(f'no result within {goal_timeout:g} s (goal cancelled)')
res = res_fut.result()
r = res.result

# 5. per-object results: failures from failure_reasons, placed = reached the place stage
failed = {}
for reason in r.failure_reasons:
    m = re.match(r'(\S+ \([^)]*\))', reason)
    failed[m.group(1) if m else reason] = reason
print(f'\nresult: {STATUS.get(res.status, res.status)}, picked {r.picked}, failed {r.failed}, '
      f'{time.monotonic() - t0:.1f} s', flush=True)
for obj, stage in stages.items():
    if obj in failed:
        print(f'  FAILED  {failed.pop(obj)}')
    elif stage == 'place':
        print(f'  PLACED  {obj}')
    else:
        print(f'  ?       {obj} (last stage {stage})')
for reason in failed.values():
    print(f'  FAILED  {reason}')
ok = res.status == GoalStatus.STATUS_SUCCEEDED and r.failed == 0
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if ok else 2)
EOF
