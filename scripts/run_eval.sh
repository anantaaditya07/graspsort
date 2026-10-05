#!/usr/bin/env bash
# GraspSort Phase 6 evaluation (architecture 8, D-17): headless sim + perception + scene manager +
# sort_task_node, then scripts/eval_run.py, then scripts/eval_summary.py (if present).
#
# Usage: scripts/run_eval.sh [eval_run.py args...]     e.g. scripts/run_eval.sh --trials 1
# Env (optional):
#   RUN_ID             run name (default: date +%Y%m%d_%H%M%S); output in data/eval/<RUN_ID>/
#   ROS_DOMAIN_ID      passed through (isolate from other ROS graphs)
#   GAZEBO_MASTER_URI  passed through (isolate from other gzservers)
#   GRASPSORT_SETUP    setup.bash to source (default: <repo>/install/setup.bash)
#   READY_TIMEOUT      s to wait for /sort_objects (default 300)
#   SHUTDOWN_TIMEOUT   s between SIGINT and SIGKILL at cleanup (default 20)
#   SUMMARY_OUT        eval_summary.py --out (default: data/eval/<RUN_ID>/results.md;
#                      set to docs/results.md for the reported run)
# Output: data/eval/<RUN_ID>/{trials.jsonl,node_metrics.jsonl,launch_*.log,eval_run.log}
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_ID="${RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
RUN_DIR="${REPO}/data/eval/${RUN_ID}"
READY_TIMEOUT="${READY_TIMEOUT:-300}"
SHUTDOWN_TIMEOUT="${SHUTDOWN_TIMEOUT:-20}"
SETUP="${GRASPSORT_SETUP:-${REPO}/install/setup.bash}"
SUMMARY_OUT="${SUMMARY_OUT:-${RUN_DIR}/results.md}"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
[[ -n "${ROS_DOMAIN_ID:-}" ]] && export ROS_DOMAIN_ID
[[ -n "${GAZEBO_MASTER_URI:-}" ]] && export GAZEBO_MASTER_URI

if [[ ! -f "${SETUP}" ]]; then
  echo "run_eval: ${SETUP} not found (build the workspace first)" >&2
  exit 1
fi
set +u  # ROS setup scripts read unset variables
# shellcheck disable=SC1090
source "${SETUP}"
set -u

mkdir -p "${RUN_DIR}"
echo "run_eval: run ${RUN_ID} -> ${RUN_DIR} (ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}," \
  "GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI:-default})"

PGIDS=()
cleanup() {
  local code=$?
  trap - EXIT INT TERM
  if ((${#PGIDS[@]})); then
    echo "run_eval: stopping ${#PGIDS[@]} launch process groups"
    local pg
    for pg in "${PGIDS[@]}"; do kill -INT -- "-${pg}" 2>/dev/null || true; done
    local end=$((SECONDS + SHUTDOWN_TIMEOUT))
    while ((SECONDS < end)); do
      local alive=0
      for pg in "${PGIDS[@]}"; do kill -0 -- "-${pg}" 2>/dev/null && alive=1; done
      ((alive)) || break
      sleep 0.5
    done
    for pg in "${PGIDS[@]}"; do kill -KILL -- "-${pg}" 2>/dev/null || true; done
  fi
  echo "run_eval: done (exit ${code}), output in ${RUN_DIR}"
  exit "${code}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# Start a launch in its own process group (setsid), log to the run dir.
start_launch() {
  local name=$1
  shift
  setsid ros2 launch "$@" >"${RUN_DIR}/launch_${name}.log" 2>&1 &
  PGIDS+=("$!")
  echo "run_eval: started ${name} (pgid $!): ros2 launch $*"
}

start_launch sim graspsort_bringup sim.launch.py gui:=false rviz:=false
start_launch perception graspsort_perception perception.launch.py
start_launch scene graspsort_scene scene.launch.py
start_launch sort graspsort_manipulation sort_task.launch.py \
  "metrics_log_path:=${RUN_DIR}/node_metrics.jsonl"

echo "run_eval: waiting up to ${READY_TIMEOUT} s for /sort_objects"
python3 - "${READY_TIMEOUT}" <<'EOF'
import sys
import rclpy
from graspsort_msgs.action import SortObjects
from rclpy.action import ActionClient
rclpy.init()
node = rclpy.create_node('run_eval_ready')
ok = ActionClient(node, SortObjects, '/sort_objects').wait_for_server(timeout_sec=float(sys.argv[1]))
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if ok else 1)
EOF
echo "run_eval: /sort_objects is up"

python3 "${REPO}/scripts/eval_run.py" --out-dir "${RUN_DIR}" --run-id "${RUN_ID}" "$@" \
  2>&1 | tee "${RUN_DIR}/eval_run.log"

if [[ -f "${REPO}/scripts/eval_summary.py" ]]; then
  python3 "${REPO}/scripts/eval_summary.py" "${RUN_DIR}" --out "${SUMMARY_OUT}" 2>&1 |
    tee "${RUN_DIR}/eval_summary.log"
else
  echo "run_eval: scripts/eval_summary.py not found, skipping the summary"
fi
