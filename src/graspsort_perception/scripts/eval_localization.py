#!/usr/bin/env python3
"""Measure object_localizer_node 3D error against Gazebo ground truth (architecture section 8).

Ground truth is /gazebo/model_states (evaluation only, never used by the robot). Each object model
origin is its bottom centre (scripts/fetch_models.sh), so the true centre is origin + height / 2.

For every ground-truth object (selected by name prefix) the script takes the latest /objects_3d
estimate of the same class that is nearest in 3D (within --max-match-dist) and reports the
position error (3D, horizontal, vertical), size and yaw. Objects without a match are reported as
MISSED. Exits non-zero if any object is missed, or if the median 3D error exceeds --pass-median.

Run in a ROS-sourced shell while the sim and perception are running:
    ros2 run graspsort_perception eval_localization.py --duration 20 --csv /tmp/err.csv
"""

import argparse
import csv
import math
import statistics
import sys
import time

import rclpy
from gazebo_msgs.msg import ModelStates
from graspsort_msgs.msg import ObjectPoseArray
from rclpy.node import Node

# Model name prefix -> (detector class name, object height [m]).
# Heights from the physical-parameter table in scripts/fetch_models.sh
# (cricket_ball r 0.0375, mustard_bottle h 0.1913). No cups in the world (D-13).
DEFAULT_OBJECTS = [
    "ball_:sports ball:0.075",
    "bottle_:bottle:0.1913",
]


def parse_objects(specs):
    table = []
    for spec in specs:
        prefix, cls, height = spec.split(":")
        table.append((prefix, cls, float(height)))
    return table


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Evaluator(Node):
    def __init__(self, args):
        super().__init__("eval_localization")
        self.objects = parse_objects(args.objects)
        self.truth = {}  # name -> (class, x, y, z_centre, yaw)
        self.estimates = {}  # id -> ObjectPose (latest)
        self.create_subscription(ModelStates, args.model_states_topic, self.on_states, 10)
        self.create_subscription(ObjectPoseArray, args.objects_topic, self.on_objects, 10)

    def on_states(self, msg):
        for name, pose in zip(msg.name, msg.pose):
            for prefix, cls, height in self.objects:
                if name.startswith(prefix):
                    p = pose.position
                    self.truth[name] = (cls, p.x, p.y, p.z + height / 2.0, yaw_of(pose.orientation))

    def on_objects(self, msg):
        for obj in msg.objects:
            self.estimates[obj.id] = obj


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--duration", type=float, default=20.0, help="collection time [s]")
    ap.add_argument("--objects", nargs="+", default=DEFAULT_OBJECTS,
                    help="prefix:class:height specs (default: %(default)s)")
    ap.add_argument("--objects-topic", default="/objects_3d")
    ap.add_argument("--model-states-topic", default="/gazebo/model_states")
    ap.add_argument("--max-match-dist", type=float, default=0.10, help="[m]")
    ap.add_argument("--pass-median", type=float, default=0.015,
                    help="pass threshold on the median 3D error [m] (architecture 8: 1.5 cm)")
    ap.add_argument("--csv", default="", help="optional per-object CSV output")
    args = ap.parse_args()

    rclpy.init()
    node = Evaluator(args)
    end = time.monotonic() + args.duration
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    rows = []
    for name in sorted(node.truth):
        cls, tx, ty, tz, tyaw = node.truth[name]
        best, best_d = None, float("inf")
        for est in node.estimates.values():
            if est.class_name != cls:
                continue
            p = est.pose.position
            d = math.dist((p.x, p.y, p.z), (tx, ty, tz))
            if d < best_d:
                best, best_d = est, d
        if best is None or best_d > args.max_match_dist:
            rows.append({"object": name, "class": cls, "status": "MISSED"})
            continue
        p = best.pose.position
        rows.append({
            "object": name, "class": cls, "status": "OK", "id": best.id,
            "err_3d_mm": 1000 * best_d,
            "err_xy_mm": 1000 * math.hypot(p.x - tx, p.y - ty),
            "err_z_mm": 1000 * (p.z - tz),
            "est_size": "%.3f %.3f %.3f" % (best.size.x, best.size.y, best.size.z),
            "est_yaw_deg": math.degrees(yaw_of(best.pose.orientation)),
            "true_yaw_deg": math.degrees(tyaw),
            "confidence": best.confidence,
        })

    print("%-9s %-12s %-6s %8s %8s %8s  %-18s %8s" % (
        "object", "class", "status", "3D mm", "xy mm", "z mm", "size x y z [m]", "yaw deg"))
    for r in rows:
        if r["status"] != "OK":
            print("%-9s %-12s %-6s" % (r["object"], r["class"], r["status"]))
            continue
        print("%-9s %-12s %-6s %8.1f %8.1f %8.1f  %-18s %8.1f" % (
            r["object"], r["class"], r["status"], r["err_3d_mm"], r["err_xy_mm"], r["err_z_mm"],
            r["est_size"], r["est_yaw_deg"]))

    ok = [r for r in rows if r["status"] == "OK"]
    missed = len(rows) - len(ok)
    if not rows:
        print("RESULT: FAIL (no ground-truth objects on %s)" % args.model_states_topic)
        sys.exit(1)
    for cls in sorted({r["class"] for r in ok}):
        errs = [r["err_3d_mm"] for r in ok if r["class"] == cls]
        print("class %-12s n=%d median 3D error %.1f mm, max %.1f mm" % (
            cls, len(errs), statistics.median(errs), max(errs)))
    median = statistics.median([r["err_3d_mm"] for r in ok]) if ok else float("inf")
    print("overall: %d/%d localized, median 3D error %.1f mm (pass < %.1f mm)" % (
        len(ok), len(rows), median, 1000 * args.pass_median))

    if args.csv:
        with open(args.csv, "w", newline="") as f:
            keys = ["object", "class", "status", "id", "err_3d_mm", "err_xy_mm", "err_z_mm",
                    "est_size", "est_yaw_deg", "true_yaw_deg", "confidence"]
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(rows)

    passed = missed == 0 and median < 1000 * args.pass_median
    print("RESULT: %s" % ("PASS" if passed else "FAIL"))
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
