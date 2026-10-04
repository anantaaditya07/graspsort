#!/usr/bin/env python3
"""Phase 0 / C: spawn candidate objects one at a time on the table and save RGB + depth
from both harness cameras (oblique /camera, top-down /camera_top).

System python in a ROS-sourced shell (source harness/env.sh first):
    /usr/bin/python3 grab_objects.py --out $C_SCRATCH/frames [--models a,b] [--clutter]

Every object is spawned as a STATIC model (no physics) resting on the table top, so the pose
is exact. For each pose the first color+depth frame whose stamp is >= settle seconds after
the spawn is saved: <out>/<cam>/<tag>.png (bgr8) and <tag>_depth.npy (float32 metres).
An empty-table frame (tag "background") is saved first; the evaluator derives the ground-truth
box from the depth difference to it. A manifest.csv lists every saved pose.
"""

import argparse
import csv
import math
import re
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

TABLE_Z = 0.75
SCRATCH = Path("/tmp/claude-1000/-home-adityachavali-graspsort/"
               "b1f9bd71-1b1a-4b44-834a-7b7601ea7da7/scratchpad/C/gzmodels")
CAMS = {"oblique": "/camera", "top": "/camera_top",
        "oblique_near": "/camera_near", "top_near": "/camera_top_near"}
POSITIONS = [(0.0, 0.0), (0.15, 0.12), (-0.15, 0.12), (0.15, -0.12), (-0.12, -0.15)]
YAWS = [0.0, 2.0]

# name -> (intended class, source kind, extra z offset so the object rests on the table)
MODELS = {
    "plastic_cup": ("cup", "osrf", 0.0),
    "pitcher_base": ("cup", "fuel:Pitcher_Base", None),
    "mustard_bottle": ("bottle", "fuel:Mustard_Bottle", None),
    "beer": ("bottle", "osrf", 0.0),
    "coke_can": ("can", "osrf", 0.0),
    "master_chef_can": ("can", "fuel:Master_Chef_Can", None),
    "potted_meat_can": ("can", "fuel:Potted_Meat_Can", None),
    "cracker_box": ("box", "fuel:Cracker_Box", None),
    "cardboard_box": ("box", "osrf", 0.15),
    "cricket_ball": ("ball", "osrf", 0.0),
    "robocup_spl_ball": ("ball", "osrf", 0.0325),
    "robocup_3Dsim_ball": ("ball", "osrf", 0.04),
}


def obj_min_z(path: Path) -> float:
    zs = [float(line.split()[3]) for line in path.open() if line.startswith("v ")]
    return min(zs)


def model_xml(name: str):
    """Return (sdf string, z offset)."""
    _, kind, zoff = MODELS[name]
    if kind == "osrf":
        xml = (SCRATCH / name / "model.sdf").read_text()
        xml = re.sub(r"(<model\s+name=\"[^\"]*\"\s*>)", r"\1<static>true</static>", xml, count=1)
        return xml, zoff
    folder = SCRATCH / "fuel" / kind.split(":", 1)[1]
    obj = folder / "textured.obj"
    xml = f"""<?xml version="1.0"?><sdf version="1.6"><model name="{name}"><static>true</static>
<link name="link"><visual name="visual"><geometry><mesh><uri>file://{obj}</uri></mesh></geometry>
</visual><collision name="collision"><geometry><mesh><uri>file://{obj}</uri></mesh></geometry>
</collision></link></model></sdf>"""
    return xml, -obj_min_z(obj)


class Grabber(Node):
    def __init__(self):
        super().__init__("phase0_c_grabber")
        self.bridge = CvBridge()
        self.latest = {}  # (cam, kind) -> msg
        for cam, ns in CAMS.items():
            self.create_subscription(Image, f"{ns}/color/image_raw",
                                     lambda m, c=cam: self.latest.__setitem__((c, "rgb"), m),
                                     qos_profile_sensor_data)
            self.create_subscription(Image, f"{ns}/depth/image_raw",
                                     lambda m, c=cam: self.latest.__setitem__((c, "depth"), m),
                                     qos_profile_sensor_data)
        self.spawn_cli = self.create_client(SpawnEntity, "/spawn_entity")
        self.delete_cli = self.create_client(DeleteEntity, "/delete_entity")

    def call(self, cli, req, timeout=10.0):
        if not cli.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"service {cli.srv_name} unavailable")
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if not fut.done():
            raise RuntimeError(f"service {cli.srv_name} timed out")
        return fut.result()

    def spawn(self, ent, xml, x, y, z, yaw):
        req = SpawnEntity.Request()
        req.name, req.xml, req.reference_frame = ent, xml, "world"
        p = Pose()
        p.position.x, p.position.y, p.position.z = x, y, z
        p.orientation.z, p.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        req.initial_pose = p
        res = self.call(self.spawn_cli, req)
        if not res.success:
            raise RuntimeError(f"spawn {ent} failed: {res.status_message}")

    def delete(self, ent):
        req = DeleteEntity.Request()
        req.name = ent
        self.call(self.delete_cli, req)

    def stamp_of(self, key):
        m = self.latest.get(key)
        return None if m is None else m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    def wait_fresh(self, after: float, timeout=20.0):
        """Spin until every (cam, kind) has a frame stamped >= after; return the msgs."""
        keys = [(c, k) for c in CAMS for k in ("rgb", "depth")]
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if all((self.stamp_of(k) or -1) >= after for k in keys):
                return {k: self.latest[k] for k in keys}
        raise RuntimeError(f"no fresh frames after t={after:.2f}")

    def newest_stamp(self):
        rclpy.spin_once(self, timeout_sec=0.05)
        s = [self.stamp_of(k) for k in self.latest]
        return max(s) if s else 0.0

    def object_visible(self, msgs, bg, thr=0.004, min_px=20):
        """True when, in every camera, >= min_px depth pixels differ from the empty table."""
        for cam in CAMS:
            d = np.asarray(self.bridge.imgmsg_to_cv2(msgs[(cam, "depth")], "passthrough"),
                           dtype=np.float32)
            diff = np.abs(np.nan_to_num(d, nan=10, posinf=10) -
                          np.nan_to_num(bg[cam], nan=10, posinf=10))
            if int((diff > thr).sum()) < min_px:
                return False
        return True

    def save(self, msgs, out: Path, tag: str):
        for cam in CAMS:
            d = out / cam
            d.mkdir(parents=True, exist_ok=True)
            rgb = self.bridge.imgmsg_to_cv2(msgs[(cam, "rgb")], desired_encoding="bgr8")
            cv2.imwrite(str(d / f"{tag}.png"), rgb)
            dm = msgs[(cam, "depth")]
            depth = self.bridge.imgmsg_to_cv2(dm, desired_encoding="passthrough")
            np.save(d / f"{tag}_depth.npy", np.asarray(depth, dtype=np.float32))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--models", default=",".join(MODELS))
    ap.add_argument("--positions", type=int, default=len(POSITIONS))
    ap.add_argument("--yaws", type=int, default=len(YAWS))
    ap.add_argument("--settle", type=float, default=2.0,
                    help="sim seconds after spawn (render lags the spawn; 0.3-1.0 s showed stale frames)")
    ap.add_argument("--max-wait-tries", type=int, default=10)
    ap.add_argument("--clutter", action="store_true", help="also save one 5-object frame")
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = Grabber()
    t = node.newest_stamp()
    while t == 0.0:
        t = node.newest_stamp()
    msgs = node.wait_fresh(t + args.settle)
    node.save(msgs, out, "background")
    bg = {c: np.asarray(node.bridge.imgmsg_to_cv2(msgs[(c, "depth")], "passthrough"),
                        dtype=np.float32) for c in CAMS}
    manifest = out / "manifest.csv"
    new = not manifest.exists()
    with manifest.open("a", newline="") as fh:
        w = csv.writer(fh)
        if new:
            w.writerow(["tag", "model", "cls", "x", "y", "z", "yaw"])
        for name in args.models.split(","):
            xml, zoff = model_xml(name)
            for pi, (x, y) in enumerate(POSITIONS[:args.positions]):
                for yi, yaw in enumerate(YAWS[:args.yaws]):
                    tag = f"{name}_p{pi}_y{yi}"
                    # unique entity name: re-using one name right after delete_entity let the
                    # rendering drop the new visual in ~15% of spawns (object missing in frame)
                    ent = f"obj_{tag}"
                    z = TABLE_Z + zoff
                    node.spawn(ent, xml, x, y, z, yaw)
                    t = node.newest_stamp()
                    msgs = node.wait_fresh(t + args.settle)
                    tries = 0
                    while not node.object_visible(msgs, bg) and tries < args.max_wait_tries:
                        tries += 1  # render still lagging: wait for newer frames
                        msgs = node.wait_fresh(node.newest_stamp() + 0.5)
                    if tries:
                        print(f"{tag}: waited {tries} extra 0.5 s windows for the render",
                              flush=True)
                    if not node.object_visible(msgs, bg):
                        print(f"WARNING {tag}: object not visible in depth", flush=True)
                    node.save(msgs, out, tag)
                    node.delete(ent)
                    w.writerow([tag, name, MODELS[name][0], x, y, f"{z:.4f}", yaw])
                    fh.flush()
                    print(f"saved {tag}", flush=True)
        if args.clutter:
            pick = ["plastic_cup", "mustard_bottle", "coke_can", "cracker_box", "cricket_ball"]
            spots = [(-0.18, -0.15), (0.0, -0.18), (0.18, 0.0), (0.0, 0.15), (-0.18, 0.12)]
            for i, (name, (x, y)) in enumerate(zip(pick, spots)):
                xml, zoff = model_xml(name)
                node.spawn(f"clutter_{i}", xml, x, y, TABLE_Z + zoff, 0.5 * i)
            t = node.newest_stamp()
            msgs = node.wait_fresh(t + args.settle)
            node.save(msgs, out, "clutter")
            for i in range(len(pick)):
                node.delete(f"clutter_{i}")
            print("saved clutter", flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
