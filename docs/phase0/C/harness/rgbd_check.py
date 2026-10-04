#!/usr/bin/env python3
"""Phase 0 / C RGB-D check (system python, ROS-sourced, harness/env.sh).

For one camera namespace: receives color, depth, camera_info for --secs wall seconds with
SensorData QoS, reports rates (wall and sim), encoding, frame ids, intrinsics, and compares the
depth at the projected pixel of known world points with the geometric ground truth.
Optionally spawns the osrf `beer` cylinder (flat top, r=0.055 m, h=0.23 m) at --beer-xy.
"""
import argparse
import math
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image

CAM_POSES = {  # world pose of the Gazebo camera link (x forward), from yolo_check.world
    "/camera": ((-0.643, 0.0, 1.516), 0.8727),
    "/camera_top": ((0.0, 0.0, 1.70), 1.5708),
}


def optical_axes(pitch):
    """World-frame unit vectors of the optical frame (x right, y down, z forward) for a
    camera link with roll=yaw=0 and the given pitch (link x = viewing direction)."""
    fwd = np.array([math.cos(pitch), 0.0, -math.sin(pitch)])
    left = np.array([0.0, 1.0, 0.0])
    up = np.cross(fwd, left)
    return -left, -up, fwd  # x_opt = right, y_opt = down, z_opt = forward


class Rx(Node):
    def __init__(self, ns):
        super().__init__("phase0_c_rgbd_check")
        self.msgs = {"color": [], "depth": [], "info": [], "dinfo": []}
        self.clock = None
        q = qos_profile_sensor_data
        self.create_subscription(Image, f"{ns}/color/image_raw", lambda m: self.msgs["color"].append(m), q)
        self.create_subscription(Image, f"{ns}/depth/image_raw", lambda m: self.msgs["depth"].append(m), q)
        self.create_subscription(CameraInfo, f"{ns}/color/camera_info", lambda m: self.msgs["info"].append(m), q)
        self.create_subscription(CameraInfo, f"{ns}/depth/camera_info", lambda m: self.msgs["dinfo"].append(m), q)
        self.create_subscription(Clock, "/clock", self.on_clock, qos_profile_sensor_data)  # /clock is best effort

    def on_clock(self, m):
        self.clock = m.clock.sec + m.clock.nanosec * 1e-9


def st(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ns", default="/camera")
    ap.add_argument("--secs", type=float, default=10.0)
    ap.add_argument("--points", default="0,0,0.75;0.2,0.15,0.75;-0.15,-0.1,0.75",
                    help="world points x,y,z;... (table top is z=0.75)")
    a = ap.parse_args()
    rclpy.init()
    n = Rx(a.ns)
    t_end = time.time() + 2.0
    while time.time() < t_end:
        rclpy.spin_once(n, timeout_sec=0.05)
    for v in n.msgs.values():
        v.clear()
    c0 = n.clock
    t0 = time.time()
    while time.time() - t0 < a.secs:
        rclpy.spin_once(n, timeout_sec=0.05)
    wall = time.time() - t0
    sim = n.clock - c0
    print(f"window: wall {wall:.1f} s, sim {sim:.2f} s (RTF {sim / wall:.2f})")
    for k, v in n.msgs.items():
        if not v:
            print(f"{k}: NO MESSAGES")
            continue
        stamps = [st(m) for m in v]
        span = stamps[-1] - stamps[0]
        exp = int(round(span * 10)) + 1  # 10 Hz sensor
        print(f"{k}: {len(v)} msgs, {len(v) / wall:.2f} Hz wall, stamp span {span:.2f} s sim -> "
              f"{(len(v) - 1) / span if span > 0 else 0:.2f} Hz sim; expected {exp} at 10 Hz sim "
              f"-> lost {exp - len(v)}; frame_id='{v[-1].header.frame_id}'")
    col, dep, info = n.msgs["color"][-1], n.msgs["depth"][-1], n.msgs["info"][-1]
    print(f"color: {col.width}x{col.height} encoding={col.encoding}")
    print(f"depth: {dep.width}x{dep.height} encoding={dep.encoding} step={dep.step}")
    print(f"camera_info: K={[round(x, 3) for x in info.k]} D={list(info.d)} model={info.distortion_model}")
    print(f"same frame_id color/depth: {col.header.frame_id == dep.header.frame_id}; "
          f"color/depth stamps equal (last pair): {st(col) == st(dep)}")
    d = CvBridge().imgmsg_to_cv2(dep, "passthrough").astype(np.float32)
    print(f"depth stats: min {np.nanmin(d):.3f} max {np.nanmax(d):.3f} nan {int(np.isnan(d).sum())} "
          f"inf {int(np.isinf(d).sum())}")
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    pos, pitch = CAM_POSES[a.ns]
    xo, yo, zo = optical_axes(pitch)
    c = np.array(pos)
    for ptxt in a.points.split(";"):
        p = np.array([float(x) for x in ptxt.split(",")])
        r = p - c
        X, Y, Z = r @ xo, r @ yo, r @ zo
        u, v = fx * X / Z + cx, fy * Y / Z + cy
        ui, vi = int(round(u)), int(round(v))
        meas = float(d[vi, ui])
        print(f"point {p.tolist()}: optical (X,Y,Z)=({X:.4f},{Y:.4f},{Z:.4f}) -> pixel "
              f"({u:.1f},{v:.1f}); GT z-depth {Z:.4f} m, euclid {np.linalg.norm(r):.4f} m; "
              f"measured depth[{vi},{ui}]={meas:.4f} m; error {meas - Z:+.4f} m")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
