#!/usr/bin/env python3
"""Measure Robotiq 2F-85 fingertip geometry via TF (robot_state_publisher, sim time) at open/closed:
pad-centre separation (opening) and z of pad centres in robotiq_85_base_link -> TCP offset candidate.
Pad centre approximated by the fingertip link inertial origin from the URDF."""
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.duration import Duration
from control_msgs.action import GripperCommand
from tf2_ros import Buffer, TransformListener
import numpy as np
from scipy.spatial.transform import Rotation as R

PADS = {'robotiq_85_left_finger_tip_link': [-0.01456706, -0.0008, 0.01649701],
        'robotiq_85_right_finger_tip_link': [0.01456706, 5e-05, 0.01649701]}
BASE = 'robotiq_85_base_link'


def main():
    rclpy.init()
    n = Node('gripper_geom', parameter_overrides=[Parameter('use_sim_time', value=True)])
    buf = Buffer(); TransformListener(buf, n)
    ac = ActionClient(n, GripperCommand, '/gripper_controller/gripper_cmd'); ac.wait_for_server()
    t = buf.lookup_transform  # noqa
    for pos in (0.0, 0.7929):
        g = GripperCommand.Goal(); g.command.position = pos; g.command.max_effort = 20.0
        f = ac.send_goal_async(g); rclpy.spin_until_future_complete(n, f)
        rf = f.result().get_result_async(); rclpy.spin_until_future_complete(n, rf)
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            rclpy.spin_once(n, timeout_sec=0.05)
        pts = {}
        for link, off in PADS.items():
            tr = buf.lookup_transform(BASE, link, rclpy.time.Time(), Duration(seconds=2)).transform
            q = [tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w]
            p = np.array([tr.translation.x, tr.translation.y, tr.translation.z]) + R.from_quat(q).apply(off)
            pts[link] = p
        l, r = pts.values()
        print(f'knuckle={pos:.4f}: left pad {np.round(l,4)} right pad {np.round(r,4)} '
              f'| pad separation x={abs(l[0]-r[0])*1000:.1f} mm | pad z={((l[2]+r[2])/2)*1000:.1f} mm')
    tr = buf.lookup_transform('tool0', BASE, rclpy.time.Time(), Duration(seconds=2)).transform
    print(f'tool0 -> {BASE}: xyz=({tr.translation.x:.4f},{tr.translation.y:.4f},{tr.translation.z:.4f})')
    tr = buf.lookup_transform('tool0', 'gripper_tcp', rclpy.time.Time(), Duration(seconds=2)).transform
    print(f'tool0 -> gripper_tcp (as defined in scratch xacro): z={tr.translation.z:.4f}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
