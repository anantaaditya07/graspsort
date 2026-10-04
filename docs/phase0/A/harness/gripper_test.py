#!/usr/bin/env python3
"""Check 3: GripperCommand open/close via gripper action controller; verify the driven joint and the
mimic joints (as reported by Gazebo through gazebo_ros2_control on /joint_states). Writes a CSV."""
import csv, sys, time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from control_msgs.action import GripperCommand

ACTION = sys.argv[1] if len(sys.argv) > 1 else '/gripper_controller/gripper_cmd'
CSV = sys.argv[2] if len(sys.argv) > 2 else 'gripper_joint_states.csv'
DRIVEN = sys.argv[3] if len(sys.argv) > 3 else 'robotiq_85_left_knuckle_joint'
# mimic joint -> multiplier (empty for the custom gripper if its second finger is separately commanded)
MIMIC = {'robotiq_85_right_knuckle_joint': -1, 'robotiq_85_left_inner_knuckle_joint': 1,
         'robotiq_85_right_inner_knuckle_joint': -1, 'robotiq_85_left_finger_tip_joint': -1,
         'robotiq_85_right_finger_tip_joint': 1} if DRIVEN.startswith('robotiq') and len(sys.argv) <= 6 else \
        {k: float(v) for k, v in (a.split('=') for a in sys.argv[6:]) if k != 'none'}  # 'none=0' -> no mimic check
TARGETS = [float(sys.argv[4]), float(sys.argv[5])] if len(sys.argv) > 5 else [0.7, 0.0]  # close, open
TOL = 0.01
JOINTS = [DRIVEN] + list(MIMIC)


class T(Node):
    def __init__(self):
        super().__init__('gripper_test', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.rows, self.last = [], None
        self.create_subscription(JointState, '/joint_states', self.cb, 50)
        self.ac = ActionClient(self, GripperCommand, ACTION)

    def cb(self, m):
        d = dict(zip(m.name, m.position))
        # gazebo_ros2_control 0.4.x publishes mimic joints as '<joint>_mimic'; map back to URDF names
        d.update({k[:-len('_mimic')]: v for k, v in d.items() if k.endswith('_mimic')})
        self.renamed = sorted(k for k in m.name if k.endswith('_mimic'))
        if all(j in d for j in JOINTS):
            self.last = d
            self.rows.append([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9] + [d[j] for j in JOINTS])


def main():
    rclpy.init()
    n = T()
    assert n.ac.wait_for_server(timeout_sec=20.0), 'gripper action server not found'
    while n.last is None:
        rclpy.spin_once(n, timeout_sec=0.1)
    ok_all = True
    print('joint names with _mimic suffix on /joint_states:', n.renamed)
    for tgt in TARGETS + TARGETS:  # close, open, close, open
        g = GripperCommand.Goal()
        g.command.position = tgt
        g.command.max_effort = 20.0
        t0 = time.monotonic()
        f = n.ac.send_goal_async(g); rclpy.spin_until_future_complete(n, f)
        gh = f.result()
        rf = gh.get_result_async(); rclpy.spin_until_future_complete(n, rf, timeout_sec=15.0)
        wall = time.monotonic() - t0
        end = time.monotonic() + 0.3
        while time.monotonic() < end:
            rclpy.spin_once(n, timeout_sec=0.05)
        if not rf.done():
            print(f'target {tgt:+.3f}: action did NOT return within 15 s'); ok_all = False; continue
        r = rf.result()
        d = n.last
        err = abs(d[DRIVEN] - tgt)
        mim = {j: abs(d[j] - m * d[DRIVEN]) for j, m in MIMIC.items()}
        mmax = max(mim.values()) if mim else 0.0
        ok = r.status == 4 and err < TOL and mmax < TOL
        ok_all &= ok
        print(f'target {tgt:+.3f}: status={r.status} (4=SUCCEEDED) reached_goal={r.result.reached_goal} '
              f'stalled={r.result.stalled} pos={r.result.position:+.4f} wall={wall:.2f}s | '
              f'driven err={err:.4f} | max mimic dev={mmax:.4f} -> {"OK" if ok else "FAIL"}')
        for j in MIMIC:
            print(f'    {j:40s} pos={d[j]:+.4f} expected={MIMIC[j]*d[DRIVEN]:+.4f}')
    with open(CSV, 'w', newline='') as f:
        w = csv.writer(f); w.writerow(['stamp'] + JOINTS); w.writerows(n.rows)
    print(f'wrote {len(n.rows)} samples to {CSV}')
    print('OVERALL', 'PASS' if ok_all else 'FAIL')
    n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
