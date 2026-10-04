#!/usr/bin/env python3
"""Check 1: send a 2-point FollowJointTrajectory goal, log /joint_states to CSV, report final error."""
import csv, sys, time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration

JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
POSE_A = [0.0, -1.0, 1.5, -2.07, -1.5708, 0.0]
POSE_B = [0.0, -1.2, 1.9, -2.2708, -1.5708, 0.0]
ACTION = sys.argv[1] if len(sys.argv) > 1 else '/joint_trajectory_controller/follow_joint_trajectory'
CSV = sys.argv[2] if len(sys.argv) > 2 else 'fjt_joint_states.csv'


class T(Node):
    def __init__(self):
        super().__init__('fjt_test', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.rows, self.last = [], None
        self.create_subscription(JointState, '/joint_states', self.cb, 50)
        self.ac = ActionClient(self, FollowJointTrajectory, ACTION)

    def cb(self, m):
        d = dict(zip(m.name, m.position))
        if all(j in d for j in JOINTS):
            self.last = [d[j] for j in JOINTS]
            self.rows.append([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9] + self.last)


def main():
    rclpy.init()
    n = T()
    assert n.ac.wait_for_server(timeout_sec=20.0), 'action server not found'
    while n.last is None:
        rclpy.spin_once(n, timeout_sec=0.1)
    g = FollowJointTrajectory.Goal()
    g.trajectory.joint_names = JOINTS
    for pos, t in ((POSE_A, 4), (POSE_B, 8)):
        p = JointTrajectoryPoint(positions=pos, velocities=[0.0] * 6,
                                 time_from_start=Duration(sec=t))
        g.trajectory.points.append(p)
    t0 = time.monotonic()
    fut = n.ac.send_goal_async(g)
    rclpy.spin_until_future_complete(n, fut)
    gh = fut.result()
    print('goal accepted:', gh.accepted)
    rf = gh.get_result_async()
    rclpy.spin_until_future_complete(n, rf)
    wall = time.monotonic() - t0
    r = rf.result().result
    # settle 0.5 s wall of extra samples
    end = time.monotonic() + 0.5
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
    err = [abs(a - b) for a, b in zip(n.last, POSE_B)]
    print(f'result error_code={r.error_code} (0=SUCCESSFUL) error_string="{r.error_string}"')
    print(f'wall time for 8 s trajectory: {wall:.2f} s')
    print('final joint error [rad]:', ' '.join(f'{j}={e:.5f}' for j, e in zip(JOINTS, err)))
    print(f'max |error| = {max(err):.5f} rad -> {"PASS" if r.error_code == 0 and max(err) < 0.01 else "FAIL"}')
    with open(CSV, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['stamp'] + JOINTS)
        w.writerows(n.rows)
    print(f'wrote {len(n.rows)} samples to {CSV}')
    n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
