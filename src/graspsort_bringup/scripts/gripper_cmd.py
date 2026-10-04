#!/usr/bin/env python3
"""Open or close the gripper through /gripper_controller/gripper_cmd (control_msgs/GripperCommand).

Target: `open`, `close`, a finger joint position [m] (0 = open, larger = more closed), or
`--width W`, the jaw gap [m] (position = (open_width - W) / 2). Per D-02 a real grasp closes to the
object width minus a small squeeze, never fully.

Prints the action status and result (position, reached_goal, stalled) and the driven joint position
measured on /joint_states. Exit code 0 only if the goal SUCCEEDED.

Example:
  ros2 run graspsort_bringup gripper_cmd.py close
  ros2 run graspsort_bringup gripper_cmd.py --width 0.06
"""
import argparse
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState

STATUS = {v: k for k, v in GoalStatus.__dict__.items() if isinstance(v, int) and k.startswith('STATUS_')}


def parse_args(argv):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument('target', nargs='?', default=None, help='open | close | <position m>')
    p.add_argument('--width', type=float, default=None, help='jaw gap target [m]')
    p.add_argument('--open-position', type=float, default=0.0, help='joint position of open [m]')
    p.add_argument('--closed-position', type=float, default=0.04,
                   help='joint position of closed [m]')
    p.add_argument('--open-width', type=float, default=0.09, help='jaw gap at position 0 [m]')
    p.add_argument('--max-effort', type=float, default=50.0, help='[N]')
    p.add_argument('--action', default='/gripper_controller/gripper_cmd')
    p.add_argument('--joint', default='gripper_left_finger_joint',
                   help='driven joint reported from /joint_states')
    p.add_argument('--server-timeout', type=float, default=30.0, help='[s]')
    p.add_argument('--settle-time', type=float, default=0.3,
                   help='wait before reading /joint_states [s]')
    a = p.parse_args(argv)
    if (a.target is None) == (a.width is None):
        p.error('give exactly one of: target (open|close|<position>) or --width')
    return a


def target_position(a):
    if a.width is not None:
        return (a.open_width - a.width) / 2.0
    if a.target == 'open':
        return a.open_position
    if a.target == 'close':
        return a.closed_position
    return float(a.target)


def main():
    a = parse_args(remove_ros_args(sys.argv)[1:])
    position = target_position(a)
    rclpy.init()
    node = rclpy.create_node('gripper_cmd',
                             parameter_overrides=[Parameter('use_sim_time', value=True)])
    js = {}
    node.create_subscription(JointState, '/joint_states',
                             lambda m: js.update(zip(m.name, m.position)), 10)
    client = ActionClient(node, GripperCommand, a.action)
    ok = False
    try:
        if not client.wait_for_server(timeout_sec=a.server_timeout):
            print(f'ERROR: action server {a.action} not available')
            return 2
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = a.max_effort
        t0 = time.monotonic()
        fut = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, fut)
        handle = fut.result()
        if not handle.accepted:
            print('ERROR: goal rejected')
            return 1
        rfut = handle.get_result_async()
        rclpy.spin_until_future_complete(node, rfut)
        wall = time.monotonic() - t0
        res = rfut.result()
        end = time.monotonic() + a.settle_time
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        measured = js.get(a.joint, float('nan'))
        ok = res.status == GoalStatus.STATUS_SUCCEEDED
        print(f'target={position:.4f} status={STATUS.get(res.status, res.status)} '
              f'result.position={res.result.position:.4f} reached_goal={res.result.reached_goal} '
              f'stalled={res.result.stalled} measured_{a.joint}={measured:.4f} wall={wall:.2f} s')
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print('RESULT: ' + ('SUCCEEDED' if ok else 'FAILED'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
