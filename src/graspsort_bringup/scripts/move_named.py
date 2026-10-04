#!/usr/bin/env python3
"""Move a MoveIt group through a list of SRDF named states.

For each state: plan with the move_group MoveGroup action (/move_action, plan only, so the real
planning time is reported), then execute that plan with the ExecuteTrajectory action
(/execute_trajectory). Joint values come from the SRDF that move_group was started with
(parameter robot_description_semantic of the move_group node).

Prints per state: MoveItErrorCodes, planning time, execution wall time, trajectory duration and
the max joint error measured on /joint_states after execution. Exit code 0 only if every state
succeeded.

Example:
  ros2 run graspsort_bringup move_named.py home ready home
  ros2 run graspsort_bringup move_named.py ready --planner-id RRTConnectkConfigDefault --repeat 3
"""
import argparse
import sys
import time
import xml.etree.ElementTree as ET

import rclpy
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState

CODES = {v: k for k, v in MoveItErrorCodes.__dict__.items() if isinstance(v, int) and k.isupper()}


def parse_args(argv):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument('states', nargs='*', default=['home', 'ready', 'home'],
                   help='SRDF group_state names, visited in order (default: home ready home)')
    p.add_argument('--group', default='ur_manipulator')
    p.add_argument('--pipeline-id', default='ompl')
    p.add_argument('--planner-id', default='RRTConnectkConfigDefault')
    p.add_argument('--planning-time', type=float, default=5.0, help='allowed planning time [s]')
    p.add_argument('--attempts', type=int, default=1, help='num_planning_attempts')
    p.add_argument('--velocity-scaling', type=float, default=0.5)
    p.add_argument('--acceleration-scaling', type=float, default=0.5)
    p.add_argument('--joint-tolerance', type=float, default=0.001,
                   help='goal joint constraint tolerance [rad or m]')
    p.add_argument('--repeat', type=int, default=1, help='repeat the whole state list N times')
    p.add_argument('--move-group-node', default='/move_group')
    p.add_argument('--move-action', default='/move_action')
    p.add_argument('--execute-action', default='/execute_trajectory')
    p.add_argument('--server-timeout', type=float, default=30.0, help='wait for servers [s]')
    p.add_argument('--settle-time', type=float, default=0.5,
                   help='wait after execution before measuring /joint_states [s]')
    p.add_argument('--plan-only', action='store_true', help='plan but do not execute')
    return p.parse_args(argv)


class MoveNamed(Node):
    def __init__(self, args):
        super().__init__('move_named',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.args = args
        self.joint_state = None
        self.create_subscription(JointState, '/joint_states', self._on_js, 10)
        self.move_client = ActionClient(self, MoveGroup, args.move_action)
        self.exec_client = ActionClient(self, ExecuteTrajectory, args.execute_action)
        self.param_client = self.create_client(
            GetParameters, args.move_group_node.rstrip('/') + '/get_parameters')

    def _on_js(self, msg):
        self.joint_state = dict(zip(msg.name, msg.position))

    def wait(self, future):
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def named_states(self):
        """{state_name: {joint: value}} for self.args.group from move_group's SRDF."""
        req = GetParameters.Request(names=['robot_description_semantic'])
        srdf = self.wait(self.param_client.call_async(req)).values[0].string_value
        if not srdf:
            raise RuntimeError('move_group has no robot_description_semantic parameter')
        states = {}
        for gs in ET.fromstring(srdf).iter('group_state'):
            if gs.get('group') == self.args.group:
                states[gs.get('name')] = {j.get('name'): float(j.get('value'))
                                          for j in gs.iter('joint')}
        return states

    def plan(self, joints):
        a = self.args
        goal = MoveGroup.Goal()
        r = goal.request
        r.group_name = a.group
        r.pipeline_id = a.pipeline_id
        r.planner_id = a.planner_id
        r.num_planning_attempts = a.attempts
        r.allowed_planning_time = a.planning_time
        r.max_velocity_scaling_factor = a.velocity_scaling
        r.max_acceleration_scaling_factor = a.acceleration_scaling
        r.start_state.is_diff = True
        r.goal_constraints.append(Constraints(joint_constraints=[
            JointConstraint(joint_name=n, position=v, tolerance_above=a.joint_tolerance,
                            tolerance_below=a.joint_tolerance, weight=1.0)
            for n, v in joints.items()]))
        goal.planning_options.plan_only = True
        handle = self.wait(self.move_client.send_goal_async(goal))
        if not handle.accepted:
            raise RuntimeError('MoveGroup goal rejected')
        return self.wait(handle.get_result_async()).result

    def execute(self, trajectory):
        goal = ExecuteTrajectory.Goal(trajectory=trajectory)
        handle = self.wait(self.exec_client.send_goal_async(goal))
        if not handle.accepted:
            raise RuntimeError('ExecuteTrajectory goal rejected')
        return self.wait(handle.get_result_async()).result

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)


def code_str(code):
    return f'{code.val} ({CODES.get(code.val, "?")})'


def main():
    args = parse_args(remove_ros_args(sys.argv)[1:])
    rclpy.init()
    node = MoveNamed(args)
    ok = True
    try:
        t = args.server_timeout
        if not (node.move_client.wait_for_server(timeout_sec=t)
                and node.exec_client.wait_for_server(timeout_sec=t)
                and node.param_client.wait_for_service(timeout_sec=t)):
            print('ERROR: move_group action servers / parameter service not available')
            return 2
        states = node.named_states()
        missing = [s for s in args.states if s not in states]
        if missing:
            print(f'ERROR: unknown named state(s) {missing} for group {args.group}; '
                  f'known: {sorted(states)}')
            return 2
        for rep in range(args.repeat):
            for name in args.states:
                target = states[name]
                res = node.plan(target)
                traj = res.planned_trajectory.joint_trajectory
                duration = (traj.points[-1].time_from_start.sec
                            + 1e-9 * traj.points[-1].time_from_start.nanosec) if traj.points else 0.0
                line = (f'[{rep + 1}/{args.repeat}] {args.group} -> {name}: plan '
                        f'{code_str(res.error_code)} planning_time={res.planning_time:.3f} s '
                        f'points={len(traj.points)} duration={duration:.2f} s')
                if res.error_code.val != MoveItErrorCodes.SUCCESS:
                    print(line + ' FAIL')
                    ok = False
                    continue
                if args.plan_only:
                    print(line + ' SUCCESS (plan only)')
                    continue
                t0 = time.monotonic()
                ex = node.execute(res.planned_trajectory)
                wall = time.monotonic() - t0
                node.spin_for(args.settle_time)
                js = node.joint_state or {}
                err = max((abs(js[j] - v) for j, v in target.items() if j in js),
                          default=float('nan'))
                success = ex.error_code.val == MoveItErrorCodes.SUCCESS
                ok = ok and success
                print(f'{line}; execute {code_str(ex.error_code)} execution_wall={wall:.2f} s '
                      f'max_joint_error={err:.5f} {"SUCCESS" if success else "FAIL"}')
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print('RESULT: ' + ('SUCCESS' if ok else 'FAILURE'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
