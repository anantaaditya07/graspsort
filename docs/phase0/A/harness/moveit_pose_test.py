#!/usr/bin/env python3
"""Check 2: plan+execute a pose target for tool0 via the MoveGroup action.
Target pose = FK (/compute_fk) of a known collision-free joint config, so it is reachable by construction.
Logs MoveItErrorCodes, planning_time, and final tool0 pose error (via a second FK of measured joints)."""
import sys, time, math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import (Constraints, PositionConstraint, OrientationConstraint,
                             BoundingVolume, RobotState, MoveItErrorCodes)
from shape_msgs.msg import SolidPrimitive

GROUP = 'ur_manipulator'
EE = 'tool0'
FRAME = 'base_link'
JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
TARGETS = {'a': [0.8, -1.3, 1.4, -1.67, -1.57, 0.0],   # tool pointing down, in front of the base
           'b': [-0.6, -1.6, 1.8, -1.77, -1.57, 0.5]}
TARGET_Q = TARGETS[sys.argv[1] if len(sys.argv) > 1 else 'a']
CODES = {v: k for k, v in MoveItErrorCodes.__dict__.items() if isinstance(v, int) and k.isupper()}


class T(Node):
    def __init__(self):
        super().__init__('moveit_pose_test', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.js = None
        self.create_subscription(JointState, '/joint_states', lambda m: setattr(self, 'js', m), 10)
        self.fk = self.create_client(GetPositionFK, '/compute_fk')
        self.ac = ActionClient(self, MoveGroup, '/move_action')

    def call(self, fut):
        rclpy.spin_until_future_complete(self, fut)
        return fut.result()

    def fk_pose(self, names, pos):
        req = GetPositionFK.Request()
        req.header.frame_id = FRAME
        req.fk_link_names = [EE]
        req.robot_state.joint_state.name = names
        req.robot_state.joint_state.position = list(pos)
        return self.call(self.fk.call_async(req)).pose_stamped[0].pose


def main():
    rclpy.init()
    n = T()
    assert n.fk.wait_for_service(timeout_sec=20) and n.ac.wait_for_server(timeout_sec=20)
    while n.js is None:
        rclpy.spin_once(n, timeout_sec=0.1)
    tgt = n.fk_pose(JOINTS, TARGET_Q)
    print(f'target {EE} in {FRAME}: p=({tgt.position.x:.4f},{tgt.position.y:.4f},{tgt.position.z:.4f}) '
          f'q=({tgt.orientation.x:.4f},{tgt.orientation.y:.4f},{tgt.orientation.z:.4f},{tgt.orientation.w:.4f})')
    g = MoveGroup.Goal()
    r = g.request
    r.group_name = GROUP
    r.planner_id = sys.argv[2] if len(sys.argv) > 2 else ''
    r.num_planning_attempts = 5
    r.allowed_planning_time = 5.0
    r.max_velocity_scaling_factor = 0.5
    r.max_acceleration_scaling_factor = 0.5
    r.start_state.is_diff = True
    pc = PositionConstraint()
    pc.header.frame_id = FRAME
    pc.link_name = EE
    sp = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.001])
    bv = BoundingVolume()
    bv.primitives.append(sp)
    bv.primitive_poses.append(tgt)
    pc.constraint_region = bv
    pc.weight = 1.0
    oc = OrientationConstraint()
    oc.header.frame_id = FRAME
    oc.link_name = EE
    oc.orientation = tgt.orientation
    oc.absolute_x_axis_tolerance = oc.absolute_y_axis_tolerance = oc.absolute_z_axis_tolerance = 0.01
    oc.weight = 1.0
    r.goal_constraints.append(Constraints(position_constraints=[pc], orientation_constraints=[oc]))
    # 1) plan only, to read planning_time (move_group leaves it 0 when it also executes)
    g.planning_options.plan_only = True
    gh = n.call(n.ac.send_goal_async(g))
    res = n.call(gh.get_result_async()).result
    print(f'plan_only: MoveItErrorCodes.val = {res.error_code.val} ({CODES.get(res.error_code.val, "?")}), '
          f'planning_time = {res.planning_time:.3f} s')
    # 2) plan + execute
    g.planning_options.plan_only = False
    t0 = time.monotonic()
    gh = n.call(n.ac.send_goal_async(g))
    print('goal accepted:', gh.accepted)
    res = n.call(gh.get_result_async()).result
    wall = time.monotonic() - t0
    code = res.error_code.val
    print(f'MoveItErrorCodes.val = {code} ({CODES.get(code, "?")})')
    print(f'planning_time = {res.planning_time:.3f} s; plan+execute wall = {wall:.2f} s; '
          f'trajectory points = {len(res.planned_trajectory.joint_trajectory.points)}')
    end = time.monotonic() + 1.0
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
    cur = n.fk_pose(list(n.js.name), list(n.js.position))
    dp = math.dist((cur.position.x, cur.position.y, cur.position.z),
                   (tgt.position.x, tgt.position.y, tgt.position.z))
    dot = abs(cur.orientation.x * tgt.orientation.x + cur.orientation.y * tgt.orientation.y +
              cur.orientation.z * tgt.orientation.z + cur.orientation.w * tgt.orientation.w)
    ang = 2 * math.acos(min(1.0, dot))
    print(f'final {EE} error (FK of measured /joint_states): position {dp*1000:.2f} mm, orientation {math.degrees(ang):.3f} deg')
    print('PASS' if code == 1 else 'FAIL')
    n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
