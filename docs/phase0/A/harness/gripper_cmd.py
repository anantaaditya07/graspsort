#!/usr/bin/env python3
"""Send one GripperCommand goal: gripper_cmd.py <position> [max_effort] [action]"""
import sys, rclpy
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
rclpy.init(); n = rclpy.create_node('gripper_cmd')
ac = ActionClient(n, GripperCommand, sys.argv[3] if len(sys.argv) > 3 else '/gripper_controller/gripper_cmd')
ac.wait_for_server()
g = GripperCommand.Goal(); g.command.position = float(sys.argv[1]); g.command.max_effort = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0
f = ac.send_goal_async(g); rclpy.spin_until_future_complete(n, f)
rf = f.result().get_result_async(); rclpy.spin_until_future_complete(n, rf)
r = rf.result()
print(f'status={r.status} position={r.result.position:.4f} reached_goal={r.result.reached_goal} stalled={r.result.stalled}')
