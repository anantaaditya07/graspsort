#!/usr/bin/env python3
"""Phase 0 / B attach test driver.

Modes:
  calib                       move to Q_GRASP and print wrist_3_link world pose
  run --candidate custom|ifra|none --runs N --tag T [--offset] [--reattach]
Each run: spawn box under flange, settle, attach (unless candidate none), lift + fast sweep
out/back, record box pose relative to wrist_3_link from /gazebo/link_states, detach, record
1 s of fall. Writes <outdir>/<tag>_runK.csv and prints a summary line per run.
"""
import argparse
import csv
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from gazebo_msgs.msg import LinkStates
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Pose
from rosgraph_msgs.msg import Clock
from trajectory_msgs.msg import JointTrajectoryPoint
from scipy.spatial.transform import Rotation as R

JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
Q_GRASP = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
LIFT = {'shoulder_lift_joint': -0.25}          # rad added at the lift pose
SWEEP = {'shoulder_pan_joint': 1.0, 'elbow_joint': -1.0, 'wrist_3_joint': 1.0}  # rad
BOX = 0.05      # m edge
MASS = 0.1      # kg
GAP = 0.002     # m between box top and flange when spawned
WRIST = 'ur::wrist_3_link'
BOXLINK = 'box::link'

BOX_SDF = f"""<?xml version='1.0'?><sdf version='1.6'><model name='box'><link name='link'>
<inertial><mass>{MASS}</mass><inertia><ixx>{MASS*BOX*BOX/6}</ixx><iyy>{MASS*BOX*BOX/6}</iyy>
<izz>{MASS*BOX*BOX/6}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
<collision name='c'><geometry><box><size>{BOX} {BOX} {BOX}</size></box></geometry></collision>
<visual name='v'><geometry><box><size>{BOX} {BOX} {BOX}</size></box></geometry></visual>
</link></model></sdf>"""


def T(p):
    m = np.eye(4)
    m[:3, :3] = R.from_quat([p.orientation.x, p.orientation.y, p.orientation.z,
                             p.orientation.w]).as_matrix()
    m[:3, 3] = [p.position.x, p.position.y, p.position.z]
    return m


class Tester(Node):
    def __init__(self, candidate):
        super().__init__('attach_tester',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.candidate = candidate
        self.sim_t = 0.0
        self.samples = []   # (sim_t, T_wrist, T_box, box_twist_lin, box_twist_ang)
        self.recording = False
        self.last = None
        self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)
        self.create_subscription(LinkStates, '/gazebo/link_states', self.on_ls, 50)
        self.traj = ActionClient(self, FollowJointTrajectory,
                                 '/joint_trajectory_controller/follow_joint_trajectory')
        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        self.del_cli = self.create_client(DeleteEntity, '/delete_entity')
        if candidate == 'custom':
            from b_attach_msgs.srv import AttachLink
            self.att = self.create_client(AttachLink, '/attach')
            self.det = self.create_client(AttachLink, '/detach')
            self.srv_type = AttachLink
        elif candidate == 'ifra':
            from linkattacher_msgs.srv import AttachLink, DetachLink
            self.att = self.create_client(AttachLink, '/ATTACHLINK')
            self.det = self.create_client(DetachLink, '/DETACHLINK')
            self.srv_type = (AttachLink, DetachLink)

    def on_clock(self, msg):
        self.sim_t = msg.clock.sec + msg.clock.nanosec * 1e-9

    def on_ls(self, msg):
        try:
            iw = msg.name.index(WRIST)
        except ValueError:
            return
        tw = T(msg.pose[iw])
        tb, lin, ang = None, None, None
        if BOXLINK in msg.name:
            ib = msg.name.index(BOXLINK)
            tb = T(msg.pose[ib])
            v = msg.twist[ib]
            lin = [v.linear.x, v.linear.y, v.linear.z]
            ang = [v.angular.x, v.angular.y, v.angular.z]
        self.last = (self.sim_t, tw, tb, lin, ang)
        if self.recording and tb is not None:
            self.samples.append(self.last)

    def spin_for(self, sim_seconds):
        t0 = self.sim_t
        while rclpy.ok() and self.sim_t - t0 < sim_seconds:
            rclpy.spin_once(self, timeout_sec=0.01)

    def call(self, cli, req, timeout=10.0):
        cli.wait_for_service(timeout_sec=timeout)
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()

    def move(self, waypoints):
        """waypoints: list of (time_from_start_s, q list). Blocks until done."""
        self.traj.wait_for_server(timeout_sec=30)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        for t, q in waypoints:
            pt = JointTrajectoryPoint()
            pt.positions = list(q)
            pt.velocities = [0.0] * 6
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        fut = self.traj.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30)
        gh = fut.result()
        assert gh.accepted, 'trajectory rejected'
        rfut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rfut, timeout_sec=120)
        return rfut.result().result.error_code

    def link_req(self, detach=False):
        if self.candidate == 'custom':
            r = self.srv_type.Request()
            r.parent_model, r.parent_link, r.child_model, r.child_link = \
                'ur', 'wrist_3_link', 'box', 'link'
        else:
            r = (self.srv_type[1] if detach else self.srv_type[0]).Request()
            r.model1_name, r.link1_name, r.model2_name, r.link2_name = \
                'ur', 'wrist_3_link', 'box', 'link'
        return r

    def attach(self):
        res = self.call(self.att, self.link_req())
        return res.success, res.message

    def detach(self):
        res = self.call(self.det, self.link_req(detach=True))
        return res.success, res.message

    def wait_last(self):
        self.last = None
        while self.last is None:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.last

    def spawn_box(self, offset):
        self.call(self.del_cli, DeleteEntity.Request(name='box'))
        # Deletion is applied asynchronously by Gazebo: wait until the box is really gone,
        # otherwise the spawn below fails with "entity already exists".
        while self.wait_last()[2] is not None:
            self.spin_for(0.05)
        _, tw, _, _, _ = self.wait_last()
        # Flange face points along world -z at Q_GRASP; place box under it on the table.
        fx, fy, fz = tw[:3, 3]
        dx, dy, yaw = (0.02, -0.015, 0.4) if offset else (0.0, 0.0, 0.0)
        req = SpawnEntity.Request(name='box', xml=BOX_SDF, reference_frame='world')
        req.initial_pose = Pose()
        req.initial_pose.position.x = fx + dx
        req.initial_pose.position.y = fy + dy
        req.initial_pose.position.z = fz - GAP - BOX / 2
        q = R.from_euler('z', yaw).as_quat()
        (req.initial_pose.orientation.x, req.initial_pose.orientation.y,
         req.initial_pose.orientation.z, req.initial_pose.orientation.w) = q
        res = self.call(self.spawn_cli, req)
        return res.success, res.status_message


def rel(tw, tb):
    return np.linalg.inv(tw) @ tb


def drift(t0, t):
    dp = np.linalg.norm(t[:3, 3] - t0[:3, 3]) * 1000.0
    da = math.degrees(np.linalg.norm(R.from_matrix(t0[:3, :3].T @ t[:3, :3]).as_rotvec()))
    return dp, da


def trajectory(fast_s):
    q_lift = list(Q_GRASP)
    for j, d in LIFT.items():
        q_lift[JOINTS.index(j)] += d
    q_out = list(q_lift)
    for j, d in SWEEP.items():
        q_out[JOINTS.index(j)] += d
    return q_lift, [(1.0, q_lift), (1.0 + fast_s, q_out), (1.0 + 2 * fast_s, q_lift)]


def one_run(n, k, args, writer_path):
    log = []
    ok, msg = n.spawn_box(args.offset)
    log.append(f'spawn: {ok} {msg}')
    n.spin_for(0.5)  # settle on table
    n.samples = []
    n.recording = True
    n.spin_for(0.1)
    deadline, retries = time.time() + 10.0, 0
    while len(n.samples) < 5:  # box must be visible in /gazebo/link_states
        rclpy.spin_once(n, timeout_sec=0.05)
        if time.time() > deadline:  # harness spawn race (see REPORT.md): respawn once
            assert retries == 0, f'box never appeared in link_states: {log}'
            n.recording = False
            log.append('HARNESS: box not in link_states after 10 s, respawn: %s %s'
                       % n.spawn_box(args.offset))
            n.spin_for(0.5)
            n.samples, n.recording = [], True
            deadline, retries = time.time() + 10.0, 1
    pre = rel(n.samples[-1][1], n.samples[-1][2])
    att_t = n.sim_t
    if args.candidate != 'none':
        ok, msg = n.attach()
        log.append(f'attach: {ok} {msg}')
        if args.reattach:
            n.spin_for(0.1)
            log.append('detach (reattach test): %s %s' % n.detach())
            n.spin_for(0.2)
            log.append('re-attach: %s %s' % n.attach())
        if args.double_attach:
            ok2, msg2 = n.attach()
            log.append(f'attach again (expect fail): {ok2} {msg2}')
    n.spin_for(0.05)
    t_attach_done = n.sim_t
    ref = rel(n.samples[-1][1], n.samples[-1][2])
    snap_mm, snap_deg = drift(pre, ref)
    q_lift, wps = trajectory(args.fast)
    err = n.move(wps)
    t_motion_end = n.sim_t
    n.spin_for(0.2)
    det_t = n.sim_t
    if args.candidate != 'none':
        ok, msg = n.detach()
        log.append(f'detach: {ok} {msg}')
    n.spin_for(1.0)
    n.recording = False
    end_t = n.sim_t
    # analysis
    rows, held_dp, held_da = [], [], []
    for (t, tw, tb, lin, ang) in n.samples:
        r_ = rel(tw, tb)
        dp, da = drift(ref, r_)
        phase = ('pre' if t < att_t else 'held' if t < det_t else 'released')
        if t_attach_done <= t < det_t:
            held_dp.append(dp)
            held_da.append(da)
        rows.append([f'{t:.4f}', phase, *[f'{x:.6f}' for x in tw[:3, 3]],
                     *[f'{x:.6f}' for x in tb[:3, 3]], f'{dp:.3f}', f'{da:.3f}',
                     f'{np.linalg.norm(lin):.4f}', f'{np.linalg.norm(ang):.4f}'])
    with open(writer_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['sim_t', 'phase', 'wrist_x', 'wrist_y', 'wrist_z', 'box_x', 'box_y', 'box_z',
                    'rel_drift_mm', 'rel_drift_deg', 'box_lin_speed', 'box_ang_speed'])
        w.writerows(rows)
    held = [s for s in n.samples if t_attach_done <= s[0] < det_t]
    rate = (len(held) - 1) / (held[-1][0] - held[0][0]) if len(held) > 1 else 0.0
    rel_ = [s for s in n.samples if s[0] >= det_t]
    z0, z1 = rel_[0][2][2, 3], rel_[-1][2][2, 3]
    allv = np.array([np.linalg.norm(s[3]) for s in n.samples])
    allpose = np.array([s[2][:3, 3] for s in n.samples])
    wrist_travel = max(np.linalg.norm(s[1][:3, 3] - held[0][1][:3, 3]) for s in held)
    box_travel = max(np.linalg.norm(s[2][:3, 3] - held[0][2][:3, 3]) for s in held)
    summary = dict(run=k, traj_err=err,
                   rel_at_attach_xyz_mm=[round(float(x) * 1000, 1) for x in ref[:3, 3]],
                   rel_at_attach_rpy_deg=[round(float(x), 1) for x in
                                          R.from_matrix(ref[:3, :3]).as_euler('xyz', degrees=True)], samples_held=len(held), rate_hz=round(rate, 1),
                   snap_mm=round(snap_mm, 3), snap_deg=round(snap_deg, 3),
                   max_drift_mm=round(max(held_dp), 3), max_drift_deg=round(max(held_da), 3),
                   wrist_travel_m=round(wrist_travel, 3), box_travel_m=round(box_travel, 3),
                   z_at_detach=round(z0, 4), z_after_1s=round(z1, 4),
                   max_box_speed=round(float(allv.max()), 3),
                   nan=bool(np.isnan(allpose).any()), sim_span=(round(att_t, 2), round(end_t, 2)),
                   motion_end=round(t_motion_end, 2))
    # return arm to grasp pose for next run (box no longer attached)
    n.move([(2.0, Q_GRASP)])
    return log, summary


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('mode', choices=['calib', 'run'])
    ap.add_argument('--candidate', default='custom', choices=['custom', 'ifra', 'none'])
    ap.add_argument('--runs', type=int, default=1)
    ap.add_argument('--tag', default='run')
    ap.add_argument('--outdir', default='.')
    ap.add_argument('--fast', type=float, default=1.5, help='seconds for each 1 rad leg')
    ap.add_argument('--offset', action='store_true')
    ap.add_argument('--double_attach', action='store_true')
    ap.add_argument('--reattach', action='store_true')
    args = ap.parse_args()
    rclpy.init()
    n = Tester(args.candidate)
    while n.sim_t == 0.0:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.move([(3.0, Q_GRASP)])
    n.spin_for(0.3)
    if args.mode == 'calib':
        _, tw, _, _, _ = n.wait_last()
        np.set_printoptions(precision=4, suppress=True)
        print('wrist_3_link world pose at Q_GRASP:\n', tw)
        q_lift, _ = trajectory(args.fast)
        n.move([(2.0, q_lift)])
        n.spin_for(0.3)
        _, tw2, _, _, _ = n.wait_last()
        print('wrist_3_link at lift pose:', tw2[:3, 3])
        n.move([(2.0, Q_GRASP)])
        return
    for k in range(1, args.runs + 1):
        log, s = one_run(n, k, args, f'{args.outdir}/{args.tag}_run{k}.csv')
        for line in log:
            print(f'[{args.tag} run{k}] {line}')
        print(f'[{args.tag} run{k}] SUMMARY {s}')
        sys.stdout.flush()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
