#!/usr/bin/env python3
"""Generate the SRDF <disable_collisions> block, same rules as the MoveIt Setup Assistant.

Used because moveit_setup_assistant/collisions_updater (MoveIt 2.5.10) hangs on this machine
before producing any output (see EVIDENCE / DECISIONS). Needs a running move_group whose SRDF has
NO disable_collisions entries (otherwise those pairs are never reported):

  ros2 launch graspsort_bringup moveit.launch.py use_sim_time:=false srdf_file:=<srdf without pairs>
  ros2 run graspsort_bringup gen_disabled_collisions.py --srdf-out <path>/graspsort.srdf

Method (deterministic for a given --seed and --samples):
  Adjacent  links with collision geometry joined by a joint, skipping links without geometry
  Default   in collision at the default state (all joints 0, clamped to limits)
  Always    in collision in >= --always-fraction of the random samples
  Never     never in collision in --samples uniform random states within the URDF limits
Collisions are taken from move_group's /check_state_validity contacts (whole robot).
"""
import argparse
import itertools
import random
import re
import sys
import xml.etree.ElementTree as ET

import rclpy
from moveit_msgs.srv import GetStateValidity
from rcl_interfaces.srv import GetParameters
from rclpy.utilities import remove_ros_args

BEGIN = '<!-- BEGIN GENERATED disable_collisions -->'
END = '<!-- END GENERATED disable_collisions -->'


def parse_args(argv):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument('--samples', type=int, default=10000, help='random states (Setup Assistant default)')
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--always-fraction', type=float, default=0.95)
    p.add_argument('--move-group-node', default='/move_group')
    p.add_argument('--service', default='/check_state_validity')
    p.add_argument('--srdf-out', default='',
                   help='SRDF to update between the BEGIN/END GENERATED markers; empty = print')
    p.add_argument('--server-timeout', type=float, default=30.0)
    return p.parse_args(argv)


class Robot:
    def __init__(self, urdf_xml):
        root = ET.fromstring(urdf_xml)
        self.geometric = sorted(l.get('name') for l in root.findall('link')
                                if l.find('collision') is not None)
        self.parent_of = {}
        self.variables = []  # (name, lower, upper) of movable, non-mimic joints
        self.mimic = {}  # name -> (source, multiplier, offset)
        for j in root.findall('joint'):
            child, parent = j.find('child').get('link'), j.find('parent').get('link')
            self.parent_of[child] = parent
            if j.get('type') == 'fixed':
                continue
            m = j.find('mimic')
            if m is not None:
                self.mimic[j.get('name')] = (m.get('joint'), float(m.get('multiplier', 1.0)),
                                             float(m.get('offset', 0.0)))
                continue
            lim = j.find('limit')
            lo = float(lim.get('lower', 0.0)) if lim is not None else -3.14159265
            hi = float(lim.get('upper', 0.0)) if lim is not None else 3.14159265
            if j.get('type') == 'continuous':
                lo, hi = -3.14159265, 3.14159265
            self.variables.append((j.get('name'), lo, hi))

    def adjacent_pairs(self):
        geo = set(self.geometric)
        pairs = set()
        for link in self.geometric:
            parent = self.parent_of.get(link)
            while parent is not None and parent not in geo:
                parent = self.parent_of.get(parent)
            if parent is not None:
                pairs.add(tuple(sorted((parent, link))))
        return pairs

    def state(self, values):
        names = [n for n, _, _ in self.variables]
        pos = list(values)
        for name, (src, mult, off) in self.mimic.items():
            names.append(name)
            pos.append(mult * values[names.index(src)] + off)
        return names, pos


def main():
    a = parse_args(remove_ros_args(sys.argv)[1:])
    rclpy.init()
    node = rclpy.create_node('gen_disabled_collisions')
    params = node.create_client(GetParameters, a.move_group_node.rstrip('/') + '/get_parameters')
    check = node.create_client(GetStateValidity, a.service)
    if not (params.wait_for_service(timeout_sec=a.server_timeout)
            and check.wait_for_service(timeout_sec=a.server_timeout)):
        print('ERROR: move_group services not available')
        return 2

    def call(client, req):
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(node, fut)
        return fut.result()

    vals = call(params, GetParameters.Request(
        names=['robot_description', 'robot_description_semantic'])).values
    urdf, srdf = vals[0].string_value, vals[1].string_value
    robot = Robot(urdf)
    # Only Adjacent pairs may be pre-disabled (they reduce contact-count saturation); any other
    # disabled pair would never be reported and would wrongly end up as Never.
    for d in ET.fromstring(srdf).iter('disable_collisions'):
        pair = tuple(sorted((d.get('link1'), d.get('link2'))))
        if pair not in robot.adjacent_pairs():
            print(f'ERROR: move_group SRDF disables non-adjacent pair {pair}; use an SRDF with '
                  'only Adjacent pairs (or none)')
            return 2
    max_pairs = [0]  # most colliding pairs seen in one state (sanity check against a contact cap)

    def colliding(values):
        req = GetStateValidity.Request()
        names, pos = robot.state(values)
        req.robot_state.joint_state.name = names
        req.robot_state.joint_state.position = pos
        res = call(check, req)
        pairs = {tuple(sorted((c.contact_body_1, c.contact_body_2))) for c in res.contacts}
        max_pairs[0] = max(max_pairs[0], len(pairs))
        return pairs

    reasons = {p: 'Adjacent' for p in robot.adjacent_pairs()}
    default = [min(max(0.0, lo), hi) for _, lo, hi in robot.variables]
    for p in colliding(default):
        reasons.setdefault(p, 'Default')
    rng = random.Random(a.seed)
    counts = {}
    for _ in range(a.samples):
        for p in colliding([rng.uniform(lo, hi) for _, lo, hi in robot.variables]):
            counts[p] = counts.get(p, 0) + 1
    for p, n in counts.items():
        if n >= a.always_fraction * a.samples:
            reasons.setdefault(p, 'Always')
    for p in itertools.combinations(robot.geometric, 2):
        if p not in counts:
            reasons.setdefault(p, 'Never')

    # no double dash inside XML comments
    lines = [f'  <!-- generated by gen_disabled_collisions.py samples={a.samples} seed={a.seed} '
             f'always_fraction={a.always_fraction}; {len(robot.geometric)} links with collision '
             f'geometry; {len(reasons)} of {len(robot.geometric) * (len(robot.geometric) - 1) // 2} '
             'pairs disabled -->']
    lines += [f'  <disable_collisions link1="{l1}" link2="{l2}" reason="{r}"/>'
              for (l1, l2), r in sorted(reasons.items())]
    block = '\n'.join(lines)
    summary = {r: sum(1 for v in reasons.values() if v == r)
               for r in ('Adjacent', 'Default', 'Always', 'Never')}
    print(f'samples={a.samples} seed={a.seed} pairs: {summary}; '
          f'pairs ever in collision: {len(counts)}; max colliding pairs in one state: {max_pairs[0]}')
    if a.srdf_out:
        with open(a.srdf_out, 'r', encoding='utf-8') as f:
            text = f.read()
        text, n = re.subn(re.escape(BEGIN) + r'.*?' + re.escape(END),
                          BEGIN + '\n' + block + '\n  ' + END, text, flags=re.S)
        if n != 1:
            print(f'ERROR: markers not found in {a.srdf_out}')
            return 1
        with open(a.srdf_out, 'w', encoding='utf-8') as f:
            f.write(text)
        print(f'updated {a.srdf_out}')
    else:
        print(block)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
