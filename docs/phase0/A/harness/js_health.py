#!/usr/bin/env python3
"""Sample /joint_states for N seconds: report driven gripper joint, worst mimic deviation and max |velocity|
(a sanity check for physics blow-ups). js_health.py <seconds> [driven] [mimic=mult ...]"""
import sys, time, rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
DUR = float(sys.argv[1]); DRIVEN = sys.argv[2] if len(sys.argv) > 2 else 'robotiq_85_left_knuckle_joint'
MIMIC = {'robotiq_85_right_knuckle_joint': -1, 'robotiq_85_left_inner_knuckle_joint': 1,
         'robotiq_85_right_inner_knuckle_joint': -1, 'robotiq_85_left_finger_tip_joint': -1,
         'robotiq_85_right_finger_tip_joint': 1} if DRIVEN.startswith('robotiq') else \
        {k: float(v) for k, v in (a.split('=') for a in sys.argv[3:])}
rclpy.init(); n = Node('js_health'); st = {'dev': 0.0, 'vmax': 0.0, 'vj': '', 'last': None, 'k': 0}
def cb(m):
    p = dict(zip(m.name, m.position)); v = dict(zip(m.name, m.velocity))
    p.update({k[:-6]: x for k, x in p.items() if k.endswith('_mimic')})
    st['k'] += 1; st['last'] = p[DRIVEN]
    st['dev'] = max([st['dev']] + [abs(p[j] - mm * p[DRIVEN]) for j, mm in MIMIC.items()])
    for j, x in v.items():
        if abs(x) > st['vmax']: st['vmax'], st['vj'] = abs(x), j
n.create_subscription(JointState, '/joint_states', cb, 50)
end = time.monotonic() + DUR
while time.monotonic() < end: rclpy.spin_once(n, timeout_sec=0.05)
print(f'  js_health over {DUR}s ({st["k"]} msgs): {DRIVEN}={st["last"]:+.4f} max_mimic_dev={st["dev"]:.4f} '
      f'max|vel|={st["vmax"]:.3f} ({st["vj"]})')
