#!/usr/bin/env python3
"""Rapid attach/detach cycles against a running sim (box must exist). Checks for hangs."""
import sys
import time
import rclpy
from b_attach_msgs.srv import AttachLink

N = int(sys.argv[1]) if len(sys.argv) > 1 else 100
CAND = sys.argv[2] if len(sys.argv) > 2 else 'custom'
rclpy.init()
n = rclpy.create_node('attach_stress')
if CAND == 'custom':
    cli = {s: n.create_client(AttachLink, '/' + s) for s in ('attach', 'detach')}
    req = {s: AttachLink.Request(parent_model='ur', parent_link='wrist_3_link',
                                 child_model='box', child_link='link') for s in cli}
else:  # IFRA_LinkAttacher
    from linkattacher_msgs.srv import AttachLink as IA, DetachLink as ID
    cli = {'attach': n.create_client(IA, '/ATTACHLINK'), 'detach': n.create_client(ID, '/DETACHLINK')}
    req = {s: t.Request(model1_name='ur', link1_name='wrist_3_link', model2_name='box',
                        link2_name='link') for s, t in (('attach', IA), ('detach', ID))}
ok, t0 = 0, time.time()
for i in range(N):
    for s in ('attach', 'detach'):
        cli[s].wait_for_service(timeout_sec=5)
        f = cli[s].call_async(req[s])
        rclpy.spin_until_future_complete(n, f, timeout_sec=5)
        if f.result() is None:
            print(f'HANG: cycle {i} {s} got no response within 5 s')
            sys.exit(1)
        ok += f.result().success
print(f'{N} attach/detach cycles, {ok}/{2 * N} successful calls, {time.time() - t0:.1f} s wall')
rclpy.shutdown()
