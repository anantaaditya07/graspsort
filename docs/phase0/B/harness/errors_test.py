#!/usr/bin/env python3
"""Error-path checks for the custom /attach, /detach services."""
import rclpy
from b_attach_msgs.srv import AttachLink

CASES = [
    ('detach', ('ur', 'wrist_3_link', 'box', 'link')),      # nothing attached yet
    ('attach', ('nope', 'wrist_3_link', 'box', 'link')),    # unknown parent model
    ('attach', ('ur', 'no_link', 'box', 'link')),           # unknown parent link
    ('attach', ('ur', 'wrist_3_link', 'ghost', 'link')),    # unknown child model
    ('attach', ('ur', 'wrist_3_link', 'ur', 'wrist_3_link')),  # same link
]
rclpy.init()
n = rclpy.create_node('attach_errors')
for srv, (pm, pl, cm, cl) in CASES:
    c = n.create_client(AttachLink, '/' + srv)
    c.wait_for_service(timeout_sec=10)
    f = c.call_async(AttachLink.Request(parent_model=pm, parent_link=pl, child_model=cm,
                                        child_link=cl))
    rclpy.spin_until_future_complete(n, f, timeout_sec=10)
    r = f.result()
    print(f'{srv}({pm},{pl},{cm},{cl}) -> success={r.success} message="{r.message}"')
rclpy.shutdown()
