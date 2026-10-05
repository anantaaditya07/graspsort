#!/usr/bin/env python3
"""Phase 6 analysis beyond eval_summary.py (EVAL ONLY: uses Gazebo ground truth).

Per-class and per-clutter success, bottle yaw error at the snapshot and at grasp time, which bottle
failures trace to yaw, perception misses per gap, and detection vs the bottle's yaw relative to the
camera's line of sight and distance to the camera.

Usage: scripts/eval_analysis.py data/eval/<run_id> [--yaw-threshold 10]
"""
import argparse
import json
import os
import math
import statistics
import yaml
from collections import Counter, defaultdict

REPO = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
ap.add_argument('run_dir')
ap.add_argument('--yaw-threshold', type=float, default=10.0,
                help='[deg] bottle yaw error counted as wrong (architecture 8 target)')
ap.add_argument('--match-radius', type=float, default=0.10,
                help='[m] node attempt target to spawn pose (same as eval_run.py)')
ap.add_argument('--layout', default=os.path.join(REPO, 'src', 'graspsort_gazebo', 'config',
                                                 'world_layout.yaml'))
args = ap.parse_args()
run, YAW_T = args.run_dir, args.yaw_threshold
GRASP_CATS = {'no_grasp_candidate', 'grasp_slipped'}  # sort_logic.hpp categories a wrong yaw can cause

trials = [json.loads(l) for l in open(f'{run}/trials.jsonl')]
attempts = [json.loads(l) for l in open(f'{run}/node_metrics.jsonl')]


def fold(d):
    d = abs(d) % 180.0
    return 180.0 - d if d > 90.0 else d


def pct(xs, p):
    xs = sorted(xs)
    if not xs:
        return float('nan')
    k = (len(xs) - 1) * p / 100.0
    f = math.floor(k)
    c = min(f + 1, len(xs) - 1)
    return xs[f] + (xs[c] - xs[f]) * (k - f)


def rate(c, n):
    return f'{100.0 * c / n:5.1f} % ({c}/{n})' if n else '   n/a'


by_goal = {t['goal_id']: t for t in trials}
cell = defaultdict(lambda: [0, 0])
cls = defaultdict(lambda: [0, 0])
cls_cell = defaultdict(lambda: [0, 0])
marg = defaultdict(lambda: [0, 0])
for t in trials:
    n, g = t['config']['n_objects'], t['config']['min_gap']
    for o in t['objects']:
        for d in (cell[(n, g)], cls[o['class']], cls_cell[(o['class'], n, g)],
                  marg[f'{n} obj'], marg[f'gap {g*100:.0f} cm'], marg['overall']):
            d[0] += o['correct']
            d[1] += 1

print(f'run {run}: {len(trials)} trials, {sum(len(t["objects"]) for t in trials)} objects')
print('\n== success (object in correct bin) ==')
for k in ('overall', '2 obj', '4 obj', 'gap 8 cm', 'gap 3 cm'):
    print(f'  {k:10s} {rate(*marg[k])}')
for k in sorted(cell):
    print(f'  {k[0]} obj gap {k[1]*100:.0f} cm  {rate(*cell[k])}')
for c in sorted(cls):
    print(f'  class {c:12s} {rate(*cls[c])}   ' + '  '.join(
        f'{n}/{g*100:.0f}cm {rate(*cls_cell[(c, n, g)])}' for (n, g) in sorted(cell)))

# Yaw at snapshot (same as eval_summary)
snap = [fold(o['localization']['yaw_err_deg']) for t in trials for o in t['objects']
        if o['class'] == 'bottle' and o['localization']['yaw_err_deg'] is not None]
print(f'\n== bottle yaw error at snapshot: median {statistics.median(snap):.1f} p95 '
      f'{pct(snap, 95):.1f} deg, n={len(snap)}, > {YAW_T:g} deg: {sum(e > YAW_T for e in snap)}')
for (n, g) in sorted(cell):
    e = [fold(o['localization']['yaw_err_deg']) for t in trials
         if (t['config']['n_objects'], t['config']['min_gap']) == (n, g)
         for o in t['objects'] if o['class'] == 'bottle' and o['localization']['yaw_err_deg'] is not None]
    if e:
        print(f'  {n} obj gap {g*100:.0f} cm: median {statistics.median(e):.1f} p95 {pct(e, 95):.1f} (n={len(e)})')

# Join node attempts to ground-truth objects (nearest same-class spawn in the same goal).
def gt_of(a):
    t = by_goal.get(a['goal_id'])
    if not t:
        return None, None
    best, bd = None, 1e9
    for o in t['objects']:
        if o['class'] != a['class']:
            continue
        d = math.hypot(o['spawn']['x'] - a['target_pose']['x'], o['spawn']['y'] - a['target_pose']['y'])
        if d < bd:
            best, bd = o, d
    return t, (best if bd < args.match_radius else None)


print('\n== bottle attempts (yaw error at grasp time = target_pose.yaw vs spawn yaw) ==')
att_rows = []
for a in attempts:
    if a['class'] != 'bottle':
        continue
    t, o = gt_of(a)
    if o is None:
        continue
    ye = fold(math.degrees(a['target_pose']['yaw'] - o['spawn']['yaw']))
    att_rows.append((a['success'], a['category'], ye, o['name'], t['trial'], t['config']))
ok = [r[2] for r in att_rows if r[0]]
bad = [r for r in att_rows if not r[0]]
if ok:
    print(f'  successful attempts: n={len(ok)}, yaw err median {statistics.median(ok):.1f} '
          f'p95 {pct(ok, 95):.1f}, max {max(ok):.1f}')
print(f'  failed attempts: n={len(bad)}')
for r in bad:
    print(f'    {r[1]:20s} yaw err {r[2]:5.1f}  {r[3]} trial {r[4]} {r[5]}')

# Final bottle failures and yaw attribution
print('\n== bottles not in the correct bin ==')
for t in trials:
    for o in t['objects']:
        if o['class'] != 'bottle' or o['correct']:
            continue
        cats = [r for r in att_rows if r[3] == o['name'] and not r[0]]
        ye = o['localization']['yaw_err_deg']
        ye = fold(ye) if ye is not None else None
        grasp = [r for r in cats if r[1] in GRASP_CATS]
        yaw_linked = bool(grasp) and ye is not None and ye > YAW_T
        print(f'  {o["name"]:16s} {t["config"]} matched={o["localization"]["matched"]} '
              f'yaw_snap={"n/a" if ye is None else f"{ye:.1f}"} attempts={[r[1] for r in cats]} '
              f'final_bin={o["final_bin"]} -> {"YAW-LINKED" if yaw_linked else "not yaw"}')

# Perception misses
print('\n== perception misses at snapshot ==')
for g in sorted({t['config']['min_gap'] for t in trials}):
    objs = [o for t in trials if t['config']['min_gap'] == g for o in t['objects']]
    miss = [o for o in objs if not o['localization']['matched']]
    tr = [t for t in trials if t['config']['min_gap'] == g]
    tmiss = sum(any(not o['localization']['matched'] for o in t['objects']) for t in tr)
    unsettled = sum(any('did not settle' in n for n in t.get('notes', [])) for t in tr)
    print(f'  gap {g*100:.0f} cm: {len(miss)}/{len(objs)} objects missed '
          f'({Counter(o["class"] for o in miss)}), {tmiss}/{len(tr)} trials with a miss, '
          f'{unsettled} trials not settled')
    for t in tr:
        for o in t['objects']:
            if not o['localization']['matched']:
                sp = o['spawn']
                print(f'    trial {t["trial"]} n={t["config"]["n_objects"]} {o["name"]:16s} '
                      f'{o["class"]:11s} x {sp["x"]:.3f} y {sp["y"]:+.3f} yaw {math.degrees(sp["yaw"]):+7.1f} '
                      f'correct={o["correct"]}')
print('\n== all misses by class (any gap) ==')
allo = [o for t in trials for o in t['objects']]
for c in sorted({o['class'] for o in allo}):
    oc = [o for o in allo if o['class'] == c]
    print(f'  {c:11s} missed {sum(not o["localization"]["matched"] for o in oc)}/{len(oc)}')

print('\n== action statuses ==', Counter(t['action']['status'] for t in trials))

# Bottle yaw relative to the camera's line of sight, and distance to the camera
_cam = yaml.safe_load(open(args.layout))['/**']['ros__parameters']['camera']
CAM = (_cam['x'], _cam['y'])
print('\n== bottle yaw relative to line of sight, folded [0, 90] deg (0 = model x axis along the view) ==')
rows = []
for t in trials:
    for o in t['objects']:
        if o['class'] != 'bottle':
            continue
        sp = o['spawn']
        bearing = math.degrees(math.atan2(sp['y'] - CAM[1], sp['x'] - CAM[0]))
        rows.append((fold(math.degrees(sp['yaw']) - bearing), o['localization']['matched'], o['correct'],
                     math.hypot(sp['x'] - CAM[0], sp['y'] - CAM[1])))
for lo, hi in ((0, 15), (15, 30), (30, 45), (45, 60), (60, 75), (75, 90.01)):
    b = [r for r in rows if lo <= r[0] < hi]
    print(f'  {lo:2d}-{min(hi, 90):2.0f} deg: n={len(b):2d} missed {sum(not r[1] for r in b):2d} '
          f'correct {sum(r[2] for r in b):2d}')
print('  by distance to the camera (xy) and line-of-sight angle < 45 / >= 45 deg:')
dists = sorted(r[3] for r in rows)
edges = [dists[0], pct(dists, 33.3), pct(dists, 66.7), dists[-1] + 1e-9]
for lo, hi in zip(edges, edges[1:]):
    for name, sel in (('< 45', lambda r: r[0] < 45), ('>=45', lambda r: r[0] >= 45)):
        b = [r for r in rows if lo <= r[3] < hi and sel(r)]
        print(f'    {lo:.3f}-{hi:.3f} m, angle {name}: n={len(b):2d} missed {sum(not r[1] for r in b):2d}')
