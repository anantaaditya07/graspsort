# GraspSort evaluation results

- Generated: 2026-10-06
- Git commit: 66b7f0b
- run_id: d22_full
- Scenario: D-17 (n_objects 2 or 4 x minimum footprint gap 8 or 3 cm, 10 trials per config planned; 2 objects = 1 ball + 1 bottle, 4 = 2 + 2; random positions and bottle yaw, fixed seeds)
- Data: 40 trials, 120 objects, 132 node metric lines (0 not joined to a trial)

## Results per configuration

| Metric | 2 obj, gap 8 cm | 2 obj, gap 3 cm | 4 obj, gap 8 cm | 4 obj, gap 3 cm | overall |
|---|---|---|---|---|---|
| Trials | 10 | 10 | 10 | 10 | 40 |
| Action status | SUCCEEDED 10 | SUCCEEDED 10 | SUCCEEDED 10 | SUCCEEDED 10 | SUCCEEDED 40 |
| Objects spawned | 20 | 20 | 40 | 40 | 120 |
| Pick success rate | 95.0 % (19/20) | 95.0 % (19/20) | 77.5 % (31/40) | 67.5 % (27/40) | 80.0 % (96/120) |
| Trials fully cleared | 90.0 % (9/10) | 90.0 % (9/10) | 30.0 % (3/10) | 20.0 % (2/10) | 57.5 % (23/40) |
| Detected at snapshot | 100.0 % (20/20) | 95.0 % (19/20) | 92.5 % (37/40) | 87.5 % (35/40) | 92.5 % (111/120) |
| 3D error sports ball median / p95 (mm) | 6.3 / 8.1 (n=10) | 5.5 / 7.5 (n=10) | 5.6 / 7.8 (n=19) | 5.6 / 7.5 (n=18) | 5.6 / 8.0 (n=57) |
| 3D error bottle median / p95 (mm) | 1.7 / 2.3 (n=10) | 2.4 / 2.7 (n=9) | 2.2 / 17.1 (n=18) | 2.1 / 71.6 (n=17) | 2.1 / 68.5 (n=54) |
| Bottle yaw error median / p95 (deg) | 9.9 / 18.1 (n=10) | 6.7 / 17.0 (n=9) | 7.9 / 36.4 (n=18) | 8.3 / 59.6 (n=17) | 8.3 / 42.7 (n=54) |
| Planning time p50 / p95 (s) | 0.072 / 0.084 (n=21) | 0.068 / 0.090 (n=19) | 0.070 / 0.114 (n=44) | 0.074 / 0.129 (n=36) | 0.072 / 0.107 (n=120) |
| Cycle time mean / p50 / p95 (s) | 12.54 / 12.32 / 13.83 (n=19) | 12.14 / 12.17 / 12.38 (n=19) | 12.53 / 12.24 / 14.31 (n=25) | 13.27 / 12.60 / 15.19 (n=24) | 12.65 / 12.31 / 14.89 (n=87) |
| Attempts per object | 1.05 (21/20, 10 trials) | 0.95 (19/20, 10 trials) | 1.10 (44/40, 10 trials) | 1.05 (42/40, 10 trials) | 1.05 (126/120, 40 trials) |

## Targets (architecture 8)

| Target | Required | Measured | Result |
|---|---|---|---|
| Pick success, low clutter (gap 8 cm) | >= 90 % | 83.3 % (50/60) | FAIL |
| 3D error median, sports ball | < 15 mm | 5.6 mm (n=57) | PASS |
| 3D error median, bottle | < 15 mm | 2.1 mm (n=54) | PASS |
| Bottle yaw error median | < 10 deg | 8.3 deg (n=54) | PASS |
| Planning time p95 | < 2 s | 0.107 s (n=120) | PASS |

## Failure breakdown (one primary label per object not in its correct bin)

| Category | 2 obj, gap 8 cm | 2 obj, gap 3 cm | 4 obj, gap 8 cm | 4 obj, gap 3 cm | overall |
|---|---|---|---|---|---|
| misdetection | 1 | 0 | 3 | 3 | 7 |
| unreachable | 0 | 0 | 0 | 1 | 1 |
| no_grasp_candidate | 0 | 0 | 2 | 3 | 5 |
| other | 0 | 0 | 0 | 2 | 2 |
| off_table | 0 | 0 | 1 | 1 | 2 |
| not_detected | 0 | 1 | 2 | 2 | 5 |
| still_on_table | 0 | 0 | 1 | 1 | 2 |
| **total not correct** | 1 | 1 | 9 | 13 | 24 |

Action failure reasons not attributable to a not-correct object: misdetection 13.

Failed attempts by category (node metrics, per attempt incl. retries and reach-check give-ups):

| Category | 2 obj, gap 8 cm | 2 obj, gap 3 cm | 4 obj, gap 8 cm | 4 obj, gap 3 cm | overall |
|---|---|---|---|---|---|
| execution_failed | 0 | 0 | 0 | 1 | 1 |
| grasp_slipped | 0 | 0 | 1 | 0 | 1 |
| no_grasp_candidate | 0 | 0 | 2 | 3 | 5 |
| other | 0 | 0 | 0 | 6 | 6 |
| plan_failed | 2 | 0 | 18 | 11 | 31 |
| unreachable | 0 | 0 | 0 | 1 | 1 |

Label precedence: action failure reason attributed to the object, else wrong_bin, off_table (final z < 0.7 m), not_detected, still_on_table, no_final_pose.

## Notes / limits

- Simulation only (Gazebo Classic); ground truth from Gazebo is used only by the evaluation scripts.
- One fixed oblique RGB-D camera (D-05), no hand-eye loop.
- Two object classes, sports ball and bottle (D-13); 2 and 4 objects instead of the PDF's 3/5/7, gaps 8 / 3 cm instead of 8 / 2 cm (D-17).
- 3D and yaw errors are from one snapshot before each goal (nearest same-class estimate within 0.10 m); unmatched objects count as not detected and have no error.
- Planning time is the planning_time field of sort_task_node per executed attempt (reach-check give-ups excluded); cycle time covers successful attempts only.
- Percentiles use linear interpolation (type 7); n is shown for every value.

## Analysis (main agent, 2026-10-06)

The tables above come from `scripts/eval_summary.py` on run `d22_full`: D-17, 40 trials, the same seeds as
the earlier runs, with D-22 (detector `conf_threshold` 0.35 -> 0.15). Earlier runs: before D-20
[results_d17_baseline.md](results_d17_baseline.md), D-20 [results_d20.md](results_d20.md). The full output of
`scripts/eval_analysis.py` is in [results_d22_analysis.txt](results_d22_analysis.txt) (d22) and
[results_d20_analysis.txt](results_d20_analysis.txt) (d20).

### Before / after

| | Baseline (d17_full) | D-20 (d20_full, before) | D-22 (d22_full, after) |
|---|---|---|---|
| Objects in the correct bin | 73.3 % (88/120) | 75.8 % (91/120) | **80.0 % (96/120)** |
| Sports ball | 93.3 % (56/60) | 96.7 % (58/60) | 98.3 % (59/60) |
| Bottle | 53.3 % (32/60) | 55.0 % (33/60) | 61.7 % (37/60) |
| Low clutter, gap 8 cm (target >= 90 %) | 73.3 % (44/60) | 76.7 % (46/60) | 83.3 % (50/60) |
| 2 objects / 4 objects | 85.0 % / 67.5 % | 87.5 % / 70.0 % | 95.0 % / 72.5 % |
| Trials fully cleared | 21/40 | 21/40 | 23/40 |
| Detected at snapshot | 99/120 | 99/120 | **111/120** |
| ... bottles | 44/60 | 44/60 | **54/60** |
| `not_detected` (primary failure label) | | 16 | **5** |
| Bottle yaw error median / p95 | 9.6 / 51.1 deg (n=44) | 9.6 / 51.1 deg (n=44) | 8.3 / 42.7 deg (n=54) |
| Failed attempts at `lower` (into the bin) | | 0 | 13 |
| plan_failed attempts | | 8 | 31 |

### What D-22 fixed

The biggest failure reason in d20_full was missed bottles (16 of 29 objects not in their bin), almost all
seen end-on. On 300 single-bottle frames the end-on bottle still scored 0.10-0.35 as `bottle`, under the 0.35
threshold (D-22). At 0.15, objects missed at the snapshot fell from 21 to 9 (bottles 16 -> 6 of 60, balls
5 -> 3), and the `not_detected` failure label from 16 to 5.

### New problem (D-23, not fixed)

At 0.15 the detector also finds objects already lying in the bins (53 ball and 26 bottle scene objects inside
the bins, vs 0 and 17 before). As planning-scene obstacles they block the Pilz LIN `lower` into bin_ball
(13 failed attempts). The ball still ends in its bin, but the action reports it as failed (the 13
"not attributable" failures above) and retries cost time. Most remaining failures at 4 objects are planning
failures and yaw-linked bottle grasps, not detection. Options for D-23 are in DECISIONS.md.

### Targets

The architecture 8 target of >= 90 % pick success at low clutter is **still not met** (83.3 %).
