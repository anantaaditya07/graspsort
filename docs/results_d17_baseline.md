# GraspSort evaluation results: D-17 baseline (before the overnight fixes)

- Generated: 2026-10-06
- Git commit: db044d1
- run_id: d17_full
- Scenario: D-17 (n_objects 2 or 4 x minimum footprint gap 8 or 3 cm, 10 trials per config planned; 2 objects = 1 ball + 1 bottle, 4 = 2 + 2; random positions and bottle yaw, fixed seeds)
- Data: 40 trials, 120 objects, 125 node metric lines (0 not joined to a trial)

## Results per configuration

| Metric | 2 obj, gap 8 cm | 2 obj, gap 3 cm | 4 obj, gap 8 cm | 4 obj, gap 3 cm | overall |
|---|---|---|---|---|---|
| Trials | 10 | 10 | 10 | 10 | 40 |
| Action status | SUCCEEDED 10 | SUCCEEDED 10 | SUCCEEDED 10 | SUCCEEDED 10 | SUCCEEDED 40 |
| Objects spawned | 20 | 20 | 40 | 40 | 120 |
| Pick success rate | 80.0 % (16/20) | 90.0 % (18/20) | 70.0 % (28/40) | 65.0 % (26/40) | 73.3 % (88/120) |
| Trials fully cleared | 60.0 % (6/10) | 80.0 % (8/10) | 30.0 % (3/10) | 40.0 % (4/10) | 52.5 % (21/40) |
| Detected at snapshot | 80.0 % (16/20) | 95.0 % (19/20) | 82.5 % (33/40) | 77.5 % (31/40) | 82.5 % (99/120) |
| 3D error sports ball median / p95 (mm) | 6.3 / 8.1 (n=10) | 5.5 / 7.5 (n=10) | 5.6 / 7.8 (n=18) | 5.7 / 7.5 (n=17) | 5.6 / 8.0 (n=55) |
| 3D error bottle median / p95 (mm) | 5.8 / 8.3 (n=6) | 5.8 / 14.8 (n=9) | 7.1 / 28.6 (n=15) | 7.3 / 72.0 (n=14) | 6.2 / 70.4 (n=44) |
| Bottle yaw error median / p95 (deg) | 12.1 / 18.7 (n=6) | 6.7 / 17.0 (n=9) | 8.3 / 39.4 (n=15) | 12.0 / 59.9 (n=14) | 9.6 / 51.1 (n=44) |
| Planning time p50 / p95 (s) | 0.077 / 0.098 (n=20) | 0.072 / 0.088 (n=23) | 0.072 / 0.096 (n=42) | 0.076 / 0.093 (n=29) | 0.074 / 0.097 (n=114) |
| Cycle time mean / p50 / p95 (s) | 13.15 / 12.37 / 15.55 (n=16) | 13.32 / 12.25 / 19.52 (n=18) | 12.73 / 12.25 / 15.04 (n=28) | 14.18 / 13.78 / 17.07 (n=26) | 13.35 / 12.47 / 16.96 (n=88) |
| Attempts per object | 1.00 (20/20, 10 trials) | 1.15 (23/20, 10 trials) | 1.05 (42/40, 10 trials) | 0.88 (35/40, 10 trials) | 1.00 (120/120, 40 trials) |

## Targets (architecture 8)

| Target | Required | Measured | Result |
|---|---|---|---|
| Pick success, low clutter (gap 8 cm) | >= 90 % | 73.3 % (44/60) | FAIL |
| 3D error median, sports ball | < 15 mm | 5.6 mm (n=55) | PASS |
| 3D error median, bottle | < 15 mm | 6.2 mm (n=44) | PASS |
| Bottle yaw error median | < 10 deg | 9.6 deg (n=44) | PASS |
| Planning time p95 | < 2 s | 0.097 s (n=114) | PASS |

## Failure breakdown (one primary label per object not in its correct bin)

| Category | 2 obj, gap 8 cm | 2 obj, gap 3 cm | 4 obj, gap 8 cm | 4 obj, gap 3 cm | overall |
|---|---|---|---|---|---|
| grasp_slipped | 0 | 1 | 2 | 0 | 3 |
| misdetection | 0 | 0 | 3 | 1 | 4 |
| unreachable | 0 | 0 | 0 | 1 | 1 |
| no_grasp_candidate | 0 | 0 | 1 | 3 | 4 |
| other | 0 | 0 | 0 | 2 | 2 |
| not_detected | 4 | 1 | 6 | 7 | 18 |
| **total not correct** | 4 | 2 | 12 | 14 | 32 |

Action failure reasons not attributable to a not-correct object: none.

Failed attempts by category (node metrics, per attempt incl. retries and reach-check give-ups):

| Category | 2 obj, gap 8 cm | 2 obj, gap 3 cm | 4 obj, gap 8 cm | 4 obj, gap 3 cm | overall |
|---|---|---|---|---|---|
| grasp_slipped | 3 | 5 | 11 | 2 | 21 |
| no_grasp_candidate | 0 | 0 | 1 | 3 | 4 |
| other | 0 | 0 | 0 | 6 | 6 |
| plan_failed | 1 | 0 | 3 | 1 | 5 |
| unreachable | 0 | 0 | 0 | 1 | 1 |

Label precedence: action failure reason attributed to the object, else wrong_bin, off_table (final z < 0.7 m), not_detected, still_on_table, no_final_pose.

## Notes / limits

- Simulation only (Gazebo Classic); ground truth from Gazebo is used only by the evaluation scripts.
- One fixed oblique RGB-D camera (D-05), no hand-eye loop.
- Two object classes, sports ball and bottle (D-13); 2 and 4 objects instead of the PDF's 3/5/7, gaps 8 / 3 cm instead of 8 / 2 cm (D-17).
- 3D and yaw errors are from one snapshot before each goal (nearest same-class estimate within 0.10 m); unmatched objects count as not detected and have no error.
- Planning time is the planning_time field of sort_task_node per executed attempt (reach-check give-ups excluded); cycle time covers successful attempts only.
- Percentiles use linear interpolation (type 7); n is shown for every value.
