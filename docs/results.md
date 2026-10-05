# GraspSort evaluation results

- Generated: 2026-10-06
- Git commit: ea6409e
- run_id: d20_full
- Scenario: D-17 (n_objects 2 or 4 x minimum footprint gap 8 or 3 cm, 10 trials per config planned; 2 objects = 1 ball + 1 bottle, 4 = 2 + 2; random positions and bottle yaw, fixed seeds)
- Data: 40 trials, 120 objects, 112 node metric lines (0 not joined to a trial)

## Results per configuration

| Metric | 2 obj, gap 8 cm | 2 obj, gap 3 cm | 4 obj, gap 8 cm | 4 obj, gap 3 cm | overall |
|---|---|---|---|---|---|
| Trials | 10 | 10 | 10 | 10 | 40 |
| Action status | SUCCEEDED 10 | SUCCEEDED 10 | SUCCEEDED 10 | SUCCEEDED 10 | SUCCEEDED 40 |
| Objects spawned | 20 | 20 | 40 | 40 | 120 |
| Pick success rate | 80.0 % (16/20) | 95.0 % (19/20) | 75.0 % (30/40) | 65.0 % (26/40) | 75.8 % (91/120) |
| Trials fully cleared | 60.0 % (6/10) | 90.0 % (9/10) | 40.0 % (4/10) | 20.0 % (2/10) | 52.5 % (21/40) |
| Detected at snapshot | 80.0 % (16/20) | 95.0 % (19/20) | 82.5 % (33/40) | 77.5 % (31/40) | 82.5 % (99/120) |
| 3D error sports ball median / p95 (mm) | 6.3 / 8.1 (n=10) | 5.5 / 7.5 (n=10) | 5.6 / 7.8 (n=18) | 5.7 / 7.5 (n=17) | 5.6 / 8.0 (n=55) |
| 3D error bottle median / p95 (mm) | 1.8 / 2.4 (n=6) | 2.4 / 2.7 (n=9) | 2.2 / 27.1 (n=15) | 2.4 / 72.0 (n=14) | 2.2 / 70.4 (n=44) |
| Bottle yaw error median / p95 (deg) | 12.1 / 18.7 (n=6) | 6.7 / 17.0 (n=9) | 8.3 / 39.4 (n=15) | 12.0 / 59.9 (n=14) | 9.6 / 51.1 (n=44) |
| Planning time p50 / p95 (s) | 0.070 / 0.089 (n=16) | 0.072 / 0.091 (n=20) | 0.073 / 0.095 (n=35) | 0.076 / 0.090 (n=29) | 0.073 / 0.091 (n=100) |
| Cycle time mean / p50 / p95 (s) | 12.82 / 12.27 / 16.05 (n=16) | 12.18 / 12.09 / 12.54 (n=19) | 12.81 / 12.14 / 15.32 (n=30) | 13.77 / 12.81 / 17.15 (n=26) | 12.96 / 12.22 / 16.19 (n=91) |
| Attempts per object | 0.80 (16/20, 10 trials) | 1.00 (20/20, 10 trials) | 0.88 (35/40, 10 trials) | 0.88 (35/40, 10 trials) | 0.88 (106/120, 40 trials) |

## Targets (architecture 8)

| Target | Required | Measured | Result |
|---|---|---|---|
| Pick success, low clutter (gap 8 cm) | >= 90 % | 76.7 % (46/60) | FAIL |
| 3D error median, sports ball | < 15 mm | 5.6 mm (n=55) | PASS |
| 3D error median, bottle | < 15 mm | 2.2 mm (n=44) | PASS |
| Bottle yaw error median | < 10 deg | 9.6 deg (n=44) | PASS |
| Planning time p95 | < 2 s | 0.091 s (n=100) | PASS |

## Failure breakdown (one primary label per object not in its correct bin)

| Category | 2 obj, gap 8 cm | 2 obj, gap 3 cm | 4 obj, gap 8 cm | 4 obj, gap 3 cm | overall |
|---|---|---|---|---|---|
| plan_failed | 0 | 0 | 0 | 1 | 1 |
| misdetection | 0 | 0 | 3 | 0 | 3 |
| unreachable | 0 | 0 | 0 | 1 | 1 |
| no_grasp_candidate | 0 | 0 | 2 | 3 | 5 |
| other | 0 | 0 | 0 | 2 | 2 |
| not_detected | 4 | 1 | 5 | 6 | 16 |
| still_on_table | 0 | 0 | 0 | 1 | 1 |
| **total not correct** | 4 | 1 | 10 | 14 | 29 |

Action failure reasons not attributable to a not-correct object: none.

Failed attempts by category (node metrics, per attempt incl. retries and reach-check give-ups):

| Category | 2 obj, gap 8 cm | 2 obj, gap 3 cm | 4 obj, gap 8 cm | 4 obj, gap 3 cm | overall |
|---|---|---|---|---|---|
| grasp_slipped | 0 | 0 | 1 | 0 | 1 |
| no_grasp_candidate | 0 | 0 | 2 | 3 | 5 |
| other | 0 | 0 | 0 | 6 | 6 |
| plan_failed | 0 | 1 | 4 | 3 | 8 |
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

The tables above come from `scripts/eval_summary.py` on run `d20_full`: D-17, 40 trials, seeds as in the
baseline, with D-20. The baseline before D-20 (run `d17_full`, commit db044d1) is in
[results_d17_baseline.md](results_d17_baseline.md). The full output of `scripts/eval_analysis.py` is in
[results_analysis.txt](results_analysis.txt) and [results_d17_baseline_analysis.txt](results_d17_baseline_analysis.txt).

### Success rate (object in its correct bin)

| | Baseline (d17_full) | D-20 (d20_full) |
|---|---|---|
| Overall | 73.3 % (88/120) | **75.8 % (91/120)** |
| Sports ball | 93.3 % (56/60) | **96.7 % (58/60)** |
| Bottle | 53.3 % (32/60) | **55.0 % (33/60)** |
| 2 objects | 85.0 % (34/40) | 87.5 % (35/40) |
| 4 objects | 67.5 % (54/80) | 70.0 % (56/80) |
| Gap 8 cm | 73.3 % (44/60) | 76.7 % (46/60) |
| Gap 3 cm | 73.3 % (44/60) | 75.0 % (45/60) |

### Bottles

| | Baseline | D-20 |
|---|---|---|
| Yaw error median / p95 | 9.6 / 51.1 deg (n=44) | 9.6 / 51.1 deg (n=44) |
| 3D error median | 6.2 mm | **2.2 mm** |
| Gripper close timeouts (fingers sent inside the bottle) | 20 of 55 closes | **2** |
| Failed pick attempts / attempts | 33 / 65 | **20 / 53** |
| Bottles not in the correct bin | 28 | 27 |
| ... never detected | 15 | 15 |
| ... yaw-linked (grasp failure, yaw error > 10 deg) | 5 (18 %) | 6 (22 %) |
| ... other (clutter, transport planning, bin full) | 8 | 6 |

- **Yaw:** n is 44, not 60, because 16 bottles were never localized. The yaw values are unchanged by D-20,
  because the axes still come from minAreaRect, and with the same seeds the snapshot is deterministic. The
  bottle's rounded cross-section limits yaw from a half view to about 0-15 deg (D-20).
- **What D-20 fixed:** the width (no more closing inside the bottle), the centre, and the end-on long-axis
  flip. It did not fix the yaw tail.

### Perception misses

- At 3 cm gap 10/60 objects were missed (7 bottles, 3 balls), in 7/20 trials. At 8 cm gap 11/60 were missed
  (9 bottles, 2 balls), in 9/20 trials. **Missing objects is not caused by clutter.**
- Misses depend on the bottle's yaw relative to the camera's line of sight (all 60 bottles, 16 missed):

| Line-of-sight angle | Bottles | Missed |
|---|---|---|
| < 45 deg (seen end-on to diagonal) | 30 | 14 |
| >= 45 deg (seen side-on) | 30 | 2 |
| < 45 deg and within 0.45 m of the camera | 14 | 8 |

- The YOLOv8n COCO detector rarely finds a mustard bottle seen end-on, especially close to the camera (steep
  view). The reference layout (yaw 0, 0.60 m from the camera) is in the better far band. Fixing this needs a
  fine-tuned detector (D-04 stretch goal), which is out of scope for D-20.

### Targets

The architecture 8 target of >= 90 % pick success at low clutter is **not met** (76.7 %). The ball alone
reaches 96.7 %. The gap comes from bottle detection (15 of the 27 bottle failures).
