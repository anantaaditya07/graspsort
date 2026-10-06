# GraspSort evaluation results

- Generated: 2026-10-06
- Git commit: b35d12f (tracked files modified)
- run_id: d23_full
- Scenario: D-17 (n_objects 2 or 4 x minimum footprint gap 8 or 3 cm, 10 trials per config planned; 2 objects = 1 ball + 1 bottle, 4 = 2 + 2; random positions and bottle yaw, fixed seeds)
- Data: 40 trials, 120 objects, 119 node metric lines (0 not joined to a trial)

## Results per configuration

| Metric | 2 obj, gap 8 cm | 2 obj, gap 3 cm | 4 obj, gap 8 cm | 4 obj, gap 3 cm | overall |
|---|---|---|---|---|---|
| Trials | 10 | 10 | 10 | 10 | 40 |
| Action status | SUCCEEDED 10 | SUCCEEDED 10 | SUCCEEDED 10 | SUCCEEDED 10 | SUCCEEDED 40 |
| Objects spawned | 20 | 20 | 40 | 40 | 120 |
| Pick success rate | 100.0 % (20/20) | 95.0 % (19/20) | 85.0 % (34/40) | 75.0 % (30/40) | 85.8 % (103/120) |
| Trials fully cleared | 100.0 % (10/10) | 90.0 % (9/10) | 50.0 % (5/10) | 40.0 % (4/10) | 70.0 % (28/40) |
| Detected at snapshot | 100.0 % (20/20) | 95.0 % (19/20) | 92.5 % (37/40) | 87.5 % (35/40) | 92.5 % (111/120) |
| 3D error sports ball median / p95 (mm) | 6.3 / 8.1 (n=10) | 5.5 / 7.5 (n=10) | 5.6 / 7.8 (n=19) | 5.6 / 7.5 (n=18) | 5.6 / 8.0 (n=57) |
| 3D error bottle median / p95 (mm) | 1.7 / 2.3 (n=10) | 2.4 / 2.7 (n=9) | 2.2 / 17.1 (n=18) | 2.1 / 71.2 (n=17) | 2.1 / 68.4 (n=54) |
| Bottle yaw error median / p95 (deg) | 9.9 / 18.1 (n=10) | 6.7 / 17.0 (n=9) | 7.9 / 36.3 (n=18) | 8.3 / 59.6 (n=17) | 8.3 / 42.6 (n=54) |
| Planning time p50 / p95 (s) | 0.070 / 0.094 (n=20) | 0.069 / 0.089 (n=19) | 0.072 / 0.096 (n=36) | 0.078 / 0.096 (n=32) | 0.072 / 0.096 (n=107) |
| Cycle time mean / p50 / p95 (s) | 12.49 / 12.37 / 13.24 (n=20) | 12.14 / 12.10 / 12.39 (n=19) | 12.93 / 12.34 / 14.54 (n=35) | 13.07 / 12.70 / 14.73 (n=32) | 12.75 / 12.34 / 14.54 (n=106) |
| Attempts per object | 1.00 (20/20, 10 trials) | 0.95 (19/20, 10 trials) | 0.90 (36/40, 10 trials) | 0.95 (38/40, 10 trials) | 0.94 (113/120, 40 trials) |

## Targets (architecture 8)

| Target | Required | Measured | Result |
|---|---|---|---|
| Pick success, low clutter (gap 8 cm) | >= 90 % | 90.0 % (54/60) | PASS |
| 3D error median, sports ball | < 15 mm | 5.6 mm (n=57) | PASS |
| 3D error median, bottle | < 15 mm | 2.1 mm (n=54) | PASS |
| Bottle yaw error median | < 10 deg | 8.3 deg (n=54) | PASS |
| Planning time p95 | < 2 s | 0.096 s (n=107) | PASS |

## Failure breakdown (one primary label per object not in its correct bin)

| Category | 2 obj, gap 8 cm | 2 obj, gap 3 cm | 4 obj, gap 8 cm | 4 obj, gap 3 cm | overall |
|---|---|---|---|---|---|
| misdetection | 0 | 0 | 1 | 0 | 1 |
| unreachable | 0 | 0 | 0 | 1 | 1 |
| no_grasp_candidate | 0 | 0 | 2 | 3 | 5 |
| other | 0 | 0 | 0 | 2 | 2 |
| off_table | 0 | 0 | 1 | 1 | 2 |
| not_detected | 0 | 1 | 2 | 2 | 5 |
| still_on_table | 0 | 0 | 0 | 1 | 1 |
| **total not correct** | 0 | 1 | 6 | 10 | 17 |

Action failure reasons not attributable to a not-correct object: none.

Failed attempts by category (node metrics, per attempt incl. retries and reach-check give-ups):

| Category | 2 obj, gap 8 cm | 2 obj, gap 3 cm | 4 obj, gap 8 cm | 4 obj, gap 3 cm | overall |
|---|---|---|---|---|---|
| grasp_slipped | 0 | 0 | 1 | 0 | 1 |
| no_grasp_candidate | 0 | 0 | 2 | 3 | 5 |
| other | 0 | 0 | 0 | 6 | 6 |
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

The tables above come from `scripts/eval_summary.py` on run `d23_full`: D-17, 40 trials, the same seeds as
every earlier run, with D-22 (detector threshold 0.15), D-23 (objects inside bin areas kept out of the
planning scene) and D-24 (each goal starts at the ready state). Earlier runs: [results_d17_baseline.md](results_d17_baseline.md),
[results_d20.md](results_d20.md), [results_d22.md](results_d22.md). `scripts/eval_analysis.py` output:
[results_analysis.txt](results_analysis.txt) (d23), [results_d22_analysis.txt](results_d22_analysis.txt),
[results_d20_analysis.txt](results_d20_analysis.txt).

### d20 / d22 / d23

| | d20 (D-20) | d22 (+ D-22) | d23 (+ D-23, D-24) |
|---|---|---|---|
| Objects in the correct bin | 75.8 % (91/120) | 80.0 % (96/120) | **85.8 % (103/120)** |
| Sports ball | 96.7 % (58/60) | 98.3 % (59/60) | 98.3 % (59/60) |
| Bottle | 55.0 % (33/60) | 61.7 % (37/60) | 73.3 % (44/60) |
| Low clutter, gap 8 cm (target >= 90 %) | 76.7 % (46/60) | 83.3 % (50/60) | **90.0 % (54/60)** |
| 2 objects / 4 objects | 87.5 % / 70.0 % | 95.0 % / 72.5 % | 97.5 % / 80.0 % |
| Trials fully cleared | 21/40 | 23/40 | 28/40 |
| Detected at snapshot (bottles) | 99/120 (44/60) | 111/120 (54/60) | 111/120 (54/60) |
| `not_detected` (primary failure label) | 16 | 5 | 5 |
| Failed `lower` into the bin | 0 | 13 | **0** |
| plan_failed attempts | 8 | 31 | **0** |
| Bottle yaw error median / p95 (deg) | 9.6 / 51.1 (n=44) | 8.3 / 42.7 (n=54) | 8.3 / 42.6 (n=54) |
| Planning time p95 (s) | 0.091 | 0.107 | 0.096 |

- **D-23:** the scene manager ignored in-bin detections (99 log lines), and no `lower` failed. The 13 balls
  that d22 reported as failed although they ended in their bin are gone (no unattributed failures).
- **D-24:** the first trial of the run (2:0.08 t0) starts from the spawn pose and now succeeds. All 10
  trials at 2 objects / 8 cm are fully cleared.
- Perception is unchanged from d22 (same seeds, deterministic snapshot), so the gain over d22 is all in
  manipulation.

### Remaining failures (17 objects, 16 of them bottles)

5 not detected, 5 no grasp candidate (yaw tail: the fitted footprint is too wide across the fingers), 2 off the
table, 2 "other" (no release slot in bin_bottle), 1 unreachable, 1 misdetection, 1 still on the table. 16 of
the 17 are in the 4-object configs.

### Targets, honestly

The architecture 8 target of **>= 90 % pick success at low clutter is met exactly: 90.0 % (54/60)**. This is
on the threshold. One more failure would miss it, and the 95 % confidence interval for 54/60 is about
80-95 % (Wilson). It is one run of 60 objects on fixed seeds, with 2 and 4 objects instead of the PDF's
3/5/7 (D-17). Read it as "about 90 %", not as a margin. At 3 cm gap the rate is 81.7 % (49/60), and bottles
at 4 objects are still weak (55-70 %).
