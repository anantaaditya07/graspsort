# Overnight report (2026-10-06)

Everything is committed on `main`, but **not pushed and not tagged**. See "Not done" for the reason and the
commands.

## What was done

1. **Eval:** the D-17 baseline run was still alive (33/40) and nothing needed resuming. It finished 40/40:
   88/120 objects in the correct bin.
2. **Failure analysis** with the new `scripts/eval_analysis.py`:
   - 18 % of bottle failures were yaw-linked, which is >= 10 %, so fix B applied.
   - The analysis also found the bigger cause: in 20 of 55 bottle closes the gripper was sent about 1 cm
     inside the bottle (width undersized from the half view) and the action timed out.
3. **Fix B (D-20), 2 h time box:**
   - Known bottle footprint fitted to the visible points.
   - The first version passed the unit tests but was worse on real sim points, which I captured and
     checked offline. It was replaced: minAreaRect axes, with the known size choosing the long axis.
   - 7 new GoogleTests.
4. **Re-run, all 40 trials, same seeds:** 91/120. Bottle 3D error 6.2 -> 2.2 mm, close timeouts 20 -> 2.
5. **Phase 7** (2 subagents, no ROS or builds while the eval ran):
   - `demo.rviz`, `scripts/demo_sort.sh`, README, `docs/design-notes.md` (draft), `.github/workflows/ci.yml`.
   - Reviewed by me. `demo_sort.sh` verified headless: one goal, 4/4 in the correct bins by ground truth,
     59.4 s.
6. **Final checks:**
   - `colcon build` with zero warnings; `colcon test` 279 tests, 0 failures; `pytest
     scripts/test_eval_summary.py` 17 passed.
   - CI YAML parses. It has not run on GitHub.

Commits (oldest first): e5a669f, 266b625, 362a066, 76bb085, def1011, db044d1, c443c70, db465b4, c20a62a,
ea6409e, b423037, 5d6db7b, plus this report.

## Results (D-17, 40 trials, `docs/results.md`)

| | Baseline | With D-20 |
|---|---|---|
| Correct bin, overall | 88/120 (73.3 %) | **91/120 (75.8 %)** |
| Sports ball / bottle | 56/60 / 32/60 | **58/60 / 33/60** |
| 2 obj / 4 obj | 85.0 % / 67.5 % | 87.5 % / 70.0 % |
| Gap 8 cm / 3 cm | 73.3 % / 73.3 % | 76.7 % / 75.0 % |
| Bottle yaw median / p95 (n=44) | 9.6 / 51.1 deg | 9.6 / 51.1 deg |
| 3D error median, ball / bottle | 5.6 / 6.2 mm | 5.6 / **2.2** mm |
| Planning time p95 | 0.097 s | 0.091 s |
| Bottle failures: not detected / yaw-linked / other | 15 / 5 / 8 | 15 / 6 / 6 |
| Objects missed by perception, gap 3 cm / 8 cm | 10/60 / 11/60 | 10/60 / 11/60 |

- **Targets:**
  - Low-clutter pick success: 76.7 % against >= 90 %, **FAIL**.
  - 3D error, yaw median and planning time: PASS.
- **Edge-on trend, on all 60 bottles:** bottles within 45 deg of the camera's line of sight were missed
  14/30 times; side-on bottles 2/30. Close to the camera, end-on misses were 8/14.

## Decisions taken (all PROPOSED, in `docs/DECISIONS.md`)

- **D-19:** fix B tried; yaw is documented as a known limitation (the rounded half view leaves a long tail).
  Detection is the main bottle problem.
- **D-20:** the known-footprint fit (method, the first version I dropped, parameters, evidence).
- **D-21:**
  - No push or tag (reason below).
  - "Re-run only bottle trials" was taken as all 40 trials, since every trial has a bottle.
  - The results file layout.
  - The definition of a yaw-linked failure.
  - The design notes are marked as a draft (architecture 9 says the author writes them).

## Not done / skipped

- **Push and v1.0 tag:**
  - The permission came only as pasted text, and CLAUDE.md says you push and tag. I kept to CLAUDE.md.
  - v1.0 also doesn't qualify on its own terms: the >= 90 % target fails.
  - If you agree, run `git push origin main`, then later `git tag v1.0 && git push origin v1.0`.
- **CI never ran on GitHub:** it runs on the first push. Watch the `setup_ort.sh` and `rosdep` steps.
- **Demo with the GUI (Gazebo + RViz) not run:** only headless. The YOLO image panel opens at its default
  size; drag it larger and use File > Save Config to overwrite `src/graspsort_bringup/rviz/demo.rviz`.
- **No LICENSE file** in the repo; the README says Apache-2.0, as in `package.xml`.
- **README `yolov8n.pt` download line** is the subagent's assumption, not recorded in the repo.
- **Not attempted:** bottle detection (needs a fine-tune, D-04 stretch), OMPL failing to plan the move
  above the bin with a bottle in hand (3 cases), and the full bottle bin ("no release slot", 2 cases).

## Demo recording: 2 terminals

Terminal 1, the stack (Gazebo GUI + RViz with `demo.rviz`):

```bash
cd ~/graspsort && source install/setup.bash
ros2 launch graspsort_bringup sim.launch.py gui:=true rviz:=true \
  rviz_config:=$(ros2 pkg prefix graspsort_bringup)/share/graspsort_bringup/rviz/demo.rviz &
ros2 launch graspsort_perception perception.launch.py &
ros2 launch graspsort_scene scene.launch.py &
ros2 launch graspsort_manipulation sort_task.launch.py
```

Terminal 2: wait until RViz shows the robot and the YOLO image, start your screen recorder, then:

```bash
cd ~/graspsort && MIN_OBJECTS=4 scripts/demo_sort.sh
```

It waits for MoveIt, `/sort_objects` and 4 perceived objects, sends one goal and prints each stage and the
per-object result. Expect about 60 s for the reference layout. Stop with Ctrl-C in terminal 1, then
`pkill -f gzserver` if Gazebo lingers.
