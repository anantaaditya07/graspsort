# GraspSort build checklist

Derived from docs/architecture.txt, section 10 (build plan), with tasks expanded from sections 4-9.
Legend: `[x]` done and verified, `[ ]` open. A phase is done only when its "Done when" passes.
Open questions blocking a phase are listed in docs/DECISIONS.md (D-xx).

## Phase 0 - Validate risky assumptions
- [x] Repo: CLAUDE.md, .gitignore (build/, install/, log/, *.onnx, *.pt, third_party/onnxruntime/,
      data/), architecture.txt present
- [x] Installed-package audit (DECISIONS.md header). The user installed robotiq_description, the
      panda resources, ur_simulation_gz and ign_ros2_control (2026-10-04)
- [x] A: UR5e + ros2_control on Gazebo Classic: controllers active, FJT error 0.00003 rad, RTF 1.00;
      MoveIt plan+execute 8/8 (docs/phase0/A)
- [x] A: gripper: Robotiq 2F-85 is unstable in contact (FAIL); custom prismatic gripper PASS
      (docs/phase0/A, check 3)
- [x] B: attach mechanism: custom plugin 5/5 runs, 0.095 mm / 0.079 deg drift, 4200-cycle stress test
      with 0 failures; IFRA hangs (docs/phase0/B)
- [x] C: YOLOv8n detection rate measured for 12 models x 4 viewpoints: only ball and bottle pass,
      oblique only, so the fallback applies (docs/phase0/C)
- [x] C: RGB-D topics: 10 Hz, 32FC1, shared optical frame, depth error <= 3.1 mm (docs/phase0/C)
- [x] Main-agent spot check of A, B and C (DECISIONS.md, end)
- [x] USER: D-01..D-08 ACCEPTED (2026-10-04); D-04 can/box fine-tune is an optional stretch goal

**Status: Phase 0 DONE (2026-10-04).**

**Done when:** all three checks pass or a fallback is decided (D-01..D-04).

## Phase 1 - World, robot, MoveIt
- [x] Workspace layout: repo root is the colcon workspace; 3 packages (graspsort_msgs,
      graspsort_gazebo, graspsort_bringup) build with zero warnings; docs/COLCON_IGNORE (D-10)
- [x] `colcon test`: 42 tests, 0 failures (registry GoogleTests, world/layout/xacro pytest,
      bringup config consistency pytest)
- [x] scripts/fetch_models.sh (D-08): pinned models, SHA-256 verified, masses, inertia, collision,
      material and pose fixed; models rest with 0.0 mm drift (D-10)
- [x] graspsort_gazebo: world with table, 3 bins, 6 objects (2 each of ball, bottle, cup), gazebo_ros_state,
      attach plugin; layout in config/world_layout.yaml
- [x] Arm + gripper + camera URDF per D-02/D-05 (gripper_tcp; camera 50 deg, 0.75 m)
- [x] graspsort_msgs/AttachLink.srv; /attach and /detach on the real finger link verified (D-03)
- [x] graspsort_bringup: own sim launch (D-01), controllers, MoveIt config (RRTConnect default + Pilz,
      D-06), named states home/ready/open/closed, CycloneDDS in every launch (tested)
- [x] Static frames: world -> table, bin_cup, bin_bottle, bin_ball (from world_layout.yaml)
- [x] Integration, headless (2026-10-04): 3 controllers active;
      `move_named.py home ready home ready home` 5/5 SUCCESS (planning 0.020-0.027 s,
      joint error <= 0.00096 rad); gripper close/open SUCCEEDED (0.0400 / 0.0000);
      /camera/color 9.98 Hz rgb8, /camera/depth 9.98 Hz 32FC1, both frame camera_color_optical_frame
- [x] Pilz LIN plan + execute 5 cm down/up from ready, within 0.1 mm (subagent C, not re-run in integration)
- [x] USER: RViz MotionPlanning plan + execute verified by the user (2026-10-04)
- [x] USER: D-09 and D-10 ACCEPTED (2026-10-04)

**Status: Phase 1 DONE (2026-10-04).**

**Done when:** the arm moves from the RViz MotionPlanning panel and from a script.

## Phase 2 - Messages + ROS-free libraries
- [x] graspsort_msgs: ObjectPose.msg, ObjectPoseArray.msg, SortObjects.action, AttachLink.srv (D-03)
- [x] projection.hpp (back-projection, percentile depth, depth-band points, table-plane footprint via
      minAreaRect, StabilityGate, TrackAssociator) + 37 GoogleTests
- [x] grasp_planner.hpp (top-down candidates, width/neighbour filtering, ranking) + GoogleTests
- [x] bin_assignment.hpp + GoogleTests (27 GoogleTests across grasp_planner and bin_assignment)

**Status: Phase 2 DONE (2026-10-04).** All unit tests are green (workspace: 163 tests).

**Done when:** all unit tests are green.

## Phase 3 - Perception
- [x] scripts/setup_ort.sh and export_yolo.py copied from SemNav (pinned ORT 1.20.1)
- [x] object_detector_node copied from SemNav with its tests; class_filter [bottle, sports ball]
      (D-04, D-13), image topic /camera/color/image_raw. Sim: 10 fps, inference p50 34 ms, balls and
      bottles 2/frame in 100/100 frames
- [x] D-11 fixed: online Gazebo model database disabled (stale cached models had replaced ours)
- [x] D-13: cups removed from the world (undetectable); 4 objects (2 balls, 2 bottles); bin_cup empty
- [x] D-12 ACCEPTED: wide depth band (0.10 m) + table-plane footprint centre, z = table + h/2
- [x] localizer.hpp (ROS-free: boxFromCenterSize, localizeBox, applyShape, ShapeTable) + GoogleTests on
      rendered oblique depth images (renderer shared in test/oblique_render.hpp)
- [x] object_localizer_node per 7.2/D-12: exact-stamp depth cache, TF at the image stamp (fails
      loudly), nearest-neighbour tracks, stability gate; /objects_3d at 10 Hz; all tunables in
      perception.yaml; table_height from world_layout.yaml via perception.launch.py
- [x] eval_localization.py vs /gazebo/model_states (2 identical runs): 4/4 localized; balls
      4.7 / 4.9 mm, bottles 11.8 / 13.1 mm; median 8.4 mm (target < 15 mm): PASS
- [x] D-14: bottle mesh rotated -24.4 deg in its OBJ frame; asset fixed (fetch_models.sh rev 2).
      Re-measured: bottle yaw error 3.3 / 4.8 deg (median 4.0 < 10), median 3D 7.5 mm. Height-band
      parameter added, default off (evidence in D-14). grasp_planner max_grasp_width 0.085 m + tests
- [x] CycloneDDS transport with sim + detector + localizer running: detector 10 fps, 0 dropped
      (re-check with MoveIt in Phase 4)
- [ ] (Optional stretch, D-04) make_dataset.py + fine-tune to add can/box; not required for Phase 3
- [ ] (Stretch, D-13, time-boxed) try Google Scanned Objects mug models as a detectable cup

**Status: Phase 3 DONE (2026-10-04)** for its "done when"; D-14 is open for Phase 4.

**Done when:** 3D pose error has been measured against ground truth.

## Phase 4 - Scene manager + grasp attach, single pick
- [x] D-14 resolved (bottle asset aligned; yaw error median 4.0 deg); grasp_planner max_grasp_width
      0.085 m + GoogleTests
- [x] Attach plugin nearest-model mode (D-15): empty child_model = nearest object within 0.02 m;
      live check passed
- [x] scene_manager_node per 7.3: table/pedestal/bins fixed, objects from /objects_3d with timeouts,
      attached objects untouched, /scene_manager/freeze (std_srvs/SetBool); 21 GoogleTests + pytests
- [x] pick_place_test (MoveGroupInterface): freeze, open, OMPL pre-grasp, Pilz LIN approach, close
      (width - squeeze), Gazebo attach + attachObject, LIN lift, OMPL above bin, LIN lower, open,
      detach (both), remove, LIN retreat, ready, unfreeze; safe recovery on failure (tested); no
      *_mimic joints sent to MoveIt (D-10); pick_place_geometry.hpp + 17 GoogleTests
- [x] pick_place_trials.py (eval harness, Gazebo reset + /gazebo/model_states check)
- [x] Verified by the main agent, headless, full stack: sports ball -> bin_ball 5/5 (11.5-12.3 s),
      bottle -> bin_bottle 5/5 (13.0-19.8 s), planning 0.05-0.10 s per trial; workspace tests 241/241
- [x] USER: D-14 (height band off by default) and D-16 ACCEPTED (2026-10-04)

**Status: Phase 4 DONE (2026-10-04)** for its "done when" (5 in a row for each class).

**Done when:** one object lands in a bin, 5 times in a row.

## Phase 5 - sort_task_node
- [x] SortObjects action server per 7.5: ordering, pre-grasp, Pilz LIN approach/retreat, grasp, attach,
      transport, place, retries, feedback, per-stage timings (sort_task_node + PickPlaceExecutor)
- [x] sort_logic.hpp (ROS-free: ordering, attempt bookkeeping, release slots) + GoogleTests; workspace
      tests 272/272, zero warnings
- [x] sort_trials.py harness; verified by the main agent, headless, reference world (2026-10-05):
      3/3 runs PASS, one goal each, 4/4 objects in the correct bin, 0 failures, wall 59.0-70.8 s,
      cycle 12.0-21.6 s, planning 0.07-0.11 s
- [x] USER: D-18 ACCEPTED, including the action_msgs dependency (2026-10-05)

**Status: Phase 5 DONE (2026-10-05).**

**Done when:** the full table is sorted from one action call.

## Phase 6 - Evaluation
- [x] eval_runner (scripts/eval_run.py, run_eval.sh): randomised layouts with a fixed seed and unique
      spawn names (D-06, D-17); headless; eval_summary.py writes results.md (17 pytests)
- [x] Full D-17 baseline, 40 trials (2026-10-06): 88/120 correct (docs/results_d17_baseline.md)
- [x] D-19 analysed (scripts/eval_analysis.py): 18 % of bottle failures yaw-linked; gripper closing
      inside the bottle (20/55 closes); 15/28 bottle failures never detected
- [x] D-20 known-footprint fit + 7 GoogleTests; re-run 40 trials: 91/120 correct, bottle 3D error 2.2 mm,
      close timeouts 2; docs/results.md with real numbers
- [ ] USER: review D-19, D-20, D-21 (PROPOSED overnight)
- [ ] Architecture 8 target >= 90 % low-clutter pick success NOT met (76.7 %): bottle detection (D-04 stretch)

**Status: Phase 6 DONE (2026-10-06)** for its "done when"; targets and D-19/D-20/D-21 await review.

**Done when:** the results table has real numbers.

## Phase 7 - Demo and docs
- [x] demo.rviz (graspsort_bringup/rviz) + scripts/demo_sort.sh; verified headless on the reference world:
      one goal, 4/4 in the correct bins (ground truth), 59.4 s
- [x] README (pitch, mermaid diagram, quick start, results, limitations, licenses)
- [x] docs/design-notes.md (draft for the author to rewrite, architecture 9)
- [x] .github/workflows/ci.yml (valid YAML; not yet run on GitHub)
- [ ] Demo with GUI (gazebo + RViz demo.rviz) not run overnight: headless only; resize the YOLO panel and
      save demo.rviz before recording
- [ ] USER: recorded video, LICENSE file, push, CI run, v1.0 tag

**Done when:** a stranger can run the demo in 15 minutes.
