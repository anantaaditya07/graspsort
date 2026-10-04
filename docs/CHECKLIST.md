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
- [ ] Workspace layout: repo root is the colcon workspace; packages per architecture section 5 build
      with zero warnings; `colcon test` runs
- [ ] scripts/fetch_models.sh (D-08): pinned object models, YCB inertials and materials patched
- [ ] graspsort_gazebo: world with table, 3 bins, 4-6 objects, `libgazebo_ros_state.so`, and a fixed
      RGB-D camera (pose per D-05, topics remapped to /camera/color/*, /camera/depth/*)
- [ ] Arm + gripper URDF per D-02 (gripper_tcp frame); controllers yaml; launch with
      RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
- [ ] graspsort_bringup: MoveIt config (OMPL with RRTConnect default + Pilz, D-06); named poses
- [ ] Static frames: world -> table, bin_<class>
- [ ] Verify: controllers active, /joint_states rate, camera topic rates + encodings

**Done when:** the arm moves from the RViz MotionPlanning panel and from a script.

## Phase 2 - Messages + ROS-free libraries
- [ ] graspsort_msgs: ObjectPose.msg, ObjectPoseArray.msg, SortObjects.action, AttachLink.srv (D-03)
- [ ] projection.hpp (back-projection, percentile depth, minAreaRect yaw, stability gating) + GoogleTests
- [ ] grasp_planner.hpp (top-down candidates, width/neighbour filtering, ranking) + GoogleTests
- [ ] bin_assignment.hpp + GoogleTests

**Done when:** all unit tests are green.

## Phase 3 - Perception
- [ ] scripts/setup_ort.sh and export_yolo.py copied from SemNav (pinned ORT 1.20.1)
- [ ] object_detector_node copied from SemNav with its tests; class_filter [cup, bottle, sports ball]
      (D-04), image topic renamed
- [ ] object_localizer_node per 7.2 (frames from header, TF at the image stamp, fail loudly)
- [ ] (Optional stretch, D-04) make_dataset.py + fine-tune to add can/box; not required for Phase 3
- [ ] Verify CycloneDDS image transport under full load (D-07)

**Done when:** 3D pose error has been measured against ground truth.

## Phase 4 - Scene manager + grasp attach, single pick
- [ ] scene_manager_node per 7.3 (table and bins fixed, objects from perception only, freeze service)
- [ ] Attach plugin per D-03, tested with the real finger link and MoveIt attachObject
- [ ] Scripted single pick and place

**Done when:** one object lands in a bin, 5 times in a row.

## Phase 5 - sort_task_node
- [ ] SortObjects action server per 7.5: ordering, pre-grasp, Pilz LIN approach/retreat, grasp, attach,
      transport, place, retries, feedback, per-stage timings

**Done when:** the full table is sorted from one action call.

## Phase 6 - Evaluation
- [ ] eval_runner: randomised layouts with a fixed seed and unique spawn names (D-06); headless
- [ ] Metrics per section 8; clutter sweep at 3/5/7 objects; results.md

**Done when:** the results table has real numbers.

## Phase 7 - Demo and docs
- [ ] Demo RViz config, demo_sort.sh, README, design-notes.md, CI, recorded video, v1.0 tag (by user)

**Done when:** a stranger can run the demo in 15 minutes.
