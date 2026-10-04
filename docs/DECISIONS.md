# GraspSort decisions log

Each entry: finding (with evidence from this machine), options, recommendation, status.
Status values: NEEDS DECISION (blocks work), INFO (verified, no conflict), ACCEPTED (user approved).
Audit date: 2026-10-04. Installed: ROS 2 Humble, Gazebo Classic 11.10.2, gazebo_ros 3.9.0,
gazebo_ros2_control 0.4.10, ros2_control / ros2_controllers 2.54, MoveIt 2.5.10 (+ pilz), ur 2.14
(ur_description 2.13), robotiq_description 0.0.1, moveit_resources_panda_*, ur_simulation_gz 0.5.0,
ign_ros2_control 0.7.21, rmw_cyclonedds_cpp 1.3.5, vision_msgs 4.1.1. net.core.rmem_max = 2147483647.

Phase 0 evidence (raw output, CSVs, reproducible harnesses): `docs/phase0/{A,B,C}/REPORT.md`.
Three investigations ran in parallel with isolated ROS_DOMAIN_ID / GAZEBO_MASTER_URI. Each check was
re-run once by the main agent as a spot check (see the end of this file).

---

## D-01 Simulator: Gazebo Classic 11 vs Fortress  - NEEDS DECISION

**PDF (3, 9):** pick whichever has a working UR5e + gripper + ros2_control simulation; do not mix.

**Finding (docs/phase0/A):**
- Classic 11: UR5e from Universal_Robots_ROS2_Gazebo_Simulation (humble, SHA 34a0417, source only,
  not in apt). joint_state_broadcaster and joint_trajectory_controller active, /joint_states
  99.8 Hz, FollowJointTrajectory final error 0.00003 rad, RTF 1.00, gzserver ~47 % of one core.
  MoveIt plan + execute SUCCESS 8/8 (final error <= 1 mm / 0.81 deg).
- Fortress (ur_simulation_gz 0.5.0, apt): arm also passes (error 0.0, RTF 1, ~81 % of one core).
  The Robotiq moves with real dynamics, but mimic joints are not published, and one `ign model` CLI call
  froze the gripper (2/2). No contact test was run.
- The attach plugin (D-03) and `/gazebo/model_states` (PDF section 6, evaluation) are Classic APIs.
  The SemNav tooling is Classic as well.

**Options**
- A. Gazebo Classic 11. Vendor the UR sim launch file + controller yaml (~200 lines, reference
  SHA 34a0417) into `graspsort_gazebo`, so there is no source dependency on an unreleased repo.
- B. Gazebo Classic 11, cloning the UR sim repo pinned at SHA 34a0417 via a fetch script.
- C. Fortress. Needs a new attach mechanism and the ros_gz ground truth instead of /gazebo/model_states.

**Recommendation:** A.

## D-02 Arm, gripper and controllers  - NEEDS DECISION

**PDF (3, 6):** UR5e; parallel-jaw gripper (e.g. Robotiq 2F-85) on a gripper action controller,
`/gripper_controller/gripper_cmd` (control_msgs/action/GripperCommand); a simple custom 2-finger
gripper URDF is an acceptable fallback. Panda fallback.

**Finding (docs/phase0/A, check 3):**
- Robotiq 2F-85 on Classic, free space: PASS (4/4 goals, all 5 mimic joints within 0.0001 rad).
- Robotiq 2F-85 on Classic, closing on an object: **FAIL**.
  - A 40 mm cube is thrown 0.2-0.8 m or tilted on every close.
  - Fingertips touching static geometry (table, bin) blow up the linkage (60-700 rad/s, mimic
    deviation up to 3 rad), and later actions never return.
  - Cause: the closed 4-bar linkage is modelled as independent mimic joints, and position commands
    are applied as one 10 ms step.
- Custom 2-finger prismatic gripper (second finger mimics the first) on the same UR5e: PASS.
  - Free-space error 0.0000 m, mimic deviation 0.0000 m.
  - Closing to the object width + 0.5 mm squeeze leaves the cube in place (< 1 mm drift in 3 s).
  - Every action returned.
  - A large overshoot still throws the object, so the close target must be width-based.
- Panda fallback: not needed.

**Options**
- A. UR5e + joint_trajectory_controller + custom 2-finger prismatic gripper with
  `position_controllers/GripperActionController` on `/gripper_controller/gripper_cmd` (interface
  as in the PDF).
  - The close target is the object width minus a small squeeze; never close fully. The grasp is
    held by the attach plugin (D-03).
  - Max opening ~90 mm, set in the URDF. This limits the objects in D-04.
  - Squeeze, open width and finger geometry are parameters / xacro args.
- B. Robotiq 2F-85 visuals with the same width-based close rule, keeping fingertips clear of the
  table and bins. Risky: the 3b blow-ups hang the action server.
- C. Robotiq on Fortress (changes D-01).
- D. Drive the gripper with a trajectory controller for a slow close. Changes the PDF section 6
  interface.

**Recommendation:** A.

## D-03 Grasp attach mechanism and its service interface  - NEEDS DECISION

**PDF (3, 6, 7.6):** fixed joint gripper <-> object on /attach, removed on /detach; "existing port
or ~150-line custom plugin"; service type "std_srvs or custom srv" (left open).

**Finding (docs/phase0/B):** pass limits were < 5 mm and < 2 deg drift over a fast sweep (0.744 m,
peak 1.33 m/s), 5/5 runs.

| | Custom WorldPlugin (151 lines, C++17) | IFRA_LinkAttacher (humble b056289, 2024-01-15, Apache-2.0) |
|---|---|---|
| Build, -Wall -Wextra -Wpedantic | 0 warnings, clang-format clean | 0 warnings |
| Max drift, 5/5 runs | 0.095 mm / 0.079 deg | 0.156 mm / 0.219 deg |
| Detach, re-attach, offset kept | all OK, 0.000 mm snap | all OK, 0.013 mm snap |
| Rapid attach/detach stress | 4200 cycles, 0 failures | gzserver hung after 231 and 318 cycles (2/2) |
| Other | several attachments at once; clear errors | one attachment per world; SetDamping error per attach; revolute joint, not fixed; /ATTACHLINK names |

- Negative control: without attach, the wrist moved 0.744 m and the box moved 0.000 m.
- After detach the box falls freely (z 0.573 -> 0.461 m in 1 s), with no NaN values.
- The first plugin version deadlocked Gazebo on the 2nd detach (physics vs world-update mutex order).
  It was fixed by pausing the world while joints change; that fixed version passed the stress test above.
- Not tested yet: the real gripper finger link (wrist_3_link stood in), MoveIt attachObject, and an
  object deleted while attached. These are covered in Phase 4.

**Options (mechanism)**
- A. Custom plugin in `graspsort_gazebo`.
- B. IFRA_LinkAttacher. It would need locking and stale-entry fixes.

**Options (service type)**
- S1. New `graspsort_msgs/srv/AttachLink.srv` (parent_model, parent_link, child_model, child_link
  -> bool success, string message), used for both /attach and /detach. Explicit, already
  prototyped. **This adds a new srv type, so it needs approval (CLAUDE.md rule 5).**
- S2. `std_srvs/Trigger`. The plugin would have to guess the object, e.g. the nearest one within a
  radius (a new tunable). Risky in clutter.
- S3. `std_srvs/SetBool`. Same object-selection problem as S2.

**Recommendation:** A + S1.

## D-04 Object set, YOLO classes and fine-tuning  - NEEDS DECISION

**PDF (2, 3, 9):** choose objects by measured detection rate, not by name. The candidates are cup,
bottle, can, box and ball. Fallback: auto-labelled synthetic dataset (make_dataset.py) + short YOLOv8n
fine-tune.

**Finding (docs/phase0/C):**
- Setup:
  - YOLOv8n COCO ONNX on CPU.
  - Pass rule: conf >= 0.35, IoU >= 0.5 against a depth-derived ground-truth box, pass at >= 80 %.
  - 12 models, 5 positions x 2 yaws, so 10 frames per model and viewpoint (small sample).
  - Inference p50 34-47 ms.
- **COCO has no "can" and no "box" class** (a conflict in the PDF's candidate list).
- Detection rates of the best model per class:

| class | model | oblique 1.0 m | oblique 0.7 m | top-down | COCO class that fires |
|---|---|---|---|---|---|
| ball | cricket_ball (osrf) | 1.00 | 1.00 | 0.80-1.00 | sports ball |
| bottle | mustard_bottle (YCB, Fuel) | 0.80 | 0.80 | 0.00 | bottle |
| cup | plastic_cup (osrf) | 0.30 | 0.70 | 0.00 | cup |
| cup | pitcher_base (YCB) | 0.50 | 0.60 | 0.00 | cup |
| can | master_chef_can (YCB) | 0.50 | 0.90 as *bottle* | 0.10-0.40 | bottle / clock |
| can | coke_can (osrf) | 0.10 | 0.40 as *cup* | 0.00-0.20 | cup / none |
| box | cracker_box (YCB) | 0.20 as *book* | 0.20 | 0.00 | book / none |

- Only 2 classes pass (ball, bottle), and only from an oblique camera. **The Phase 0 check fails
  (< 3 classes)**, so the PDF fallback applies.
- In clutter (5 objects), the can and box are never detected.
- Misses repeat exactly frame to frame, so temporal averaging will not recover them.
- Unusable models:
  - master_chef_can: ~103 mm wide, more than the 90 mm gripper opening.
  - cardboard_box: 0.5 m wide.
  - robocup_spl_ball: detected as "orange".
- Model sources and licences: osrf/gazebo_models @8163eb4 (CC BY 3.0) and Gazebo Fuel YCB (CC BY 4.0).
  All are pinned in docs/phase0/C/REPORT.md section 5.
- The Fuel YCB SDFs have nonsense inertials (0.6-2.5 g) and black materials (no diffuse colour).
  Both need fixing when they are vendored.

**Options**
- A. Start with three classes, **ball (cricket_ball), bottle (mustard_bottle), cup (plastic_cup)**,
  with the oblique camera (D-05).
  - Accept cup at 0.70 for now.
  - Run make_dataset.py + fine-tune as a planned step in Phase 3 to raise cup and add can and box
    (graspable models: coke_can, a small box).
  - If the fine-tune is cut (it is in the PDF cut order), the demo runs with 3 classes.
- B. Two classes only (ball, bottle) until a fine-tune exists. Every class passes the bar, but this is
  a weak sorting demo with 2 bins.
- C. Fine-tune first in Phase 3 for all five classes, before any perception integration. Slowest
  path, best final numbers.
- D. One more time-boxed (45 min) sweep: 35-40 deg pitch, extra cup models from Google Scanned
  Objects (CC BY 4.0, a new download source that needs approval).

**Recommendation:** A. Class names stay COCO names in the detector (`cup`, `bottle`,
`sports ball`); bins are mapped per class in bin_assignment.

## D-05 Camera viewpoint: oblique instead of "fixed overhead"  - NEEDS DECISION (deviation from PDF)

**PDF (1, 4):** "fixed overhead camera (no hand-eye loop)".

**Finding (docs/phase0/C):** from straight down (0.65 m and 0.95 m) YOLOv8n detects **only balls**.
Bottle and cup are 0.00 from top-down and pass or come close only at ~50 deg pitch. The depth
pipeline is equally accurate from both views (errors <= 3.1 mm oblique, 0.0 mm top-down).

**Options**
- A. Fixed oblique camera, ~50 deg pitch, 0.7-1.0 m from the table centre, mounted opposite the arm.
  It is still fixed and from the URDF, so there is no hand-eye loop. The localizer already uses TF, so
  the PDF algorithms are unchanged; footprint/yaw from the depth mask sees side pixels as well, which
  Phase 3 handles by projecting to the table plane (already in 7.2).
- B. Keep the top-down camera and rely on the fine-tune (D-04 C). Blocks every class except ball
  until the fine-tune works.

**Recommendation:** A. The exact pose becomes a parameter in Phase 1, picked so the arm does not
occlude the table during pre-grasp.

## D-06 Reuse of the UR MoveIt config, planners and simulation gotchas  - INFO

Recorded from docs/phase0/A, B and C; no conflict with the PDF.
- **MoveIt config:**
  - ur_moveit_config loads only OMPL and has no default planner. The GraspSort MoveIt config must
    set `RRTConnectkConfigDefault` and add the Pilz pipeline (PDF 3: Pilz LIN for approach/retreat).
  - One plan in 6 took 5.0 s and one returned INVALID_MOTION_PLAN. The PDF's retry rule (7.5)
    covers this, and planning time is a Phase 6 metric.
- **URDF/xacro:** gazebo_ros2_control 0.4 sends the URDF through a YAML parser, so a comment
  containing ": " stops the controller manager from starting. Rule: no colon+space in URDF/xacro
  comments.
- **Joint and topic names:**
  - Mimic joints appear on /joint_states with a `_mimic` suffix (TF is unaffected).
  - The camera plugin publishes depth on `/camera/color/depth/...`. Three `<remapping>` rules give
    the PDF names `/camera/depth/image_raw` and `/camera/depth/camera_info`.
  - Colour and depth share `camera_color_optical_frame` (optical convention verified). Depth is
    32FC1 in metres along the camera axis.
- **Gazebo world and spawning:**
  - `/gazebo/model_states` needs `libgazebo_ros_state.so` in the world (PDF section 6, eval only).
  - Spawn names must be unique. Reusing a name right after a delete lost about 16 % of spawns in
    C, which matters for the Phase 6 eval_runner.
  - The `/clock` publish rate defaults to 10 Hz, and subscribers need SensorData QoS.

## D-07 Middleware: CycloneDDS (replaces SemNav's Fast DDS profile)  - INFO

The PDF says CycloneDDS "from SemNav D-19/D-20". SemNav actually used a Fast DDS SHM profile plus
sysctl. For GraspSort, `rmw_cyclonedds_cpp` 1.3.5 is installed, and the raised UDP buffers
(rmem_max 2147483647) are already active.

Measured in C: 640x480 RGB-D at 10 Hz with a SensorData subscriber lost 0/100 frames on one camera
and 1/100 on the other. No extra config is needed now; this gets re-checked under full load in
Phase 3.

## D-08 Pinning external assets (fetch script, not git)  - NEEDS DECISION

All Phase 0 downloads stayed in the session scratchpad. Phase 1 needs reproducible sources for:
- the UR sim launch/config (D-01, vendored copy or pinned clone)
- the object models: osrf/gazebo_models @8163eb4 and Fuel YCB with sha256 (D-04)
- yolov8n.onnx: re-exported with SemNav's export_yolo.py, ONNX sha256 838d3332...;
  yolov8n.pt is AGPL-3.0

**Options**
- A. Same approach as SemNav D-14: `scripts/fetch_models.sh` downloads pinned versions into a
  gitignored `src/graspsort_gazebo/models_external/` and patches the YCB inertials and materials.
  Licences are noted in the README.
- B. Vendor the small osrf models into git (CC BY 3.0 with attribution). YCB stays fetched.

**Recommendation:** A.

---

## Spot checks by the main agent (2026-10-04)
- A: relaunched `ur_sim_control.launch.py ur_type:=ur5e`. `ros2 control list_controllers` showed
  joint_state_broadcaster and joint_trajectory_controller active; `/joint_states` 99.896 Hz.
- B: one fresh custom-plugin run gave max drift 0.089 mm / 0.063 deg, box travel 0.749 m vs wrist
  0.744 m, and after detach z went 0.5733 -> 0.4608 with no NaN.
- C: re-scored all 479 saved frames. results_summary.csv (68 rows, columns 1-8) is identical. Inference
  mean is 34.6 ms with the machine idle (47.8 ms during the parallel runs).
