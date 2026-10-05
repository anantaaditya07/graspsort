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

## D-01 Simulator: Gazebo Classic 11 vs Fortress  - ACCEPTED

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

**Final (accepted 2026-10-04):** A. Gazebo Classic 11; UR sim launch + controller yaml vendored into graspsort_gazebo (reference SHA 34a0417).

**Update (user, Phase 1, 2026-10-04):** the adapted launch and controllers yaml live in
`graspsort_bringup` (launch/sim.launch.py, config/controllers.yaml; header credits SHA 34a0417), not in
graspsort_gazebo. Nothing from the UR sim repo is fetched or built (user answer: "own launch only").

## D-02 Arm, gripper and controllers  - ACCEPTED

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

**Final (accepted 2026-10-04):** A. UR5e + joint_trajectory_controller + custom 2-finger prismatic gripper on `/gripper_controller/gripper_cmd`; width-based close (never fully closed); squeeze/open width are parameters.

## D-03 Grasp attach mechanism and its service interface  - ACCEPTED

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

**Final (accepted 2026-10-04):** A + S1. Custom attach plugin in graspsort_gazebo; new `graspsort_msgs/srv/AttachLink.srv` for /attach and /detach (approved addition per CLAUDE.md rule 5).

## D-04 Object set, YOLO classes and fine-tuning  - ACCEPTED

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

**Final (accepted 2026-10-04):** A, amended by the user: classes ball (cricket_ball), bottle (mustard_bottle), cup (plastic_cup, 0.70 accepted). The make_dataset.py + fine-tune to add can/box is an **optional stretch goal, not a Phase 3 requirement**.

**Amended by D-13 (2026-10-04):** cups dropped (0 detections in the real world), so the classes are ball and bottle.

## D-05 Camera viewpoint: oblique instead of "fixed overhead"  - ACCEPTED (deviation from PDF)

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

**Final (accepted 2026-10-04):** A. Fixed oblique camera (~50 deg pitch, 0.7-1.0 m from table centre), pose as parameters. Deviation from PDF "fixed overhead".

## D-06 Reuse of the UR MoveIt config, planners and simulation gotchas  - ACCEPTED

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

**Final (accepted 2026-10-04):** adopted as rules: RRTConnect default + Pilz pipeline in the MoveIt config; no colon+space in URDF/xacro comments; `_mimic` suffix handling; depth topic remaps; libgazebo_ros_state.so in the world; unique spawn names; SensorData QoS on /clock.

## D-07 Middleware: CycloneDDS (replaces SemNav's Fast DDS profile)  - ACCEPTED

The PDF says CycloneDDS "from SemNav D-19/D-20". SemNav actually used a Fast DDS SHM profile plus
sysctl. For GraspSort, `rmw_cyclonedds_cpp` 1.3.5 is installed, and the raised UDP buffers
(rmem_max 2147483647) are already active.

Measured in C: 640x480 RGB-D at 10 Hz with a SensorData subscriber lost 0/100 frames on one camera
and 1/100 on the other. No extra config is needed now; this gets re-checked under full load in
Phase 3.

**Final (accepted 2026-10-04):** CycloneDDS with the existing raised UDP buffers; no extra config; re-check under full load in Phase 3.

## D-08 Pinning external assets (fetch script, not git)  - ACCEPTED

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

**Final (accepted 2026-10-04):** A. scripts/fetch_models.sh downloads pinned assets into gitignored `src/graspsort_gazebo/models_external/` and patches YCB inertials/materials; licences in README.

## D-09 SRDF disabled-collision pairs generated by our own script  - ACCEPTED

**Finding (Phase 1, subagent C):** the MoveIt Setup Assistant `collisions_updater` tool produced no output in any run;
two runs hung for more than 3.5 h until they were killed. The SRDF needs disabled collision pairs.

**What was done:** `graspsort_bringup/scripts/gen_disabled_collisions.py` applies the Setup Assistant rules
(adjacent, default, never in collision) using move_group's `/check_state_validity` over 10000 random states.
Seeds 0 and 1 gave identical sets of 20 pairs. The procedure is documented in the SRDF header.

**Options**
- A. Keep the script (reproducible, in the repo).
- B. Time-box one more attempt at collisions_updater and replace the pairs if it works.

**Recommendation:** A.

**Final (accepted 2026-10-04):** A.

## D-10 Phase 1 implementation choices and deviations  - ACCEPTED

- **Bottle mass:** mustard_bottle is 0.300 kg instead of the real 0.603 kg. This keeps it within about 3x of the 0.1 kg
  attach validation (D-03). One value in fetch_models.sh.
- **Collision shapes:** all objects use primitive collision fitted to the mesh bounding box (sphere, cylinder, box)
  instead of mesh collision. The bottle can only be gripped across its 0.067 m side with the 90 mm gripper. Cup
  collision is within 5 mm of its visual surface.
- **Camera distance:** 0.75 m from the aim point instead of 0.9-1.0 m (D-05 allowed 0.7-1.0 m). Phase 0 C measured cup
  detection at 0.70 at 0.7 m versus 0.30 at 1.0 m. It is set in world_layout.yaml and as a xacro arg.
- **graspsort_msgs created in Phase 1:** it contains only AttachLink.srv, because the user's Phase 1 prompt needs the
  attach services. The other messages stay in Phase 2.
- **gazebo_ros export:** `plugin_path="${prefix}/../../lib"`, because gazebo_ros replaces `${prefix}` with the share
  directory.
- **`_mimic` joint names:** move_group aborts when it receives a /joint_states message containing
  `gripper_right_finger_joint_mimic`, for example via /compute_fk. Phase 4/5 code must drop `*_mimic` names before any
  MoveIt request (extends D-06).
- **`docs/COLCON_IGNORE`:** added so colcon does not build the Phase 0 harness packages under docs/phase0.
- **Evaluation-only object poses:** world_layout.yaml holds the object poses for evaluation only. They must never feed
  the planning scene (CLAUDE.md rule).

**Final (accepted 2026-10-04):** all of the above accepted by the user.

## D-11 Gazebo silently used unpatched online model copies  - FIXED (INFO)

**Symptom (Phase 3):** both cups were thrown off the table at spawn, before the arm moved.
Phase 1 had measured drift only after the objects settled, so it missed this.

**Root cause:** `~/.gazebo/models/` held **unpatched online copies** of `plastic_cup` and `cricket_ball`
(downloaded 2026-10-04 16:43, while an interrupted Phase 1 subagent run had no models_external on its path).
gzserver downloads from `models.gazebosim.org` by default, and that cache took priority over
`graspsort_gazebo/models_external`.
- With the cache present, the original osrf cup (inertia about 3x too large) was ejected.
- A renamed copy of our patched cup stayed exactly in place, which proved the cache was the cause.

**Fix:**
- `sim.launch.py` sets `GAZEBO_MODEL_DATABASE_URI=''`, so a missing model fails loudly instead of being
  fetched. A bringup test checks this.
- The two stale cache folders were moved (not deleted) to the session scratchpad. `ground_plane` and `sun`
  were left alone.
- After the fix, all 6 objects stay at their spawn poses (probe of the world alone).

## D-12 Localizer method under the oblique camera  - ACCEPTED (deviation from PDF 7.2)

**PDF 7.2:**
- Depth: 20th-percentile depth in the central 50 % of the box.
- Position: back-project the box centre, then push in by half the object depth.
- Footprint and yaw: pixels within +/- 2 cm of the object depth, projected to the table, minAreaRect.

**Finding (Phase 2, subagent A, noise-free depth rendered from the real camera pose):** with the 50 deg oblique
camera (D-05), the +/- 2 cm band sees only the front face of an object.

| | PDF method (2 cm band, push-in) | Band 0.10 m, footprint centre for x, y, z = table + height/2 |
|---|---|---|
| Bottle footprint | 0.067 x 0.049, **yaw wrong by 90 deg** | within 1.5 mm, 0.5 deg |
| Cup footprint | 0.064 x 0.045 (true 0.065 x 0.065) | within 1.5 mm |
| Bottle centre | 36-45 mm off | within 2 mm |
| Cup centre | 19 mm off | within 2 mm |
| Ball centre | under 1 mm | 5.4 mm |
| Height | exact | exact |

A 90 deg yaw error would make the grasp planner close across the bottle's 0.097 m side, which is wider than
the 0.09 m gripper opening.

**Options** (all three are parameter changes; projection.hpp supports each)
- A. PDF method as written. It misses the 1.5 cm target for bottle and cup, and bottle yaw is wrong.
- B. `depth_band` 0.10 m; position from the footprint centre (x, y), z = table height + height/2.
- C. As B, but keep push-in for spherical objects (ball), set by a per-class parameter.

**Recommendation:** B. It is a deviation from PDF 7.2 (centre and band), with every tunable kept as a
parameter.

**Final (accepted 2026-10-04, user):** B. Wide depth band (`depth_band` 0.10 m); x and y from the table-plane
footprint centre, z = table height + height/2. This deviates from PDF 7.2 (band width and centre method). All values
are ROS parameters.

## D-13 The plastic cup is not detected in the real world  - ACCEPTED

**Finding (Phase 3, sim + object_detector_node, 100 frames, default world after the D-11 fix):**
- Bottles: 2/frame, 100/100 frames, conf 0.64-0.74.
- Balls: 2/frame, 100/100 frames, conf 0.44-0.83.
- Cups: **0 frames.**
- The same cup mesh re-coloured opaque white or opaque red is also 0/100. The osrf cup is a plain tapered
  cylinder with no visible rim, so this is not a material problem.
- Phase 0 C's 0.70 for plastic_cup was measured on a different, plain table and background, so D-04's cup
  choice does not hold in this world.

**Options**
- A. Drop cups for now: sort ball and bottle (2 bins in use). Re-add cups if the fine-tune happens.
- B. Make the D-04 fine-tune (make_dataset.py, auto-labelled from ground truth) a **required** Phase 3 step, so
  cups (and optionally can/box) are detected.
- C. Try other cup or mug models. Google Scanned Objects (CC BY 4.0) is a new download source and needs
  approval. Phase 0 C's YCB pitcher_base (0.6 detection) is too wide for the 90 mm gripper.

**Recommendation:** A now, so localizer verification continues on 4 objects, and B as the next step if you want
3 classes. This reverses your earlier "fine-tune is optional" call, so it is your decision.

**Final (accepted 2026-10-04, user):**
- Drop cups now, and **also remove them from the world**: an undetected object is invisible to MoveIt (the
  scene comes from perception only), so the arm could hit it.
- Sort 2 classes, balls (`sports ball`) and bottles. `bin_cup` stays and is empty.
- Detector class_filter is `[bottle, sports ball]`.
- Stretch goal, time-boxed: try Google Scanned Objects mug models (option C; this approves GSO as a download
  source for that experiment only).
- `fetch_models.sh` still fetches plastic_cup (unused). It is harmless and kept for that experiment.

## D-14 Bottle footprint and yaw on the real mesh  - ACCEPTED

**Finding (Phase 3 verification, real sim, 2 identical runs of eval_localization.py):**

| object | 3D error | footprint (est.) | true footprint | yaw est. | yaw true |
|---|---|---|---|---|---|
| ball_1 / ball_2 | 4.7 / 4.9 mm | 0.074 / 0.072 diam. | 0.075 | n/a | n/a |
| bottle_1 | 11.8 mm | 0.086 x 0.056 | 0.097 x 0.067 (collision box) | -64.8 deg | 0 |
| bottle_2 | 13.1 mm | 0.070 x 0.058 | 0.097 x 0.067 | -11.9 deg | 0 |

- Position meets the architecture 8 target: median 8.4 mm, under 15 mm.
- **Bottle yaw misses the "boxes: median yaw error under 10 deg" target.**
- Likely cause: the visual mesh is a real mustard bottle (tapered, with a narrow neck and cap), not the box used in
  the synthetic tests. minAreaRect over the visible points of that irregular shape gives an undersized footprint
  and an unstable long axis. The depth camera sees the visual mesh; the gripper hits the collision box.

**Options (to decide before Phase 4 bottle grasps)**
- A. Fit the footprint only to points in a height slice (e.g. the lower 60 % of the object, below the neck),
  set by a parameter. This is the likely main fix.
- B. Treat bottles as cylinders (yaw ignored; grasp across the measured short side). This loses the box
  orientation.
- C. Leave it and rely on Phase 4 grasp retries.

**Recommendation:** A, measured with eval_localization.py (with yaw error added for boxes) before Phase 4.

**User decision (2026-10-04):** A (fit the footprint to a lower height band below the neck, parameterised), plus a
grasp_planner safety check `max_grasp_width` (default 0.085 m).

**What the measurements then showed (2026-10-04):**
1. **The ground truth was wrong, not only the estimate.** The YCB mustard-bottle scan is rotated **-24.4 deg about z
   inside its own OBJ frame**: cv::minAreaRect of all 8194 vertices gives a 0.0959 x 0.0582 m body at -24.446 deg.
   - fetch_models.sh had built the collision box as the axis-aligned box around that rotated body
     (0.097 x 0.067).
   - So the collision box did not match what the camera sees, and "true yaw = model yaw = 0" was wrong by 24.4 deg.
2. **The lower band does not help, so it is implemented but off.** An offline sweep on a real sim depth frame
   (band 0.05/0.10/0.15 m x height fraction 0.3-1.0, corrected bottle):
   - a fraction of 0.8 gives exactly the same result as 1.0;
   - any fraction of 0.7 or less breaks yaw (47-66 deg errors), because from the 50 deg camera the bottle's far edge
     is only visible on its top surfaces;
   - for a flat-topped box any cut below 1.0 removes the top face (a unit test shows the footprint collapsing).
   - So `footprint.max_height_fraction` exists as you asked, but defaults to **1.0 (off)**. **This departs from the
     accepted option A and needs your OK.**

**Final (implemented 2026-10-04):**
- fetch_models.sh (PATCH_REV 2): the bottle visual is rotated +24.446 deg about its body centre, so the body's long
  side is the model x axis. The collision box is now the body's own footprint, 0.0958 x 0.0582 x 0.1913.
  Model yaw is now the body yaw.
- projection.hpp / localizer: parameter `footprint.max_height_fraction`, default 1.0, plus a unit test.
- eval_localization.py: yaw error for box-shaped estimates (modulo 180 deg), with pass criterion median < 10 deg.
- grasp_planner: `max_grasp_width` (0.085 m). It rejects a candidate if the footprint extent along its closing axis
  (box |sx cos d| + |sy sin d|, cylinder diameter) exceeds the limit. It is checked first, with its own rejection
  reason. 4 GoogleTests.
- **Re-measured** (sim, 2 identical runs):

| object | 3D error | yaw error |
|---|---|---|
| ball_1 / ball_2 | 4.7 / 4.9 mm | n/a |
| bottle_1 / bottle_2 | 15.7 / 10.0 mm | 3.3 / 4.8 deg |

  Median 3D error 7.5 mm (< 15) and median box yaw error 4.0 deg (< 10): PASS.
- **Limits:**
  - Both bottles stand at yaw 0, so only one orientation was measured. Phase 6 randomised yaws will test more.
  - The estimated bottle footprint is undersized along the view (0.063-0.075 vs 0.096 m), because the back half is
    hidden. It is safe for grasping: the closing axis follows the short side, and max_grasp_width guards it.

**User (2026-10-04):** accepted, including `footprint.max_height_fraction` defaulting to 1.0 (off).

## D-15 Phase 4 interfaces: grasp attach target, MoveIt attach, scene freeze  - ACCEPTED (user, 2026-10-04)

**Gap:** the architecture does not say how the robot names the Gazebo model for /attach (it only knows perceived
ids; ground truth is evaluation-only). 7.3 and 7.5 also overlap on who attaches the object in MoveIt.

**Final (user):**
- **Attach target:** `/attach` and `/detach` (AttachLink.srv, unchanged) with an empty `child_model` mean "the nearest
  non-static model whose link bounding box is within `max_attach_distance` (SDF, 0.02 m) of the parent link". On
  detach, an empty child means everything on that link. This is sim-side physics emulation; the robot never reads
  ground truth. (commit b72eaf9)
- **MoveIt attach:** the pick code calls MoveGroupInterface attachObject/detachObject on the scene manager's object
  (`object_<track id>`), as 7.5 says. The scene manager has no attach services; it never removes or modifies an
  attached object.
- **Freeze:** `/scene_manager/freeze`, std_srvs/SetBool. No new message types.

**Scene manager implementation choices (INFO):**
- Only the table top is a collision object, not the legs: the leg inset is not in world_layout.yaml, and the arm cannot
  reach under the table.
- Extra parameter `update_yaw` (0.1 rad, boxes, modulo pi), besides `update_distance` (0.01 m).
- A moved object gets a new track id from perception, so the old `object_<n>` stays until `remove_timeout` (2 s).
  This is why the pick code must freeze the scene before picking.
- A placed object that perception still sees (e.g. in a bin) is re-added after unfreeze.

## D-16 Phase 4 pick-and-place implementation choices  - ACCEPTED

Found by subagent B while reaching 5/5 + 5/5 (docs/CHECKLIST.md Phase 4); all are parameters or local behaviour.
- **Held object vs table:** while an object is attached, MoveIt may allow it to touch the table (allowed collision
  matrix entry). The perceived box of a held object reached slightly into the table, so MoveIt saw the lift start
  in collision and nudged the start state by 0.078. The entry is removed when the object is removed from the scene,
  and during recovery.
- **IK-seeded free moves:** free moves (OMPL) go to a joint goal from IK seeded with the current arm pose. A free move
  is accepted only if the following straight-line move (approach or lower) also plans from its end pose. This fixed a
  bin_cup sweep in the first failed trial. The node reads graspsort_bringup's kinematics.yaml, which its launch file
  passes in.
- **Parameter file key:** pick_place.yaml uses the `/**` key, because a node-name key silently overrode
  `-p target_class:=...`.
- **Values:** pregrasp_height 0.10 (PDF); squeeze 0.0005 (D-02); release_clearance 0.02; transport_clearance 0.05;
  lift_height 0.15; retreat_height 0.10; velocity scaling 0.5 for free moves, 0.1 for straight-line moves; up to 3
  grasp candidates (first plus 2 retries, 7.5).
- **Limits of the test:** one fixed layout. The same ball_2 and bottle_2 were picked every time, with the bottle at
  yaw 0. Phase 6 randomised layouts are the real test.

**Recommendation:** accept as is.

**Final (accepted 2026-10-04, user):** accepted as is.

## D-17 Evaluation scenario (Phase 6)  - ACCEPTED (user, 2026-10-04; deviation from architecture 8)

**PDF 8:** clutter sweep with 3, 5 and 7 objects, minimum spacing 2 cm vs 8 cm; randomised layouts through the Gazebo
spawn service with a fixed seed per run.

**User (Phase 6 prompt):**
- 2 and 4 objects, minimum spacing 8 cm vs 3 cm, 10 trials per configuration (40 trials).
- Random positions **and** random bottle yaw.

The smaller counts follow from D-13: only 2 detectable classes, with 2 objects each in the reference world.

**Interpretation (main agent):**
- **Spacing:** the minimum **gap between object footprints**, i.e. centre distance minus both circumscribed
  footprint radii, which matches the PDF's intent of clutter around the fingers.
- **Object mix:** 2 objects = 1 ball + 1 bottle; 4 objects = 2 balls + 2 bottles.
- **Positions:** uniform within the reachable, camera-visible object area of world_layout.yaml, away from the bins.
- **Seeds:** fixed per trial (base seed + trial index).
- **Spawning:** objects are deleted and spawned with unique names (D-06).
- **Ground truth:** used only by the evaluation scripts.

## D-18 Phase 5 sort_task_node implementation choices  - PROPOSED (needs user review)

Found by the main agent while reviewing the Phase 5 code against architecture 7.5 (2026-10-05).
Ordering, pre-grasp, LIN approach/retreat, grasp/attach, transport, 2 retries and feedback follow 7.5.
Not in the PDF:
- **Release slots in a bin** (sort_logic.hpp chooseReleaseSlot, `release_slot.*` parameters): the
  second object of a class is released beside the first instead of on top of it. Each bin holds 2 objects.
- **Re-detect between objects:** after each place the node waits until /objects_3d is stable
  (`sort.settle_*`), then re-orders. Picked objects are matched across track ids by `sort.match_radius`.
- **Reach check:** the "IK check" in 7.5 uses move_group's existing `/compute_ik` service (collision-aware)
  on the best `reach_check_candidates` grasp. This adds no new interface.
- **New dependency:** `action_msgs` exec_depend (GoalStatus in sort_trials.py). It is part of core ROS 2,
  but CLAUDE.md rule 5 asks for approval.
- **Metrics:** per-attempt JSON lines go to `metrics_log_path` (default off), used by eval_summary.py. This
  is the 7.5 "log per-stage timings".

**Recommendation:** accept as is.

## D-19 Bottle yaw error with random yaw (Phase 6)  - OPEN (needs user decision)

**Finding:** the smoke run (data/eval/smokeA_1, 1 trial per D-17 config) gives a median bottle yaw error of
17.6 deg (n=5, p95 51.3; values 7-59 deg). The target in architecture 8 is < 10 deg. Phase 3 measured 4.0 deg,
but only with the bottles at yaw 0. eval_summary.py folds the error into [0, 90] correctly, so the error comes
from perception.
**Likely cause (not yet proven):** as D-14 notes, the camera sees only the front half of a bottle. The
minAreaRect footprint is short along the view direction, so at yaws away from 0 its long axis tilts toward the
camera's view direction. The picks still worked at up to 19 deg error, because the closing axis follows the
short side and max_grasp_width guards it. The worst case (59 deg, 4 objects / 3 cm gap) was in the trial with
the unreachable / no-candidate failures.

**Options:**
- **A.** Run the full 40-trial D-17 evaluation as is (~50 min) and report the yaw result honestly, with the
  cause. That gives n=60 bottles instead of 5, and a baseline for any fix.
- **B.** Time-boxed fix (2 h) inside the existing 7.2 design: fit the known bottle footprint from ShapeTable to
  the visible depth-band points instead of a free minAreaRect, then re-measure. If it doesn't help in time,
  fall back to A.
- **C.** Keep the algorithm and change the metric, e.g. only score the closing-axis error that matters for
  grasping. This deviates from architecture 8.

**Recommendation:** A first. 5 samples are too few to tell a perception problem from bad luck. Decide B from
the full data (B means a second 40-trial run).

---

## Spot checks by the main agent (2026-10-04)
- A: relaunched `ur_sim_control.launch.py ur_type:=ur5e`. `ros2 control list_controllers` showed
  joint_state_broadcaster and joint_trajectory_controller active; `/joint_states` 99.896 Hz.
- B: one fresh custom-plugin run gave max drift 0.089 mm / 0.063 deg, box travel 0.749 m vs wrist
  0.744 m, and after detach z went 0.5733 -> 0.4608 with no NaN.
- C: re-scored all 479 saved frames. results_summary.csv (68 rows, columns 1-8) is identical. Inference
  mean is 34.6 ms with the machine idle (47.8 ms during the parallel runs).
