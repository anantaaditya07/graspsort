# GraspSort design notes

> **DRAFT for the author to rewrite.** Architecture section 9 says: "write design-notes.md
> yourself". This draft collects the facts from docs/DECISIONS.md and the Phase 0 reports, so the
> rewrite has the numbers in one place. Every number below comes from those files.

Each entry covers the problem, the options, the choice and why, the evidence, and a one-liner for an interview.

---

## D-02 Gripper: custom 2-finger prismatic instead of Robotiq 2F-85

- **Problem:** the PDF suggests a Robotiq 2F-85 on a GripperActionController. On Gazebo Classic it passed in
  free space (4/4 goals, mimic joints within 0.0001 rad) but failed in contact. A 40 mm cube was thrown
  0.2-0.8 m or tilted on every close. Fingertips touching the table or bin blew up the linkage (60-700 rad/s,
  mimic deviation up to 3 rad), and later actions never returned. Cause: the closed 4-bar linkage is
  modelled as independent mimic joints, and position commands are applied as one 10 ms step.
- **Options:** A, a custom prismatic 2-finger gripper on the same `/gripper_controller/gripper_cmd`
  interface. B, Robotiq visuals with careful closing. C, Robotiq on Gazebo Fortress. D, a trajectory-driven
  slow close (changes the PDF interface).
- **Chosen:** A. The interface stays as in the PDF. Close to object width minus a 0.5 mm squeeze, never
  fully. Max opening 90 mm.
- **Evidence:** custom gripper free-space error 0.0000 m; the cube stays within 1 mm for 3 s after closing;
  every action returned. A large overshoot still throws the object, so the close target must be width-based.
- **Interview:** "The Robotiq model's closed linkage isn't stable in ODE contact, so I kept the action
  interface and swapped the hardware model for one I could make stable. The close command is width-based,
  because position overshoot is what throws objects."

## D-03 Grasp attach: custom ~150-line Gazebo WorldPlugin + AttachLink.srv

- **Problem:** friction grasps in Gazebo slip. The PDF asks for a fixed joint on /attach, removed on
  /detach: "existing port or ~150-line custom plugin". The service type was left open.
- **Options:** custom WorldPlugin vs IFRA_LinkAttacher (Apache-2.0). Service: a new `AttachLink.srv` vs
  `std_srvs/Trigger` or `SetBool` (which would have to guess the object).
- **Chosen:** custom plugin + `graspsort_msgs/srv/AttachLink.srv`.
- **Evidence (5/5 runs, sweep of 0.744 m at up to 1.33 m/s):**

| | Custom plugin (151 lines) | IFRA_LinkAttacher |
|---|---|---|
| Max drift | 0.095 mm / 0.079 deg | 0.156 mm / 0.219 deg |
| Attach/detach stress | 4200 cycles, 0 failures | gzserver hung after 231 and 318 cycles |
| Other | several attachments at once | one per world, revolute joint |

  Negative control: without attach, the wrist moved 0.744 m and the box 0.000 m. The first plugin version
  deadlocked on the 2nd detach (physics vs world-update mutex order). Fixed by pausing the world while
  joints change.
- **Interview:** "I measured both options under the same sweep and a stress test. The existing port hung
  gzserver after a few hundred cycles. Mine ran 4200 cycles, after I fixed a lock-order deadlock I found
  on the way."

## D-05 Camera: fixed oblique instead of fixed overhead

- **Problem:** the PDF says "fixed overhead camera". From straight down (0.65 and 0.95 m), YOLOv8n detected
  only balls. Bottle and cup were at 0.00.
- **Options:** A, a fixed oblique camera (~50 deg pitch, opposite the arm, still from the URDF, no
  hand-eye loop). B, keep top-down and depend on a fine-tune.
- **Chosen:** A. Pose in `world_layout.yaml`: x 0.9721, z 1.3245, pitch 0.8727 rad, yaw pi, 0.75 m from
  the object-area centre.
- **Evidence:** bottle detection 0.80 oblique vs 0.00 top-down (Phase 0 C). Depth error <= 3.1 mm oblique.
- **Interview:** "COCO's bottle class is learned from side views, so I moved the camera to match the
  training distribution instead of fighting it. That cost me a harder localizer (D-12)."

## D-11 Gazebo silently used unpatched online model copies

- **Problem:** in Phase 3 both cups were thrown off the table at spawn.
- **Root cause:** `~/.gazebo/models/` held unpatched online copies (cup inertia about 3x too large),
  downloaded while a run had no `models_external` on its path. gzserver's online model database took
  priority. A renamed copy of the patched cup stayed in place, which proved it.
- **Fix:** `sim.launch.py` sets `GAZEBO_MODEL_DATABASE_URI=''`, so a missing model fails loudly. A bringup
  test checks this.
- **Interview:** "The physics bug wasn't in my model, it was in a cache I didn't know was on the path. I
  proved it with a renamed copy, then made missing models fail loudly instead of downloading silently."

## D-12 Localizer: wide depth band + table-plane footprint centre

- **Problem:** PDF 7.2 uses a +/-2 cm depth band and pushes the box centre in by half the object depth.
  With the 50 deg camera, that band sees only the front face.
- **Evidence (noise-free rendered depth from the real camera pose):**

| | PDF method | Band 0.10 m, footprint centre, z = table + h/2 |
|---|---|---|
| Bottle footprint | 0.067 x 0.049, yaw wrong by 90 deg | within 1.5 mm, 0.5 deg |
| Bottle centre | 36-45 mm off | within 2 mm |
| Ball centre | under 1 mm | 5.4 mm |

  A 90 deg yaw error would make the planner close across the 0.097 m side, wider than the 0.09 m opening.
- **Chosen:** B. `depth_band` 0.10 m; x, y from the table-plane footprint centre; z = table height + h/2.
  All values are parameters.
- **Interview:** "The written algorithm assumed a top-down view. I quantified the failure on rendered depth
  before changing anything, then made the change a parameter."

## D-13 Cups dropped: two classes, ball and bottle

- **Problem:** in the real world, cups were detected in 0/100 frames (also when re-coloured white or red).
  Bottles and balls were 100/100. The Phase 0 cup rate (0.70) was measured on a different table and
  background.
- **Options:** drop cups; make the fine-tune required; try other cup/mug models.
- **Chosen:** drop cups **and remove them from the world**. An undetected object is invisible to MoveIt
  (the scene comes from perception only), so the arm could hit it. A Google Scanned Objects mug test is a
  time-boxed stretch goal.
- **Interview:** "Because the planning scene is perception-only, an undetected object is a collision
  hazard, not just a missed pick. So removing it was a safety decision."

## D-14 Bottle footprint and yaw: the ground truth was wrong too

- **Problem:** bottle yaw estimates were -64.8 and -11.9 deg for bottles standing at yaw 0.
- **Finding:** the YCB mustard-bottle scan is rotated -24.4 deg about z inside its own OBJ frame
  (minAreaRect of all 8194 vertices). The collision box was built axis-aligned around the rotated body
  (0.097 x 0.067). So the collision box didn't match what the camera saw, and "true yaw = 0" was off by
  24.4 deg. The accepted fix, fitting only a lower height band, did not help (fractions <= 0.7 gave
  47-66 deg errors). It is implemented but defaults to off.
- **Chosen:** rotate the visual back in `fetch_models.sh`; collision box = the body's own footprint
  0.0958 x 0.0582 x 0.1913 m; plus a `max_grasp_width` (0.085 m) safety check in the grasp planner.
- **Evidence:** re-measured yaw error 3.3 / 4.8 deg (median 4.0 < 10), median 3D error 7.5 mm (< 15).
  Limit: only measured at yaw 0. See D-19.
- **Interview:** "Before tuning the estimator I checked the ground truth, and it was wrong. The scanned
  mesh was rotated inside its own file."

## D-15 Phase 4 interfaces: attach target, MoveIt attach, scene freeze

- **Gap:** the robot knows only perceived ids; the Gazebo model name is ground truth. 7.3 and 7.5 overlap
  on who attaches the object in MoveIt.
- **Chosen:**
  - `/attach` with an empty `child_model` = the nearest non-static model within 0.02 m of the finger link.
    This is sim-side physics emulation, and the robot never reads ground truth.
  - The pick code calls `attachObject` / `detachObject` on the scene manager's `object_<track id>`.
  - `/scene_manager/freeze` (std_srvs/SetBool) stops perception updates during a pick. A moved object gets
    a new track id, so freezing is required. No new message types.
- **Interview:** "Ground truth stays in the simulator. The attach plugin only emulates physics ('whatever
  is touching the finger'), so the robot code would carry over to a real gripper."

## D-16 Pick-and-place implementation choices

- **Held object vs table:** the perceived box of a held object reached slightly into the table, so MoveIt
  saw the lift start in collision. An allowed-collision entry, held object vs table, is added while
  attached and removed on release and in recovery.
- **IK-seeded free moves:** OMPL moves go to a joint goal from IK seeded with the current pose. A free move
  is accepted only if the following straight-line (Pilz LIN) move also plans from its end pose. This fixed
  a sweep through a bin.
- **Parameter file key `/**`:** a node-name key silently overrode `-p` on the command line.
- **Values:** pre-grasp 0.10 m (PDF), squeeze 0.0005 m, lift 0.15 m, velocity scaling 0.5 free / 0.1 LIN,
  up to 3 grasp candidates.
- **Evidence:** ball 5/5 (11.5-12.3 s), bottle 5/5 (13.0-19.8 s), planning 0.05-0.10 s per trial.
- **Interview:** "Planning a free move that ends where the next straight-line move can't start is a
  classic trap. I made each free move conditional on the next LIN move being plannable."

## D-17 Evaluation scenario

- **PDF 8:** 3, 5 and 7 objects; minimum spacing 2 vs 8 cm; randomised spawns with a fixed seed.
- **Chosen (user):** 2 and 4 objects (D-13 leaves 2 classes x 2 objects); minimum footprint gap 8 vs 3 cm;
  10 trials per configuration (40 trials); random positions and random bottle yaw.
- **Definitions:** gap = centre distance minus both circumscribed footprint radii. 2 objects = 1 ball +
  1 bottle; 4 = 2 + 2. Seed = base seed + trial index. Spawn names are unique (D-06: reusing a name after a
  delete lost about 16 % of spawns). Ground truth is used only by the eval scripts.
- **Interview:** "I defined the scenario before building the evaluator, so it can show the effect of
  clutter, and I logged every deviation from the plan."

## D-18 sort_task_node implementation choices

- Ordering, pre-grasp, LIN approach/retreat, attach, transport, 2 retries and feedback follow 7.5.
- Beyond the PDF:
  - **Release slots:** a second object of a class is released beside the first (each bin holds 2).
  - **Re-detect after each place:** wait until `/objects_3d` is stable, then re-order.
  - **Reach check:** the "IK check" uses move_group's existing `/compute_ik` (collision-aware).
  - **Dependency:** `action_msgs` (accepted).
- **Evidence:** reference world, 3/3 runs, one goal each, 4/4 objects in the correct bin, 0 failures,
  59.0-70.8 s per run, planning 0.07-0.11 s.
- **Interview:** "Each pick changes the scene, so the node re-perceives between objects instead of
  executing a plan made from a stale snapshot."

## D-19 Bottle yaw error with random yaw (OPEN)

- **Finding (smoke run, n=5):** median bottle yaw error 17.6 deg (p95 51.3) vs the < 10 deg target. Phase 3
  had 4.0 deg, but only at yaw 0. The summary script folds the error into [0, 90] correctly, so the error
  comes from perception.
- **Likely cause:** the camera sees only the front half of a bottle, so the minAreaRect footprint is short
  along the view and its long axis tilts toward the view direction.
- **Options:** A, report as is; B, time-boxed fit of the known bottle footprint to the visible points;
  C, score only the closing-axis error.
- **Outcome:** `<!-- D-19 OUTCOME: full 40-trial results and decision -->`
- **Interview:** `<!-- TODO after the decision -->`
