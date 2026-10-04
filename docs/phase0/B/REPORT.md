# Phase 0 / B: Grasp-attach mechanism (feeds D-03)

Time box 2 h. Actual: about 45 min (11:45 to 12:30 IST, 2026-10-04). Headless gzserver only. Isolation: `ROS_DOMAIN_ID=42`, `GAZEBO_MASTER_URI=http://localhost:11352`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`. Only process groups I started were killed.

## Summary

| Candidate | Build (-Wall -Wextra -Wpedantic) | 5/5 drift max | Detach | Re-attach | Offset kept | Rapid attach/detach | Negative control |
|---|---|---|---|---|---|---|---|
| Custom plugin `libgraspsort_attach.so` (151 lines) | 0 warnings | 0.095 mm / 0.079° (5/5 at 1.5 s legs, 2/2 at 0.75 s legs) | z 0.573 → 0.461, no NaN | yes | 0.000 mm / 0.000° snap with a (15, -20, 27) mm, 22.9° yaw offset | 4200 cycles, 0 failures (1000 while moving) | box 0.000 m vs wrist 0.744 m |
| IFRA_LinkAttacher (b056289) | 0 warnings | 0.156 mm / 0.219° | yes | yes | 0.013 mm / 0.021° | gzserver hung after 231 and 318 cycles (2/2 sessions) | same harness |

Thresholds: < 5 mm and < 2°. Decision rule: use the custom plugin if it passes (the architecture prefers it), with IFRA as the fallback. **Result: custom plugin.**

## Setup
- **World** (`harness/attach_test.world.in`):
  - ODE quick solver, 50 iterations, SOR 1.3, step 0.001, update rate 1000 Hz, erp 0.2, cfm 0.
  - Ground plane and sun.
  - Static 0.3 m table with its top at z = 0.4358.
  - `gazebo_ros_state` publishing `/gazebo/link_states` at 100 Hz.
- **Robot:** `ur_description` 2.13, `ur.urdf.xacro ur_type:=ur5e name:=ur sim_gazebo:=true simulation_controllers:=controllers.yaml`. Controllers run at 500 Hz: `joint_state_broadcaster` and `joint_trajectory_controller` with position command.
- **Calibration:** at Q_GRASP = [0, -1.57, 1.57, -1.57, -1.57, 0], the flange (`wrist_3_link`) is at (0.492, 0.133, 0.488) with its z axis pointing down.
- **Each run:**
  1. Spawn a 5 cm, 0.1 kg box on the table 2 mm under the flange and let it settle for 0.5 s.
  2. Attach `ur/wrist_3_link` to `box/link`.
  3. Send one trajectory: lift (`shoulder_lift` -0.25 rad, 1 s), then +1 rad shoulder_pan, -1 rad elbow and +1 rad wrist_3 out and back (1.5 s or 0.75 s per leg).
  4. Detach and record for 1 s.
- **Measurement:** relative pose taken from the same `link_states` message for wrist and box, about 102 samples per sim second.
- **Speeds:** peak wrist speed 0.67 m/s (1.33 m/s on the 0.75 s legs); box up to 1.37 m/s.
- **Note:** `/clock` is published at 10 Hz, so the `sim_t` column in the CSVs is quantized to 0.1 s.

## Commands
```
source harness/env.sh
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
harness/start_sim.sh custom|ifra <log>    # prints PGID
python3 harness/attach_test.py run --candidate custom|ifra|none --runs 5 --tag X [--fast 0.75] [--offset --double_attach] [--reattach]
python3 harness/errors_test.py
python3 harness/stress_test.py 1000 custom|ifra
```

## Real output (trimmed)
```
CXX_FLAGS = -O3 -DNDEBUG -fPIC -Wall -Wextra -Wpedantic ... -std=gnu++17   (no warning lines; clang-format --dry-run -Werror clean)
[custom_main run1..5] max_drift_mm 0.094/0.094/0.093/0.090/0.093  max_drift_deg 0.073/0.073/0.071/0.064/0.071
[custom_offset run1] attach again (expect fail): False already attached: ur::wrist_3_link__box::link
[custom_offset2 run1] rel_at_attach [15.0,-20.0,27.0] mm yaw -112.9 deg, snap 0.0 mm, max_drift 0.122 mm / 0.155 deg
[ifra_main run1..5] max_drift_mm 0.149/0.131/0.156/0.118/0.041
box z after detach (20 ms steps): 0.5733 0.5712 0.5652 0.5552 0.5414 0.5236 0.5019 0.4763 0.4620 0.4624
[negative run1] wrist_travel 0.744 m, box_travel 0.0 m, max_box_speed 0.0
errors: not attached / unknown model 'nope' / unknown link 'no_link' in model 'ur' / unknown model 'ghost' / same link -> all success=False
custom stress: 200, 1000 (arm moving), 3x1000 cycles -> all calls successful; post-stress drift 0.092 mm
ifra stress: HANG cycle 231 detach (arm moving); HANG cycle 318 detach (fresh sim) -> gzserver thread at 99.9% CPU, no /clock
ifra log: [Err] ODEJoint::SetDamping: index[1] is out of bounds (DOF() = 1)   (once per attach, 320x)
```
Custom sessions logged only two warnings: the controller period notice and the JTC deprecation notice.

## Versions
Gazebo 11.10.2, gazebo_ros 3.9.0, gazebo_ros2_control 0.4.10, controller_manager 2.54.2, joint_trajectory_controller 2.54.0, ur_description 2.13.0, rmw_cyclonedds_cpp 1.3.5, gcc 11.4.0.

IFRA: `b056289ba93ccb549db98926dcbb9679642d0c8d`, last commit 2024-01-15, Apache-2.0. Services `/ATTACHLINK` and `/DETACHLINK` (`linkattacher_msgs`, fields model1/link1/model2/link2). Not packaged for Humble.

## Problems and fixes
1. **Deadlock in plugin v1 (fixed).**
   - v1 locked the physics mutex in the service callback.
   - Gazebo's `RemoveJoint()` calls `World::SetPaused()`, which takes the world-update mutex. The physics thread takes world-update first and then physics, so the locks were taken in opposite order.
   - The 2nd detach hung Gazebo (`logs/gzserver_custom_v1_deadlock_tail.log`).
   - Fix: a RAII guard that pauses the world while the joint is changed, then restores the previous pause state. v2 passed 4200 cycles.
2. **IFRA hang (not fixed).** It changes joints from the ROS thread with no synchronisation. The root cause is unconfirmed because ptrace is restricted. Other issues in the source:
   - a global `IsAttached` flag, so only one attachment in the world
   - `GV_joints` is never erased
   - a zero-limit revolute joint instead of fixed
   - `SetDamping` index 1 on a 1-DOF joint
   - upper-case service names
3. **Harness:** `/clock` is best-effort, so the subscriber needs sensor-data QoS.
4. **Harness:** delete is asynchronous, so the harness now waits for the box to disappear before respawning. Once in about 25 runs, a spawned box did not appear within 10 s; the harness respawns once and logs `HARNESS:`. This happens before any attach call and was seen with both candidates.

## Recommendation for D-03
Use the custom Gazebo Classic world plugin in `graspsort_gazebo`:
- On attach: `CreateJoint(name, "fixed", parent, child)` then `Init()`. On detach: `Detach()` then `RemoveJoint()`. Both run with the world paused.
- Attachments are tracked in a map, so several can exist at once.
- The current offset is preserved (no snap), and errors come back as clear messages.
- Joint type and service names are SDF parameters.

IFRA is the fallback; it would need the locking and stale-entry fixes first.

Service interface options:
- **A (recommended):** `graspsort_msgs/srv/AttachLink.srv` for both `/attach` and `/detach`. Explicit, works for any perceived object, already prototyped. This adds a new srv, so it needs approval.
- **B:** `std_srvs/Trigger`. The plugin would have to pick the nearest object within a radius (a new tunable), which is risky in clutter.
- **C:** `std_srvs/SetBool`. Same object-selection problem as B.

Not tested: the real gripper finger link (D-02 pending), MoveIt `attachObject`, objects deleted while attached.
