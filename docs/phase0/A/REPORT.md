# Phase 0 / Subagent A: arm, gripper and controller simulation check

Date: 2026-10-04, 11:45 to 12:25 IST (about 40 min wall time; the box was 2 h).
Machine: Ubuntu 22.04, 20 cores, 15 GB RAM, Intel iGPU. All runs were headless (gzserver / `ign gazebo -s`).
Isolation for every shell: ROS_DOMAIN_ID=41, GAZEBO_MASTER_URI=http://localhost:11351,
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp, and IGN_PARTITION=graspsort_A for Fortress (see harness/env.sh).
Every process I started ran under `setsid timeout ...` and was stopped by killing its process group. At the end
no process of mine was left running.

## Summary
| # | Check | Result | Key number |
|---|---|---|---|
| 1 | UR5e on Gazebo Classic 11 (ur_simulation_gazebo, humble) | PASS | JSB and JTC active; /joint_states 99.8 Hz (configured 100); 2-pose FJT SUCCESSFUL, final max abs joint error 0.00003 rad; RTF 1.00 |
| 2 | MoveIt 2 (ur_moveit_config, use_sim_time) plan and execute to a tool0 pose | PASS | MoveItErrorCodes 1 (SUCCESS) in 8 of 8; RRTConnect planning 0.035-0.060 s (one outlier at 5.0 s); final error 0.55-0.99 mm and 0.35-0.81 deg |
| 3a | Robotiq 2F-85 + GripperActionController on Classic, free space | PASS | 4 of 4 goals SUCCEEDED; driven error 0.0001 rad; 5 mimic joints within 0.0001 rad |
| 3b | Robotiq 2F-85 on Classic, closing on an object | FAIL | 40 mm cube is ejected or tilted on every close. Fingertips touching static geometry make the chain blow up (60-700 rad/s, mimic deviation up to 3 rad, later actions hang) |
| 3c | Custom 2-finger prismatic gripper (mimic) on Classic | PASS | Free space: error 0.0000 m, mimic deviation 0.0000 m. Close to width plus 0.5 mm squeeze: cube drift under 1 mm in 3 s, mimic deviation 0.0004 m |
| 4a | UR5e on Fortress (ur_simulation_gz), smoke test | PASS | /joint_states 495 Hz (update rate 500); FJT error 0.00000 rad; RTF 1 |
| 4b | Robotiq on Fortress (ign_ros2_control mimic), smoke test | PASS with caveat | 8 of 8 goals SUCCEEDED, about 1.5 s per stroke with dynamics; mimic joints not on /joint_states; running the `ign model` CLI once froze the gripper |

## Versions and SHAs
- Universal_Robots_ROS2_Gazebo_Simulation: branch humble, commit 34a041738bd8f736f6f85e8a1c73dd91396d7ee1 (2026-06-16)
- Gazebo Classic 11.10.2
- gazebo_ros 3.9.0, gazebo_ros2_control 0.4.10
- controller_manager 2.54.2, joint_trajectory_controller and gripper_controllers 2.54.0
- ur_description 2.13.0, ur_moveit_config and ur_controllers 2.14.0
- robotiq_description 0.0.1
- MoveIt 2.5.10
- ur_simulation_gz 0.5.0, ign_ros2_control 0.7.21, ros_gz_sim 0.244.26

Scratch build: `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`. Built in 0.98 s with no warnings.

## Check 1: UR5e on Gazebo Classic
Commands:
    ros2 launch ur_simulation_gazebo ur_sim_control.launch.py ur_type:=ur5e gazebo_gui:=false launch_rviz:=false
    ros2 control list_controllers
    ros2 topic hz /joint_states
    gz stats
    python3 harness/fjt_test.py
Output:
    joint_state_broadcaster active / joint_trajectory_controller active
    average rate: 99.828 (std dev 0.00024 s)
    Factor[1.00]
    result error_code=0 "Goal successfully reached!", wall 8.05 s for an 8 s trajectory
    max |error| = 0.00003 rad -> PASS
The arm passed via-pose A at t=4.06 s with error 0.0000 rad (csv/c1_fjt_joint_states.csv, 859 samples).
gzserver used 47 % of one core.
Warnings (harmless): "Desired controller update period (0.01 s) is slower than the gazebo simulation period
(0.001 s)" and the JTC deprecation "allow_nonzero_velocity_at_trajectory_end".

## Check 2: MoveIt
Commands:
    ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e use_sim_time:=true launch_rviz:=false launch_servo:=false
    python3 harness/moveit_pose_test.py <a|b> [planner_id]
- With use_sim_time:=true, ur_moveit_config switches the default controller to joint_trajectory_controller, so
  the names match the sim.
- The script builds a reachable target with /compute_fk, then sends a MoveGroup action (group ur_manipulator,
  link tool0, 1 mm and 0.01 rad tolerance). It sends the goal once as plan-only to read planning_time, because
  move_group reports 0 when it also executes. It then plans and executes, and measures the final pose by FK on
  the measured joint states.
- Results:
  - 8 of 8 executions returned SUCCESS (1), ending within 0.55-0.99 mm and 0.35-0.81 deg.
  - RRTConnect plan-only times: 0.060, 0.035, 0.042, 5.007, 0.052, 0.040 s.
  - One plan-only call returned -2 INVALID_MOTION_PLAN ("Invalid states at index 31 32"). The next call succeeded.
- Notes:
  - ompl_planning.yaml has no default planner. Set planner_id RRTConnectkConfigDefault explicitly.
  - ur_moveit_config loads only the OMPL pipeline. Pilz is installed but not configured, so the graspsort MoveIt
    config must add it.
  - Expected log messages: "No 3D sensor plugin(s) defined for octomap updates" (ERROR level, expected because
    there is no octomap) and "Falling back to using the move_group node namespace".

## Check 3: gripper on Classic
### 3a. Robotiq in free space
Harness: ur5e_robotiq.urdf.xacro, ur5e_robotiq_controllers.yaml, ur5e_robotiq_sim.launch.py.
- The xacro builds ur_robot with sim_gazebo, then ur_to_robotiq on tool0, then robotiq_gripper with
  include_ros2_control=false.
- A second ros2_control block uses gazebo_ros2_control/GazeboSystem. It commands robotiq_85_left_knuckle_joint
  and declares the 5 linkage joints with the mimic and multiplier params. gazebo_ros2_control 0.4.10 accepted
  2 ros2_control blocks.
- gripper_controller has type position_controllers/GripperActionController and serves
  /gripper_controller/gripper_cmd.

Results:
- 4 of 4 goals SUCCEEDED for targets 0.7 and 0.0. Driven error 0.0001 rad, mimic deviation 0.0000-0.0001 rad,
  read from Gazebo through GazeboSystem.
- The arm with the gripper mounted still passes FJT, with final error 0.00004 rad.

Findings:
- Mimic joints are published as `<joint>_mimic`. TF is still fine, because robot_state_publisher uses the URDF
  mimic tags.
- Each position command is applied in one 10 ms step (0.0 to 0.7 rad in dt 0.010 s). The velocity limit and
  max_effort have no effect.
- Geometry, measured via TF in robotiq_85_base_link:
  - open: 106.4 mm between pad centres, z 114.8 mm
  - closed: 22.4 mm between pad centres, z 128.3 mm
  - tool0 to robotiq_85_base_link is 0.011 m
  - TCP candidate: gripper_tcp = robotiq_85_base_link + 0.130 m, which is tool0 + 0.141 m
- Startup error "Parameter 'hold_joints' has already been declared" comes from the second GazeboSystem instance
  and is harmless.

### 3b. Robotiq closing on an object
Setup (grasp_contact_test.sh): tool-down pose, a static pedestal, a 40 mm 0.1 kg cube, close, then 3 s of cube
pose and joint-health logging, then open.

| Close target | Setup | Cube | Gripper health | Open afterwards |
|---|---|---|---|---|
| 0.7929, 0.50, 0.46, 0.42, 0.30 | fingertips into the static pedestal | thrown 0.2-0.8 m | 60-710 rad/s, mimic dev 0.25-3.1 rad | never returned |
| 0.30 | pedestal lowered, no contact | did not move | OK | OK |
| 0.46 | pedestal lowered, contact | tilted 0.385 rad, then thrown 0.43 m | transient 20 rad/s, dev 0.099, recovered | OK |
| 0.7929 | pedestal lowered | thrown 0.25 m | OK | OK |

Conclusion: the linkage is driven joint by joint, the closed loop is not modelled, and commands arrive as a
step. The gripper is not stable in contact, and contact with a static table or bin hangs the sim. FAIL.

### 3c. Custom prismatic gripper
Design:
- gripper_base_link: a 40 mm box on tool0.
- 2 fingers, 20x10x100 mm boxes, on prismatic joints along y.
- q = 0 is open (90 mm opening). q = 0.04 is closed.
- The right finger mimics the left finger.
- gripper_tcp = base + 0.12 m. goal_tolerance is 0.002.

Results:
- Free space: 4 of 4 goals SUCCEEDED, error 0.0000 m, mimic deviation 0.0000 m.
- Contact (cube contact at q = 0.025):
  - close to 0.0245: the cube did not move.
  - close to 0.0255: the fingers stop at 0.0250-0.0252, the cube drifts less than 1 mm, mimic deviation 0.0004.
  - close to 0.027: the cube creeps 16 mm in 3 s.
  - close to 0.04: the cube is thrown 0.68 m, but the gripper stays healthy.
  - Every open goal returned.

PASS.

## Check 4: Fortress (optional)
- ur_simulation_gz with gazebo_gui:=false: /joint_states 495 Hz, /clock 989 Hz, RTF 1, FJT error 0.0 rad,
  about 81 % of one core.
- Robotiq with sim_ignition:
  - 8 of 8 goals SUCCEEDED, about 1.5 s per stroke with real dynamics.
  - Mimic joints are not published.
  - Gotcha: one `ign model -m ur -l <link>` call froze the gripper. Every close goal then returned stalled at
    0.0. Reproduced 2 of 2; a restart fixes it.
- I did not run a contact test on Fortress.

## Frames, controllers and topics
- UR frames: world (robot at the origin, plus a ground_plane link), base_link, base, shoulder_link ...
  wrist_3_link, flange, tool0, ft_frame.
- Robotiq frames: ur_to_robotiq_link, gripper_mount_link, robotiq_85_base_link, gripper_tcp (candidate).
- Custom gripper frames: gripper_base_link, gripper_{left,right}_finger_link, gripper_tcp.
- Controller manager: /controller_manager at 100 Hz (Classic) or 500 Hz (Fortress package default).
- Actions: /joint_trajectory_controller/follow_joint_trajectory and /gripper_controller/gripper_cmd.
- Topics: /joint_states, /dynamic_joint_states, /tf, /tf_static, /robot_description, /clock.
- Gazebo services: /spawn_entity and /delete_entity. /gazebo/model_states is not published by the default world;
  the GraspSort world needs libgazebo_ros_state.so.
- MoveIt: /move_action, /compute_fk, group ur_manipulator.

## Problems hit and how they were solved
1. "Couldn't parse parameter override rule: --param robot_description:=": gazebo_ros2_control 0.4 sends the URDF
   through the rcl YAML parser, and a comment with ": " broke it. Rule: no colon+space anywhere in a URDF.
2. setsid forks inside a background shell, so `$!` was not the process group. I took the PGID from ps and
   stopped each run with `kill -INT -<pgid>`.
3. The `_mimic` joint-name suffix made the first test wait forever. The script now maps the names.
4. My first Robotiq contact tests were confounded by fingertips reaching the static pedestal. I reran them with
   the pedestal lowered and kept both results, because fingertip-into-table is a real case for GraspSort.
5. MoveGroup planning_time is 0 when the goal also executes, so I added a plan-only request.

## Time spent
About 40 min of the 2 h box: check 1 about 5 min, check 2 about 7 min, check 3 about 20 min, Fortress about 8 min.

## Recommendations
- D-01: Gazebo Classic 11. Copy the ur_simulation_gazebo launch and config (reference SHA 34a0417) into
  graspsort_gazebo. Fallback: Fortress.
- D-02: UR5e + joint_trajectory_controller + custom 2-finger prismatic gripper with
  position_controllers/GripperActionController on /gripper_controller/gripper_cmd.
  - Close to object width minus about 0.5 mm per side, never fully closed, then attach.
  - Make the squeeze and open width ROS parameters.
  - Option B: Robotiq for visuals with the same rule. Risky, because fingertip contact with static geometry
    breaks the sim.
  - Option C: Robotiq on Fortress, which would change D-01.
  - Option D: a trajectory controller for the gripper, which changes the section 6 interface.
  - Panda fallback: not needed.

## For approval
- D-01 and D-02 as above.
- Copy (vendor) the UR sim launch and config, or clone the repo pinned at SHA 34a0417.
- Add the Pilz pipeline to the graspsort MoveIt config.
- The no-colon-space-in-URDF rule.
