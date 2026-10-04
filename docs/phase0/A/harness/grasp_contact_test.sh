#!/bin/bash
# Check 3b: close the gripper on a 4 cm cube (on a static pedestal) and watch cube pose + gripper joint health.
# usage: grasp_contact_test.sh <close_pos> <open_pos> <action> <driven_joint> [mimic=mult ...]
H=$(dirname "$(realpath "$0")")
CLOSE=${1:-0.7929}; OPEN=${2:-0.0}; ACT=${3:-/gripper_controller/gripper_cmd}; DRV=${4:-robotiq_85_left_knuckle_joint}; shift 4; MIM="$@"
X=${CUBE_X:-0.5537}; Y=${CUBE_Y:-0.1333}; Z=${CUBE_Z:-0.0705}; PZ=${PED_Z:-0}
echo "== move arm to tool-down pose"; timeout 60 python3 -u $H/fjt_test_down.py /joint_trajectory_controller/follow_joint_trajectory /dev/null | grep -E "result|max"
echo "== open"; timeout 15 python3 $H/gripper_cmd.py $OPEN 20 $ACT || echo "open: action did not return within 15 s"
python3 $H/js_health.py 1 $DRV $MIM
echo "== spawn pedestal + cube"
ros2 run gazebo_ros spawn_entity.py -entity pedestal -file $H/pedestal_cube.sdf -x $X -y $Y -z $PZ 2>&1 | grep -o "Successfully.*"
ros2 run gazebo_ros spawn_entity.py -entity cube -file $H/cube.sdf -x $X -y $Y -z $Z 2>&1 | grep -o "Successfully.*"
sleep 2; echo "  cube pose before close: $(gz model -m cube -p)"
echo "== close to $CLOSE"; timeout 15 python3 $H/gripper_cmd.py $CLOSE 20 $ACT || echo "close: action did not return within 15 s"
for i in 1 2 3; do python3 $H/js_health.py 1 $DRV $MIM; echo "  cube pose: $(gz model -m cube -p)"; done
echo "== open again"; timeout 15 python3 $H/gripper_cmd.py $OPEN 20 $ACT || echo "open: action did not return within 15 s"
python3 $H/js_health.py 1 $DRV $MIM; echo "  cube pose: $(gz model -m cube -p)"
echo "== cleanup"; ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: cube}" >/dev/null; ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: pedestal}" > /dev/null; echo done
