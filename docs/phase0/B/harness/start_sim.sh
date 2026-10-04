#!/bin/bash
# Usage: start_sim.sh <custom|ifra|none> <logfile>
# Starts the harness in its own process group; prints PGID. Stop with: kill -- -<PGID>
set -e
H=$(cd "$(dirname "$0")" && pwd)
source $H/env.sh
# Table geometry (from calibration run: flange pose at Q_GRASP, see REPORT.md)
TABLE_X=${TABLE_X:-0.49}; TABLE_Y=${TABLE_Y:-0.13}; TABLE_TOP=${TABLE_TOP:-0.4358}
TABLE_CZ=$(python3 -c "print($TABLE_TOP/2)"); TABLE_H=$TABLE_TOP
case "$1" in
  custom) P='<plugin name="graspsort_attach" filename="libgraspsort_attach.so"><joint_type>fixed</joint_type></plugin>';;
  ifra)   P='<plugin name="gazebo_link_attacher" filename="libgazebo_link_attacher.so"/>';;
  none)   P='';;
esac
W=$B/attach_test_$1.world
sed -e "s|@ATTACH_PLUGIN@|$P|" -e "s|@TABLE_X@|$TABLE_X|" -e "s|@TABLE_Y@|$TABLE_Y|" \
    -e "s|@TABLE_CZ@|$TABLE_CZ|" -e "s|@TABLE_H@|$TABLE_H|g" $H/attach_test.world.in > $W
setsid timeout 3600 ros2 launch $H/attach_test.launch.py world:=$W controllers:=$H/controllers.yaml > "$2" 2>&1 < /dev/null &
sleep 1; ps -o pgid= -p $! | tr -d ' '
