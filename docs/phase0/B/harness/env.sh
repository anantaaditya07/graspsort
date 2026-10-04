# Source in every shell (isolation from other agents' Gazebo instances).
B=/tmp/claude-1000/-home-adityachavali-graspsort/b1f9bd71-1b1a-4b44-834a-7b7601ea7da7/scratchpad/B
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source $B/ws/install/setup.bash
export ROS_DOMAIN_ID=42
export GAZEBO_MASTER_URI=http://localhost:11352
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export GAZEBO_PLUGIN_PATH=$B/ws/install/b_attach_plugin/lib:$B/ws/install/ros2_linkattacher/lib:$GAZEBO_PLUGIN_PATH
