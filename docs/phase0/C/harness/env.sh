# Source in every shell used for the Phase 0 C harness (isolation from agents A/B).
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
export ROS_DOMAIN_ID=43
export GAZEBO_MASTER_URI=http://localhost:11353
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export C_SCRATCH=/tmp/claude-1000/-home-adityachavali-graspsort/b1f9bd71-1b1a-4b44-834a-7b7601ea7da7/scratchpad/C
export GAZEBO_MODEL_PATH=$C_SCRATCH/gzmodels:${GAZEBO_MODEL_PATH}
