cmake_minimum_required(VERSION 3.8)
project(graspsort_bringup NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/move_named.py
  scripts/gripper_cmd.py
  scripts/gen_disabled_collisions.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Consistency checks between controllers.yaml, the SRDF, the robot URDF and the launch files.
  # GRASPSORT_ROBOT_XACRO overrides the robot xacro under test (e.g. a stand-in); empty means
  # graspsort_gazebo/urdf/graspsort_robot.urdf.xacro if installed, else the Phase 0 harness copy.
  set(GRASPSORT_ROBOT_XACRO "" CACHE STRING "Robot xacro checked by test_config_consistency")
  ament_add_pytest_test(test_config_consistency test/test_config_consistency.py
    ENV
      GRASPSORT_BRINGUP_SRC=${CMAKE_CURRENT_SOURCE_DIR}
      GRASPSORT_ROBOT_XACRO=${GRASPSORT_ROBOT_XACRO}
    TIMEOUT 120)
endif()

ament_package()
