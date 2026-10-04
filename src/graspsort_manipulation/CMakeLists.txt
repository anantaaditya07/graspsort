cmake_minimum_required(VERSION 3.8)
project(graspsort_manipulation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(graspsort_msgs REQUIRED)

# Header-only, ROS-free libraries (grasp_planner.hpp, bin_assignment.hpp, pick_place_geometry.hpp).
install(DIRECTORY include/ DESTINATION include)

# Phase 4 scripted single pick-and-place (architecture 7.5).
add_executable(pick_place_test src/pick_place_test.cpp)
target_include_directories(pick_place_test PRIVATE include)
target_include_directories(pick_place_test SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(pick_place_test rclcpp rclcpp_action moveit_ros_planning_interface
  moveit_msgs control_msgs geometry_msgs tf2_ros std_srvs graspsort_msgs)
install(TARGETS pick_place_test DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/pick_place_trials.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_grasp_planner test/test_grasp_planner.cpp)
  target_include_directories(test_grasp_planner PRIVATE include)
  target_include_directories(test_grasp_planner SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIR})

  ament_add_gtest(test_bin_assignment test/test_bin_assignment.cpp)
  target_include_directories(test_bin_assignment PRIVATE include)

  ament_add_gtest(test_pick_place_geometry test/test_pick_place_geometry.cpp)
  target_include_directories(test_pick_place_geometry PRIVATE include)
  target_include_directories(test_pick_place_geometry SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIR})
endif()

ament_export_include_directories(include)
ament_export_dependencies(eigen3_cmake_module Eigen3)
ament_package()
