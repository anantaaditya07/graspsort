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

# Header-only, ROS-free libraries (grasp_planner.hpp, bin_assignment.hpp, pick_place_geometry.hpp,
# sort_logic.hpp) and the pick_place_executor.hpp header.
install(DIRECTORY include/ DESTINATION include)

set(ROS_DEPS rclcpp rclcpp_action moveit_ros_planning_interface moveit_msgs control_msgs
  geometry_msgs tf2_ros std_srvs graspsort_msgs)

# One pick-and-place of a perceived object (architecture 7.5), shared by pick_place_test and
# sort_task_node.
add_library(pick_place_executor SHARED src/pick_place_executor.cpp)
target_include_directories(pick_place_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_include_directories(pick_place_executor SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(pick_place_executor ${ROS_DEPS})
install(TARGETS pick_place_executor
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)

# Phase 4 scripted single pick-and-place.
add_executable(pick_place_test src/pick_place_test.cpp)
target_link_libraries(pick_place_test pick_place_executor)
ament_target_dependencies(pick_place_test ${ROS_DEPS})

# Phase 5 SortObjects action server (architecture 7.5).
add_executable(sort_task_node src/sort_task_node.cpp)
target_link_libraries(sort_task_node pick_place_executor)
ament_target_dependencies(sort_task_node ${ROS_DEPS})

install(TARGETS pick_place_test sort_task_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/pick_place_trials.py scripts/sort_trials.py
  DESTINATION lib/${PROJECT_NAME})

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

  ament_add_gtest(test_sort_logic test/test_sort_logic.cpp)
  target_include_directories(test_sort_logic PRIVATE include)
  target_include_directories(test_sort_logic SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIR})
endif()

ament_export_include_directories(include)
ament_export_dependencies(eigen3_cmake_module Eigen3)
ament_package()
