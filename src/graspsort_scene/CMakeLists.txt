cmake_minimum_required(VERSION 3.8)
project(graspsort_scene)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(graspsort_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Header-only, ROS-free scene logic (scene_logic.hpp).
install(DIRECTORY include/ DESTINATION include)

# scene_manager_node (architecture 7.3).
add_executable(scene_manager_node src/scene_manager_node.cpp)
target_include_directories(scene_manager_node PRIVATE include)
ament_target_dependencies(scene_manager_node geometry_msgs graspsort_msgs moveit_msgs
  moveit_ros_planning_interface rclcpp shape_msgs std_srvs)
install(TARGETS scene_manager_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(test_scene_logic test/test_scene_logic.cpp)
  target_include_directories(test_scene_logic PRIVATE include)

  # Launch sets CycloneDDS; scene.yaml + world_layout.yaml cover every declared parameter.
  ament_add_pytest_test(test_scene_config test/test_scene_config.py
    ENV GRASPSORT_SCENE_SRC=${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_export_include_directories(include)
ament_package()
