cmake_minimum_required(VERSION 3.8)
project(graspsort_manipulation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# Header-only, ROS-free libraries (grasp_planner.hpp, bin_assignment.hpp).
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_grasp_planner test/test_grasp_planner.cpp)
  target_include_directories(test_grasp_planner PRIVATE include)
  target_include_directories(test_grasp_planner SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIR})

  ament_add_gtest(test_bin_assignment test/test_bin_assignment.cpp)
  target_include_directories(test_bin_assignment PRIVATE include)
endif()

ament_export_include_directories(include)
ament_export_dependencies(eigen3_cmake_module Eigen3)
ament_package()
