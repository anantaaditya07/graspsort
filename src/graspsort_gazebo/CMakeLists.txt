cmake_minimum_required(VERSION 3.8)
project(graspsort_gazebo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(graspsort_msgs REQUIRED)

# Attach world plugin (D-03): libgraspsort_attach.so
add_library(graspsort_attach SHARED src/attach_plugin.cpp)
target_include_directories(graspsort_attach PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# Gazebo headers are third-party. Mark them SYSTEM so their warnings do not count against us.
target_include_directories(graspsort_attach SYSTEM PRIVATE ${GAZEBO_INCLUDE_DIRS})
ament_target_dependencies(graspsort_attach gazebo_dev gazebo_ros rclcpp graspsort_msgs)

install(TARGETS graspsort_attach LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config urdf worlds DESTINATION share/${PROJECT_NAME})

# Object models fetched by scripts/fetch_models.sh (D-08). Installed only if present.
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/models_external)
  install(DIRECTORY models_external DESTINATION share/${PROJECT_NAME})
else()
  message(STATUS "models_external/ not found; run scripts/fetch_models.sh to fetch the object models")
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(test_attachment_registry test/test_attachment_registry.cpp)
  target_include_directories(test_attachment_registry PRIVATE include)

  ament_add_pytest_test(test_world_layout test/test_world_layout.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    ENV GRASPSORT_GAZEBO_SRC=${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 120)
endif()

ament_export_include_directories(include)
ament_package()
