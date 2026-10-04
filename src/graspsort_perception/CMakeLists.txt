cmake_minimum_required(VERSION 3.8)
project(graspsort_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(graspsort_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc dnn)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vision_msgs REQUIRED)

# --- ONNX Runtime (pinned 1.20.1, fetched by scripts/setup_ort.sh; copied from SemNav) ---
set(GRASPSORT_ORT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/onnxruntime"
    CACHE PATH "ONNX Runtime install root (from scripts/setup_ort.sh)")
get_filename_component(GRASPSORT_ORT_ROOT "${GRASPSORT_ORT_ROOT}" ABSOLUTE)
if(NOT EXISTS "${GRASPSORT_ORT_ROOT}/lib/libonnxruntime.so"
   OR NOT EXISTS "${GRASPSORT_ORT_ROOT}/include/onnxruntime_cxx_api.h")
  message(FATAL_ERROR
    "ONNX Runtime not found in ${GRASPSORT_ORT_ROOT}.\n"
    "Run: scripts/setup_ort.sh   (from the repo root), then rebuild.")
endif()
add_library(onnxruntime::onnxruntime SHARED IMPORTED)
set_target_properties(onnxruntime::onnxruntime PROPERTIES
  IMPORTED_LOCATION "${GRASPSORT_ORT_ROOT}/lib/libonnxruntime.so.1.20.1"
  IMPORTED_SONAME "libonnxruntime.so.1"
  INTERFACE_INCLUDE_DIRECTORIES "${GRASPSORT_ORT_ROOT}/include"
  INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${GRASPSORT_ORT_ROOT}/include")

# Ship the runtime library with the package so `ros2 run` finds it.
file(GLOB GRASPSORT_ORT_LIBS "${GRASPSORT_ORT_ROOT}/lib/libonnxruntime.so*")
install(FILES ${GRASPSORT_ORT_LIBS} DESTINATION lib)

# Default model_path parameter: the repo's models/yolov8n.onnx as an absolute path, baked in at
# compile time (as in SemNav), so it works from the install space and from the source tree as
# long as the repo is present. Model files are not in git (D-08: scripts/export_yolo.py).
set(GRASPSORT_MODEL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../models/yolov8n.onnx")
get_filename_component(GRASPSORT_MODEL_PATH "${GRASPSORT_MODEL_PATH}" ABSOLUTE)

# Header-only, ROS-free logic (detector pieces + projection.hpp for the localizer).
install(DIRECTORY include/ DESTINATION include)

add_executable(object_detector_node src/object_detector_node.cpp)
target_include_directories(object_detector_node PRIVATE include)
target_link_libraries(object_detector_node onnxruntime::onnxruntime ${OpenCV_LIBS})
ament_target_dependencies(object_detector_node cv_bridge rclcpp sensor_msgs std_msgs vision_msgs)
target_compile_definitions(object_detector_node PRIVATE
  GRASPSORT_DEFAULT_MODEL_PATH="${GRASPSORT_MODEL_PATH}")
set_target_properties(object_detector_node PROPERTIES INSTALL_RPATH "$ORIGIN/..")
install(TARGETS object_detector_node DESTINATION lib/${PROJECT_NAME})

# object_localizer_node (architecture 7.2, D-12): detections + depth -> /objects_3d.
add_executable(object_localizer_node src/object_localizer_node.cpp)
target_include_directories(object_localizer_node PRIVATE include)
target_include_directories(object_localizer_node SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIR})
target_link_libraries(object_localizer_node opencv_core opencv_imgproc)
ament_target_dependencies(object_localizer_node cv_bridge geometry_msgs graspsort_msgs rclcpp
  sensor_msgs tf2 tf2_ros vision_msgs)
install(TARGETS object_localizer_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/eval_localization.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  # Hello-world Ort::Session before any ROS code.
  ament_add_gtest(test_ort_session test/test_ort_session.cpp)
  target_link_libraries(test_ort_session onnxruntime::onnxruntime)
  target_compile_definitions(test_ort_session PRIVATE
    GRASPSORT_TEST_MODEL_PATH="${GRASPSORT_MODEL_PATH}")

  # ROS-free pre/post-processing (architecture 7.1 tests, copied from SemNav).
  foreach(t test_letterbox test_yolo_decode test_rolling_stats test_yolo_detector)
    ament_add_gtest(${t} test/${t}.cpp)
    target_include_directories(${t} PRIVATE include)
    target_link_libraries(${t} onnxruntime::onnxruntime ${OpenCV_LIBS})
  endforeach()
  target_compile_definitions(test_yolo_detector PRIVATE
    GRASPSORT_TEST_MODEL_PATH="${GRASPSORT_MODEL_PATH}")

  # Localizer geometry (architecture 7.2), header-only: Eigen + OpenCV core/imgproc.
  foreach(t test_projection test_localizer)
    ament_add_gtest(${t} test/${t}.cpp)
    target_include_directories(${t} PRIVATE include)
    target_include_directories(${t} SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIR})
    target_link_libraries(${t} opencv_core opencv_imgproc)
  endforeach()

  # Launch sets CycloneDDS; perception.yaml class_filter matches D-04.
  ament_add_pytest_test(test_perception_config test/test_perception_config.py
    ENV GRASPSORT_PERCEPTION_SRC=${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_package()
