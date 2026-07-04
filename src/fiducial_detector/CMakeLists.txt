cmake_minimum_required(VERSION 3.8)
project(fiducial_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2 -march=native)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc highgui aruco calib3d)
find_package(Eigen3 REQUIRED)

set(COMMON_SRCS
  src/nodes/visualization.cpp
  src/core/pose_estimator.cpp
  src/core/fps_monitor.cpp
  src/core/detector_parameters.cpp
  src/core/dictionary_manager.cpp
  src/core/confidence_system.cpp
  src/core/marker_decoder.cpp
  src/core/benchmark_runner.cpp
  src/core/gate_alignment.cpp
  src/core/marker_generator.cpp
)

add_executable(aruco_node
  src/main/main.cpp
  src/core/aruco.cpp
  ${COMMON_SRCS}
)

add_executable(calibration_node
  src/main/calibration_main.cpp
  src/nodes/calibration_node.cpp
  ${COMMON_SRCS}
)

foreach(tgt aruco_node calibration_node)
  target_include_directories(${tgt} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
  )
  ament_target_dependencies(${tgt}
    rclcpp rclcpp_components
    std_msgs sensor_msgs geometry_msgs
    tf2 tf2_geometry_msgs
    cv_bridge image_transport
  )
  target_link_libraries(${tgt} ${OpenCV_LIBS} Eigen3::Eigen)
endforeach()

# ── capture_node ──────────────────────────────────────────────────────────
add_executable(capture_node
  src/main/capture_main.cpp
  src/nodes/capture_node.cpp
)
target_include_directories(capture_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(capture_node rclcpp sensor_msgs cv_bridge)
target_link_libraries(capture_node ${OpenCV_LIBS})

option(USE_REALSENSE "Enable Intel RealSense support in capture_node" OFF)
if(USE_REALSENSE)
  find_package(realsense2 REQUIRED)
  target_compile_definitions(capture_node PRIVATE FIDUCIAL_USE_REALSENSE)
  target_link_libraries(capture_node ${realsense2_LIBRARY})
  target_include_directories(capture_node PRIVATE ${realsense2_INCLUDE_DIR})
  message(STATUS "capture_node: Intel RealSense support ENABLED")
else()
  message(STATUS "capture_node: RealSense OFF (rebuild with -DUSE_REALSENSE=ON)")
endif()

# ── CUDA GPU acceleration (preprocessing: CLAHE, blur, sharpen) ───────────
option(USE_CUDA "Enable CUDA GPU acceleration for preprocessing" OFF)
if(USE_CUDA)
  if(NOT DEFINED OpenCV_CUDA_VERSION AND NOT DEFINED CUDA_VERSION)
    find_package(CUDA QUIET)
  endif()
  if(NOT OpenCV_CUDA_VERSION)
    message(WARNING
      "USE_CUDA=ON but OpenCV has no CUDA support. "
      "Rebuild OpenCV with -DWITH_CUDA=ON -DWITH_CUDNN=ON. "
      "Falling back to CPU preprocessing.")
  else()
    foreach(tgt aruco_node calibration_node)
      target_compile_definitions(${tgt} PRIVATE FIDUCIAL_USE_CUDA)
    endforeach()
    message(STATUS "CUDA preprocessing: ENABLED "
      "(OpenCV CUDA ${OpenCV_CUDA_VERSION})")
  endif()
else()
  message(STATUS "CUDA preprocessing: OFF (rebuild with -DUSE_CUDA=ON)")
endif()

# ── dict_test_node — DICT_7X7_50 50/50 validation ─────────────────────────
add_executable(dict_test_node src/main/dict_test_main.cpp)
target_include_directories(dict_test_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(dict_test_node rclcpp std_msgs sensor_msgs geometry_msgs cv_bridge image_transport)
target_link_libraries(dict_test_node ${OpenCV_LIBS})

install(TARGETS aruco_node calibration_node capture_node dict_test_node
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY utils/ DESTINATION include/utils)
install(DIRECTORY launch/  DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/  DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
ament_package()
