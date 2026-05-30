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
find_package(PkgConfig REQUIRED)
pkg_check_modules(APRILTAG REQUIRED apriltag)
set(COMMON_SRCS
  src/visualization.cpp
  src/pose_estimator.cpp
  src/fps_monitor.cpp
  src/detector_parameters.cpp
  src/dictionary_manager.cpp
  src/confidence_system.cpp
  src/marker_decoder.cpp
  src/charuco_handler.cpp
  src/board_handler.cpp
  src/custom_dictionary.cpp
  src/benchmark_runner.cpp
  src/board_generator.cpp
  src/apriltag_backend.cpp
  src/hybrid_detector.cpp
  src/display_utils.cpp
  src/marker_generator.cpp
)
add_executable(aruco_node
  src/main.cpp
  src/aruco.cpp
  ${COMMON_SRCS}
)
add_executable(calibration_node
  src/calibration_main.cpp
  src/calibration_node.cpp
  ${COMMON_SRCS}
)
foreach(tgt aruco_node calibration_node)
  target_include_directories(${tgt} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${APRILTAG_INCLUDE_DIRS}
  )
  ament_target_dependencies(${tgt}
    rclcpp rclcpp_components
    std_msgs sensor_msgs geometry_msgs
    tf2 tf2_geometry_msgs
    cv_bridge image_transport
  )
  target_link_libraries(${tgt} ${OpenCV_LIBS} Eigen3::Eigen ${APRILTAG_LIBRARIES})
endforeach()
# ── capture_node (C++ unified camera publisher) ───────────────────────────
add_executable(capture_node
  src/capture_main.cpp
  src/capture_node.cpp
)
target_include_directories(capture_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(capture_node
  rclcpp sensor_msgs cv_bridge
)
target_link_libraries(capture_node ${OpenCV_LIBS})
# Optional: compile with librealsense2 support
option(USE_REALSENSE "Enable Intel RealSense support in capture_node" OFF)
if(USE_REALSENSE)
  find_package(realsense2 REQUIRED)
  target_compile_definitions(capture_node PRIVATE FIDUCIAL_USE_REALSENSE)
  target_link_libraries(capture_node ${realsense2_LIBRARY})
  target_include_directories(capture_node PRIVATE ${realsense2_INCLUDE_DIR})
  message(STATUS "capture_node: Intel RealSense support ENABLED")
else()
  message(STATUS "capture_node: RealSense support OFF (rebuild with -DUSE_REALSENSE=ON to enable)")
endif()

# ── dict_test_node (standalone CLI: generate + detect + verify all marker IDs) ─
add_executable(dict_test_node
  src/dict_test_main.cpp
  src/marker_generator.cpp
  src/dictionary_manager.cpp
)
target_include_directories(dict_test_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(dict_test_node ${OpenCV_LIBS})

install(TARGETS aruco_node calibration_node capture_node dict_test_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY launch/  DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/  DESTINATION share/${PROJECT_NAME}/config)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
ament_package()
