#!/usr/bin/env python3
"""Generate fully-integrated aruco.cpp, aruco.hpp, CMakeLists.txt, detector.yaml, launch files."""
import os, textwrap
WS   = "/home/arilix/Documents/vtol/vtol aruco/fiducial_ws"
PKG  = WS + "/src/fiducial_detector"
SRC  = PKG + "/src"
INC  = PKG + "/include/fiducial_detector"
CFG  = PKG + "/config"
LCH  = PKG + "/launch"

def write(path, content):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write(content)
    print(f"  WROTE {os.path.basename(path)}  ({content.count(chr(10))} lines)")

# ─── CMakeLists.txt ───────────────────────────────────────────────────────────
CMAKE = r"""cmake_minimum_required(VERSION 3.8)
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
  )
  ament_target_dependencies(${tgt}
    rclcpp rclcpp_components
    std_msgs sensor_msgs geometry_msgs
    tf2 tf2_geometry_msgs
    cv_bridge image_transport
  )
  target_link_libraries(${tgt} ${OpenCV_LIBS} Eigen3::Eigen)
endforeach()
install(TARGETS aruco_node calibration_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY launch/  DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/  DESTINATION share/${PROJECT_NAME}/config)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
ament_package()
"""
write(PKG + "/CMakeLists.txt", CMAKE.lstrip())

# ─── calibration.launch.py ────────────────────────────────────────────────────
CAL_LAUNCH = r"""from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('output_yaml',  default_value='/tmp/camera_calibration.yaml'),
        DeclareLaunchArgument('calib_mode',   default_value='CHARUCO'),
        Node(
            package='fiducial_detector',
            executable='calibration_node',
            name='calibration_node',
            output='screen',
            parameters=[{
                'camera_topic': LaunchConfiguration('camera_topic'),
                'output_yaml':  LaunchConfiguration('output_yaml'),
                'calib_mode':   LaunchConfiguration('calib_mode'),
                'charuco_cols': 7,
                'charuco_rows': 5,
                'charuco_sq':   0.035,
                'charuco_mk':   0.0175,
                'min_frames':   15,
                'show_window':  True,
            }],
        ),
    ])
"""
write(LCH + "/calibration.launch.py", CAL_LAUNCH.lstrip())

# ─── detector.yaml ────────────────────────────────────────────────────────────
YAML = r"""# ═══════════════════════════════════════════════════
# Fiducial Detector — Full Configuration
# ═══════════════════════════════════════════════════
aruco_node:
  ros__parameters:
    # ── Core ──────────────────────────────────────
    marker_size:          0.05
    camera_topic:         "/camera/image_raw"
    dictionary_type:      "DICT_4X4_50"
    detection_mode:       "SINGLE"     # SINGLE|AUTO|MULTI|BENCHMARK
    benchmark_on_start:   false

    # ── Enable flags ──────────────────────────────
    enable_charuco:       true
    enable_gridboard:     false
    enable_diamond:       false
    enable_custom_dict:   false
    custom_dict_path:     ""

    # ── Display ───────────────────────────────────
    show_window:          true
    show_rejected:        true
    show_corner_labels:   true
    show_orientation_arrow: true
    show_confidence:      true
    debug_windows:        false       # toggle with 'd' key

    # ── Alignment ─────────────────────────────────
    alignment_tolerance:  50
    smoothing_alpha:      0.4
    max_missed_frames:    5

    # ── Camera intrinsics ─────────────────────────
    camera_matrix:
      - 640.0
      - 0.0
      - 320.0
      - 0.0
      - 640.0
      - 240.0
      - 0.0
      - 0.0
      - 1.0
    dist_coeffs: [0.0, 0.0, 0.0, 0.0, 0.0]

    # ── ChArUco board ─────────────────────────────
    charuco_cols:         7
    charuco_rows:         5
    charuco_sq:           0.035
    charuco_mk:           0.0175

    # ── GridBoard ─────────────────────────────────
    gridboard_cols:       5
    gridboard_rows:       7
    gridboard_marker_size: 0.04
    gridboard_sep:        0.01

    # ── ArUco Detector Parameters ─────────────────
    adaptiveThreshWinSizeMin:    3
    adaptiveThreshWinSizeMax:    23
    adaptiveThreshWinSizeStep:   10
    adaptiveThreshConstant:      7.0
    minMarkerPerimeterRate:      0.03
    maxMarkerPerimeterRate:      4.0
    polygonalApproxAccuracyRate: 0.03
    minCornerDistanceRate:       0.05
    minDistanceToBorder:         3
    minMarkerDistanceRate:       0.05
    perspectiveRemovePixelPerCell:          8
    perspectiveRemoveIgnoredMarginPerCell:  0.13
    maxErroneousBitsInBorderRate: 0.35
    errorCorrectionRate:          0.6
    detectInvertedMarker:         false
    cornerRefinementMethod:       1   # 0=NONE 1=SUBPIX 2=CONTOUR 3=APRILTAG
    cornerRefinementWinSize:      5
    cornerRefinementMaxIterations: 30
    cornerRefinementMinAccuracy:   0.1
"""
write(CFG + "/detector.yaml", YAML.lstrip())

print("ALL INTEGRATION FILES OK")
