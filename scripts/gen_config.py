#!/usr/bin/env python3
"""Generate main.cpp, CMakeLists.txt, package.xml, launch files, config/detector.yaml."""
import os
BASE_PKG = "/home/arilix/Documents/vtol/vtol aruco/fiducial_ws/src/fiducial_detector"
SRC      = os.path.join(BASE_PKG, "src")
LAUNCH   = os.path.join(BASE_PKG, "launch")
CONFIG   = os.path.join(BASE_PKG, "config")
for d in [SRC, LAUNCH, CONFIG]:
    os.makedirs(d, exist_ok=True)
MAIN_CPP = r"""#include "fiducial_detector/aruco.hpp"
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
    rclcpp::ExecutorOptions(), 4 );
  auto node = std::make_shared<fiducial_detector::FiducialDetector>();
  executor->add_node(node);
  RCLCPP_INFO(node->get_logger(),
    "Spinning on MultiThreadedExecutor (4 threads)");
  executor->spin();
  rclcpp::shutdown();
  return 0;
}
"""
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
find_package(OpenCV 4 REQUIRED
  COMPONENTS core imgproc highgui aruco calib3d)
find_package(Eigen3 REQUIRED)
add_executable(aruco_node
  src/main.cpp
  src/aruco.cpp
  src/visualization.cpp
  src/pose_estimator.cpp
  src/fps_monitor.cpp
  src/detector_parameters.cpp
)
target_include_directories(aruco_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(aruco_node
  rclcpp rclcpp_components
  std_msgs sensor_msgs geometry_msgs
  tf2 tf2_geometry_msgs
  cv_bridge image_transport
)
target_link_libraries(aruco_node
  ${OpenCV_LIBS}
  Eigen3::Eigen
)
install(TARGETS aruco_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY launch/  DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/  DESTINATION share/${PROJECT_NAME}/config)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
ament_package()
"""
PKG_XML = r"""<?xml version="1.0"?>
<?xml-model href="http:
            schematypens="http:
<package format="3">
  <name>fiducial_detector</name>
  <version>2.0.0</version>
  <description>
    Production-ready ROS 2 Humble multi-marker fiducial detection system.
    Full OpenCV ArUco pipeline: ArUco, ChArUco, AprilTag (via OpenCV dict),
    ARTag-compatible. Includes pose estimation, alignment guidance, and ROS 2
    publisher pipeline.
  </description>
  <maintainer email="dev@example.com">Fiducial Detector</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>rclcpp_components</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>tf2</depend>
  <depend>tf2_geometry_msgs</depend>
  <depend>cv_bridge</depend>
  <depend>image_transport</depend>
  <build_depend>libopencv-dev</build_depend>
  <build_depend>libeigen3-dev</build_depend>
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
"""
DETECTOR_YAML = r"""# ═══════════════════════════════════════════════════════════════════════════
aruco_node:
  ros__parameters:
    marker_size: 0.05
    camera_topic: "/camera/image_raw"
    dictionary_type: "DICT_4X4_50"
    enable_charuco: true
    show_window: true
    show_rejected: true
    alignment_tolerance: 50
    smoothing_alpha: 0.4
    max_missed_frames: 5
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
    adaptiveThreshWinSizeMin: 3
    adaptiveThreshWinSizeMax: 23
    adaptiveThreshWinSizeStep: 10
    adaptiveThreshConstant: 7.0
    minMarkerPerimeterRate: 0.03
    maxMarkerPerimeterRate: 4.0
    polygonalApproxAccuracyRate: 0.03
    minCornerDistanceRate: 0.05
    minDistanceToBorder: 3
    minMarkerDistanceRate: 0.05
    perspectiveRemovePixelPerCell: 8
    perspectiveRemoveIgnoredMarginPerCell: 0.13
    maxErroneousBitsInBorderRate: 0.35
    errorCorrectionRate: 0.6
    detectInvertedMarker: false
    cornerRefinementMethod: 1
    cornerRefinementWinSize: 5
    cornerRefinementMaxIterations: 30
    cornerRefinementMinAccuracy: 0.1
"""
WEBCAM_LAUNCH = r"""#!/usr/bin/env python3
"""
WEBCAM_LAUNCH += '"""webcam.launch.py — USB webcam + fiducial_detector"""\n'
WEBCAM_LAUNCH += r"""from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    pkg = get_package_share_directory('fiducial_detector')
    params = os.path.join(pkg, 'config', 'detector.yaml')
    device_arg = DeclareLaunchArgument('device', default_value='/dev/video0',
        description='V4L2 device path')
    size_arg = DeclareLaunchArgument('marker_size', default_value='0.05')
    dict_arg  = DeclareLaunchArgument('dictionary_type', default_value='DICT_4X4_50')
    camera_node = Node(
        package='v4l2_camera', executable='v4l2_camera_node',
        name='v4l2_camera',
        parameters=[{'video_device': LaunchConfiguration('device'),
                     'image_size': [640, 480]}],
        remappings=[('/image_raw', '/camera/image_raw')],
        output='screen')
    detector_node = Node(
        package='fiducial_detector', executable='aruco_node',
        name='aruco_node',
        parameters=[params, {
            'camera_topic':    '/camera/image_raw',
            'marker_size':     LaunchConfiguration('marker_size'),
            'dictionary_type': LaunchConfiguration('dictionary_type'),
        }], output='screen')
    return LaunchDescription([
        device_arg, size_arg, dict_arg,
        LogInfo(msg='Starting USB webcam fiducial detection...'),
        camera_node, detector_node,
    ])
"""
ROS_TOPIC_LAUNCH = r"""#!/usr/bin/env python3
"""
ROS_TOPIC_LAUNCH += '"""ros_topic.launch.py — existing ROS2 camera topic"""\n'
ROS_TOPIC_LAUNCH += r"""from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    pkg = get_package_share_directory('fiducial_detector')
    params = os.path.join(pkg, 'config', 'detector.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('camera_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('marker_size',  default_value='0.05'),
        DeclareLaunchArgument('dictionary_type', default_value='DICT_4X4_50'),
        LogInfo(msg='Starting fiducial_detector on ROS topic...'),
        Node(
            package='fiducial_detector', executable='aruco_node',
            name='aruco_node',
            parameters=[params, {
                'camera_topic':    LaunchConfiguration('camera_topic'),
                'marker_size':     LaunchConfiguration('marker_size'),
                'dictionary_type': LaunchConfiguration('dictionary_type'),
            }], output='screen'),
    ])
"""
files = {
    os.path.join(SRC,      "main.cpp"):              MAIN_CPP,
    os.path.join(BASE_PKG, "CMakeLists.txt"):        CMAKE,
    os.path.join(BASE_PKG, "package.xml"):           PKG_XML,
    os.path.join(CONFIG,   "detector.yaml"):         DETECTOR_YAML,
    os.path.join(LAUNCH,   "webcam.launch.py"):      WEBCAM_LAUNCH,
    os.path.join(LAUNCH,   "ros_topic.launch.py"):   ROS_TOPIC_LAUNCH,
}
for path, content in files.items():
    with open(path, "w") as f:
        f.write(content.lstrip("\n"))
    print(f"  WROTE {os.path.basename(path)}  ({content.count(chr(10))} lines)")
print("CONFIG FILES OK")
