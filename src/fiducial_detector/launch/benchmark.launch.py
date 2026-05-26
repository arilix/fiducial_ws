#!/usr/bin/env python3
"""benchmark.launch.py — Run fiducial_detector in BENCHMARK mode."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    pkg = get_package_share_directory("fiducial_detector")
    params = os.path.join(pkg, "config", "detector.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("camera_topic", default_value="/camera/image_raw"),
        LogInfo(msg="Starting fiducial_detector in BENCHMARK mode..."),
        Node(
            package="fiducial_detector", executable="aruco_node",
            name="aruco_node",
            parameters=[params, {
                "camera_topic":     LaunchConfiguration("camera_topic"),
                "detection_mode":   "BENCHMARK",
                "benchmark_on_start": True,
                "show_window":      True,
            }], output="screen"),
    ])
