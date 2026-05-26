#!/usr/bin/env python3
"""ros_topic.launch.py — existing ROS2 camera topic"""
from launch import LaunchDescription
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
