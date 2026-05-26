#!/usr/bin/env python3
"""realsense.launch.py — Intel RealSense + fiducial_detector"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('fiducial_detector')
    params = os.path.join(pkg, 'config', 'detector.yaml')
    
    size_arg = DeclareLaunchArgument('marker_size', default_value='0.05')
    dict_arg = DeclareLaunchArgument('dictionary_type', default_value='DICT_4X4_50')
    
    # Menyertakan launch file dari realsense2_camera bawaan ROS 2
    rs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ]),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'false', # Mematikan depth sementara untuk hemat resource
            'rgb_camera.profile': '640x480x30'
        }.items()
    )

    # Node Fiducial Detector
    detector_node = Node(
        package='fiducial_detector', executable='aruco_node',
        name='aruco_node',
        parameters=[params, {
            # Topic standar Intel RealSense di ROS 2
            'camera_topic':    '/camera/camera/color/image_raw',
            'marker_size':     LaunchConfiguration('marker_size'),
            'dictionary_type': LaunchConfiguration('dictionary_type'),
        }], output='screen')

    return LaunchDescription([
        size_arg, dict_arg,
        LogInfo(msg='Starting Intel RealSense fiducial detection...'),
        rs_launch,
        detector_node,
    ])
