#!/usr/bin/env python3
"""webcam.launch.py — USB webcam + fiducial_detector"""
from launch import LaunchDescription
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
