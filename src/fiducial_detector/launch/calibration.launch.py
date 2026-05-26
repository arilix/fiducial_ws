from launch import LaunchDescription
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
