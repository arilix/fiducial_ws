#!/usr/bin/env python3
"""Generate remaining project files: launch, config, README"""
import os
BASE = "/home/arilix/Documents/vtol/vtol aruco/fiducial_ws/src/fiducial_detector"
PARAMS_YAML = """\
aruco_node:
  ros__parameters:
    marker_size: 0.05
    camera_topic: "/camera/image_raw"
    dictionary_type: "DICT_4X4_50"
    enable_apriltag: true
    enable_charuco: true
    alignment_tolerance: 50
    smoothing_alpha: 0.4
    max_missed_frames: 5
    show_window: true
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
"""
WEBCAM_LAUNCH = """\
#!/usr/bin/env python3
\"\"\"
webcam.launch.py
Launches v4l2_camera (USB webcam) + fiducial_detector on the same machine.
Requires:
  sudo apt install ros-humble-v4l2-camera
\"\"\"
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    pkg_share = get_package_share_directory('fiducial_detector')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')
    device_arg = DeclareLaunchArgument(
        'device', default_value='/dev/video0',
        description='Video4Linux device path')
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        parameters=[{
            'video_device': LaunchConfiguration('device'),
            'image_size':   [640, 480],
            'camera_frame_id': 'camera',
        }],
        remappings=[
            ('/image_raw', '/camera/image_raw'),
        ],
        output='screen',
    )
    detector_node = Node(
        package='fiducial_detector',
        executable='aruco_node',
        name='aruco_node',
        parameters=[params_file, {
            'camera_topic': '/camera/image_raw',
        }],
        output='screen',
    )
    return LaunchDescription([
        device_arg,
        LogInfo(msg='Starting USB webcam fiducial detection...'),
        camera_node,
        detector_node,
    ])
"""
ROS_TOPIC_LAUNCH = """\
#!/usr/bin/env python3
\"\"\"
ros_topic.launch.py
Launches only the fiducial_detector subscribing to an existing ROS 2 camera topic.
Use this when a camera node is already running (e.g. RealSense, compressed stream).
\"\"\"
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    pkg_share = get_package_share_directory('fiducial_detector')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')
    topic_arg = DeclareLaunchArgument(
        'camera_topic', default_value='/camera/image_raw',
        description='Existing camera image_raw topic to subscribe')
    size_arg = DeclareLaunchArgument(
        'marker_size', default_value='0.05',
        description='Physical marker side length in meters')
    dict_arg = DeclareLaunchArgument(
        'dictionary_type', default_value='DICT_4X4_50',
        description='ArUco dictionary')
    detector_node = Node(
        package='fiducial_detector',
        executable='aruco_node',
        name='aruco_node',
        parameters=[params_file, {
            'camera_topic':     LaunchConfiguration('camera_topic'),
            'marker_size':      LaunchConfiguration('marker_size'),
            'dictionary_type':  LaunchConfiguration('dictionary_type'),
        }],
        output='screen',
    )
    return LaunchDescription([
        topic_arg,
        size_arg,
        dict_arg,
        LogInfo(msg='Starting fiducial_detector on existing ROS topic...'),
        detector_node,
    ])
"""
README = """\
Real-time detection of **ArUco**, **AprilTag**, **ARTag**, and **ChArUco** markers
with 6DOF pose estimation, UI overlay, and alignment guidance.
---
```
[USB Webcam / ROS Camera Topic]
         │
         ▼  /camera/image_raw
  ┌──────────────────────────┐
  │  aruco_node              │
  │  (FiducialDetector)      │
  │  ├─ detectAruco()        │
  │  ├─ detectARTag()        │
  │  ├─ detectAprilTag()     │
  │  ├─ detectCharuco()      │
  │  ├─ estimatePose()       │
  │  ├─ smoothMarkers()      │
  │  ├─ drawUI()             │
  │  └─ publishResults()     │
  └──────────────────────────┘
         │
   ┌─────┼────────────────────┐
   ▼     ▼                    ▼
/fiducial/pose   /fiducial/debug_image
/fiducial/alignment           /fiducial/fps
```
---
```bash
sudo apt update
sudo apt install -y \\
  ros-humble-rclcpp \\
  ros-humble-sensor-msgs \\
  ros-humble-geometry-msgs \\
  ros-humble-std-msgs \\
  ros-humble-cv-bridge \\
  ros-humble-image-transport \\
  ros-humble-tf2 \\
  ros-humble-tf2-geometry-msgs \\
  ros-humble-v4l2-camera
sudo apt install -y \\
  libopencv-dev \\
  python3-opencv \\
  libeigen3-dev \\
  libapriltag-dev \\
  libapriltag3
sudo apt install -y \\
  python3-colcon-common-extensions \\
  python3-rosdep
```
---
```bash
mkdir -p ~/fiducial_ws/src
cp -r fiducial_detector ~/fiducial_ws/src/
cd ~/fiducial_ws
rosdep install --from-paths src --ignore-src -r -y
```
---
```bash
cd ~/fiducial_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select fiducial_detector
source install/setup.bash
```
---
```bash
source ~/fiducial_ws/install/setup.bash
ros2 launch fiducial_detector webcam.launch.py device:=/dev/video0
```
Override parameters:
```bash
ros2 launch fiducial_detector webcam.launch.py \\
  device:=/dev/video0 \\
  marker_size:=0.08
```
```bash
ros2 launch fiducial_detector ros_topic.launch.py \\
  camera_topic:=/camera/color/image_raw \\
  marker_size:=0.05
```
```bash
ros2 run fiducial_detector aruco_node \\
  --ros-args -p camera_topic:=/camera/image_raw -p marker_size:=0.05
```
---
| Topic                   | Type                         | Description            |
|-------------------------|------------------------------|------------------------|
| `/fiducial/pose`        | geometry_msgs/PoseStamped    | 6DOF pose of marker[0] |
| `/fiducial/debug_image` | sensor_msgs/Image            | Annotated frame        |
| `/fiducial/alignment`   | std_msgs/String              | Alignment status string |
| `/fiducial/fps`         | std_msgs/Float32             | Current FPS (1 Hz pub) |
Alignment values: `NO_MARKER`, `TARGET_LOCK`, `GESER_KIRI`, `GESER_KANAN`,
`NAIK`, `TURUN`, combinations like `GESER_KIRI|NAIK`, etc.
---
| Parameter             | Type    | Default            | Description                          |
|-----------------------|---------|--------------------|--------------------------------------|
| `marker_size`         | double  | `0.05`             | Physical side length (meters)        |
| `camera_topic`        | string  | `/camera/image_raw`| Camera topic                         |
| `dictionary_type`     | string  | `DICT_4X4_50`      | ArUco dictionary                     |
| `enable_apriltag`     | bool    | `true`             | Enable AprilTag detection            |
| `enable_charuco`      | bool    | `true`             | Enable ChArUco detection             |
| `alignment_tolerance` | int     | `50`               | Pixel tolerance for TARGET_LOCK      |
| `smoothing_alpha`     | double  | `0.4`              | EMA alpha (0=no update, 1=raw)       |
| `max_missed_frames`   | int     | `5`                | Frames before dropping a track       |
| `show_window`         | bool    | `true`             | Show OpenCV window                   |
| `camera_matrix`       | double[]| identity           | 3x3 camera matrix (row-major)        |
| `dist_coeffs`         | double[]| zeros              | Distortion coefficients              |
---
```python
import cv2
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
for marker_id in range(5):
    img = cv2.aruco.generateImageMarker(dictionary, marker_id, 300)
    cv2.imwrite(f"aruco_{marker_id}.png", img)
    print(f"Saved aruco_{marker_id}.png")
```
Print at 50mm x 50mm for `marker_size=0.05`.
---
```bash
pip install apriltag-generator
python3 -c "
import apriltag
import cv2
"
```
Or download directly: https:
---
```bash
ros2 run camera_calibration cameracalibrator \\
  --size 9x6 --square 0.025 \\
  image:=/camera/image_raw camera:=/camera
```
---
```bash
ros2 bag record /camera/image_raw -o my_bag
ros2 bag play my_bag --loop &
ros2 launch fiducial_detector ros_topic.launch.py
```
---
- Set `show_window: false` for headless operation
- Reduce `tag_detector_->quad_decimate` to 4.0 if AprilTag is slow
- Use `image_transport` compressed subscriber for network cameras:
  ```bash
  ros2 run fiducial_detector aruco_node \\
    --ros-args -p camera_topic:=/camera/image_raw/compressed
  ```
- Enable GPU-accelerated OpenCV (requires OpenCV built with CUDA):
  ```bash
  python3 -c "import cv2; print(cv2.cuda.getCudaEnabledDeviceCount())"
  ```
---
| Problem                          | Solution                                                |
|----------------------------------|---------------------------------------------------------|
| `No module: apriltag`            | `sudo apt install libapriltag-dev libapriltag3`        |
| `cv_bridge exception`            | Ensure camera publishes `bgr8` or `rgb8`                |
| Markers not detected             | Ensure adequate lighting; check `dictionary_type`       |
| Low FPS on Jetson                | Set `quad_decimate=4.0`, reduce resolution             |
| Camera not found (`/dev/video0`) | `ls /dev/video*` and update `device` launch arg        |
| `solvePnP failed`                | Set correct `camera_matrix` from calibration            |
| Build error: `apriltag.h`        | `sudo apt install libapriltag-dev`                      |
| `image_transport` not found      | `sudo apt install ros-humble-image-transport`           |
"""
files = {
    os.path.join(BASE, "config", "params.yaml"):    PARAMS_YAML,
    os.path.join(BASE, "launch", "webcam.launch.py"): WEBCAM_LAUNCH,
    os.path.join(BASE, "launch", "ros_topic.launch.py"): ROS_TOPIC_LAUNCH,
    os.path.join(BASE, "README.md"):                README,
}
for path, content in files.items():
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write(content)
    print(f"Written: {path}")
print("ALL FILES GENERATED")
