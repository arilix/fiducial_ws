# fiducial_detector — ROS 2 Humble Multi-Marker Detection System

Real-time detection of **ArUco**, **AprilTag**, **ARTag**, and **ChArUco** markers
with 6DOF pose estimation, UI overlay, and alignment guidance.

---

## System Architecture

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

## 1. Install Dependencies

```bash
# ROS 2 Humble base
sudo apt update
sudo apt install -y \
  ros-humble-rclcpp \
  ros-humble-sensor-msgs \
  ros-humble-geometry-msgs \
  ros-humble-std-msgs \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-tf2 \
  ros-humble-tf2-geometry-msgs \
  ros-humble-v4l2-camera

# Vision libraries
sudo apt install -y \
  libopencv-dev \
  python3-opencv \
  libeigen3-dev \
  libapriltag-dev \
  libapriltag3

# Build tools
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep
```

---

## 2. Workspace Setup

```bash
mkdir -p ~/fiducial_ws/src
cp -r fiducial_detector ~/fiducial_ws/src/
cd ~/fiducial_ws
rosdep install --from-paths src --ignore-src -r -y
```

---

## 3. Build

```bash
cd ~/fiducial_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select fiducial_detector
source install/setup.bash
```

---

## 4. Run

### Option A — USB Webcam

```bash
source ~/fiducial_ws/install/setup.bash
ros2 launch fiducial_detector webcam.launch.py device:=/dev/video0
```

Override parameters:
```bash
ros2 launch fiducial_detector webcam.launch.py \
  device:=/dev/video0 \
  marker_size:=0.08
```

### Option B — Existing ROS 2 Camera Topic

```bash
ros2 launch fiducial_detector ros_topic.launch.py \
  camera_topic:=/camera/color/image_raw \
  marker_size:=0.05
```

### Option C — Direct run (no launch)

```bash
ros2 run fiducial_detector aruco_node \
  --ros-args -p camera_topic:=/camera/image_raw -p marker_size:=0.05
```

---

## 5. Published Topics

| Topic                   | Type                         | Description            |
|-------------------------|------------------------------|------------------------|
| `/fiducial/pose`        | geometry_msgs/PoseStamped    | 6DOF pose of marker[0] |
| `/fiducial/debug_image` | sensor_msgs/Image            | Annotated frame        |
| `/fiducial/alignment`   | std_msgs/String              | Alignment status string |
| `/fiducial/fps`         | std_msgs/Float32             | Current FPS (1 Hz pub) |

Alignment values: `NO_MARKER`, `TARGET_LOCK`, `GESER_KIRI`, `GESER_KANAN`,
`NAIK`, `TURUN`, combinations like `GESER_KIRI|NAIK`, etc.

---

## 6. ROS 2 Parameters

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

## 7. Generate ArUco Markers

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

## 8. Generate AprilTag

```bash
# Using official generator tool
pip install apriltag-generator
python3 -c "
import apriltag
import cv2
# Download tag36h11 images from:
# https://github.com/AprilRobotics/apriltag-imgs
# Print tag36h11_id_XX.png at physical size matching marker_size param
"
```

Or download directly: https://github.com/AprilRobotics/apriltag-imgs/tree/master/tag36h11

---

## 9. Camera Calibration

```bash
# Print a 9x6 chessboard: https://calib.io/pages/camera-calibration-pattern-generator
# Move board in front of camera (collect 20+ images)

ros2 run camera_calibration cameracalibrator \
  --size 9x6 --square 0.025 \
  image:=/camera/image_raw camera:=/camera

# Results saved to ~/.ros/camera_info/camera.yaml
# Copy values to config/params.yaml: camera_matrix and dist_coeffs
```

---

## 10. Testing with rosbag

```bash
# Record
ros2 bag record /camera/image_raw -o my_bag

# Playback + detect
ros2 bag play my_bag --loop &
ros2 launch fiducial_detector ros_topic.launch.py
```

---

## 11. Jetson / Mini-PC Optimizations

- Set `show_window: false` for headless operation
- Reduce `tag_detector_->quad_decimate` to 4.0 if AprilTag is slow
- Use `image_transport` compressed subscriber for network cameras:
  ```bash
  ros2 run fiducial_detector aruco_node \
    --ros-args -p camera_topic:=/camera/image_raw/compressed
  ```
- Enable GPU-accelerated OpenCV (requires OpenCV built with CUDA):
  ```bash
  # Check CUDA support:
  python3 -c "import cv2; print(cv2.cuda.getCudaEnabledDeviceCount())"
  ```

---

## 12. Troubleshooting

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
