# fiducial_detector — ROS 2 Humble Hybrid Multi-Marker Detection System

Real-time detection of **ArUco**, **AprilTag 3**, **ChArUco**, and **GridBoard** markers with 6DOF pose estimation, hybrid OpenCV + AprilTag3 detection, CLAHE preprocessing, and VTOL alignment guidance.

> **KRTI 2026** — Precision landing on 2000×2000 mm orange waypoint with 500×500 mm ArUco marker.

---

## System Architecture

```
[capture_node]  ←── USB Webcam (cv::VideoCapture / V4L2)
      or         ←── Intel RealSense D4xx (librealsense2 SDK)
[external topic] ←── Any ROS 2 image publisher
        │
        ▼  /camera/image_raw
┌──────────────────────────────────────────┐
│  aruco_node  (FiducialDetector)          │
│  ├─ preprocessFrame()                    │
│  │   ├─ CLAHE contrast enhancement       │
│  │   ├─ Optional sharpen / blur          │
│  ├─ HybridDetector                       │
│  │   ├─ OpenCV ArUco backend             │
│  │   ├─ AprilTag 3 backend (libapriltag) │
│  │   └─ FUSION: weighted score merge     │
│  ├─ detectCharuco() / detectBoard()      │
│  ├─ estimatePoses() — solvePnP           │
│  ├─ computeConfidence()                  │
│  └─ renderAnnotations() + displayLoop()  │
└──────────────────────────────────────────┘
        │
 ┌──────┼──────────────────────────────┐
 ▼      ▼                              ▼
/fiducial/pose    /fiducial/debug_image
/fiducial/alignment           /fiducial/fps
```

### Executables

| Executable | Description |
|---|---|
| `aruco_node` | Main fiducial detection node |
| `capture_node` | Unified C++ camera publisher (webcam + RealSense) |
| `calibration_node` | Camera intrinsic calibration via ChArUco |

---

## Prerequisites

```bash
# ROS 2 Humble deps
sudo apt install -y \
  ros-humble-rclcpp ros-humble-sensor-msgs \
  ros-humble-geometry-msgs ros-humble-std-msgs \
  ros-humble-cv-bridge ros-humble-image-transport \
  ros-humble-tf2 ros-humble-tf2-geometry-msgs

# OpenCV + build tools
sudo apt install -y \
  libopencv-dev libeigen3-dev \
  libapriltag-dev libapriltag3 \
  python3-colcon-common-extensions

# Intel RealSense SDK (optional — for realsense.launch.xml)
sudo apt install -y librealsense2-dev librealsense2-udev-rules
```

---

## Build

### Standard (webcam only)

```bash
cd ~/Documents/vtol/vtol\ aruco/fiducial_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select fiducial_detector \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### With Intel RealSense support

```bash
colcon build --packages-select fiducial_detector \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DUSE_REALSENSE=ON
source install/setup.bash
```

---

## Launch Files

All launch files are pure **XML** — no Python dependency.

| Launch file | Camera source | Notes |
|---|---|---|
| `webcam.launch.xml` | HP webcam `/dev/video6` via `capture_node` | Default |
| `realsense.launch.xml` | Intel RealSense D4xx via librealsense2 | Requires `USE_REALSENSE=ON` |
| `ros_topic.launch.xml` | Existing ROS image topic | No camera node started |
| `benchmark.launch.xml` | Any topic | BENCHMARK mode |
| `calibration.launch.xml` | Any topic | Camera intrinsic calibration |

See [launch.md](launch.md) for detailed usage examples.

---

## Quick Start

```bash
# USB Webcam (HP Wide Vision HD on /dev/video6)
ros2 launch fiducial_detector webcam.launch.xml

# Specific device
ros2 launch fiducial_detector webcam.launch.xml device_id:=2

# Intel RealSense (build with USE_REALSENSE=ON first)
ros2 launch fiducial_detector realsense.launch.xml

# From existing ROS topic
ros2 launch fiducial_detector ros_topic.launch.xml \
  camera_topic:=/camera/color/image_raw

# AUTO dictionary detection (KRTI 2026 recommended)
ros2 launch fiducial_detector webcam.launch.xml \
  detection_mode:=AUTO \
  marker_size:=0.5
```

---

## Supported Dictionaries

### OpenCV ArUco

| Dictionary | Bits | Markers |
|---|---|---|
| `DICT_4X4_50` / `100` / `250` / `1000` | 4×4 | 50–1000 |
| `DICT_5X5_50` — `DICT_5X5_1000` | 5×5 | 50–1000 |
| `DICT_6X6_50` — `DICT_6X6_1000` | 6×6 | 50–1000 |
| `DICT_7X7_50` — `DICT_7X7_1000` | 7×7 | 50–1000 |
| `DICT_ARUCO_ORIGINAL` | 5×5 | 1024 |

### AprilTag (via native AprilTag 3 backend)

| Dictionary | Family | Markers |
|---|---|---|
| `DICT_APRILTAG_16h5` | tag16h5 | 30 |
| `DICT_APRILTAG_25h9` | tag25h9 | 35 |
| `DICT_APRILTAG_36h10` | tag36h10 | 2320 |
| `DICT_APRILTAG_36h11` | tag36h11 | 587 |

### AUTO Mode Subset (KRTI 2026)

AUTO mode tries these dictionaries automatically:
`DICT_4X4_50`, `DICT_4X4_100`, `DICT_5X5_50`, `DICT_5X5_100`, `DICT_6X6_50`, `DICT_6X6_100`, `DICT_6X6_250`, `DICT_APRILTAG_36h11`

---

## ROS 2 Parameters

### Core

| Parameter | Type | Default | Description |
|---|---|---|---|
| `camera_topic` | string | `/camera/image_raw` | Input image topic |
| `marker_size` | double | `0.05` | Physical side length (metres) |
| `dictionary_type` | string | `DICT_4X4_50` | Dictionary name or `AUTO` |
| `detection_mode` | string | `SINGLE` | `SINGLE`/`AUTO`/`MULTI`/`BENCHMARK` |
| `show_window` | bool | `true` | OpenCV GUI window |
| `alignment_tolerance` | int | `50` | Pixel tolerance for centering |

### Hybrid Detector

| Parameter | Type | Default | Description |
|---|---|---|---|
| `use_opencv_detector` | bool | `true` | OpenCV ArUco backend |
| `use_native_apriltag` | bool | `true` | Native AprilTag 3 backend |
| `use_detector_fusion` | bool | `true` | Dual-backend fusion scoring |
| `apriltag_threads` | int | `4` | AprilTag decoder threads |
| `apriltag_decimate` | float | `1.0` | Quad decimation (1=full res) |
| `apriltag_sharpening` | double | `0.25` | Decode sharpening |
| `apriltag_max_hamming` | int | `1` | Max Hamming distance |
| `apriltag_min_margin` | double | `40.0` | Min decision margin (anti-ghost) |

### Preprocessing

| Parameter | Type | Default | Description |
|---|---|---|---|
| `enable_clahe` | bool | `true` | CLAHE contrast enhancement |
| `clahe_clip_limit` | double | `2.0` | CLAHE clip limit |
| `enable_sharpen` | bool | `false` | Unsharp mask |
| `enable_blur` | bool | `false` | Gaussian blur |

### capture_node Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `source` | string | `webcam` | `webcam` or `realsense` |
| `device_id` | int | `6` | V4L2 device index |
| `device_path` | string | `""` | `/dev/videoX` path (overrides `device_id`) |
| `width` / `height` | int | `640`/`480` | Resolution |
| `fps_limit` | double | `30.0` | Max capture FPS |

---

## Published Topics

| Topic | Type | Description |
|---|---|---|
| `/fiducial/pose` | `geometry_msgs/PoseStamped` | 6DOF pose of primary marker |
| `/fiducial/debug_image` | `sensor_msgs/Image` | Annotated frame |
| `/fiducial/alignment` | `std_msgs/String` | `POSISI_CENTERING` / `GESER_KIRI` / etc. |
| `/fiducial/fps` | `std_msgs/Float32` | Current detection FPS |
| `/fiducial/current_dict` | `std_msgs/String` | Active dictionary name (AUTO mode) |

---

## Generate Markers

ArUco markers from [chev.me/arucogen](https://chev.me/arucogen/) — recommended for KRTI 2026.

AprilTag images: [github.com/AprilRobotics/apriltag-imgs](https://github.com/AprilRobotics/apriltag-imgs)

Print at physical size matching `marker_size` parameter (e.g. 500mm = `marker_size:=0.5`).

---

## Troubleshooting

| Problem | Solution |
|---|---|
| `v4l2_camera` crashes Z16 error | Use `webcam.launch.xml` (C++ `capture_node`) instead |
| Camera not found | `ls /dev/video*` — use `device_id:=N` arg |
| `libapriltag.h` not found | `sudo apt install libapriltag-dev` |
| RealSense not opening | Build with `-DUSE_REALSENSE=ON` |
| Low FPS on Jetson | Set `apriltag_decimate:=2.0`, `show_window:=false` |
| `solvePnP` failed | Run `calibration.launch.xml` first |
| Ghost AprilTag detections | Increase `apriltag_min_margin` to `60`–`80` |
