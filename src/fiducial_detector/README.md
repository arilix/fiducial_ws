# fiducial_detector — ROS 2 Humble Hybrid Multi-Marker Detection System

Real-time detection of **ArUco**, **AprilTag 3**, **ChArUco**, and **GridBoard** markers
with 6DOF pose estimation, hybrid OpenCV + AprilTag3 detection, CLAHE preprocessing, and VTOL alignment guidance.

---

## System Architecture

```
[USB Webcam / Intel RealSense / ROS Topic]
         │
         ▼  /camera/image_raw
  ┌──────────────────────────────────┐
  │  aruco_node (FiducialDetector)   │
  │  ├─ preprocessFrame()           │
  │  │   ├─ CLAHE enhancement       │
  │  │   ├─ Optional blur           │
  │  │   └─ Optional sharpen        │
  │  ├─ HybridDetector              │
  │  │   ├─ OpenCV ArUco backend    │
  │  │   ├─ AprilTag 3 backend      │
  │  │   ├─ Auto-selection          │
  │  │   └─ Fusion scoring          │
  │  ├─ detectCharuco()             │
  │  ├─ detectBoard()               │
  │  ├─ estimatePoses()             │
  │  ├─ computeConfidence()         │
  │  └─ renderAnnotations()         │
  └──────────────────────────────────┘
         │
   ┌─────┼──────────────────────┐
   ▼     ▼                      ▼
/fiducial/pose   /fiducial/debug_image
/fiducial/alignment        /fiducial/fps
```

### Hybrid Detection Pipeline

```
Dictionary Selection
       │
       ├── DICT_APRILTAG_*  → Native AprilTag 3 detector (libapriltag3)
       ├── DICT_4X4_*       → OpenCV ArUco detector
       ├── DICT_5X5_*       → OpenCV ArUco detector
       ├── DICT_6X6_*       → OpenCV ArUco detector
       ├── DICT_7X7_*       → OpenCV ArUco detector
       └── FUSION mode      → Both backends + weighted scoring
                               final = opencv×0.4 + apriltag×0.6
```

---

## Prerequisites

```bash
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

sudo apt install -y \
  libopencv-dev \
  python3-opencv \
  libeigen3-dev \
  libapriltag-dev \
  libapriltag3

# Intel RealSense (optional)
sudo apt install -y \
  ros-humble-realsense2-camera \
  ros-humble-realsense2-description

sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep
```

---

## Build

```bash
cd ~/Documents/vtol/vtol\ aruco/fiducial_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select fiducial_detector
source install/setup.bash
```

---

## Execution Modes

### A — USB Webcam (V4L2)

```bash
source ~/Documents/vtol/vtol\ aruco/fiducial_ws/install/setup.bash
ros2 launch fiducial_detector webcam.launch.py device:=/dev/video0
```

### B — ROS 2 Camera Topic

```bash
ros2 launch fiducial_detector ros_topic.launch.py \
  camera_topic:=/camera/color/image_raw \
  marker_size:=0.05
```

### C — Intel RealSense

```bash
ros2 launch fiducial_detector realsense.launch.py \
  marker_size:=0.05 \
  dictionary_type:=DICT_4X4_50
```

### D — Standalone

```bash
ros2 run fiducial_detector aruco_node \
  --ros-args -p camera_topic:=/camera/image_raw -p marker_size:=0.05
```

### E — AprilTag 36h11 Detection (Native)

```bash
ros2 launch fiducial_detector realsense.launch.py \
  dictionary_type:=DICT_APRILTAG_36h11 \
  marker_size:=0.05
```

### F — AprilTag 16h5 Detection

```bash
ros2 launch fiducial_detector realsense.launch.py \
  dictionary_type:=DICT_APRILTAG_16h5 \
  marker_size:=0.05
```

---

## Supported Dictionaries

### OpenCV ArUco (via OpenCV backend)

| Dictionary | Marker Bits | Total Markers | Border Bits |
|---|---|---|---|
| `DICT_4X4_50` | 4×4 | 50 | 1 |
| `DICT_4X4_100` | 4×4 | 100 | 1 |
| `DICT_4X4_250` | 4×4 | 250 | 1 |
| `DICT_4X4_1000` | 4×4 | 1000 | 1 |
| `DICT_5X5_50` — `DICT_5X5_1000` | 5×5 | 50–1000 | 1 |
| `DICT_6X6_50` — `DICT_6X6_1000` | 6×6 | 50–1000 | 1 |
| `DICT_7X7_50` — `DICT_7X7_1000` | 7×7 | 50–1000 | 1 |
| `DICT_ARUCO_ORIGINAL` | 5×5 | 1024 | 1 |

### AprilTag (via native AprilTag 3 backend)

| Dictionary | Family | Total Markers | Border Bits |
|---|---|---|---|
| `DICT_APRILTAG_16h5` | tag16h5 | 30 | 2 |
| `DICT_APRILTAG_25h9` | tag25h9 | 35 | 2 |
| `DICT_APRILTAG_36h10` | tag36h10 | 2320 | 2 |
| `DICT_APRILTAG_36h11` | tag36h11 | 587 | 2 |

### Extended AprilTag Families (native only)

| Family | Description |
|---|---|
| `tagStandard41h12` | Recommended — higher density |
| `tagStandard52h13` | Maximum tag count |
| `tagCircle21h7` | Circular tags |
| `tagCircle49h12` | Larger circular tags |
| `tagCustom48h12` | Nested/recursive tags |

---

## ROS 2 Parameters

### Core Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `marker_size` | double | `0.05` | Physical marker side length (meters) |
| `camera_topic` | string | `/camera/image_raw` | Camera image topic |
| `dictionary_type` | string | `DICT_4X4_50` | ArUco/AprilTag dictionary |
| `detection_mode` | string | `SINGLE` | `SINGLE`/`AUTO`/`MULTI`/`BENCHMARK` |
| `show_window` | bool | `true` | Show OpenCV window |
| `alignment_tolerance` | int | `50` | Pixel tolerance for centering |
| `smoothing_alpha` | double | `0.4` | EMA smoothing alpha (0–1) |

### Hybrid Detector Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `use_opencv_detector` | bool | `true` | Enable OpenCV ArUco backend |
| `use_native_apriltag` | bool | `true` | Enable native AprilTag 3 backend |
| `use_detector_fusion` | bool | `true` | Enable dual-backend fusion scoring |
| `apriltag_family` | string | `tag36h11` | AprilTag family for native detector |
| `apriltag_threads` | int | `4` | AprilTag detector threads |
| `apriltag_decimate` | float | `1.0` | Quad decimation (1.0=full res, 2.0=half) |
| `apriltag_blur` | float | `0.0` | Gaussian blur sigma for quad detection |
| `apriltag_refine_edges` | bool | `true` | Snap to strong gradients |
| `apriltag_sharpening` | double | `0.25` | Decode image sharpening |
| `apriltag_debug` | bool | `false` | Write debug images to CWD |

### Preprocessing Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `enable_clahe` | bool | `true` | CLAHE contrast enhancement |
| `clahe_clip_limit` | double | `2.0` | CLAHE clip limit |
| `enable_sharpen` | bool | `false` | Unsharp mask sharpening |
| `enable_blur` | bool | `false` | Gaussian blur (3×3) |

---

## Published Topics

| Topic | Type | Description |
|---|---|---|
| `/fiducial/pose` | geometry_msgs/PoseStamped | 6DOF pose of marker[0] |
| `/fiducial/debug_image` | sensor_msgs/Image | Annotated frame |
| `/fiducial/alignment` | std_msgs/String | Alignment status |
| `/fiducial/fps` | std_msgs/Float32 | Current FPS |

---

## Generate Markers

### ArUco Marker

```python
import cv2

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
for marker_id in range(5):
    img = cv2.aruco.generateImageMarker(dictionary, marker_id, 300)
    cv2.imwrite(f"aruco_{marker_id}.png", img)
```

### AprilTag Marker

Download tag images from: https://github.com/AprilRobotics/apriltag-imgs

Recommended families:
- **tag36h11** — Best balance of robustness and range
- **tagStandard41h12** — Higher data density

Print at physical size matching `marker_size` parameter (e.g., 50mm × 50mm for `marker_size:=0.05`).

---

## Tuning Tips

### Meningkatkan Jarak Deteksi

```yaml
apriltag_decimate: 1.0    # Jangan turunkan resolusi
apriltag_sharpening: 0.5  # Tingkatkan sharpening
enable_clahe: true        # Aktifkan CLAHE
clahe_clip_limit: 3.0     # Tingkatkan kontras
```

### Meningkatkan Kecepatan (Jetson/Mini-PC)

```yaml
apriltag_decimate: 2.0    # Deteksi quad di resolusi setengah
apriltag_threads: 4       # Sesuaikan jumlah CPU core
show_window: false        # Matikan GUI di headless
enable_clahe: false       # Kurangi preprocessing overhead
```

---

## Troubleshooting

| Problem | Solution |
|---|---|
| AprilTag not detected | Pastikan `dictionary_type:=DICT_APRILTAG_36h11` |
| ArUco markers not detected | Periksa `dictionary_type` sesuai marker |
| `apriltag.h` not found | `sudo apt install libapriltag-dev` |
| Low FPS on Jetson | Set `apriltag_decimate=2.0`, `enable_clahe=false` |
| Camera not found | `ls /dev/video*` dan update `device` launch arg |
| `solvePnP failed` | Set `camera_matrix` dari kalibrasi kamera |
| Build error: OpenCV | `sudo apt install libopencv-dev` |
