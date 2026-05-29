# Panduan Launch — fiducial_detector (ROS 2 Humble)

Semua launch file menggunakan format **XML** (tanpa Python). Pipeline kamera sepenuhnya C++.

---

## 1. Build

```bash
cd ~/Documents/vtol/vtol\ aruco/fiducial_ws
source /opt/ros/humble/setup.bash

# Tanpa RealSense
colcon build --packages-select fiducial_detector \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

# Dengan Intel RealSense
colcon build --packages-select fiducial_detector \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DUSE_REALSENSE=ON

source install/setup.bash
```

---

## 2. Daftar Launch File

| File | Keterangan |
|---|---|
| `webcam.launch.xml` | USB webcam via C++ `capture_node` |
| `realsense.launch.xml` | Intel RealSense via librealsense2 SDK |
| `ros_topic.launch.xml` | Subscribe topic ROS existing |
| `benchmark.launch.xml` | Mode BENCHMARK (profiling performa) |
| `calibration.launch.xml` | Kalibrasi kamera via ChArUco board |

---

## 3. webcam.launch.xml

```bash
# Default (HP Wide Vision HD — /dev/video6)
ros2 launch fiducial_detector webcam.launch.xml

# Device lain berdasarkan index
ros2 launch fiducial_detector webcam.launch.xml device_id:=2

# Device berdasarkan path
ros2 launch fiducial_detector webcam.launch.xml device_path:=/dev/video4

# Dengan marker KRTI 2026
ros2 launch fiducial_detector webcam.launch.xml \
  marker_size:=0.5 \
  dictionary_type:=DICT_6X6_100 \
  detection_mode:=AUTO

# Resolusi tinggi
ros2 launch fiducial_detector webcam.launch.xml \
  width:=1280 height:=720 fps_limit:=30.0
```

**Arguments:**

| Argument | Default | Keterangan |
|---|---|---|
| `device_id` | `6` | V4L2 index (`/dev/video6`) |
| `device_path` | `""` | Path eksplisit, override `device_id` |
| `width` / `height` | `640`/`480` | Resolusi kamera |
| `fps_limit` | `30.0` | Batas FPS capture |
| `marker_size` | `0.05` | Ukuran fisik marker (metre) |
| `dictionary_type` | `DICT_4X4_50` | Nama dictionary atau `AUTO` |
| `detection_mode` | `AUTO` | `SINGLE`/`AUTO`/`MULTI`/`BENCHMARK` |
| `show_window` | `true` | Tampilkan jendela OpenCV |

---

## 4. realsense.launch.xml

> Build terlebih dahulu dengan: `colcon build --cmake-args -DUSE_REALSENSE=ON`

```bash
# Default 640x480 @ 30fps
ros2 launch fiducial_detector realsense.launch.xml

# Resolusi tinggi
ros2 launch fiducial_detector realsense.launch.xml \
  width:=1280 height:=720

# Dengan marker KRTI 2026
ros2 launch fiducial_detector realsense.launch.xml \
  marker_size:=0.5 \
  dictionary_type:=AUTO \
  detection_mode:=AUTO
```

**Arguments:**

| Argument | Default | Keterangan |
|---|---|---|
| `width` / `height` | `640`/`480` | Resolusi color stream |
| `fps_limit` | `30.0` | FPS color stream |
| `marker_size` | `0.05` | Ukuran fisik marker (metre) |
| `dictionary_type` | `DICT_4X4_50` | Nama dictionary atau `AUTO` |
| `detection_mode` | `AUTO` | Mode deteksi |
| `show_window` | `true` | Tampilkan jendela OpenCV |

---

## 5. ros_topic.launch.xml

Digunakan saat kamera sudah berjalan sebagai node terpisah (misal `realsense2_camera`).

```bash
# Default topic
ros2 launch fiducial_detector ros_topic.launch.xml

# Intel RealSense via realsense2_camera package
ros2 launch fiducial_detector ros_topic.launch.xml \
  camera_topic:=/camera/camera/color/image_raw

# Kamera custom
ros2 launch fiducial_detector ros_topic.launch.xml \
  camera_topic:=/my_cam/image_raw \
  marker_size:=0.1 \
  dictionary_type:=DICT_APRILTAG_36h11
```

---

## 6. benchmark.launch.xml

```bash
# Benchmark pada topic default
ros2 launch fiducial_detector benchmark.launch.xml

# Benchmark pada topic lain
ros2 launch fiducial_detector benchmark.launch.xml \
  camera_topic:=/camera/image_raw
```

Output benchmark mencakup FPS rata-rata, latency, dan jumlah marker terdeteksi per dictionary.

---

## 7. calibration.launch.xml

```bash
# Kalibrasi default (ChArUco 7x5)
ros2 launch fiducial_detector calibration.launch.xml

# Simpan ke file custom
ros2 launch fiducial_detector calibration.launch.xml \
  output_yaml:=/home/arilix/calib_realsense.yaml

# Dengan kamera RealSense
ros2 launch fiducial_detector calibration.launch.xml \
  camera_topic:=/camera/camera/color/image_raw \
  output_yaml:=/home/arilix/calib_realsense.yaml \
  min_frames:=20

# Board ukuran berbeda
ros2 launch fiducial_detector calibration.launch.xml \
  charuco_cols:=9 charuco_rows:=6 \
  charuco_sq:=0.025 charuco_mk:=0.0125
```

**Keyboard control di jendela kalibrasi:**

| Key | Aksi |
|---|---|
| `SPACE` | Capture frame kalibrasi |
| `s` | Simpan dan selesai |
| `q` / `ESC` | Keluar tanpa menyimpan |

---

## 8. Mode Standalone (Tanpa Launch File)

```bash
# ArUco standar
ros2 run fiducial_detector aruco_node --ros-args \
  -p camera_topic:=/camera/image_raw \
  -p marker_size:=0.05 \
  -p dictionary_type:=DICT_4X4_50 \
  -p detection_mode:=AUTO

# AprilTag 36h11
ros2 run fiducial_detector aruco_node --ros-args \
  -p camera_topic:=/camera/image_raw \
  -p dictionary_type:=DICT_APRILTAG_36h11 \
  -p use_native_apriltag:=true \
  -p apriltag_threads:=4 \
  -p marker_size:=0.05

# Mode FUSION (kedua backend aktif)
ros2 run fiducial_detector aruco_node --ros-args \
  -p camera_topic:=/camera/image_raw \
  -p use_opencv_detector:=true \
  -p use_native_apriltag:=true \
  -p use_detector_fusion:=true \
  -p marker_size:=0.05
```

---

## 9. Memantau Output

```bash
# Pose marker
ros2 topic echo /fiducial/pose

# Status alignment (POSISI_CENTERING / GESER_KIRI / MAJU / dll)
ros2 topic echo /fiducial/alignment

# FPS
ros2 topic echo /fiducial/fps

# Dictionary aktif (AUTO mode)
ros2 topic echo /fiducial/current_dict

# Visualisasi gambar debug
ros2 run rqt_image_view rqt_image_view
# Pilih: /fiducial/debug_image
```

---

## 10. Tuning

### Jarak Jauh / Outdoor

```yaml
apriltag_decimate: 1.0
apriltag_sharpening: 0.5
enable_clahe: true
clahe_clip_limit: 3.0
apriltag_min_margin: 40.0
```

### Headless / Jetson / Mini-PC

```yaml
apriltag_decimate: 2.0
apriltag_threads: 4
show_window: false
enable_clahe: false
```

### Anti Ghost-Detection AprilTag

```yaml
apriltag_max_hamming: 1
apriltag_min_margin: 60.0
```

---

## 11. Identifikasi Device Kamera

```bash
# List semua video device
ls /dev/video*

# Cek format pixel per device
v4l2-ctl -d /dev/video6 --list-formats-ext

# Cek nama kamera
v4l2-ctl -d /dev/video6 --info | grep "Card type"
```

Cari device dengan format `YUYV` atau `MJPG` — itu kamera RGB.
Device dengan `Z16` adalah depth stream RealSense (tidak kompatibel).
