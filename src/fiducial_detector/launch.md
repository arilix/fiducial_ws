# Panduan Menjalankan Fiducial Detector (ROS 2 Humble)

Panduan singkat untuk menjalankan *node* pendeteksi multi-marker hybrid (`fiducial_detector`) dengan dukungan OpenCV ArUco + AprilTag 3 native.

## 1. Kompilasi (Build)

```bash
cd ~/Documents/vtol/vtol\ aruco/fiducial_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select fiducial_detector
```

## 2. Source Setup

```bash
cd ~/Documents/vtol/vtol\ aruco/fiducial_ws
source install/setup.bash
```

## 3. Menjalankan Deteksi ArUco Standar

### USB Webcam

```bash
ros2 launch fiducial_detector webcam.launch.py device:=/dev/video0
```

### Intel RealSense

```bash
ros2 launch fiducial_detector realsense.launch.py marker_size:=0.05
```

### ROS 2 Topic Eksternal

```bash
ros2 launch fiducial_detector ros_topic.launch.py \
  camera_topic:=/camera/color/image_raw \
  marker_size:=0.05
```

---

## 4. Menjalankan Deteksi AprilTag (Native AprilTag 3)

Sistem secara otomatis menggunakan native AprilTag 3 detector saat dictionary AprilTag dipilih.

### AprilTag 36h11 (Rekomendasi)

```bash
ros2 launch fiducial_detector realsense.launch.py \
  dictionary_type:=DICT_APRILTAG_36h11 \
  marker_size:=0.05
```

### AprilTag 16h5

```bash
ros2 launch fiducial_detector realsense.launch.py \
  dictionary_type:=DICT_APRILTAG_16h5 \
  marker_size:=0.05
```

### AprilTag 25h9

```bash
ros2 launch fiducial_detector realsense.launch.py \
  dictionary_type:=DICT_APRILTAG_25h9 \
  marker_size:=0.05
```

### AprilTag 36h10

```bash
ros2 launch fiducial_detector realsense.launch.py \
  dictionary_type:=DICT_APRILTAG_36h10 \
  marker_size:=0.05
```

### Webcam + AprilTag

```bash
ros2 launch fiducial_detector webcam.launch.py \
  device:=/dev/video0 \
  dictionary_type:=DICT_APRILTAG_36h11 \
  marker_size:=0.05
```

---

## 5. Mode Standalone (Tanpa Launch File)

```bash
ros2 run fiducial_detector aruco_node --ros-args \
  -p camera_topic:=/camera/image_raw \
  -p marker_size:=0.05 \
  -p dictionary_type:=DICT_APRILTAG_36h11 \
  -p use_native_apriltag:=true \
  -p apriltag_threads:=4 \
  -p enable_clahe:=true
```

---

## 6. Mode AUTO (Auto-Detect Dictionary)

Deteksi otomatis dictionary terbaik berdasarkan marker di depan kamera:

```bash
ros2 launch fiducial_detector realsense.launch.py \
  dictionary_type:=DICT_4X4_50 \
  marker_size:=0.05
```

Sistem akan mencoba beberapa dictionary dan memilih yang paling banyak mendeteksi marker.

---

## 7. Mode Fusion (OpenCV + AprilTag3 Bersamaan)

Aktifkan fusion untuk menjalankan kedua backend sekaligus:

```bash
ros2 run fiducial_detector aruco_node --ros-args \
  -p camera_topic:=/camera/image_raw \
  -p dictionary_type:=DICT_APRILTAG_36h11 \
  -p use_opencv_detector:=true \
  -p use_native_apriltag:=true \
  -p use_detector_fusion:=true \
  -p marker_size:=0.05
```

Fusion scoring: `final_score = opencv_score × 0.4 + apriltag_score × 0.6`

---

## 8. Tuning AprilTag untuk Jarak Jauh

```bash
ros2 run fiducial_detector aruco_node --ros-args \
  -p camera_topic:=/camera/image_raw \
  -p dictionary_type:=DICT_APRILTAG_36h11 \
  -p apriltag_decimate:=1.0 \
  -p apriltag_sharpening:=0.5 \
  -p enable_clahe:=true \
  -p clahe_clip_limit:=3.0 \
  -p marker_size:=0.15
```

---

## 9. Tuning untuk Jetson / Mini-PC

```bash
ros2 run fiducial_detector aruco_node --ros-args \
  -p camera_topic:=/camera/image_raw \
  -p apriltag_decimate:=2.0 \
  -p apriltag_threads:=4 \
  -p show_window:=false \
  -p enable_clahe:=false \
  -p marker_size:=0.05
```

---

## 10. Memantau Hasil Deteksi

### Terminal

Log output otomatis menampilkan:
- Backend yang aktif (OPENCV / APRILTAG3 / FUSION)
- Jumlah marker terdeteksi
- FPS
- Status alignment (POSISI_CENTERING / GESER_KIRI / MAJU / dll)

### ROS Topics

```bash
# Lihat pose marker
ros2 topic echo /fiducial/pose

# Lihat status alignment
ros2 topic echo /fiducial/alignment

# Lihat FPS
ros2 topic echo /fiducial/fps

# Visualisasi gambar debug
ros2 run rqt_image_view rqt_image_view
# Pilih topic: /fiducial/debug_image
```

---

## 11. Dictionary Quick Reference

| Perintah Launch | Backend | Kegunaan |
|---|---|---|
| `dictionary_type:=DICT_4X4_50` | OpenCV | Marker ArUco standar |
| `dictionary_type:=DICT_5X5_250` | OpenCV | ArUco 5×5 banyak marker |
| `dictionary_type:=DICT_APRILTAG_36h11` | AprilTag3 | Paling stabil, jarak jauh |
| `dictionary_type:=DICT_APRILTAG_16h5` | AprilTag3 | Tag kecil, jumlah sedikit |
| `dictionary_type:=DICT_APRILTAG_25h9` | AprilTag3 | Balance size/count |
| `dictionary_type:=DICT_ARUCO_ORIGINAL` | OpenCV | Kompatibilitas lama |

---

## 12. Mendapatkan Marker AprilTag

Download gambar tag siap cetak dari:

https://github.com/AprilRobotics/apriltag-imgs

Pilih folder sesuai family:
- `tag36h11/` — Rekomendasi untuk VTOL
- `tag16h5/` — Ukuran kecil

Cetak pada ukuran fisik sesuai parameter `marker_size` (contoh: 50mm × 50mm untuk `marker_size:=0.05`).
