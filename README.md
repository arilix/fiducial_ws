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

## Mathematical Models & Alignment Logic

### 1. Pose Estimation (Perspective-n-Point / PnP)

Ketika sebuah marker terdeteksi, titik sudut 2D pada citra ($u_i$) dicocokkan dengan koordinat 3D fisik ($X_i$) di dunia nyata. Sistem menggunakan algoritma PnP iteratif untuk meminimalkan *reprojection error* guna mendapatkan rotasi ($R$) dan translasi ($t$) kamera relatif terhadap posisi marker:

$$ \min_{R, t} \sum_{i=1}^{4} \left\| u_i - \pi(K, D, R X_i + t) \right\|^2 $$

Dimana:
*   $K$ adalah Matriks Intrinsik Kamera:
    $$ K = \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix} $$
*   $D$ mewakili koefisien distorsi lensa.
*   $\pi$ adalah fungsi proyeksi dari 3D ke 2D.

### 2. Distance Calculation (Euclidean Distance)

Jarak absolut linear dari kamera (pesawat) menuju titik tengah marker dihitung dari vektor translasi ($t_x, t_y, t_z$):

$$ d = \sqrt{t_x^2 + t_y^2 + t_z^2} $$

### 3. Centering Alignment Logic

Untuk menuntun posisi VTOL melakukan pendaratan presisi, posisi absolut marker pada layar ($c_x, c_y$) dihitung deviasinya terhadap *center point* gambar resolusi kamera ($\frac{W}{2}, \frac{H}{2}$).

Dengan toleransi *alignment* $\tau$:
$$ \Delta x = c_x - \frac{W}{2} $$
$$ \Delta y = c_y - \frac{H}{2} $$

Aturan panduan arah (*Alignment String*):
*   Jika $\Delta x < -\tau$ $\Rightarrow$ **GESER_KIRI**
*   Jika $\Delta x > \tau$ $\Rightarrow$ **GESER_KANAN**
*   Jika $\Delta y < -\tau$ $\Rightarrow$ **MUNDUR** (Drone harus bergerak mundur)
*   Jika $\Delta y > \tau$ $\Rightarrow$ **MAJU** (Drone harus bergerak maju)
*   Jika $|\Delta x| \le \tau$ dan $|\Delta y| \le \tau$ $\Rightarrow$ **POSISI_CENTERING**

> **Catatan UI:** Antarmuka (UI) citra video dari kamera kini disederhanakan (*minimalist UI*) menggunakan *crosshair* berwarna hitam pekat, tanpa gangguan indikator tulisan. Semua informasi status arahan penjajaran (*alignment*) dan kecepatan pemrosesan (FPS) diarahkan dan dapat dibaca langsung pada konsol/terminal node ROS.

---

## 1. Instalasi Dependensi (Prerequisites)

Sistem ini membutuhkan instalasi beberapa paket sistem operasi dasar, *library computer vision*, serta komponen inti ROS 2 Humble. Jalankan perintah berikut di terminal Ubuntu 22.04 Anda:

```bash
# 1. Komponen Dasar ROS 2 Humble
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

# 2. Library Visi Komputer & Matematika
sudo apt install -y \
  libopencv-dev \
  python3-opencv \
  libeigen3-dev \
  libapriltag-dev \
  libapriltag3

# 3. Driver Intel RealSense (Opsional untuk Opsi D)
sudo apt install -y \
  ros-humble-realsense2-camera \
  ros-humble-realsense2-description

# 4. Alat Kompilasi (Build Tools)
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep
```

---

## 2. Persiapan Lingkungan Kerja (Workspace Setup)

Siapkan struktur direktori standar *colcon workspace* dan selesaikan seluruh dependensi paket secara otomatis menggunakan `rosdep`.

```bash
# Membuat direktori source
mkdir -p ~/fiducial_ws/src
# Salin folder fiducial_detector ke dalam src/
cp -r fiducial_detector ~/fiducial_ws/src/

# Inisialisasi dependensi ROS
cd ~/fiducial_ws
sudo rosdep init # Jika belum pernah dijalankan
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

---

## 3. Proses Kompilasi (Build System)

Lakukan kompilasi (*build*) kode C++ menjadi *executable node*. Kami menggunakan *flag* `--symlink-install` agar perubahan konfigurasi atau file *launch* tidak memerlukan kompilasi ulang.

```bash
cd ~/fiducial_ws
# Memuat environment ROS 2 bawaan sistem
source /opt/ros/humble/setup.bash

# Memulai kompilasi khusus untuk paket fiducial_detector
colcon build --symlink-install --packages-select fiducial_detector

# Memuat hasil kompilasi ke dalam sesi terminal saat ini
source install/setup.bash
```

---

## 4. Cara Menjalankan Node (Execution Modes)

Sistem ini bersifat sangat modular dan dapat dieksekusi melalui beberapa mode suplai gambar kamera yang berbeda.

### Opsi A — Melalui Kamera USB Lokal (V4L2)

Gunakan file *launch* ini jika Anda menggunakan kamera fisik (webcam) yang langsung terhubung ke *port* USB *host machine* komputer Anda (misalnya: `/dev/video0`).

```bash
source ~/fiducial_ws/install/setup.bash
ros2 launch fiducial_detector webcam.launch.py device:=/dev/video0
```

Anda juga dapat mengatur ukuran *marker* secara dinamis:
```bash
ros2 launch fiducial_detector webcam.launch.py \
  device:=/dev/video0 \
  marker_size:=0.08
```

### Opsi B — Berlangganan Topik Kamera ROS 2 Eksternal

Pilih metode ini apabila sistem Anda mendapatkan *feed* gambar dari node lain atau dari jaringan terdistribusi (misalnya Gazebo Simulator atau *network stream*).

```bash
ros2 launch fiducial_detector ros_topic.launch.py \
  camera_topic:=/camera/color/image_raw \
  marker_size:=0.05
```

### Opsi C — Eksekusi Langsung Secara Mandiri (Standalone)

Apabila Anda tidak ingin menggunakan sistem *launch file*, jalankan node utama `aruco_node` secara langsung sambil mendeklarasikan parameter.

```bash
ros2 run fiducial_detector aruco_node \
  --ros-args -p camera_topic:=/camera/image_raw -p marker_size:=0.05
```

### Opsi D — Kamera Intel RealSense (D435/D455 dsb)

Jika Anda menggunakan Intel RealSense (seperti yang ditunjukkan oleh `lsusb`), sistem kini menyediakan *launch file* terintegrasi. Hal ini akan menjalankan driver `realsense2_camera` secara langsung dan menghubungkannya dengan *detector node* kita.

```bash
ros2 launch fiducial_detector realsense.launch.py \
  marker_size:=0.05
```

> **Catatan:** Pastikan Anda telah menginstal `ros-humble-realsense2-camera` (bisa dilihat di panduan Instalasi Dependensi bagian #3).

---

## 5. Published Topics

| Topic                   | Type                         | Description            |
|-------------------------|------------------------------|------------------------|
| `/fiducial/pose`        | geometry_msgs/PoseStamped    | 6DOF pose of marker[0] |
| `/fiducial/debug_image` | sensor_msgs/Image            | Annotated frame        |
| `/fiducial/alignment`   | std_msgs/String              | Alignment status string |
| `/fiducial/fps`         | std_msgs/Float32             | Current FPS (1 Hz pub) |

Alignment values: `NO_MARKER`, `POSISI_CENTERING`, `GESER_KIRI`, `GESER_KANAN`,
`MUNDUR`, `MAJU`, combinations like `GESER_KIRI|MUNDUR`, etc.

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
