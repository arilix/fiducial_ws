# fiducial_detector — Unified Realtime Fiducial Perception Framework

Real-time detection of **ArUco**, **AprilTag**, **ChArUco**, dan **GridBoard** markers
dengan 6DOF pose estimation, hybrid backend (OpenCV + AprilTag3), dan alignment guidance untuk precision landing VTOL.

---

## Arsitektur Sistem

```
┌──────────────────────────────────────────────────────────────┐
│  aruco_node (FiducialDetector)                               │
│                                                              │
│  [V4L2 Webcam / Intel RealSense / ROS Topic]                │
│          │                                                   │
│          ▼ imageCallback()                                   │
│  ┌────────────────────────┐                                  │
│  │  HybridDetector        │  ← OpenCV ArUco (FUSION)        │
│  │  FUSION backend        │  ← AprilTag3 (libapriltag)      │
│  └────────────────────────┘                                  │
│          │                                                   │
│  ┌────────────────────────┐                                  │
│  │  PoseEstimator         │  ← solvePnP IPPE_SQUARE         │
│  │  (6DOF per marker)     │  ← Quaternion + distance        │
│  └────────────────────────┘                                  │
│          │                                                   │
│  ┌────────────────────────┐                                  │
│  │  Visualizer + Logger   │  ← Frame annotated + FPS label  │
│  │  Position guidance     │  ← Arahan posisi di terminal    │
│  │  Centering detection   │  ← POSISI CENTERING alert       │
│  └────────────────────────┘                                  │
└──────────────────────────────────────────────────────────────┘
         │              │              │              │
   /fiducial/     /fiducial/     /fiducial/     /fiducial/
      pose       debug_image    alignment          fps
```

---

## Instalasi Dependensi

```bash
# ROS 2 Humble core
sudo apt update && sudo apt install -y \
  ros-humble-rclcpp \
  ros-humble-sensor-msgs \
  ros-humble-geometry-msgs \
  ros-humble-std-msgs \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-tf2 \
  ros-humble-tf2-geometry-msgs

# OpenCV + AprilTag3 (wajib)
sudo apt install -y \
  libopencv-dev \
  libeigen3-dev \
  libapriltag-dev \
  libapriltag3

# Alat build
sudo apt install -y python3-colcon-common-extensions python3-rosdep

# Intel RealSense (opsional — tidak perlu build flag khusus)
sudo apt install -y ros-humble-realsense2-camera
```

---

## Build

```bash
cd ~/Documents/vtol/vtol\ aruco/fiducial_ws
source /opt/ros/humble/setup.bash

# Standard build — sudah termasuk dukungan semua sumber kamera
colcon build --packages-select fiducial_detector \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

> **Tidak perlu** flag `-DUSE_REALSENSE=ON`. RealSense kini menggunakan
> paket `realsense2_camera` resmi (ROS2 driver terpisah).

---

## Cara Menjalankan

Source workspace dulu di setiap terminal baru:

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/vtol/vtol\ aruco/fiducial_ws/install/setup.bash
```

### ⭐ all_in_one — Webcam Bawaan Laptop (Direkomendasikan)

Satu node, satu perintah, tanpa `capture_node` terpisah:

```bash
# Default: DICT_7X7_50, /dev/video0, tanpa GUI
ros2 launch fiducial_detector all_in_one.launch.xml

# Dengan jendela preview OpenCV (show_window fix aktif)
ros2 launch fiducial_detector all_in_one.launch.xml show_window:=true

# Pilih device kamera lain
ros2 launch fiducial_detector all_in_one.launch.xml capture_device_id:=2

# Via path eksplisit
ros2 launch fiducial_detector all_in_one.launch.xml \
  capture_device_path:=/dev/video4

# Resolusi + dictionary custom
ros2 launch fiducial_detector all_in_one.launch.xml \
  capture_width:=1280 capture_height:=720 \
  dictionary_type:=DICT_4X4_50
```

### Intel RealSense D4xx

Tidak memerlukan build flag khusus — menggunakan paket `realsense2_camera` yang sudah ter-install.

```bash
# Default: DICT_7X7_50, 640×480 @ 30fps
ros2 launch fiducial_detector realsense.launch.xml

# Dengan preview window
ros2 launch fiducial_detector realsense.launch.xml show_window:=true

# Resolusi tinggi
ros2 launch fiducial_detector realsense.launch.xml \
  width:=1280 height:=720 fps_limit:=30

# Marker ukuran besar (misal landing pad KRTI)
ros2 launch fiducial_detector realsense.launch.xml \
  marker_size:=0.5 dictionary_type:=DICT_7X7_50
```

> Camera topic RealSense: `/camera/camera/color/image_raw`
> Gunakan `detection_mode:=SINGLE` (default) atau `MULTI`.
> `AUTO` mode tidak direkomendasikan dengan RealSense.

### USB Webcam via capture_node terpisah

```bash
ros2 launch fiducial_detector webcam.launch.xml
ros2 launch fiducial_detector webcam.launch.xml device_id:=0 show_window:=true
```

### Via ROS Topic yang sudah ada

```bash
ros2 launch fiducial_detector ros_topic.launch.xml \
  camera_topic:=/camera/image_raw
```

---

## Output Terminal — Format Log

Setiap frame yang mendeteksi marker mencetak log lengkap ke terminal:

```
[Frame 418] 4 marker(s) | FPS=29.8 | Lat=33.5ms
  [0] Family=DICT_7X7_50    ID=2    Conf=0%   Backend=OPENCV     FPS=29.8  Center=(390,360)
       Pose=(0.028, 0.055, 0.305)m  Dist=0.311m
       Posisi ID=2: GESER_KIRI|MAJU
  [1] Family=DICT_7X7_50    ID=3    Conf=0%   Backend=OPENCV     FPS=29.8  Center=(499,313)
       Pose=(0.085, 0.035, 0.325)m  Dist=0.337m
       Posisi ID=3: GESER_KANAN|MAJU
  [2] Family=DICT_7X7_50    ID=4    Conf=0%   Backend=OPENCV     FPS=29.8  Center=(332,240)
       Pose=(0.001, -0.019, 0.305)m  Dist=0.305m
  *** POSISI CENTERING *** — ID=4 tepat di tengah kamera
  [3] Family=DICT_7X7_50    ID=1    Margin=72.3  Backend=APRILTAG3  FPS=29.8  Center=(442,170)
       Pose=(0.057, -0.038, 0.329)m  Dist=0.336m
       Posisi ID=1: GESER_KANAN|MUNDUR
```

**Keterangan kolom:**

| Kolom | Keterangan |
|---|---|
| `[Frame N]` | Nomor frame sejak startup |
| `FPS=xx.x` | Frame rate deteksi realtime (rolling window 60 frame) |
| `Lat=xx.xms` | Latency antar frame (ms) |
| `Family` | Dictionary yang aktif |
| `ID` | Nomor ID marker yang terdeteksi |
| `Conf` | Confidence deteksi OpenCV (0–100%) |
| `Margin` | Decision margin AprilTag — semakin tinggi semakin yakin |
| `Hamming` | Bit error yang dikoreksi (0=sempurna, 1=1 bit error) |
| `Backend` | `OPENCV` / `APRILTAG3` / `FUSION` |
| `Center=(x,y)` | Koordinat pixel pusat marker dalam frame |
| `Pose=(X,Y,Z)m` | Posisi marker relatif kamera dalam metre |
| `Dist=x.xxxm` | Jarak Euclidean kamera→marker dalam metre |
| `Posisi ID=N` | Arahan: `GESER_KIRI`, `GESER_KANAN`, `MAJU`, `MUNDUR`, kombinasi, atau `*** POSISI CENTERING ***` |

**Nilai arahan posisi:**

| Nilai | Artinya |
|---|---|
| `GESER_KIRI` | Marker terlalu jauh ke kiri — geser kamera ke kiri |
| `GESER_KANAN` | Marker terlalu jauh ke kanan — geser kamera ke kanan |
| `MAJU` | Marker terlalu jauh ke bawah frame — maju/turun |
| `MUNDUR` | Marker terlalu jauh ke atas frame — mundur/naik |
| `GESER_KIRI\|MAJU` | Kombinasi dua arah |
| `*** POSISI CENTERING ***` | Marker tepat di tengah, dalam toleransi `alignment_tolerance` (default 50px) |

---

## Preview Window (show_window:=true)

Saat `show_window:=true`, jendela **"Fiducial Detector"** muncul menampilkan:

- **Kotak deteksi** berwarna per jenis marker (hijau=ArUco, biru=AprilTag)
- **Axis pose** 3D (X merah, Y hijau, Z biru) di setiap marker yang ter-estimasi
- **Label `FPS: xx.x`** berwarna kuning di atas setiap marker terdeteksi
- **Crosshair + kotak toleransi** di tengah frame
- **Teks arahan** (GESER KIRI / KANAN / MAJU / TURUN / POSISI CENTERING) di tepi frame

### Keyboard Shortcut

| Key | Fungsi |
|---|---|
| `ESC` | Keluar dari program |
| `d` | Toggle semua debug windows (Cells, Threshold, Contours, Rejected) |
| `c` | Toggle Marker Cells window |
| `t` | Toggle Threshold window |
| `n` | Toggle Contours window |
| `r` | Toggle Rejected candidates window |

> **Tanpa display?** Gunakan `rqt_image_view /fiducial/debug_image` di terminal lain
> untuk preview gambar annotated tanpa membuka window di node itu sendiri.

---

## Published Topics

| Topic | Type | Deskripsi |
|---|---|---|
| `/fiducial/pose` | `geometry_msgs/PoseStamped` | 6DOF pose marker pertama yang valid |
| `/fiducial/debug_image` | `sensor_msgs/Image` | Frame bgr8 dengan semua overlay |
| `/fiducial/alignment` | `std_msgs/String` | Status alignment: `POSISI_CENTERING` / `GESER_KIRI` / dll |
| `/fiducial/fps` | `std_msgs/Float32` | FPS realtime, di-publish setiap 1 detik |
| `/fiducial/rejected_candidates` | `std_msgs/String` | JSON: `{"rejected_count":N,"detected_count":M}` |

```bash
# Monitor semua output sekaligus (buka terminal terpisah tiap baris)
ros2 topic echo /fiducial/alignment
ros2 topic echo /fiducial/pose
ros2 topic echo /fiducial/fps
ros2 run rqt_image_view rqt_image_view /fiducial/debug_image
```

---

## Dictionary yang Didukung

**ArUco (OpenCV):**
```
DICT_4X4_50    DICT_4X4_100    DICT_4X4_250    DICT_4X4_1000
DICT_5X5_50    DICT_5X5_100    DICT_5X5_250    DICT_5X5_1000
DICT_6X6_50    DICT_6X6_100    DICT_6X6_250    DICT_6X6_1000
DICT_7X7_50    DICT_7X7_100    DICT_7X7_250    DICT_7X7_1000
DICT_ARUCO_ORIGINAL    DICT_ARUCO_MIP_36h12
```

**AprilTag (via libapriltag3):**
```
tag16h5    tag25h9    tag36h10    tag36h11
tagStandard41h12    tagStandard52h13
tagCircle21h7       tagCircle49h12    tagCustom48h12
```

> **Default project:** `DICT_7X7_50` — direkomendasikan untuk VTOL landing.

---

## Matematika Pose Estimation

### PnP (Perspective-n-Point)

Minimisasi reprojection error keempat sudut marker:

$$\min_{R,\, t} \sum_{i=1}^{4} \left\| u_i - \pi(K,\, D,\, R\,X_i + t) \right\|^2$$

- $K$ = matriks intrinsik kamera (fx, fy, cx, cy)
- $D$ = koefisien distorsi
- $\pi$ = fungsi proyeksi perspektif
- Algoritma: **IPPE_SQUARE** — optimal untuk marker planar persegi

### Jarak Euclidean

$$d = \sqrt{t_x^2 + t_y^2 + t_z^2}$$

### Alignment Logic (Precision Landing)

Toleransi $\tau$ = `alignment_tolerance` (default **50 px**):

$$\Delta x = c_x - \tfrac{W}{2}, \quad \Delta y = c_y - \tfrac{H}{2}$$

| Kondisi | Output terminal | Output topic |
|---|---|---|
| $\|\Delta x\| \le \tau$ dan $\|\Delta y\| \le \tau$ | `*** POSISI CENTERING ***` | `POSISI_CENTERING` |
| $\Delta x < -\tau$ | `Posisi ID=N: GESER_KIRI` | `GESER_KIRI` |
| $\Delta x > +\tau$ | `Posisi ID=N: GESER_KANAN` | `GESER_KANAN` |
| $\Delta y < -\tau$ | `Posisi ID=N: MUNDUR` | `MUNDUR` |
| $\Delta y > +\tau$ | `Posisi ID=N: MAJU` | `MAJU` |

---

## Generate Marker

### Python (DICT_7X7_50)

```python
import cv2

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_50)

for marker_id in [1, 2, 3, 4]:
    img = cv2.aruco.generateImageMarker(dictionary, marker_id, 500)
    cv2.imwrite(f"marker_7x7_id{marker_id}.png", img)
    print(f"Saved marker_7x7_id{marker_id}.png")
```

Print pada ukuran fisik sesuai `marker_size` yang diset (default 5 cm × 5 cm).

### Menggunakan dict_test_node (CLI)

```bash
ros2 run fiducial_detector dict_test_node
```

---

## Kalibrasi Kamera

Intrinsik default (placeholder) di `config/detector.yaml`:
```yaml
camera_matrix: [640, 0, 320, 0, 640, 240, 0, 0, 1]
dist_coeffs:   [0, 0, 0, 0, 0]
```

Untuk akurasi pose terbaik, lakukan kalibrasi:

```bash
ros2 launch fiducial_detector calibration.launch.xml \
  camera_topic:=/camera/image_raw \
  output_yaml:=/home/arilix/camera_calib.yaml

# Keyboard: SPACE=capture, s=simpan, ESC=keluar
```

Salin hasil `camera_matrix` dan `dist_coeffs` ke `config/detector.yaml`, lalu rebuild.

---

## Troubleshooting

| Problem | Solusi |
|---|---|
| Jendela tidak muncul (`show_window:=true`) | Pastikan environment punya `DISPLAY`. Cek dengan `echo $DISPLAY` |
| `show_window:=true` diabaikan | Bug lama sudah diperbaiki — pastikan sudah rebuild setelah update |
| `exit code -11` (SIGSEGV) saat RealSense | Gunakan `detection_mode:=SINGLE` (default), bukan `AUTO` |
| Marker tidak terdeteksi | Cek pencahayaan. Pastikan `dictionary_type` sesuai marker yang dicetak |
| FPS rendah | Kurangi resolusi, atau aktifkan `apriltag_decimate:=2.0` |
| Kamera tidak ditemukan | `ls /dev/video*` → sesuaikan `capture_device_id` |
| `libapriltag.so` not found | `sudo apt install libapriltag-dev libapriltag3` |
| `realsense2_camera_node` tidak ada | `sudo apt install ros-humble-realsense2-camera` |
| Pose tidak akurat | Kalibrasi kamera, update `camera_matrix` di `detector.yaml` |
| Preview tidak muncul (headless) | `ros2 run rqt_image_view rqt_image_view /fiducial/debug_image` |
| Build error `apriltag.h` | `sudo apt install libapriltag-dev` |

---

## Dokumentasi Lengkap

Panduan lengkap semua launch file, parameter, dan tuning:
[`src/fiducial_detector/launch.md`](src/fiducial_detector/launch.md)
