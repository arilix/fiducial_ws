# fiducial\_detector — DICT\_7X7\_50 Visual Gate Centering

**Production-grade DICT_7X7_50 tracking system** untuk autonomous VTOL gate centering dan navigation.  
Real-time detection, 6DOF pose estimation, gate alignment state machine, dan ROS 2 Humble.

> **KRTI 2026** — Visual gate traversal menggunakan DICT\_7X7\_50 markers (ID 0–49).  
> Validated: **50/50 PASS** · **~30 FPS** · **~33 ms latency** (HP Webcam / Intel RealSense D4xx)

---

## Status Terbaru — Board Besar + Marker Kecil

Sistem tetap memakai **DICT_7X7_50 only** dan sekarang mendukung board lomba dengan marker besar serta marker kecil yang menempel pada satu bidang.

- `detectAruco()` menjalankan deteksi normal, ROI rescue, tab rescue, dan full-frame upscale fallback.
- `detectTopTabMarkers()` mencari tab kecil di sisi marker besar, termasuk prediksi posisi **atas dan bawah**.
- Marker kecil yang berhasil diselamatkan akan menulis log: `Predictive small-tab detected ID=...`.
- Stabilizer membedakan track `ID:big` dan `ID:small`, sehingga marker besar dan kecil dengan ID sama tidak saling overwrite.
- Marker kecil ditahan lebih lama (`14 frame`) untuk mengurangi flicker saat deteksi hanya muncul intermittent.
- Rescue berat tidak dijalankan setiap frame: predictive crop berjalan berkala dan full-frame upscale hanya saat marker utama hilang, agar FPS tetap layak.
- Profil default sekarang lebih ringan untuk Jetson/UAV: `publish_debug_image=false`, `enable_split_rescue=false`, `enable_full_frame_fallback=false`, `small_marker_rescue_period=6`, `predictive_roi_limit=12`.

Catatan fisik: marker kecil tetap butuh cukup piksel. Jika terlalu jauh, kurang cahaya, atau blur, deteksi akan lebih sulit walaupun algoritma rescue aktif.

---

## System Architecture

```
[capture_node]   ←── USB Webcam / Laptop Camera (cv::VideoCapture / V4L2)
       or        ←── Intel RealSense D4xx (librealsense2 SDK, opsional)
[external topic] ←── Any ROS 2 image publisher
        │
        ▼  /camera/image_raw
┌────────────────────────────────────────────────────────┐
│  aruco_node  (FiducialDetector)                        │
│  ├─ preprocessFrame()                                  │
│  │   ├─ CLAHE contrast enhancement (CPU / GPU*)        │
│  │   └─ Optional blur / unsharp mask                   │
│  ├─ detectAruco() — DICT_7X7_50 only                   │
│  │   ├─ cv::aruco::detectMarkers() + SUBPIX refine     │
│  │   ├─ splitCombinedBoard() ROI rescue                │
│  │   ├─ detectTopTabMarkers() small-tab rescue         │
│  │   └─ full-frame upscale fallback                    │
│  ├─ estimatePoses() — solvePnP + EMA smoothing         │
│  ├─ computeConfidence() — reprojection + Hamming       │
│  └─ GateAlignmentEngine — state machine                │
│       SEARCH → DETECTED → TRACKING                     │
│       → CENTERING → ALIGNED → GATE_READY               │
│       → PASS_THROUGH                                   │
└────────────────────────────────────────────────────────┘
        │
 ┌──────┴─────────────────────────────────────────────┐
 ▼                                                     ▼
/fiducial/pose          /fiducial/alignment  (JSON)
/fiducial/debug_image   /fiducial/fps
/fiducial/rejected_candidates
```

\* GPU preprocessing memerlukan OpenCV build dengan CUDA

### Executables

| Executable | Deskripsi |
|---|---|
| `aruco_node` | Main gate centering node |
| `capture_node` | C++ camera publisher (webcam + RealSense) |
| `calibration_node` | Kalibrasi intrinsik kamera (chessboard) |
| `dict_test_node` | DICT_7X7_50 50/50 validation (ID 0–49) |

---

## Prerequisites — Install Sebelum Build

### 1. ROS 2 Humble (wajib)

```bash
sudo apt update
sudo apt install -y \
  ros-humble-rclcpp \
  ros-humble-rclcpp-components \
  ros-humble-sensor-msgs \
  ros-humble-geometry-msgs \
  ros-humble-std-msgs \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-tf2 \
  ros-humble-tf2-geometry-msgs
```

### 2. OpenCV + Eigen + Build Tools (wajib)

```bash
sudo apt install -y \
  libopencv-dev \
  libeigen3-dev \
  python3-colcon-common-extensions \
  build-essential \
  cmake
```

> Verifikasi OpenCV terinstall: `python3 -c "import cv2; print(cv2.__version__)"` → harus ≥ 4.x

### 3. Deteksi Kamera (wajib untuk webcam)

```bash
# Tool untuk identifikasi /dev/videoX
sudo apt install -y v4l-utils

# Cek kamera yang terdeteksi
ls /dev/video*
v4l2-ctl --list-devices
```

### 4. Intel RealSense SDK (opsional — hanya jika pakai RealSense)

```bash
sudo apt install -y \
  librealsense2-dev \
  librealsense2-udev-rules \
  ros-humble-realsense2-camera

# Verifikasi kamera RealSense terdeteksi
rs-enumerate-devices | head -5
```

> `libapriltag-dev` **tidak diperlukan** — AprilTag backend telah dihapus dari build ini.

---

## Build

### ⚠️ Penting: Workspace Rename & Absolute Path

> **Jika workspace di-rename atau dipindah ke device/path lain, folder `install/` HARUS dihapus dan di-rebuild ulang.**
>
> Ini terjadi karena `colcon build` menulis **absolute path** ke dalam file `install/setup.sh`,
> `install/local_setup.sh`, dan `install/*/share/.../package.sh` saat build.
> Jika path berubah, file-file ini menjadi invalid dan kamera/launch tidak bisa berjalan.
>
> **Solusi:**
> ```bash
> # Hapus install/ dan build/ lama, lalu rebuild dari awal
> rm -rf install/ build/ log/
> colcon build --packages-select fiducial_detector ...
> source install/setup.bash
> ```

---

### Standard — Webcam (paling umum)

```bash
# Masuk ke direktori workspace (sesuaikan path jika berbeda)
cd ~/Documents/vtol/vtol\ aruco/fiducial_ws

# Source ROS 2 base
source /opt/ros/humble/setup.bash

# Build paket
colcon build --packages-select fiducial_detector \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DUSE_REALSENSE=OFF

# Source hasil build (wajib setiap terminal baru)
source install/setup.bash
```

### Dengan Intel RealSense

```bash
colcon build --packages-select fiducial_detector \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DUSE_REALSENSE=ON

source install/setup.bash
```

### Dengan CUDA GPU Acceleration (opsional)

> Memerlukan OpenCV dikompilasi dengan `-DWITH_CUDA=ON`.  
> Jika tidak ada GPU, sistem otomatis fallback ke CPU — tidak crash.

```bash
colcon build --packages-select fiducial_detector \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DUSE_REALSENSE=ON -DUSE_CUDA=ON

source install/setup.bash
```

### Validasi Setelah Build (wajib)

```bash
ros2 run fiducial_detector dict_test_node
# Expected output: 50/50 PASS ✓
```

---

## Quick Start — Webcam

```bash
# === Di setiap terminal baru, selalu source dulu: ===
source /opt/ros/humble/setup.bash
source ~/Documents/vtol/vtol\ aruco/fiducial_ws/install/setup.bash
# (Ganti path di atas jika nama folder workspace berbeda)

# Cek device kamera dulu
ls /dev/video*
v4l2-ctl --list-devices

# Jalankan dengan webcam /dev/video0
ros2 launch fiducial_detector webcam.launch.xml device_id:=0

# Dengan preview window
ros2 launch fiducial_detector webcam.launch.xml device_id:=0 show_window:=true

# Ganti device jika kamera ada di /dev/video2
ros2 launch fiducial_detector webcam.launch.xml device_id:=2

# Pakai RealSense
ros2 launch fiducial_detector realsense.launch.xml show_window:=true

# RealSense FPS mode: pakai profil 30 FPS yang didukung kamera
ros2 launch fiducial_detector realsense.launch.xml \
  show_window:=false width:=640 height:=480 fps_limit:=30

# Debug desktop saja; ini lebih berat karena menggambar window OpenCV
ros2 launch fiducial_detector realsense.launch.xml \
  show_window:=true width:=640 height:=480 fps_limit:=30

# RealSense untuk marker kecil: pakai resolusi tinggi hanya jika profile didukung
ros2 launch fiducial_detector realsense.launch.xml \
  show_window:=true width:=1280 height:=720 fps_limit:=15

# Subscribe ke topic yang sudah ada (misal dari RealSense)
ros2 launch fiducial_detector ros_topic.launch.xml \
  camera_topic:=/camera/camera/color/image_raw

# Kalibrasi kamera
ros2 launch fiducial_detector calibration.launch.xml \
  output_yaml:=/home/arilix/camera_calib.yaml
```

---

## Launch Files

| File | Camera Source | Deskripsi |
|---|---|---|
| `webcam.launch.xml` | USB Webcam via `capture_node` | Default `/dev/video0` |
| `realsense.launch.xml` | Intel RealSense D4xx | Requires `USE_REALSENSE=ON` build |
| `ros_topic.launch.xml` | Existing ROS image topic | Tidak ada camera node |
| `calibration.launch.xml` | Any ROS image topic | Kalibrasi chessboard |
| `dict_test.launch.xml` | — | Validasi 50/50 |

### Common Arguments (Semua Launch)

| Argument | Default | Deskripsi |
|---|---|---|
| `marker_size` | `0.05` | Ukuran fisik marker (metre) |
| `show_window` | `false` | Tampilkan window OpenCV |
| `cuda` | `false` | `true` = GPU preprocessing |

### `webcam.launch.xml` Arguments

| Argument | Default | Deskripsi |
|---|---|---|
| `device_id` | `0` | V4L2 device index (`/dev/video0`) |
| `device_path` | `""` | Path eksplisit, override `device_id` |
| `width` / `height` | `640`/`480` | Resolusi |
| `fps_limit` | `30.0` | Max capture FPS |
| `marker_size` | `0.05` | Ukuran fisik marker (metre) |

### Window Key Controls

| Key | Aksi |
|-----|------|
| `q` / `ESC` | Quit |
| `d` | Toggle semua debug windows |
| `c` | Toggle marker cells window |
| `t` | Toggle threshold window |
| `n` | Toggle contour window |
| `r` | Toggle rejected candidates window |

---

## Gate Alignment State Machine

```
         ┌──────────┐
    ─────►  SEARCH  │  tidak ada marker
         └────┬─────┘
              │ first detection
         ┌────▼─────┐
         │ DETECTED │  (1 frame)
         └────┬─────┘
              │ ≥3 consecutive frames
         ┌────▼─────┐
         │ TRACKING │  stabil terdeteksi
         └────┬─────┘
              │ |error| > tolerance
      ┌───────▼──────┐          ┌─────────┐
      │  CENTERING   │◄─────────┤ ALIGNED │
      └───────┬──────┘ drift    └────┬────┘
              │ |error| ≤ tol         │ ≥10 frame konsekutif
              │                 ┌────▼──────┐
              └────────────────►│ GATE_READY│
                                └────┬──────┘
                                     │ external signal
                                ┌────▼───────┐
                                │PASS_THROUGH│
                                └────────────┘
```

**Tolerance:** `alignment_tolerance` (default: 50 px)  
**Stable frames:** `alignment_stable_frames` (default: 10 frames)

---

## ROS 2 Topics

### Published

| Topic | Type | Deskripsi |
|---|---|---|
| `/fiducial/pose` | `geometry_msgs/PoseStamped` | 6DOF pose marker utama |
| `/fiducial/debug_image` | `sensor_msgs/Image` | Frame ter-annotasi |
| `/fiducial/alignment` | `std_msgs/String` | JSON gate alignment |
| `/fiducial/fps` | `std_msgs/Float32` | Detection FPS |
| `/fiducial/rejected_candidates` | `std_msgs/String` | JSON rejected count |

### Format `/fiducial/alignment`

```json
{
  "error_x": +23.0,
  "error_y": -24.0,
  "distance": 0.162,
  "alignment_state": "GATE_READY",
  "marker_id": 1
}
```

- `error_x` — pixel offset horizontal (+ = marker di kanan frame center)
- `error_y` — pixel offset vertikal (+ = marker di bawah frame center)
- `distance` — jarak ke marker dalam meter

### Monitor Real-time

```bash
ros2 topic echo /fiducial/alignment
ros2 topic echo /fiducial/pose
ros2 topic hz   /fiducial/fps
```

---

## Terminal Output

```
[Detector]
  Family=DICT_7X7_50  ID=1  Confidence=86%  FPS=29.5  Latency=33.4ms

[Pose]
  XYZ=(0.011, -0.006, 0.164)m  Distance=0.164m

[Tracking]
  State=GATE_READY

[Alignment]
  State=GATE_READY  ErrorX=+41  ErrorY=-27

[Gate]
  State=GATE_READY  Distance=0.164m
```

---

## ROS 2 Parameters

### Core

| Parameter | Type | Default | Deskripsi |
|---|---|---|---|
| `camera_topic` | string | `/camera/image_raw` | Input image topic |
| `marker_size` | double | `0.05` | Ukuran fisik marker (metre) |
| `show_window` | bool | `false` | OpenCV display window |
| `alignment_tolerance` | int | `50` | Pixel tolerance untuk ALIGNED |
| `alignment_stable_frames` | int | `10` | Frame konsekutif untuk GATE_READY |
| `smoothing_alpha` | double | `0.4` | EMA pose smoothing (0=freeze, 1=raw) |
| `max_missed_frames` | int | `5` | Frame miss sebelum reset ke SEARCH |
| `cuda` | bool | `false` | `true` = GPU preprocessing (perlu `USE_CUDA=ON` build) |

### Preprocessing

| Parameter | Type | Default | Deskripsi |
|---|---|---|---|
| `enable_clahe` | bool | `true` | CLAHE contrast enhancement |
| `clahe_clip_limit` | double | `2.0` | CLAHE clip limit |
| `enable_sharpen` | bool | `false` | Unsharp mask |
| `enable_blur` | bool | `false` | Gaussian blur |

### DICT\_7X7\_50 Detector

| Parameter | Type | Default | Deskripsi |
|---|---|---|---|
| `adaptiveThreshWinSizeMin/Max` | int | `3`/`33` | Adaptive threshold window range |
| `minMarkerPerimeterRate` | double | `0.015` | Min marker perimeter (deteksi jauh) |
| `errorCorrectionRate` | double | `0.6` | Error correction rate |
| `detectInvertedMarker` | bool | `true` | Deteksi marker terbalik |
| `cornerRefinementMethod` | int | `1` | Subpixel corner refinement |

### `capture_node`

| Parameter | Type | Default | Deskripsi |
|---|---|---|---|
| `source` | string | `webcam` | `webcam` atau `realsense` |
| `device_id` | int | `0` | V4L2 device index |
| `device_path` | string | `""` | `/dev/videoX` (override device_id) |
| `width` / `height` | int | `640`/`480` | Resolusi |
| `fps_limit` | double | `30.0` | Max capture FPS |

---

## CUDA GPU Acceleration

CUDA mengakselerasi tahap preprocessing frame (CLAHE, blur, unsharp mask).  
Deteksi ArUco marker sendiri tetap berjalan di CPU.

| Kondisi | Hasil |
|---------|-------|
| `cuda:=false` (default) | CPU preprocessing — selalu jalan |
| `cuda:=true` + build `USE_CUDA=ON` + ada GPU | **GPU CLAHE/blur/sharpen aktif** |
| `cuda:=true` + build `USE_CUDA=ON` + tidak ada GPU | WARN → fallback CPU |
| `cuda:=true` + build `USE_CUDA=OFF` | WARN → fallback CPU (tidak crash) |

---

## Camera Calibration

```bash
# 1. Siapkan chessboard 9x6 fisik
# 2. Launch calibration (kamera harus aktif terpisah)
ros2 launch fiducial_detector calibration.launch.xml

# Key controls:
#   SPACE → capture frame (butuh ≥15 frame dari berbagai sudut)
#   'c'   → hitung kalibrasi
#   's'   → simpan YAML
#   'q'   → quit

# 3. Salin nilai camera_matrix dan dist_coeffs ke config/detector.yaml
```

---

## Generate Markers

```bash
# Via browser (direkomendasikan):
# https://chev.me/arucogen/
# Dictionary: 7x7, Marker ID: 0-49
```

Print pada ukuran fisik yang sesuai `marker_size` (contoh: `marker_size:=0.15` → cetak 150mm × 150mm).

---

## Troubleshooting

| Problem | Penyebab | Solusi |
|---|---|---|
| Camera tidak ditemukan | Device ID salah | `ls /dev/video*` → `v4l2-ctl --list-devices` → sesuaikan `device_id:=N` |
| `source install/setup.bash` error setelah rename | Path hardcoded di `install/` | `rm -rf install/ build/` lalu `colcon build` ulang |
| Kamera tidak buka di device lain | Install belum dilakukan | Install semua prerequisite → rebuild dari awal |
| FPS rendah | `show_window:=true` berat | `show_window:=false`, kurangi resolusi |
| FPS turun ke 10–15 | Small-marker rescue terlalu berat / full-frame upscale aktif | Versi terbaru menjalankan rescue berat berkala; kalau masih berat pakai `fps_limit:=15` atau `show_window:=false` |
| Jetson drop parah | Debug image/overlay dan rescue ROI masih berat | Pastikan `show_window:=false`, `publish_debug_image:=false`, `enable_split_rescue:=false`, `enable_full_frame_fallback:=false` |
| Deteksi tidak stabil | Lighting buruk / tolerance kecil | Naikkan `alignment_tolerance`, cek pencahayaan |
| Marker kecil hanya terbaca dekat | Piksel marker kecil terlalu sedikit / blur / exposure | Kalau device support, pakai `1280x720@15`; kalau butuh FPS, pakai `640x480@30` dan dekatkan board |
| RealSense FPS mentok 15 | Profile yang diminta invalid lalu fallback ke `640x480x15` | Pakai profile valid, misalnya `width:=640 height:=480 fps_limit:=30`; cek log `Open profile` |
| Marker kecil flicker | Rescue hanya kena beberapa frame | Cek log `Predictive small-tab detected ID=...`; stabilizer menahan small marker 14 frame |
| `solvePnP` failed | Intrinsik kamera belum dikalibrasi | Jalankan `calibration.launch.xml` |
| RealSense tidak terbuka | Flag build salah | Rebuild dengan `-DUSE_REALSENSE=ON` |
| `cuda:=true` tapi WARN | OpenCV tanpa CUDA | Rebuild dengan `-DUSE_CUDA=ON` atau OpenCV perlu CUDA |
| `dict_test_node` FAIL | OpenCV versi lama | Verifikasi OpenCV ≥ 4.x |
| Window tidak muncul di SSH | `DISPLAY` tidak di-set | `export DISPLAY=:0` sebelum launch |

---

## File Structure

```
fiducial_detector/
├── CMakeLists.txt
├── package.xml
├── README.md
├── launch.md
├── src/
│   ├── core/
│   │   ├── aruco.cpp               # Main detection logic
│   │   ├── gate_alignment.cpp      # State machine + error_x/error_y
│   │   ├── detector_parameters.cpp # Hardcoded 7x7 optimal profile
│   │   ├── dictionary_manager.cpp  # DICT_7X7_50 only
│   │   ├── pose_estimator.cpp      # solvePnP + EMA smoothing
│   │   ├── visualization.cpp       # OpenCV UI overlay
│   │   ├── confidence_system.cpp   # Reprojection + Hamming confidence
│   │   ├── benchmark_runner.cpp    # FPS/latency benchmark
│   │   ├── marker_decoder.cpp      # Bit-level debug decoder
│   │   ├── marker_generator.cpp    # Generate marker images
│   │   └── fps_monitor.cpp         # Rolling FPS counter
│   ├── nodes/
│   │   ├── capture_node.cpp        # Camera publisher (webcam + RealSense)
│   │   ├── calibration_node.cpp    # Chessboard calibration
│   │   └── visualization.cpp
│   └── main/
│       ├── main.cpp                # Entry point aruco_node
│       ├── capture_main.cpp        # Entry point capture_node
│       ├── calibration_main.cpp    # Entry point calibration_node
│       └── dict_test_main.cpp      # 50/50 PASS validator
├── utils/                          # Header files
│   ├── aruco.h
│   ├── capture_node.h
│   ├── gate_alignment.h
│   ├── detector_parameters.h
│   └── ...
├── config/
│   ├── detector.yaml               # All tunable parameters
│   └── params.yaml
└── launch/
    ├── webcam.launch.xml           # USB webcam pipeline
    ├── realsense.launch.xml        # Intel RealSense pipeline
    ├── ros_topic.launch.xml        # External topic input
    ├── calibration.launch.xml      # Camera calibration
    └── dict_test.launch.xml        # Dictionary validation
```

---

## Build Options Summary

| CMake Flag | Default | Deskripsi |
|---|---|---|
| `-DCMAKE_BUILD_TYPE` | `RelWithDebInfo` | `Release` untuk produksi |
| `-DUSE_REALSENSE` | `OFF` | Intel RealSense support di `capture_node` |
| `-DUSE_CUDA` | `OFF` | GPU preprocessing (CLAHE/blur/sharpen) |
