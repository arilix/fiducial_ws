# Panduan Launch — fiducial\_detector (ROS 2 Humble)

Semua launch file menggunakan format **XML** (tanpa Python). Pipeline kamera sepenuhnya C++.

---

## 1. Install Dependensi (Sebelum Build)

Jalankan sekali di device baru sebelum melakukan build pertama kali.

### ROS 2 Humble Packages

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

### OpenCV + Build Tools

```bash
sudo apt install -y \
  libopencv-dev \
  libeigen3-dev \
  python3-colcon-common-extensions \
  build-essential \
  cmake \
  v4l-utils
```

> Verifikasi: `python3 -c "import cv2; print(cv2.__version__)"` → harus ≥ 4.x

### Intel RealSense SDK (opsional)

Hanya diperlukan jika menggunakan kamera Intel RealSense D4xx.

```bash
sudo apt install -y \
  librealsense2-dev \
  librealsense2-udev-rules \
  ros-humble-realsense2-camera

# Verifikasi
rs-enumerate-devices | head -5
```

---

## 2. Build Workspace

### ⚠️ PENTING: Jika Workspace Di-rename atau Dipindah ke Device Lain

> **Masalah:** Colcon build menyimpan **absolute path** secara hardcoded di dalam:
> - `install/setup.sh`
> - `install/local_setup.sh`  
> - `install/fiducial_detector/share/fiducial_detector/package.sh`
>
> Jika folder workspace di-rename (contoh: `fiducial_ws` → `robot_ws`), atau dicopy ke device
> lain dengan path berbeda, file-file tersebut menjadi invalid — kamera tidak bisa berjalan,
> launch file tidak ditemukan, dan node tidak bisa di-source.
>
> **Solusi wajib setelah rename/pindah:**
> ```bash
> cd /path/ke/workspace/baru
> rm -rf install/ build/ log/
> source /opt/ros/humble/setup.bash
> colcon build --packages-select fiducial_detector --cmake-args -DCMAKE_BUILD_TYPE=Release
> source install/setup.bash
> ```

### Build Standard — Webcam (Paling Umum)

```bash
# Masuk ke direktori workspace
cd ~/Documents/vtol/vtol\ aruco/fiducial_ws
# (Ganti path di atas sesuai lokasi workspace aktual di device kamu)

# Source ROS 2 base
source /opt/ros/humble/setup.bash

# Build
colcon build --packages-select fiducial_detector \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DUSE_REALSENSE=OFF

# Source hasil build
source install/setup.bash
```

### Build dengan Intel RealSense

```bash
colcon build --packages-select fiducial_detector \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DUSE_REALSENSE=ON

source install/setup.bash
```

### Build dengan CUDA GPU Acceleration (opsional)

> Memerlukan OpenCV yang dikompilasi dengan `-DWITH_CUDA=ON`.  
> Jika tidak ada GPU, sistem otomatis fallback ke CPU — tidak crash.

```bash
colcon build --packages-select fiducial_detector \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DUSE_REALSENSE=ON -DUSE_CUDA=ON

source install/setup.bash
```

### Source di Setiap Terminal Baru

```bash
# Setiap kali buka terminal baru, wajib source dua baris ini:
source /opt/ros/humble/setup.bash
source ~/Documents/vtol/vtol\ aruco/fiducial_ws/install/setup.bash
```

### Validasi Setelah Build

```bash
# Harus output: 50/50 PASS ✓
ros2 run fiducial_detector dict_test_node
```

---

## 3. Identifikasi Device Kamera

Lakukan sebelum menjalankan launch file untuk mengetahui nomor `/dev/videoX` yang benar.

```bash
# List semua video device
ls /dev/video*

# Lihat nama perangkat per device
v4l2-ctl --list-devices

# Detail format per device
v4l2-ctl -d /dev/video0 --list-formats-ext

# Cari format YUYV atau MJPG (itu kamera RGB)
# Format Z16 = depth stream RealSense, tidak bisa dipakai langsung
```

**Contoh output `v4l2-ctl --list-devices`:**
```
HP Wide Vision HD Camera (usb-0000:06:00.3-3):
    /dev/video0   ← pakai ini untuk webcam
    /dev/video1
```

---

## 4. Daftar Launch File

| File | Keterangan |
|---|---|
| `webcam.launch.xml` | USB webcam / laptop camera via `capture_node` C++ terpisah |
| `realsense.launch.xml` | Intel RealSense D4xx via `realsense2_camera` package |
| `ros_topic.launch.xml` | Subscribe ke ROS topic yang sudah ada |
| `calibration.launch.xml` | Kalibrasi kamera via ChArUco board |
| `dict_test.launch.xml` | Validasi DICT_7X7_50 50/50 |

---

## 5. webcam.launch.xml ⭐

Dua node berjalan: `capture_node` (V4L2 → ROS topic) + `aruco_node` (deteksi).  
Direkomendasikan untuk USB webcam dan laptop camera.

### Contoh Perintah

```bash
# Webcam di /dev/video0 (paling umum)
ros2 launch fiducial_detector webcam.launch.xml device_id:=0

# Dengan preview window OpenCV
ros2 launch fiducial_detector webcam.launch.xml device_id:=0 show_window:=true

# Webcam di device lain
ros2 launch fiducial_detector webcam.launch.xml device_id:=2

# Via path eksplisit
ros2 launch fiducial_detector webcam.launch.xml device_path:=/dev/video4

# Resolusi tinggi
ros2 launch fiducial_detector webcam.launch.xml \
  device_id:=0 width:=1280 height:=720 fps_limit:=30.0

# Ubah ukuran marker fisik (default 5 cm)
ros2 launch fiducial_detector webcam.launch.xml \
  device_id:=0 marker_size:=0.15 show_window:=true
```

### Arguments

| Argument | Default | Keterangan |
|---|---|---|
| `device_id` | `0` | V4L2 index (`/dev/video0`) |
| `device_path` | `""` | Path eksplisit, override `device_id` |
| `width` / `height` | `640`/`480` | Resolusi capture |
| `fps_limit` | `30.0` | Batas FPS capture |
| `marker_size` | `0.05` | Ukuran fisik marker (metre) |
| `show_window` | `false` | Tampilkan jendela OpenCV |
| `cuda` | `false` | GPU CLAHE/blur/sharpen (perlu `USE_CUDA=ON`) |

> **Topic internal:** `capture_node` publish ke `/camera/image_raw`,
> `aruco_node` subscribe dari `/camera/image_raw`.

---

## 6. realsense.launch.xml

Menggunakan paket `realsense2_camera` resmi sebagai driver kamera.

### Prasyarat

```bash
sudo apt install ros-humble-realsense2-camera
rs-enumerate-devices | head -5  # verifikasi kamera terdeteksi
```

### Contoh Perintah

```bash
# Default: DICT_7X7_50, 1280x720 @ 30fps, tanpa GUI
ros2 launch fiducial_detector realsense.launch.xml

# Dengan preview window
ros2 launch fiducial_detector realsense.launch.xml show_window:=true

# Resolusi tinggi eksplisit
ros2 launch fiducial_detector realsense.launch.xml \
  width:=1280 height:=720 fps_limit:=30

# Marker kecil lebih stabil: turunkan FPS agar exposure lebih longgar
ros2 launch fiducial_detector realsense.launch.xml \
  width:=1280 height:=720 fps_limit:=15 show_window:=true

# Jika ruangan redup atau board masih blur, coba 10 FPS
ros2 launch fiducial_detector realsense.launch.xml \
  width:=1280 height:=720 fps_limit:=10 show_window:=true

# Marker ukuran besar
ros2 launch fiducial_detector realsense.launch.xml \
  marker_size:=0.5
```

### Arguments

| Argument | Default | Keterangan |
|---|---|---|
| `width` / `height` | `1280`/`720` | Resolusi color stream |
| `fps_limit` | `30` | FPS kamera (harus integer) |
| `marker_size` | `0.05` | Ukuran fisik marker (metre) |
| `show_window` | `false` | Tampilkan jendela OpenCV |
| `cuda` | `false` | GPU preprocessing |

> **Camera topic:** `/camera/camera/color/image_raw`  
> Warning IMU calibration (`ds-calib-parsers.cpp:36`) saat startup adalah normal — tidak mempengaruhi color stream.

### Catatan Marker Kecil

Marker kecil pada board lomba hanya stabil jika ukurannya masih cukup besar di frame. Detektor sudah memiliki **small-tab rescue** dengan log:

```text
Predictive small-tab detected ID=...
```

Jika log tersebut hanya muncul saat board dekat kamera, penyebab umumnya adalah:
- marker kecil terlalu sedikit piksel saat jauh,
- motion blur karena board/kamera bergerak,
- exposure terlalu pendek akibat FPS tinggi,
- cahaya kurang atau glare pada kertas.

Solusi praktis: gunakan `1280x720`, coba `fps_limit:=15` atau `fps_limit:=10`, dan tambah cahaya terang yang menyebar.

---

## 7. ros_topic.launch.xml

Digunakan saat kamera sudah berjalan sebagai node terpisah dan mempublish image topic.  
Hanya menjalankan `aruco_node` yang subscribe ke topic tersebut.

### Contoh Perintah

```bash
# Default: /camera/image_raw
ros2 launch fiducial_detector ros_topic.launch.xml

# Dari RealSense yang sudah jalan
ros2 launch fiducial_detector ros_topic.launch.xml \
  camera_topic:=/camera/camera/color/image_raw

# Dengan GUI
ros2 launch fiducial_detector ros_topic.launch.xml \
  camera_topic:=/camera/image_raw show_window:=true
```

### Arguments

| Argument | Default | Keterangan |
|---|---|---|
| `camera_topic` | `/camera/image_raw` | ROS image topic yang akan di-subscribe |
| `marker_size` | `0.05` | Ukuran fisik marker (metre) |
| `show_window` | `false` | Tampilkan jendela OpenCV |

---

## 8. calibration.launch.xml

Kalibrasi kamera menggunakan ChArUco board terintegrasi.  
Menghasilkan file YAML berisi `camera_matrix` dan `dist_coeffs`.

### Contoh Perintah

```bash
# Default: simpan ke /tmp/camera_calib.yaml
ros2 launch fiducial_detector calibration.launch.xml

# Output ke path custom
ros2 launch fiducial_detector calibration.launch.xml \
  output_yaml:=/home/arilix/calib_webcam.yaml

# Dari RealSense
ros2 launch fiducial_detector calibration.launch.xml \
  camera_topic:=/camera/camera/color/image_raw \
  output_yaml:=/home/arilix/calib_realsense.yaml
```

### Keyboard Control

| Key | Aksi |
|---|---|
| `SPACE` | Capture frame kalibrasi (butuh min. 20 frame berbeda sudut) |
| `s` | Hitung kalibrasi dan simpan ke YAML |
| `q` / `ESC` | Keluar tanpa menyimpan |

### Setelah Kalibrasi

Salin nilai ke `config/detector.yaml`:
```yaml
camera_matrix: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
dist_coeffs:   [k1, k2, p1, p2, k3]
```
Lalu rebuild: `colcon build --packages-select fiducial_detector`

---

## 9. Output Terminal (Format Log)

```
[Detector]
  Family=DICT_7X7_50  ID=2  Confidence=83%  FPS=29.9  Latency=36.0ms
[Pose]
  XYZ=(0.010, 0.007, 0.205)m  Distance=0.205m
[Tracking]
  State=GATE_READY
[Alignment]
  State=GATE_READY  ErrorX=+29  ErrorY=+17
[Gate]
  State=GATE_READY  Distance=0.205m
```

### Keterangan Kolom

| Kolom | Keterangan |
|---|---|
| `Family` | Dictionary yang aktif |
| `ID` | Nomor ID marker yang terdeteksi |
| `Confidence` | Confidence score (0–100%) |
| `FPS` | Frame rate deteksi realtime |
| `Latency` | Waktu proses per frame (ms) |
| `XYZ=(X,Y,Z)m` | Posisi marker relatif ke kamera (metre) |
| `Distance` | Jarak Euclidean ke marker (metre) |
| `State` | Status gate alignment state machine |
| `ErrorX/Y` | Pixel offset dari center frame |

### State Machine Output

| State | Arti |
|---|---|
| `SEARCH` | Tidak ada marker terdeteksi |
| `DETECTED` | Marker baru pertama kali terlihat |
| `TRACKING` | Marker stabil terdeteksi |
| `CENTERING` | Bergerak menuju center, ErrorX/Y masih > tolerance |
| `ALIGNED` | Marker dalam toleransi center |
| `GATE_READY` | Sudah aligned ≥10 frame berturut-turut |

---

## 10. Memantau ROS Topics

```bash
# Arahan posisi realtime (alignment state + error)
ros2 topic echo /fiducial/alignment

# Pose 6DOF marker pertama
ros2 topic echo /fiducial/pose

# FPS realtime
ros2 topic echo /fiducial/fps

# Preview frame annotated
ros2 run rqt_image_view rqt_image_view /fiducial/debug_image

# Statistik deteksi/rejected
ros2 topic echo /fiducial/rejected_candidates
```

---

## 11. Keyboard Shortcut (saat show\_window=true)

| Key | Fungsi |
|---|---|
| `ESC` / `q` | Keluar dari program |
| `d` | Toggle semua debug windows (aktif/nonaktif sekaligus) |
| `c` | Toggle Marker Cells window |
| `t` | Toggle Threshold window |
| `n` | Toggle Contours window |
| `r` | Toggle Rejected candidates window |

---

## 12. Parameter Penting (config/detector.yaml)

```yaml
# Core
marker_size:          0.05            # ukuran fisik marker (metre)
alignment_tolerance:  50              # toleransi centering (pixel)
alignment_stable_frames: 10          # frame konsekutif untuk GATE_READY

# Kamera intrinsik (update setelah kalibrasi)
camera_matrix: [640, 0, 320, 0, 640, 240, 0, 0, 1]
dist_coeffs:   [0, 0, 0, 0, 0]

# Display
show_window:          false           # jendela OpenCV

# Preprocessing
enable_clahe:         true           # kontras adaptif (bagus untuk outdoor)
clahe_clip_limit:     2.0
enable_sharpen:       false
enable_blur:          false
```

---

## 13. Troubleshooting

| Problem | Penyebab | Solusi |
|---|---|---|
| Camera tidak terbuka | Device ID salah | `ls /dev/video*` → `v4l2-ctl --list-devices` → sesuaikan `device_id:=N` |
| Launch gagal setelah rename workspace | Path hardcoded di `install/` | `rm -rf install/ build/ log/` → rebuild dari awal |
| Launch gagal di device baru | Dependensi belum diinstall | Install semua prerequisites di bagian 1 → rebuild |
| `source install/setup.bash` not found | Belum pernah build | Jalankan `colcon build` terlebih dahulu |
| FPS rendah | Resolusi tinggi / show_window aktif | `show_window:=false`, kurangi resolusi |
| Deteksi tidak stabil | Pencahayaan buruk / tolerance kecil | Naikkan `alignment_tolerance`, tambah lampu |
| Marker kecil harus dekat | Resolusi piksel kecil kurang / blur / exposure | RealSense `1280x720`, coba `fps_limit:=15` atau `10`, tambah cahaya menyebar |
| Marker kecil flicker | Small-tab rescue hanya intermittent | Cek log `Predictive small-tab detected ID=...`; stabilizer menahan small marker 14 frame |
| `solvePnP` failed / pose tidak akurat | Intrinsik kamera placeholder | Jalankan `calibration.launch.xml` → update `detector.yaml` |
| RealSense tidak terbuka | Build flag salah | Rebuild dengan `-DUSE_REALSENSE=ON` |
| `cuda:=true` tapi warning | OpenCV tanpa CUDA | Rebuild `-DUSE_CUDA=ON` atau OpenCV perlu dikompilasi ulang |
| Window tidak muncul di SSH | DISPLAY tidak di-set | `export DISPLAY=:0` sebelum launch |
| Marker tidak terdeteksi | Dictionary tidak cocok | Sesuaikan dictionary dengan marker yang dicetak (`DICT_7X7_50`) |
