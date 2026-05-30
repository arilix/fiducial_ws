# Panduan Launch — fiducial_detector (ROS 2 Humble)

Semua launch file menggunakan format **XML** (tanpa Python). Pipeline kamera sepenuhnya C++.

---

## 1. Build

```bash
cd ~/Documents/vtol/vtol\ aruco/fiducial_ws
source /opt/ros/humble/setup.bash

# Standard build — semua sumber kamera sudah didukung
colcon build --packages-select fiducial_detector \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

> Tidak perlu flag `-DUSE_REALSENSE=ON`. RealSense menggunakan paket `realsense2_camera` resmi.

Source workspace di setiap terminal baru:
```bash
source /opt/ros/humble/setup.bash
source ~/Documents/vtol/vtol\ aruco/fiducial_ws/install/setup.bash
```

---

## 2. Daftar Launch File

| File | Keterangan |
|---|---|
| `all_in_one.launch.xml` | ⭐ **Utama** — kamera internal (tanpa node terpisah), webcam laptop/USB |
| `webcam.launch.xml` | USB webcam via `capture_node` C++ terpisah |
| `realsense.launch.xml` | Intel RealSense D4xx via `realsense2_camera` package |
| `ros_topic.launch.xml` | Subscribe ke ROS topic yang sudah ada |
| `benchmark.launch.xml` | Profiling performa detector |
| `calibration.launch.xml` | Kalibrasi kamera via ChArUco board |

---

## 3. all_in_one.launch.xml ⭐

**Direkomendasikan** untuk webcam bawaan laptop atau USB webcam.
`aruco_node` membuka kamera langsung via V4L2 — tidak ada `capture_node` terpisah.

### Contoh Perintah

```bash
# Default: DICT_7X7_50, /dev/video0, tanpa GUI
ros2 launch fiducial_detector all_in_one.launch.xml

# Dengan jendela preview OpenCV
ros2 launch fiducial_detector all_in_one.launch.xml show_window:=true

# Device kamera lain berdasarkan index
ros2 launch fiducial_detector all_in_one.launch.xml capture_device_id:=2

# Device berdasarkan path eksplisit
ros2 launch fiducial_detector all_in_one.launch.xml \
  capture_device_path:=/dev/video4

# Ganti dictionary
ros2 launch fiducial_detector all_in_one.launch.xml \
  dictionary_type:=DICT_4X4_50

# Resolusi tinggi
ros2 launch fiducial_detector all_in_one.launch.xml \
  capture_width:=1280 capture_height:=720 capture_fps:=30.0

# Kombinasi lengkap
ros2 launch fiducial_detector all_in_one.launch.xml \
  capture_device_id:=0 \
  capture_width:=640 \
  capture_height:=480 \
  dictionary_type:=DICT_7X7_50 \
  detection_mode:=SINGLE \
  marker_size:=0.05 \
  show_window:=true
```

### Arguments

| Argument | Default | Keterangan |
|---|---|---|
| `dictionary_type` | `DICT_7X7_50` | ArUco dictionary |
| `detection_mode` | `SINGLE` | `SINGLE` / `MULTI` / `BENCHMARK` |
| `marker_size` | `0.05` | Ukuran fisik marker (metre) |
| `capture_device_id` | `0` | V4L2 index (`/dev/videoN`) |
| `capture_device_path` | `""` | Path eksplisit, mengoverride `capture_device_id` |
| `capture_width` | `640` | Lebar frame kamera (pixel) |
| `capture_height` | `480` | Tinggi frame kamera (pixel) |
| `capture_fps` | `30.0` | Target FPS kamera |
| `show_window` | `false` | `true` = tampilkan jendela OpenCV |
| `enable_charuco` | `false` | `true` = aktifkan ChArUco detection |

> **show_window:** Membuka jendela `Fiducial Detector` dengan overlay marker, FPS label kuning,
> crosshair, dan teks arahan. Tekan `d` untuk debug windows, `ESC` untuk keluar.
>
> **Catatan teknis:** Parameter dari launch args mengoverride `config/detector.yaml`.
> Tidak ada konflik `--ros-args` ganda — launch file ini sudah diperbaiki menggunakan
> `<param from="..."/>` sebagai pengganti `args="--ros-args --params-file ..."`.

---

## 4. Output Terminal (Format Log)

Setiap frame yang mendeteksi satu atau lebih marker mencetak ke terminal:

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

### Keterangan Kolom

| Kolom | Keterangan |
|---|---|
| `[Frame N]` | Nomor frame kumulatif sejak startup |
| `FPS=xx.x` | Frame rate deteksi realtime (rolling window 60 frame) |
| `Lat=xx.xms` | Latency antar frame (interval kamera) |
| `Family` | Dictionary yang sedang aktif |
| `ID` | Nomor ID marker yang terdeteksi |
| `Conf` | Confidence OpenCV (0–100%) — hanya untuk backend OPENCV |
| `Margin` | Decision margin AprilTag — semakin tinggi semakin yakin |
| `Hamming` | Bit error yang dikoreksi (0=sempurna, 1=satu bit error) |
| `Backend` | `OPENCV` / `APRILTAG3` / `FUSION` |
| `Center=(x,y)` | Koordinat pixel pusat marker dalam frame |
| `Pose=(X,Y,Z)m` | Posisi marker relatif terhadap kamera (metre) |
| `Dist=x.xxxm` | Jarak Euclidean kamera ke marker (metre) |
| `Posisi ID=N` | Arahan arah — lihat tabel di bawah |

### Nilai Arahan Posisi

| Output Terminal | Makna | Output `/fiducial/alignment` |
|---|---|---|
| `Posisi ID=N: GESER_KIRI` | Marker terlalu ke kiri frame | `GESER_KIRI` |
| `Posisi ID=N: GESER_KANAN` | Marker terlalu ke kanan frame | `GESER_KANAN` |
| `Posisi ID=N: MAJU` | Marker terlalu ke bawah frame | `MAJU` |
| `Posisi ID=N: MUNDUR` | Marker terlalu ke atas frame | `MUNDUR` |
| `Posisi ID=N: GESER_KIRI\|MAJU` | Kombinasi dua arah | `GESER_KIRI\|MAJU` |
| `*** POSISI CENTERING ***` | Marker tepat di tengah | `POSISI_CENTERING` |

Toleransi centering diatur via parameter `alignment_tolerance` (default **50 px**).

### FPS Overlay di Preview Window

Saat `show_window:=true`, setiap marker yang terdeteksi mendapat label **`FPS: xx.x`**
berwarna kuning di atas kotak deteksinya. Ini menunjukkan frame rate deteksi realtime
langsung pada marker yang bersangkutan.

---

## 5. webcam.launch.xml

Dua node: `capture_node` (V4L2 → ROS topic) + `aruco_node` (deteksi).
Berguna jika ingin memisahkan capture dari deteksi (misal capture di satu proses, deteksi di lain).

### Contoh Perintah

```bash
# Default: /dev/video6 (HP webcam), DICT_4X4_50, AUTO mode, dengan GUI
ros2 launch fiducial_detector webcam.launch.xml

# Device berdasarkan index
ros2 launch fiducial_detector webcam.launch.xml device_id:=0

# Device berdasarkan path
ros2 launch fiducial_detector webcam.launch.xml device_path:=/dev/video4

# Resolusi tinggi + dictionary custom
ros2 launch fiducial_detector webcam.launch.xml \
  device_id:=0 \
  width:=1280 height:=720 fps_limit:=30.0 \
  dictionary_type:=DICT_7X7_50 \
  detection_mode:=SINGLE

# Tanpa GUI
ros2 launch fiducial_detector webcam.launch.xml show_window:=false
```

### Arguments

| Argument | Default | Keterangan |
|---|---|---|
| `device_id` | `6` | V4L2 index (`/dev/video6` = HP webcam default) |
| `device_path` | `""` | Path eksplisit, override `device_id` |
| `width` / `height` | `640`/`480` | Resolusi capture |
| `fps_limit` | `30.0` | Batas FPS capture |
| `marker_size` | `0.05` | Ukuran fisik marker (metre) |
| `dictionary_type` | `DICT_4X4_50` | ArUco dictionary |
| `detection_mode` | `AUTO` | `SINGLE` / `AUTO` / `MULTI` / `BENCHMARK` |
| `show_window` | `true` | Tampilkan jendela OpenCV |

> **Topic internal:** `capture_node` publish ke `/camera/image_raw`,
> `aruco_node` subscribe dari `/camera/image_raw`.

---

## 6. realsense.launch.xml

Menggunakan paket `realsense2_camera` resmi sebagai driver kamera.
**Tidak memerlukan build flag `-DUSE_REALSENSE=ON`.**

### Prasyarat

```bash
sudo apt install ros-humble-realsense2-camera
```

Pastikan kamera RealSense terhubung via USB 3.x dan terdeteksi:
```bash
rs-enumerate-devices | head -5
```

### Contoh Perintah

```bash
# Default: DICT_7X7_50, 640x480 @ 30fps, SINGLE mode, dengan GUI
ros2 launch fiducial_detector realsense.launch.xml

# Tanpa GUI (headless)
ros2 launch fiducial_detector realsense.launch.xml show_window:=false

# Resolusi tinggi
ros2 launch fiducial_detector realsense.launch.xml \
  width:=1280 height:=720 fps_limit:=30

# Marker ukuran besar (misal landing pad KRTI 2026)
ros2 launch fiducial_detector realsense.launch.xml \
  marker_size:=0.5 \
  dictionary_type:=DICT_7X7_50

# Multi-dictionary mode
ros2 launch fiducial_detector realsense.launch.xml \
  detection_mode:=MULTI

# Kombinasi lengkap
ros2 launch fiducial_detector realsense.launch.xml \
  width:=640 height:=480 fps_limit:=30 \
  dictionary_type:=DICT_7X7_50 \
  detection_mode:=SINGLE \
  marker_size:=0.05 \
  show_window:=true
```

### Arguments

| Argument | Default | Keterangan |
|---|---|---|
| `width` / `height` | `640`/`480` | Resolusi color stream |
| `fps_limit` | `30` | FPS kamera (harus integer) |
| `marker_size` | `0.05` | Ukuran fisik marker (metre) |
| `dictionary_type` | `DICT_7X7_50` | ArUco dictionary |
| `detection_mode` | `SINGLE` | `SINGLE` atau `MULTI` |
| `show_window` | `true` | Tampilkan jendela OpenCV |

### Catatan Teknis

- Camera topic: **`/camera/camera/color/image_raw`** (dari `realsense2_camera_node` namespace `/camera`)
- Jangan set `name=` di tag node realsense — biarkan `camera_name=camera` yang mengatur nama node dan prefix topic
- `AUTO` mode **tidak direkomendasikan** dengan RealSense: ada pre-existing crash di HybridDetector saat dictionary switching dengan RealSense frames
- Warning IMU calibration (`ds-calib-parsers.cpp:36`) dan HID power adalah normal untuk D455 — tidak mempengaruhi color stream

---

## 7. ros_topic.launch.xml

Digunakan saat kamera sudah berjalan sebagai node terpisah dan mempublish image topic.
Hanya menjalankan `aruco_node` yang subscribe ke topic tersebut.

### Contoh Perintah

```bash
# Default: /camera/image_raw, DICT_4X4_50, AUTO mode
ros2 launch fiducial_detector ros_topic.launch.xml

# Intel RealSense via realsense2_camera yang sudah jalan
ros2 launch fiducial_detector ros_topic.launch.xml \
  camera_topic:=/camera/camera/color/image_raw \
  dictionary_type:=DICT_7X7_50 \
  detection_mode:=SINGLE

# Kamera custom
ros2 launch fiducial_detector ros_topic.launch.xml \
  camera_topic:=/my_camera/image_raw \
  marker_size:=0.1 \
  dictionary_type:=DICT_6X6_50

# Dengan GUI
ros2 launch fiducial_detector ros_topic.launch.xml \
  camera_topic:=/camera/image_raw \
  show_window:=true
```

### Arguments

| Argument | Default | Keterangan |
|---|---|---|
| `camera_topic` | `/camera/image_raw` | ROS image topic yang akan di-subscribe |
| `marker_size` | `0.05` | Ukuran fisik marker (metre) |
| `dictionary_type` | `DICT_4X4_50` | ArUco dictionary |
| `detection_mode` | `AUTO` | `SINGLE` / `AUTO` / `MULTI` |
| `show_window` | `true` | Tampilkan jendela OpenCV |

---

## 8. benchmark.launch.xml

Mode profiling performa — menjalankan deteksi pada dataset gambar internal.

```bash
ros2 launch fiducial_detector benchmark.launch.xml
```

Output mencakup FPS rata-rata, latency per frame, dan jumlah marker terdeteksi per dictionary.

---

## 9. calibration.launch.xml

Kalibrasi kamera menggunakan ChArUco board terintegrasi.
Menghasilkan file YAML berisi `camera_matrix` dan `dist_coeffs`.

### Contoh Perintah

```bash
# Default: ChArUco 7x5, simpan ke /tmp/camera_calib.yaml
ros2 launch fiducial_detector calibration.launch.xml

# Output ke path custom
ros2 launch fiducial_detector calibration.launch.xml \
  output_yaml:=/home/arilix/calib_realsense.yaml

# Board berbeda
ros2 launch fiducial_detector calibration.launch.xml \
  charuco_cols:=9 charuco_rows:=6 \
  charuco_sq:=0.025 charuco_mk:=0.0125

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
Rebuild: `colcon build --packages-select fiducial_detector`

---

## 10. Memantau ROS Topics

```bash
# Arahan posisi realtime (POSISI_CENTERING / GESER_KIRI / MAJU / dll)
ros2 topic echo /fiducial/alignment

# Pose 6DOF marker pertama
ros2 topic echo /fiducial/pose

# FPS realtime (update setiap 1 detik)
ros2 topic echo /fiducial/fps

# Preview frame annotated (di terminal terpisah)
ros2 run rqt_image_view rqt_image_view /fiducial/debug_image

# Statistik deteksi/rejected
ros2 topic echo /fiducial/rejected_candidates
```

---

## 11. Keyboard Shortcut (saat show_window=true)

| Key | Fungsi |
|---|---|
| `ESC` | Keluar dari program |
| `d` | Toggle semua debug windows (aktif/nonaktif sekaligus) |
| `c` | Toggle Marker Cells window |
| `t` | Toggle Threshold window |
| `n` | Toggle Contours window |
| `r` | Toggle Rejected candidates window |

---

## 12. Parameter Penting (config/detector.yaml)

Parameter runtime utama yang bisa di-override via launch args:

```yaml
# Core
dictionary_type:      "DICT_7X7_50"   # dictionary ArUco/AprilTag
detection_mode:       "SINGLE"        # SINGLE|MULTI|AUTO|BENCHMARK
marker_size:          0.05            # ukuran fisik marker (metre)
alignment_tolerance:  50              # toleransi centering (pixel)

# Kamera intrinsik (update setelah kalibrasi)
camera_matrix: [640, 0, 320, 0, 640, 240, 0, 0, 1]
dist_coeffs:   [0, 0, 0, 0, 0]

# Display
show_window:          false           # jendela OpenCV

# Hybrid Detector
use_opencv_detector:  true
use_native_apriltag:  true
use_detector_fusion:  true
apriltag_decimate:    1.0             # 1.0=full res, 2.0=setengah (lebih cepat)
apriltag_min_margin:  40.0           # semakin tinggi = semakin selektif

# Preprocessing
enable_clahe:         true           # kontras adaptif (bagus untuk outdoor)
clahe_clip_limit:     2.0
```

---

## 13. Tuning Per Skenario

### Jarak Jauh / Outdoor (>2m)

```yaml
apriltag_decimate:    1.0
apriltag_sharpening:  0.5
enable_clahe:         true
clahe_clip_limit:     3.0
apriltag_min_margin:  40.0
```

### Headless / Jetson / Onboard VTOL

```yaml
apriltag_decimate:    2.0   # setengah resolusi = ~2x lebih cepat
apriltag_threads:     4
show_window:          false
enable_clahe:         false
```

### Mengurangi False Positive AprilTag

```yaml
apriltag_max_hamming: 0    # 0=bit perfect only, 1=1 bit error ok
apriltag_min_margin:  60.0
```

### Multi-Marker (misal grid landing pad)

```bash
ros2 launch fiducial_detector all_in_one.launch.xml detection_mode:=MULTI
```

---

## 14. Identifikasi Device Kamera

```bash
# List semua video device
ls /dev/video*

# Cek format pixel per device
v4l2-ctl -d /dev/video0 --list-formats-ext

# Cek nama perangkat
v4l2-ctl -d /dev/video0 --info | grep "Card type"

# Cek RealSense terdeteksi
rs-enumerate-devices | head -10
```

Cari format `YUYV` atau `MJPG` — itu kamera RGB.
Format `Z16` = depth stream RealSense, tidak bisa dipakai langsung.

---

## 15. Troubleshooting

| Problem | Penyebab | Solusi |
|---|---|---|
| Jendela tidak muncul | `DISPLAY` tidak di-set | `export DISPLAY=:0` sebelum launch |
| `show_window:=true` tidak berpengaruh | `args=` lama yang konflik | Sudah diperbaiki — pastikan rebuild |
| SIGSEGV saat RealSense | Pre-existing bug di `AUTO` mode | Gunakan `detection_mode:=SINGLE` |
| RealSense tidak terdeteksi | Driver tidak ter-install | `sudo apt install ros-humble-realsense2-camera` |
| Kamera tidak terbuka | Device ID salah | `ls /dev/video*` → cek dengan `v4l2-ctl` |
| Marker tidak terdeteksi | Dictionary tidak cocok | Sesuaikan `dictionary_type` dengan marker yang dicetak |
| FPS rendah (<15) | Resolusi terlalu tinggi / CPU berat | Aktifkan `apriltag_decimate:=2.0` |
| Pose tidak akurat | Intrinsik kamera placeholder | Kalibrasi kamera, update `detector.yaml` |
| `libapriltag.so` not found | Library tidak ter-install | `sudo apt install libapriltag-dev libapriltag3` |
