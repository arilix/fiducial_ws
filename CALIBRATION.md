# Kalibrasi Kamera — Intel RealSense

Panduan kalibrasi intrinsik kamera RealSense untuk sistem ArUco gate-centering.

---

## Pilih Metode

RealSense memiliki **factory calibration** bawaan yang cukup akurat. Ada dua jalur:

| Metode | Waktu | Akurasi | Kapan Digunakan |
|--------|-------|---------|-----------------|
| **A — Baca dari RealSense** | 1 menit | Baik (pabrik) | Default, pertama kali setup |
| **B — Kalibrasi Manual** | 15-30 menit | Sangat baik | Setelah benturan, lensa berembun, atau reproj error > 1.0 px |

---

## Metode A — Baca Intrinsics Langsung dari RealSense

RealSense mempublish intrinsics lewat topic `/camera/camera/color/camera_info`. Tidak perlu papan chessboard.

> **Penting untuk marker kecil:** pakai intrinsics dari resolusi yang sama dengan launch deteksi. Default `realsense.launch.xml` memakai **424×240 @ 15 FPS** agar aman di Jetson/USB2. Jika kamu mengubah ke 640×480, 1280×720, atau resolusi lain, baca ulang `camera_info` karena `cx`, `cy`, dan focal length efektif berubah.

### Step 1 — Jalankan RealSense driver

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch realsense2_camera rs_launch.py enable_color:=true enable_depth:=false
```

### Step 2 — Baca intrinsics

```bash
ros2 topic echo /camera/camera/color/camera_info --once
```

Output yang muncul:

```
header:
  frame_id: camera_color_optical_frame
height: 720
width: 1280
distortion_model: plumb_bob
d: [-0.056, 0.067, 0.001, -0.000, -0.021]
k: [636.12, 0.0, 641.89,  0.0, 636.12, 365.41,  0.0, 0.0, 1.0]
...
```

### Step 3 — Pindahkan ke detector.yaml

Buka `config/detector.yaml`, ganti bagian Camera Intrinsics:

```yaml
# ── Camera Intrinsics ─────────────────────────
camera_matrix:
  - 636.120000   # fx  (K[0])
  - 0.000000
  - 641.890000   # cx  (K[2])
  - 0.000000
  - 636.120000   # fy  (K[4])
  - 365.410000   # cy  (K[5])
  - 0.000000
  - 0.000000
  - 1.000000
dist_coeffs: [-0.056, 0.067, 0.001, -0.000, -0.021]   # dari field 'd'
```

> **Mapping field `k` → camera_matrix:**
> `k: [fx, 0, cx, 0, fy, cy, 0, 0, 1]`
> Index: `[0]=fx  [2]=cx  [4]=fy  [5]=cy`

### Step 4 — Sesuaikan resolusi

Pastikan resolusi di `realsense.launch.xml` cocok dengan yang dipakai saat baca `camera_info`.
Default launch file: **424×240**.

Kalau ingin 640×480, jalankan:
```bash
ros2 launch realsense2_camera rs_launch.py \
  rgb_camera.color_profile:=640,480,30
```
Lalu baca ulang `camera_info` karena `cx`, `cy` berubah mengikuti resolusi.

---

## Metode B — Kalibrasi Manual dengan Chessboard

Gunakan ini jika kamera pernah benturan, factory calibration terasa meleset, atau ingin akurasi maksimal.

### Persiapan

#### Cetak Papan Chessboard

Gunakan papan **9×6 inner corners** (10×7 kotak). Ukuran default: **2.5 cm per kotak**.

Cetak di kertas A4/A3 → tempel di permukaan datar (kardus tebal atau kaca).

> **Wajib:** Ukur ukuran kotak fisik dengan penggaris setelah cetak. Jangan pakai asumsi.

Cetak gratis: https://calib.io/calib_printer → pilih **Checkerboard**, inner corners 9×6.

#### Pastikan RealSense sudah jalan

```bash
ros2 launch realsense2_camera rs_launch.py enable_color:=true enable_depth:=false
ros2 topic hz /camera/camera/color/image_raw
```

### Step 1 — Build

```bash
cd ~/Documents/vtol/vtol\ aruco/fiducial_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### Step 2 — Launch Calibration Node

```bash
ros2 launch fiducial_detector calibration.launch.xml \
  camera_topic:=/camera/camera/color/image_raw \
  output_yaml:=/tmp/realsense_calib.yaml
```

Jika ukuran kotak berbeda dari 2.5 cm (misal 3 cm):
```bash
ros2 launch fiducial_detector calibration.launch.xml \
  camera_topic:=/camera/camera/color/image_raw \
  chess_square:=0.030 \
  output_yaml:=/tmp/realsense_calib.yaml
```

### Step 3 — Capture Frame

Jendela **Calibration** akan muncul. Pojok chessboard terdeteksi otomatis (titik-titik berwarna).

| Tombol | Aksi |
|--------|------|
| `SPACE` | Capture frame saat ini |
| `c` | Hitung kalibrasi |
| `s` | Simpan hasil ke YAML |
| `q` / `ESC` | Keluar |

**Capture minimal 20 frame dengan variasi berikut:**

| Variasi | Contoh |
|---------|--------|
| Jarak dekat | 25–40 cm dari kamera |
| Jarak sedang | 50–80 cm |
| Miring kiri/kanan | ±30° |
| Miring atas/bawah | ±30° |
| Pojok frame | Geser papan ke tiap sudut layar |
| Tengah frame | Papan lurus di tengah |

> Tekan `SPACE` hanya saat gambar **tajam** dan **seluruh papan terlihat**.

### Step 4 — Kalibrasi dan Simpan

```
Tekan [c]  →  tunggu: "Chess calib done: reproj=X.XXX"
Tekan [s]  →  simpan
```

Terminal menampilkan hasil siap-paste:

```
═══ Paste into config/detector.yaml ═══
camera_matrix:
  - 635.812345
  - 0.000000
  - 640.123456
  - 0.000000
  - 635.456789
  - 364.789012
  - 0.000000
  - 0.000000
  - 1.000000
dist_coeffs: [-0.054321, 0.065432, 0.000123, -0.000045, -0.019876]
# Reproj error: 0.3812 px  (good if < 1.0)
════════════════════════════════════════
```

Dua file tersimpan:

| File | Isi |
|------|-----|
| `/tmp/realsense_calib.yaml` | Format OpenCV FileStorage |
| `/tmp/realsense_calib_ros_params.yaml` | Format ROS params — siap paste ke detector.yaml |

### Step 5 — Terapkan ke detector.yaml

Salin nilai dari terminal atau dari `/tmp/realsense_calib_ros_params.yaml` ke `config/detector.yaml`:

```yaml
# ── Camera Intrinsics ─────────────────────────
camera_matrix:
  - 635.812345   # fx
  - 0.000000
  - 640.123456   # cx
  - 0.000000
  - 635.456789   # fy
  - 364.789012   # cy
  - 0.000000
  - 0.000000
  - 1.000000
dist_coeffs: [-0.054321, 0.065432, 0.000123, -0.000045, -0.019876]
```

---

## Menilai Kualitas Kalibrasi

| Reprojection Error | Kualitas |
|-------------------|---------|
| < 0.5 px | Sangat baik |
| 0.5 – 1.0 px | Baik, layak terbang |
| 1.0 – 2.0 px | Cukup, pose estimate kurang presisi |
| > 2.0 px | Buruk — ulangi dengan lebih banyak variasi |

---

## Verifikasi

Setelah menyimpan ke `detector.yaml`, jalankan dan cek pose:

```bash
source install/setup.bash
ros2 launch fiducial_detector realsense.launch.xml \
  width:=1280 height:=720 fps_limit:=15 show_window:=true
ros2 topic echo /fiducial/pose
```

Pegang marker 5×5 cm di depan kamera pada jarak 50 cm.
Nilai `position.z` harus sekitar **0.50** (dalam meter).

Untuk board dengan marker kecil, cek juga terminal. Jika rescue aktif akan muncul:

```text
Predictive small-tab detected ID=...
```

Jika marker kecil hanya terbaca saat dekat, itu biasanya bukan masalah kalibrasi saja. Cek pencahayaan, motion blur, resolusi piksel marker kecil, dan coba `fps_limit:=10`.

---

## Troubleshooting

**`/camera/camera/color/image_raw` tidak muncul:**
```bash
# Cek device terdeteksi
rs-enumerate-devices | grep Serial

# Pastikan USB 3.0 — USB 2.0 tidak cukup bandwidth untuk color 1280×720
lsusb -t | grep -i intel
```

**Chessboard tidak terdeteksi di jendela kalibrasi:**
- Seluruh papan harus terlihat (tidak terpotong frame)
- Hindari cahaya langsung yang memantul di kertas
- Pastikan ukuran `chess_cols` dan `chess_rows` sesuai papan yang dicetak

**Reproj error tinggi (> 1.5 px):**
- Tambah frame dari sudut lebih bervariasi
- Hanya `SPACE` saat gambar benar-benar tajam (tidak motion blur)
- Ukur ulang `chess_square` dengan penggaris fisik

**Marker kecil sulit terbaca dari jauh:**
- Gunakan resolusi `1280x720`
- Coba `fps_limit:=15` atau `fps_limit:=10` untuk mengurangi blur/exposure issue
- Tambahkan cahaya menyebar, hindari glare langsung pada kertas
- Pastikan marker kecil terlihat minimal sekitar 70–100 px per sisi di window

**RealSense tidak bisa dibuka (permission denied):**
```bash
sudo usermod -aG plugdev $USER
# logout lalu login ulang
```

---

## Kapan Kalibrasi Ulang

- Kamera pernah jatuh atau benturan keras
- Estimasi jarak marker meleset > 5% dari jarak ukur fisik
- Kamera baru (cek dulu Metode A, kalibrasi ulang hanya jika reproj > 1.0 px)
- Resolusi streaming diubah (cx, cy berubah — minimal ulangi Metode A)
