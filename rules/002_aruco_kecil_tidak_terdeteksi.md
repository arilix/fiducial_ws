# [002] ArUco Kecil Tidak Terdeteksi (Small Marker Detection Failure)

**Tanggal ditemukan:** 2026-07-05  
**Status:** ✅ Solved / Tuned (Implemented 2026-07-05, tuned live RealSense 2026-07-05)
**Komponen terdampak:** `src/core/aruco.cpp`, `src/core/detector_parameters.cpp`  
**Prioritas:** 🔴 KRITIS (syarat lomba: marker kecil di dalam board wajib terdeteksi)

---

## 📋 Problem Statement

Program **tidak dapat mendeteksi marker ArUco berukuran kecil** yang ada di dalam board lomba. Board lomba memiliki kombinasi marker besar (utama) dan marker kecil (tab/sub-marker) yang ditempatkan menempel pada atau berdekatan dengan marker utama.

Meskipun sudah ada 3 fungsi baru yang ditambahkan (`splitCombinedBoard`, `detectTopTabMarkers`, `stabilizeDetections`), **marker kecil masih sering gagal terdeteksi** atau hanya terdeteksi sesekali (flicker).

### Gejala yang Diamati

```
# Marker besar (utama) terdeteksi dengan baik:
[Board] 1 marker(s) terdeteksi | FPS=25.0 | Lat=40ms
  [ID=3] Conf=87%  XYZ=(0.120, -0.005, 0.450)m  Dist=0.466m

# Tapi marker kecil yang menempel di atas marker besar → TIDAK terdeteksi
# Padahal secara visual terlihat jelas di kamera
```

---

## 🔍 Root Cause Analysis

### Penyebab #1 — Resolusi Piksel Terlalu Rendah untuk Marker Kecil

Marker kecil pada jarak normal (0.5–1.5m) hanya menempati **20–50 piksel** di frame kamera.
DICT_7X7_50 membutuhkan grid 7×7 + 1 border bit per sisi = total grid 9×9 piksel minimum.
Dengan hanya 20–50 piksel, setiap "cell" dalam marker hanya mendapat **2–5 piksel** — terlalu sedikit untuk thresholding yang andal.

```
Marker 7x7 butuh 9x9 grid (dengan border):

Resolusi marker di frame kamera:
┌─────────────────────────────┐
│ ~50px = 5.5px per cell      │ ← ambang batas, sering gagal
│ ~30px = 3.3px per cell      │ ← hampir pasti gagal
│ ~80px = 8.9px per cell      │ ← biasanya terdeteksi
└─────────────────────────────┘

Minimum yang dibutuhkan: ~6px per cell → ~54px total marker side
```

### Penyebab #2 — `detectTopTabMarkers()` Upscale Belum Cukup Agresif

Fungsi `detectTopTabMarkers()` saat ini menggunakan `TAB_SCALE = 4.0` dan `INTER_CUBIC`. Ini sudah cukup baik, **tapi ada beberapa masalah**:

1. **ROI estimasi posisi tab terlalu rigid** — Hanya mencari di atas marker besar (`y = marker.y - 34% height`). Jika marker kecil ada di posisi lain (samping, bawah, tumpang tindih), ia terlewat.

2. **Filter `min_big_side`** terlalu ketat — `std::min(gray.cols, gray.rows) / 8`. Pada resolusi 640×480, ini = 60px. Marker besar harus >60px agar tab-nya dicari. Jika kamera agak jauh, marker besar bisa <60px dan tab search dilewati sepenuhnya.

3. **Parameter `perspectiveRemovePixelPerCell = 18`** di tab detection — Ini tinggi dan bagus untuk resolusi crop yang sudah di-upscale, tapi jika upscale-nya menghasilkan gambar yang blurry (interpolasi), cell sampling bisa mengambil piksel yang sudah ter-interpolasi (blurred boundary).

### Penyebab #3 — `splitCombinedBoard()` Upscale Hanya 2x untuk ROI Kecil

```cpp
// Baris 617-618 di aruco.cpp saat ini:
const double roi_scale =
    std::max(crop.cols, crop.rows) < 170 ? 2.0 : 1.0;
```

Threshold 170px dan scale 2.0 mungkin tidak cukup. Marker kecil yang hanya menempati 30px di ROI, setelah di-upscale 2x menjadi 60px — **masih marginal** untuk 7x7 detection.

### Penyebab #4 — Adaptive Threshold Window Terlalu Besar untuk Marker Kecil

Di `apply7x7Profile()`, window threshold max = 53. Untuk marker kecil (30-50px), window besar ini mencakup area di luar marker → threshold lokal terdistorsi oleh latar belakang.

### Penyebab #5 — `minMarkerPerimeterRate` Masih Terlalu Tinggi

Saat ini `minMarkerPerimeterRate = 0.015`. Untuk frame 640×480:
```
min perimeter = 0.015 × max(640, 480) × 4 = 0.015 × 640 × 4 = 38.4 piksel
min side      = 38.4 / 4 ≈ 9.6 piksel
```

Ini secara teori cukup kecil, **TAPI** di `splitCombinedBoard()` ROI yang sudah di-crop, dimensi frame-nya jauh lebih kecil:
```
ROI crop = 120×120 piksel
min perimeter = 0.006 × 120 × 4 = 2.88 piksel  ← param di split
marker kecil di crop = ~30px → perimeter = 120px → SEHARUSNYA cukup

Tapi jika crop ROI terlalu ketat dan marker kecil terpotong di edge → GAGAL
```

### Penyebab #6 — Marker Kecil Kehilangan Quiet Zone Saat Menempel

Pada gambar `Aruco/Aruco-ketentuan.jpg`, marker kecil valid dan memakai `DICT_7X7_50`, tetapi sebagian marker kecil tidak terbaca ketika menempel ke marker besar. Penyebabnya bukan dictionary, melainkan **quiet zone putih di sisi sambungan hilang/menyatu dengan area hitam marker besar**.

Solusi yang dipakai:
- crop prediktif di sekitar tab atas/bawah marker besar,
- padding putih buatan di sekitar crop,
- preprocessing `equalizeHist` + sharpen + Otsu + inverted binary fallback,
- dedupe marker berdasarkan ID dan jarak center.

### Penyebab #7 — Jarak, Cahaya, dan Motion Blur

Saat board jauh, marker kecil hanya punya sedikit piksel. Deteksi bisa berhasil dekat kamera tetapi sulit saat jauh karena:
- resolusi marker kecil turun di bawah ~70 px per sisi,
- blur dari gerakan tangan/board,
- exposure pendek pada FPS tinggi,
- noise karena cahaya redup,
- glare pada kertas putih.

Solusi praktis: RealSense `1280x720`, coba `fps_limit:=15` atau `10`, tambah cahaya menyebar, dan jaga board tidak terlalu miring.

---

## ✅ Solusi & Strategi yang Direkomendasikan

### Solusi A — Perbesar Scale Factor di `splitCombinedBoard()` (MUDAH)

Ubah logic upscale agar lebih agresif untuk ROI yang sangat kecil:

```cpp
// SEBELUM (baris 617-618):
const double roi_scale =
    std::max(crop.cols, crop.rows) < 170 ? 2.0 : 1.0;

// SESUDAH — multi-tier upscale:
const int max_dim = std::max(crop.cols, crop.rows);
double roi_scale;
if      (max_dim < 60)  roi_scale = 5.0;   // sangat kecil → upscale agresif
else if (max_dim < 100) roi_scale = 4.0;   // kecil
else if (max_dim < 170) roi_scale = 2.5;   // medium-kecil
else                    roi_scale = 1.0;   // cukup besar

// Ganti INTER_LINEAR dengan INTER_CUBIC untuk upscale besar:
int interp = (roi_scale > 2.0) ? cv::INTER_CUBIC : cv::INTER_LINEAR;
cv::resize(padded, detect_img, cv::Size(), roi_scale, roi_scale, interp);
```

**Alasan:** Marker kecil butuh setidaknya ~70px per sisi setelah upscale agar 7x7 grid bisa di-sampling dengan baik. INTER_CUBIC menghasilkan edge yang lebih tajam dibanding INTER_LINEAR pada upscale besar.

### Solusi B — Perluas Area Pencarian `detectTopTabMarkers()` (MEDIUM)

Saat ini tab hanya dicari **di atas** marker besar. Perluas ke **4 sisi**:

```cpp
// SEBELUM — hanya top:
const int y = marker_rect.y - static_cast<int>(marker_rect.height * 0.34);

// SESUDAH — cari di 4 sisi (top, bottom, left, right):
struct TabRegion { int x, y, w, h; };
std::vector<TabRegion> tabs = {
    // Top tab
    { marker_rect.x + marker_rect.width/2 - tab_width/2,
      marker_rect.y - static_cast<int>(marker_rect.height * 0.34),
      tab_width, tab_height },
    // Bottom tab
    { marker_rect.x + marker_rect.width/2 - tab_width/2,
      marker_rect.y + marker_rect.height - static_cast<int>(marker_rect.height * 0.08),
      tab_width, tab_height },
    // Left tab
    { marker_rect.x - static_cast<int>(marker_rect.width * 0.34),
      marker_rect.y + marker_rect.height/2 - tab_height/2,
      tab_height, tab_width },
    // Right tab
    { marker_rect.x + marker_rect.width - static_cast<int>(marker_rect.width * 0.08),
      marker_rect.y + marker_rect.height/2 - tab_height/2,
      tab_height, tab_width },
};
// Buat ROI untuk masing-masing dan jalankan detection
```

### Solusi C — Tambahkan Preprocessing Khusus untuk Crop Kecil (MEDIUM)

Sebelum menjalankan `detectMarkers()` di ROI kecil, terapkan preprocessing tambahan:

```cpp
// Setelah upscale, terapkan sharpening ringan untuk memulihkan edge:
cv::Mat sharpened;
cv::GaussianBlur(upscaled, sharpened, cv::Size(0, 0), 1.5);
cv::addWeighted(upscaled, 1.8, sharpened, -0.8, 0, sharpened);

// Opsional: Otsu threshold untuk memaksimalkan kontras hitam-putih
cv::Mat binary;
cv::threshold(sharpened, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

// Coba detect di kedua versi (sharpened grayscale + binary)
```

### Solusi D — Turunkan `min_big_side` Filter (MUDAH)

```cpp
// SEBELUM (baris 650):
const int min_big_side = std::min(gray.cols, gray.rows) / 8;  // 60px @ 480p

// SESUDAH — lebih permisif:
const int min_big_side = std::min(gray.cols, gray.rows) / 12;  // 40px @ 480p
```

Ini memungkinkan pencarian tab bahkan ketika marker besar terlihat kecil di kamera (jarak lebih jauh).

### Solusi E — Multi-Scale Detection Pass (ADVANCED)

Tambahkan detection pass pada gambar yang sudah di-upscale secara keseluruhan (bukan per-ROI):

```cpp
// Di detectAruco(), setelah splitCombinedBoard dan detectTopTabMarkers:
// Pass tambahan: upscale seluruh frame 1.5x, deteksi ulang
if (ids.size() < 3) {  // threshold: jika belum banyak marker terdeteksi
    cv::Mat upscaled_full;
    cv::resize(gray, upscaled_full, cv::Size(), 1.5, 1.5, cv::INTER_CUBIC);

    auto dp_small = cloneDetectorParams(det_params_mgr_->params());
    dp_small->minMarkerPerimeterRate = 0.008;
    dp_small->adaptiveThreshWinSizeMax = 25;  // lebih kecil untuk marker kecil
    dp_small->adaptiveThreshWinSizeStep = 2;

    std::vector<int> small_ids;
    std::vector<std::vector<cv::Point2f>> small_corners, small_rejected;
    cv::aruco::detectMarkers(upscaled_full, aruco_dict_, small_corners,
        small_ids, dp_small, small_rejected);

    for (std::size_t i = 0; i < small_ids.size(); ++i) {
        auto mapped = small_corners[i];
        for (auto& pt : mapped) {
            pt.x /= 1.5f;
            pt.y /= 1.5f;
        }
        appendUniqueMarker(corners, ids, std::move(mapped), small_ids[i]);
    }
}
```

---

## ✅ Implementasi 2026-07-05

Perubahan yang sudah diterapkan:

1. `splitCombinedBoard()` memakai **multi-tier ROI upscale**:
   - `<60px → 5x`
   - `<100px → 4x`
   - `<170px → 2.5x`
   - besar cukup → 1x
2. `detectTopTabMarkers()` sekarang mencari tab di **4 sisi** marker besar: top, bottom, left, right.
3. Filter `min_big_side` diturunkan dari `frame/8` menjadi `frame/12`.
4. Crop kecil diberi preprocessing tambahan:
   - histogram equalization
   - `INTER_CUBIC` upscale 4x
   - sharpening ringan
   - fallback Otsu binary jika grayscale gagal
5. Ditambahkan fallback **full-frame 1.5x** hanya saat marker terdeteksi masih kurang dari 3.
6. Ditambahkan **predictive small-tab crop** untuk tab atas dan bawah marker besar.
7. Stabilizer menahan marker kecil lebih lama agar hasil rescue tidak flicker.

Verifikasi lokal:

```bash
colcon build --packages-select fiducial_detector --cmake-args -DCMAKE_BUILD_TYPE=Release -DUSE_REALSENSE=OFF
ros2 run fiducial_detector dict_test_node
```

Hasil: `50/50 PASS`.

---

## 📊 Analisa Kode Saat Ini — Perubahan Baru yang Sudah Ada

| Fungsi Baru | Status | Keterangan |
|-------------|--------|------------|
| `splitCombinedBoard()` (baris 539-640) | ✅ Sudah ada | ROI-based detection, upscale 2x untuk ROI <170px |
| `detectTopTabMarkers()` (baris 642-717) | ✅ Sudah ada | Upscale 4x, cari tab di atas marker besar |
| `stabilizeDetections()` | ✅ Sudah ada | EMA smoothing corners + hold marker kecil 14 frame |
| `cloneDetectorParams()` (baris 12-18) | ✅ Sudah ada | Deep copy parameter untuk multi-pass |
| `appendUniqueMarker()` (baris 55-73) | ✅ Sudah ada | Deduplikasi berdasarkan ID + jarak |
| Predictive small-tab crop | ✅ Sudah ada | Cari tab atas/bawah dengan variasi ukuran, gap, dan offset X |

### Bagian yang Perlu Ditingkatkan

| Area | Masalah | Solusi Ref |
|------|---------|------------|
| `splitCombinedBoard` baris 617-618 | Upscale hanya 2x, kurang untuk marker <50px | Solusi A |
| `detectTopTabMarkers` baris 649-665 | Hanya cari di atas (top), tab bisa di posisi lain | Solusi B |
| `detectTopTabMarkers` baris 650-651 | `min_big_side` = frame/8 (60px), terlalu ketat | Solusi D |
| Tidak ada preprocessing setelah upscale | Edge blur setelah interpolasi | Solusi C |
| Tidak ada full-frame upscale pass | Marker kecil di area tanpa ROI terlewat | Solusi E |

---

## 🧪 Cara Verifikasi

### Test 1 — Marker Kecil Terpisah

```bash
# Cetak marker 7x7 kecil (3cm × 3cm), letakkan di jarak 0.5m
# Harus terdeteksi tanpa marker besar di dekatnya
ros2 launch fiducial_detector realsense.launch.xml show_window:=true
```

Ekspektasi: `[ID=X] Conf=XX%` muncul di terminal.

### Test 2 — Marker Kecil Menempel di Marker Besar

```bash
# Gunakan board lomba asli: marker kecil menempel di atas marker besar
# Tunjukkan ke kamera jarak 0.5–1.5m
```

Ekspektasi:
- `[Board] 2+ marker(s) terdeteksi`
- Kedua marker (besar + kecil) punya bounding box hijau

### Test 3 — Jarak Jauh

```bash
# Tunjukkan board di jarak 2m+
# Marker kecil akan sangat kecil di frame (~15-25px)
```

Ekspektasi: Setidaknya marker besar terdeteksi. Marker kecil mungkin hanya terdeteksi secara intermittent — ini normal. `stabilizeDetections()` men-hold marker kecil selama 14 frame.

### Test 4 — RealSense untuk Marker Kecil

```bash
ros2 launch fiducial_detector realsense.launch.xml \
  width:=1280 height:=720 fps_limit:=15 show_window:=true
```

Ekspektasi terminal saat rescue aktif:

```text
Predictive small-tab detected ID=...
```

Jika hanya terbaca dekat kamera, coba:

```bash
ros2 launch fiducial_detector realsense.launch.xml \
  width:=1280 height:=720 fps_limit:=10 show_window:=true
```

### Test 5 — Debug Window ROI

Untuk verifikasi bahwa ROI crop dan upscale bekerja, tambahkan debug `imshow` sementara:

```cpp
// Di splitCombinedBoard(), setelah baris 622:
if (show_window_) {
    cv::imshow("ROI-" + std::to_string(rois.size()), detect_img);
}

// Di detectTopTabMarkers(), setelah baris 700:
if (show_window_) {
    cv::imshow("TabROI-" + std::to_string(tab_rois.size()), upscaled);
}
```

---

## 📊 Parameter Kritis untuk Marker Kecil

| Parameter | Nilai Saat Ini | Rekomendasi | Di Mana |
|-----------|---------------|-------------|---------|
| `roi_scale` (split) | 2.0 (jika <170px) | 2.5-5.0 multi-tier | `splitCombinedBoard()` baris 617 |
| `TAB_SCALE` (tab) | 4.0 | 4.0 (OK, bisa 5.0 jika masih gagal) | `detectTopTabMarkers()` baris 698 |
| `min_big_side` | frame/8 (=60px) | frame/12 (=40px) | `detectTopTabMarkers()` baris 650 |
| `perspectiveRemovePixelPerCell` (tab) | 18 | 14-18 (turunkan jika terlalu strict) | `detectTopTabMarkers()` baris 681 |
| `maxErroneousBitsInBorderRate` (tab) | 0.50 | 0.50-0.55 | `detectTopTabMarkers()` baris 683 |
| `errorCorrectionRate` (tab) | 0.75 | 0.75-0.80 (lebih toleran) | `detectTopTabMarkers()` baris 684 |
| `adaptiveThreshWinSizeStep` (tab) | 2 | 2 (OK, sudah rapat) | `detectTopTabMarkers()` baris 674 |
| Interpolation method | INTER_CUBIC | INTER_CUBIC (OK) | `detectTopTabMarkers()` baris 700 |

---

## 🔗 Hubungan dengan Issue Sebelumnya

- **[001]** ArUco tergabung → Sudah diperbaiki dengan `splitCombinedBoard()` dan parameter tuning
- **[002]** ini adalah kelanjutan: meskipun board berhasil di-split, marker **kecil** di dalam split masih terlalu kecil resolusinya untuk terdeteksi → perlu upscale lebih agresif dan preprocessing tambahan

---

## 💡 Catatan dari Analisa Kode

1. **`stabilizeDetections()` sudah bagus** — EMA corner smoothing (alpha=0.55) + hold 14 frame untuk marker kecil membantu marker yang terdeteksi intermittent. Tapi ini hanya membantu **jika marker pernah terdeteksi** setidaknya 1 frame; kualitas input kamera tetap menentukan.

2. **`appendUniqueMarker()` sudah benar** — Menggunakan distance-based deduplication, sehingga marker yang sama terdeteksi di multiple passes tidak terduplikasi.

3. **Logging sudah di-throttle** — `frame_count_ % 10 != 0` (baris 788) mengurangi spam terminal. Ini bagus untuk performance tapi pastikan saat debugging marker kecil, throttle ini diturunkan sementara untuk melihat setiap frame.

---

## 🔧 Implementasi yang Sudah Diterapkan (2026-07-05)

Semua solusi A–E sudah diimplementasikan di `aruco.cpp`:

| Solusi | Perubahan | Keterangan |
|--------|-----------|------------|
| A — Multi-tier upscale | Scale 6x/4.5x/3x/2x + `INTER_CUBIC` + **sharpening** setelah upscale | Sebelumnya 5x/4x/2.5x tanpa sharpening |
| B — 4-sisi tab search | Sudah ada sebelumnya (top/bottom/left/right) | Tidak perlu ubah |
| C — Preprocessing crop | `equalizeHist` untuk ROI <150px + Otsu binary fallback + **inverted binary** fallback | Baru ditambahkan |
| D — `min_big_side` | Sudah frame/12 sebelumnya | Tidak perlu ubah |
| E — Full-frame upscale | Scale naik 1.5x → **2.0x** + sharpening + parameter lebih toleran | Error rate 0.50, correction 0.75 |
| Tab scale | TAB_SCALE adaptive: 6x (<40px) / 5x (<70px) / 4x (sisanya) | Sebelumnya fixed 4x |
| Tab detection | Tambah **inverted binary pass** (3 versi: sharp→binary→inverted) | Sebelumnya hanya 2 versi |
| Predictive top/bottom-tab crop | Crop prediktif di atas dan bawah marker besar | Menangani board real ketika tab kecil berada di atas atau bawah marker besar |
| Predictive ROI variation | Size ratio `0.25/0.30/0.35/0.40`, gap `-0.03/0.02/0.07`, offset X `-0.08/0/0.08`, max 36 ROI tiap rescue frame | Lebih mudah kena saat board miring/posisi tab sedikit meleset tanpa drop FPS berlebihan |
| Predictive scale | Scale crop kecil `5x–7x` + sharpen + binary + inverted | Menaikkan resolusi sampling untuk 7x7 grid |
| Stabilizer key | Tracking dibedakan `ID:big` dan `ID:small` | Mencegah marker besar dan kecil dengan ID sama saling overwrite/flicker |
| Small marker hold | Small marker di-hold 14 frame; big marker 7 frame | Mengurangi flicker ketika tab kecil hanya terbaca intermittent |
| Runtime log | `Predictive small-tab detected ID=...` | Indikator terminal bahwa rescue crop berhasil |
| FPS tuning | Semua ROI rescue kecil jalan tiap 3 frame; predictive ROI max 36; full-frame upscale hanya saat tidak ada marker dan tiap 10 frame | Mengurangi latency dari rescue berat sambil tetap dibantu hold 14 frame |

**Catatan performa:** jika log menunjukkan latency `66–132ms`, bottleneck ada di deteksi CPU, bukan di RealSense. Penyebab utamanya adalah full-frame upscale dan banyak ROI rescue. Versi terbaru membatasi rescue berat agar FPS lebih tinggi.

**Catatan RealSense:** jika startup menulis `Given value, 1280,720,30 is invalid` lalu `Open profile: Color ... 640x480 ... FPS: 15`, maka input kamera memang hanya 15 FPS. Untuk mengejar >20 FPS, gunakan profil yang didukung seperti:

```bash
ros2 launch fiducial_detector realsense.launch.xml \
  width:=640 height:=480 fps_limit:=30 show_window:=true
```

**Build status:** ✅ `colcon build` berhasil (2026-07-05)

**Validasi offline gambar user (2026-07-05):**

```text
Aruco/Aruco-ketentuan.jpg count 8 ids [3, 4, 2, 1, 1, 3, 4, 2]
Aruco/Aruco-pisah.jpg      count 8 ids [3, 4, 2, 1, 2, 1, 3, 4]
```

*Log diperbarui oleh: Antigravity AI | Tanggal: 2026-07-05*
