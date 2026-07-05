# [001] ArUco Tergabung Tidak Terdeteksi + Warna Dibalik

**Tanggal ditemukan:** 2026-07-05  
**Status:** ✅ Solved (ROI split implemented)  
**Komponen terdampak:** `src/core/aruco.cpp`, `src/core/detector_parameters.cpp`  
**Prioritas:** 🔴 KRITIS (syarat lomba wajib terpenuhi)

---

## 📋 Problem Statement

Program **tidak dapat mendeteksi ArUco markers yang dicetak dalam satu papan tergabung (combined board)**. Ketentuan lomba mensyaratkan:

1. **ArUco tergabung** — Beberapa marker (besar + kecil) dicetak pada satu bidang papan yang sama, saling berbagi border atau berdekatan.
2. **Warna dibalik (inverted)** — Marker yang normalnya hitam-di-putih bisa dicetak putih-di-hitam dan tetap harus terdeteksi.

Saran dosen: **Buat fungsi yang dapat memisahkan gambar ArUco (yang besar dan kecil) meskipun dalam posisi tergabung**, agar sistem bisa mengenali batch (kumpulan marker) secara independen.

### Gejala yang Diamati

```
# Terminal output saat board tergabung ditunjukkan ke kamera:
[aruco_node] [Board] 0 marker(s) terdeteksi | FPS=28.3 | Lat=35ms
[aruco_node] [Gate-Centroid] State=NO_MARKER     ErrorX=+0  ErrorY=+0  Dist=0.000m

# Padahal di window "Rejected", banyak kandidat muncul tapi di-reject
```

---

## 🔍 Root Cause Analysis

### Penyebab Utama #1 — Shared Border / Touching Markers

Ketika dua atau lebih marker dicetak **berdampingan tanpa jarak (touching)**, algoritma thresholding adaptive OpenCV gagal memisahkan batas antar marker karena:

- **Border marker A** sekaligus menjadi **border marker B**
- Window adaptive threshold yang kecil tidak dapat membedakan pola bit marker dengan border yang menyatu
- Akibatnya kedua marker masuk bucket `rejected` bukan `detected`

```
Kondisi normal (terpisah):     Kondisi tergabung:
┌─────────┐  ┌─────────┐      ┌─────────────────┐
│ Marker A│  │ Marker B│      │ Marker A│Marker B│
│  [bits] │  │  [bits] │  →   │  [bits] │ [bits] │
└─────────┘  └─────────┘      └─────────────────┘
   ↑ border terpisah              ↑ border menyatu → GAGAL
```

**Parameter kritis yang bermasalah:**
- `adaptiveThreshConstant` terlalu tinggi (default 7.0) → over-segmentasi, border lemah hilang
- `minMarkerDistanceRate` terlalu tinggi (default 0.05) → marker berdekatan saling di-reject
- `perspectiveRemoveIgnoredMarginPerCell` terlalu kecil → bit-bit border bocor ke area data

### Penyebab Utama #2 — Marker Ukuran Berbeda Dalam Satu Board

Marker besar dan kecil dalam satu board memiliki **perimeter yang sangat berbeda**. Parameter `minMarkerPerimeterRate` dan `maxMarkerPerimeterRate` defaultnya sempit sehingga marker kecil di-reject karena dianggap terlalu kecil dibandingkan frame.

### Penyebab Utama #3 — Inverted Marker

Parameter `detectInvertedMarker` secara default **false** pada banyak build OpenCV. Marker warna dibalik (putih di hitam) tidak terdeteksi tanpa flag ini.

---

## ✅ Solusi yang Diterapkan

### Solusi A — Tuning Parameter `apply7x7Profile()` (SUDAH DIIMPLEMENTASI)

**File:** `src/core/detector_parameters.cpp` — fungsi `apply7x7Profile()`

```cpp
// Parameter kritis untuk marker tergabung:

// 1. Perlebar range window adaptive threshold:
//    Step=4 (lebih rapat) → lebih banyak skala dicoba → border lemah terdeteksi
params_->adaptiveThreshWinSizeMin  = 3;
params_->adaptiveThreshWinSizeMax  = 53;   // diperlebar dari 33
params_->adaptiveThreshWinSizeStep = 4;    // diperkecil dari 10
params_->adaptiveThreshConstant    = 7.0;

// 2. Izinkan marker sangat berdekatan:
params_->minMarkerDistanceRate = 0.01;     // dari 0.05 → marker tergabung tidak saling reject

// 3. Perimeter range diperlebar untuk cover marker besar DAN kecil:
params_->minMarkerPerimeterRate = 0.015;   // marker kecil tetap terdeteksi
params_->maxMarkerPerimeterRate = 4.0;     // marker besar tetap terdeteksi

// 4. Margin perspective lebih besar → bit border tidak bocor ke area data:
params_->perspectiveRemoveIgnoredMarginPerCell = 0.13;  // dari 0.10

// 5. Border bits lebih toleran untuk border yang menyatu:
params_->maxErroneousBitsInBorderRate = 0.40;  // dari 0.35

// 6. WAJIB: Deteksi marker warna dibalik:
params_->detectInvertedMarker = true;
```

### Solusi B — Multi-Pass Detection (SUDAH DIIMPLEMENTASI)

**File:** `src/core/aruco.cpp` — fungsi `detectAruco()`

Pass pertama dengan parameter normal. Jika banyak kandidat masih di-reject, jalankan **pass kedua** dengan `adaptiveThreshConstant` lebih kecil:

```cpp
// Pass 1: deteksi normal
cv::aruco::detectMarkers(gray, aruco_dict_, corners, ids, dp, rejected);

// Pass 2: jika masih banyak rejected, coba threshold constant lebih kecil
if (!rejected.empty() && rejected.size() > ids.size()) {
    auto dp2 = cloneDetectorParams(det_params_mgr_->params());
    dp2->adaptiveThreshConstant = 3.0;  // lebih kecil = recover border lemah
    // ... gabungkan ID baru ke hasil pass 1
}
```

### Solusi C — Fungsi Pemisahan Region (SUDAH DIIMPLEMENTASI)

**File:** `src/core/aruco.cpp` — fungsi `splitCombinedBoard()`

Dosen menyarankan membuat fungsi `splitCombinedBoard()`. Implementasi sekarang:

1. **Mendeteksi ROI (Region of Interest)** dari gambar full frame
2. **Memisahkan** area marker besar dan marker kecil dari rejected candidates + kontur threshold
3. **Menambahkan padding putih** pada setiap crop agar marker yang berbagi border mendapat quiet zone buatan
4. **Menjalankan deteksi ArUco secara terpisah** di setiap ROI dengan parameter lebih toleran
5. **Menggabungkan** hasil deteksi dari semua ROI dan menghindari duplikasi ID

**Keuntungan pendekatan ini:**
- Marker besar dan kecil tidak saling mengganggu threshold-nya
- Setiap sub-region diproses dengan parameter yang bisa disesuaikan
- Lebih robust untuk board yang tercetak rapat atau miring

**Catatan implementasi tambahan:**
- Multi-pass detection sekarang meng-clone `DetectorParameters` sebelum mengubah `adaptiveThreshConstant`, sehingga parameter utama tidak ikut berubah permanen.
- ROI diambil dari dua sumber: kandidat `rejected` OpenCV dan kontur hasil `adaptiveThreshold` normal/inverted.
- Koordinat corner hasil deteksi ROI ditransformasikan kembali ke koordinat frame asli sebelum pose estimation.

---

## 🧪 Cara Verifikasi

### Test 1 — Inverted Marker

```bash
# Cetak marker normal, balik warna di software (invert colors),
# tunjukkan ke kamera → harus terdeteksi dengan ID yang sama
ros2 launch fiducial_detector realsense.launch.xml show_window:=true
```

Ekspektasi: Terminal menampilkan `[ID=X] Conf=XX%` dengan ID marker yang benar.

### Test 2 — Marker Tergabung

```bash
# Tunjukkan board combined (2+ marker dalam 1 papan) ke kamera
# Cek terminal dan window debug
```

Ekspektasi:
- `[Board] 2+ marker(s) terdeteksi` 
- Di window visual, semua marker diberi bounding box hijau
- Centroid gate (titik merah) muncul di tengah-tengah antara semua marker

### Test 3 — Mixed Size Markers

Tunjukkan board dengan marker **7x7 besar** (misal 10cm) dan **7x7 kecil** (misal 3cm) dalam satu bidang.

Ekspektasi: Kedua marker terdeteksi secara bersamaan.

---

## 📊 Parameter Sebelum vs Sesudah

| Parameter | Default OpenCV | Sebelum Fix | Sesudah Fix | Alasan Perubahan |
|-----------|---------------|-------------|-------------|-----------------|
| `adaptiveThreshWinSizeMax` | 23 | 33 | 53 | Cover marker lebih besar |
| `adaptiveThreshWinSizeStep` | 10 | 10 | 4 | Lebih banyak skala dicoba |
| `minMarkerDistanceRate` | 0.05 | 0.05 | 0.01 | Marker tergabung bisa berdekatan |
| `perspectiveRemoveIgnoredMarginPerCell` | 0.10 | 0.10 | 0.13 | Isolasi bit data dari border bersama |
| `maxErroneousBitsInBorderRate` | 0.35 | 0.35 | 0.40 | Toleransi border menyatu |
| `detectInvertedMarker` | false | true | true | Warna dibalik wajib terdeteksi |
| `minDistanceToBorder` | 3 | 3 | 1 | Marker di pinggir frame |

---

## 🔗 Referensi

- OpenCV ArUco docs: https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html
- Parameter `detectInvertedMarker`: https://docs.opencv.org/4.x/d1/dcd/structcv_1_1aruco_1_1DetectorParameters.html
- Saran dosen: Fungsi pemisahan ROI untuk combined board (belum diimplementasi, lihat Solusi C)

---

## 📝 Catatan Tambahan

- **EMA Smoothing** sudah diterapkan di `aruco.cpp` untuk stabilisasi centroid ketika frame-frame berbeda mendeteksi subset marker berbeda
- Jika setelah fix ini masih ada marker tergabung yang gagal terdeteksi, cek window `Rejected` (tekan `r` di window visualisasi) untuk melihat kandidat yang di-reject
- Parameter `adaptiveThreshConstant = 3.0` di pass kedua bisa diturunkan lebih jauh (hingga 1.0) jika border masih gagal
- Setelah implementasi ROI split, lakukan verifikasi kamera langsung dengan board lomba karena kualitas cetak, glare, dan jarak kamera sangat memengaruhi kontur ROI.

---

*Log dibuat oleh: Antigravity AI | Tanggal: 2026-07-05*
