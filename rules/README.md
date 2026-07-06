# 📁 Rules & Troubleshooting Log

Folder ini berisi dokumentasi masalah yang ditemukan selama pengembangan sistem deteksi ArUco VTOL, beserta analisis penyebab dan solusi yang diterapkan.

## 📋 Daftar Masalah

| No | File | Judul | Status |
|----|------|-------|--------|
| 001 | [001_aruco_tergabung_tidak_terdeteksi.md](./001_aruco_tergabung_tidak_terdeteksi.md) | ArUco tergabung (combined board) tidak terdeteksi + warna dibalik | ✅ Solved |
| 002 | [002_aruco_kecil_tidak_terdeteksi.md](./002_aruco_kecil_tidak_terdeteksi.md) | ArUco kecil tidak terdeteksi (small marker detection) | ✅ Solved / Tuned |
| 003 | [003_struktur_code_fiducial_detector.md](./003_struktur_code_fiducial_detector.md) | Struktur code `fiducial_detector` dan pola implementasi ulang | 📘 Reference |

---

## ✅ Status Terbaru Deteksi

- Dictionary wajib dan aktif: `DICT_7X7_50`.
- Board tergabung ditangani oleh `splitCombinedBoard()` dan ROI rescue.
- Marker kecil yang menempel pada marker besar ditangani oleh `detectTopTabMarkers()` dan predictive top/bottom crop.
- Log runtime untuk marker kecil hasil rescue: `Predictive small-tab detected ID=...`.
- Marker kecil tetap paling sensitif terhadap jarak, cahaya, blur, dan resolusi piksel.

Rekomendasi run RealSense untuk validasi marker kecil:

```bash
ros2 launch fiducial_detector realsense.launch.xml \
  width:=1280 height:=720 fps_limit:=15 show_window:=true
```

Jika ruangan redup atau board blur:

```bash
ros2 launch fiducial_detector realsense.launch.xml \
  width:=1280 height:=720 fps_limit:=10 show_window:=true
```

---

## 🏷️ Konvensi Penamaan

```
NNN_nama_masalah_singkat.md
```

- `NNN` = nomor urut 3 digit (001, 002, dst.)
- Nama file menggunakan snake_case
- Setiap file berisi: **Problem Statement**, **Root Cause Analysis**, **Solution**, **Code Snippet**, **Verifikasi**

---

## 📌 Aturan Penambahan Log Baru

1. Buat file baru di folder ini dengan format nama di atas
2. Tambahkan entry baru ke tabel di atas
3. Isi template dengan lengkap (lihat file yang sudah ada sebagai contoh)
4. Tandai status: `🔍 Investigating` / `🔧 In Progress` / `✅ Solved` / `❌ Wontfix`
