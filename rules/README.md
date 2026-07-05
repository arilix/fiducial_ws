# 📁 Rules & Troubleshooting Log

Folder ini berisi dokumentasi masalah yang ditemukan selama pengembangan sistem deteksi ArUco VTOL, beserta analisis penyebab dan solusi yang diterapkan.

## 📋 Daftar Masalah

| No | File | Judul | Status |
|----|------|-------|--------|
| 001 | [001_aruco_tergabung_tidak_terdeteksi.md](./001_aruco_tergabung_tidak_terdeteksi.md) | ArUco tergabung (combined board) tidak terdeteksi + warna dibalik | ✅ Solved |
| 002 | [002_aruco_kecil_tidak_terdeteksi.md](./002_aruco_kecil_tidak_terdeteksi.md) | ArUco kecil tidak terdeteksi (small marker detection) | ✅ Solved |

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
