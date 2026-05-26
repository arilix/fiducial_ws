# Panduan Menjalankan Fiducial Detector (ROS 2 Humble)

Dokumen ini berisi panduan singkat untuk menjalankan *node* pendeteksi multi-marker (`fiducial_detector`).

## 1. Kompilasi (Build) Workspace

Jika Anda baru pertama kali menggunakan package ini atau baru saja melakukan perubahan kode, Anda harus melakukan kompilasi terlebih dahulu. Buka terminal dan jalankan:

```bash
cd ~/Documents/vtol/vtol\ aruco/fiducial_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select fiducial_detector
```

## 2. Source Setup File

Setiap kali Anda membuka terminal baru untuk menjalankan *node* ROS 2 dari workspace ini, Anda perlu me-load environment-nya:

```bash
cd ~/Documents/vtol/vtol\ aruco/fiducial_ws
source install/setup.bash
```

## 3. Pilihan Cara Menjalankan (Launch)

Ada dua skenario utama untuk menjalankan sistem ini, tergantung pada sumber gambar kamera Anda.

### Opsi A: Menggunakan USB Webcam Langsung
Gunakan perintah ini jika Anda menggunakan webcam laptop bawaan atau kamera USB eksternal yang terhubung ke PC/Mini-PC:

```bash
ros2 launch fiducial_detector webcam.launch.py device:=/dev/video0
```

> **Catatan:** 
> - Ubah `/dev/video0` sesuai dengan ID kamera yang terbaca di sistem Anda (cek dengan `ls /dev/video*`).
> - Anda dapat mengatur ukuran fisik marker (dalam satuan meter) dengan menambahkan parameter, contohnya: `marker_size:=0.08` (untuk marker 8 cm).

### Opsi B: Menggunakan Topik Kamera ROS 2 yang Sudah Ada
Gunakan perintah ini jika kamera Anda sudah dijalankan oleh *node* lain (misalnya dari driver kamera realsense, simulasi gazebo, dll), sehingga Anda hanya perlu men-subscribe topik gambarnya:

```bash
ros2 launch fiducial_detector ros_topic.launch.py camera_topic:=/camera/color/image_raw marker_size:=0.05
```

> **Catatan:**
> - Ubah `/camera/color/image_raw` sesuai dengan topik gambar yang ada di sistem Anda (cek dengan `ros2 topic list`).

## 4. Memantau Hasil Deteksi

Setelah *node* berjalan, Anda dapat memantau hasil deteksi secara real-time pada topik-topik berikut:

- **Pose Estimasi 6DOF:** `/fiducial/pose` (Tipe data: `geometry_msgs/PoseStamped`)
- **Gambar Hasil Debug (Visualisasi):** `/fiducial/debug_image` (Tipe data: `sensor_msgs/Image`)
- **Status Penyelarasan (Alignment):** `/fiducial/alignment` (Tipe data: `std_msgs/String`)

Untuk melihat gambar visualisasi secara langsung, buka terminal baru, jalankan `source /opt/ros/humble/setup.bash`, lalu jalankan:
```bash
ros2 run rqt_image_view rqt_image_view
```
Lalu pilih topik `/fiducial/debug_image` pada antarmuka *rqt*.
