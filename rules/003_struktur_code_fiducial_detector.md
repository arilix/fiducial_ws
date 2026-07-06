# [003] Struktur Code `fiducial_detector` dan Pola Implementasi Ulang

**Tanggal dibuat:** 2026-07-06  
**Status:** 📘 Reference / Architecture Guide  
**Tujuan:** Menjelaskan struktur code agar bisa ditiru untuk project ROS 2 + OpenCV berikutnya dengan rapi.

---

## 1. Gambaran Besar

Project ini adalah package ROS 2 C++ untuk mendeteksi marker fiducial ArUco, menghitung pose, menilai confidence, membuat debug visualization, lalu mem-publish hasilnya ke topic ROS.

Secara arsitektur, project ini dibagi menjadi 4 lapisan:

```text
launch/config
  ↓
main executable
  ↓
ROS node orchestration
  ↓
core logic modules
```

Pembagian ini bagus karena logika deteksi tidak dicampur semua ke `main()`. File `main()` hanya menjalankan node, sedangkan detail algoritma berada di class dan modul terpisah.

---

## 2. Struktur Folder

```text
src/fiducial_detector/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── detector.yaml
│   └── params.yaml
├── launch/
│   ├── calibration.launch.xml
│   ├── dict_test.launch.xml
│   ├── realsense.launch.xml
│   ├── ros_topic.launch.xml
│   └── webcam.launch.xml
├── src/
│   ├── core/
│   │   ├── aruco.cpp
│   │   ├── benchmark_runner.cpp
│   │   ├── confidence_system.cpp
│   │   ├── detector_parameters.cpp
│   │   ├── dictionary_manager.cpp
│   │   ├── fps_monitor.cpp
│   │   ├── gate_alignment.cpp
│   │   ├── marker_decoder.cpp
│   │   ├── marker_generator.cpp
│   │   └── pose_estimator.cpp
│   ├── main/
│   │   ├── calibration_main.cpp
│   │   ├── capture_main.cpp
│   │   ├── dict_test_main.cpp
│   │   └── main.cpp
│   └── nodes/
│       ├── calibration_node.cpp
│       ├── capture_node.cpp
│       └── visualization.cpp
└── utils/
    ├── aruco.h
    ├── benchmark_runner.h
    ├── calibration_node.h
    ├── capture_node.h
    ├── confidence_system.h
    ├── detector_parameters.h
    ├── dictionary_manager.h
    ├── fps_monitor.h
    ├── gate_alignment.h
    ├── marker_decoder.h
    ├── marker_generator.h
    ├── pose_estimator.h
    └── visualization.h
```

### Makna setiap folder

| Folder | Fungsi | Isi ideal |
|--------|--------|-----------|
| `config/` | Parameter runtime | YAML untuk topic, ukuran marker, tuning detector, kamera |
| `launch/` | Cara menjalankan sistem | Launch RealSense, webcam, existing topic, calibration |
| `src/main/` | Entry point executable | `main()` kecil, hanya init ROS dan spin node |
| `src/nodes/` | Integrasi ROS khusus node | Subscribe, publish, node tambahan seperti camera/calibration |
| `src/core/` | Logika utama non-trivial | Deteksi, pose, confidence, alignment, dictionary, FPS |
| `utils/` | Header public package | Deklarasi class, struct, interface antar modul |

Catatan: nama folder `utils/` di project ini sebenarnya berisi header public. Untuk project selanjutnya, nama yang lebih eksplisit bisa memakai `include/<nama_package>/`.

---

## 3. Pola Build di `CMakeLists.txt`

`CMakeLists.txt` memakai pola:

1. Cari dependency dengan `find_package()`.
2. Kumpulkan source bersama di `COMMON_SRCS`.
3. Buat beberapa executable.
4. Terapkan include, dependency, dan link library ke target.
5. Install binary, header, launch, dan config.

Contoh pola penting:

```cmake
set(COMMON_SRCS
  src/nodes/visualization.cpp
  src/core/pose_estimator.cpp
  src/core/fps_monitor.cpp
  src/core/detector_parameters.cpp
  src/core/dictionary_manager.cpp
  src/core/confidence_system.cpp
  src/core/marker_decoder.cpp
  src/core/benchmark_runner.cpp
  src/core/gate_alignment.cpp
  src/core/marker_generator.cpp
)

add_executable(aruco_node
  src/main/main.cpp
  src/core/aruco.cpp
  ${COMMON_SRCS}
)
```

Kelebihannya:

- Modul core bisa dipakai ulang oleh beberapa executable.
- `main.cpp` tetap kecil.
- Jika menambah fitur baru, cukup tambahkan `.cpp` ke `COMMON_SRCS` bila dipakai bersama.

Untuk project berikutnya, pakai pola:

```text
COMMON_SRCS = semua modul reusable
add_executable(app_node src/main/main.cpp src/core/app.cpp ${COMMON_SRCS})
add_executable(tool_node src/main/tool_main.cpp ${COMMON_SRCS})
```

---

## 4. Entry Point: `src/main/`

Folder `src/main/` berisi file yang tugasnya hanya membuat executable.

Contoh `main.cpp`:

```cpp
rclcpp::init(argc, argv);
auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
auto node = std::make_shared<fiducial_detector::FiducialDetector>();
executor->add_node(node);
executor->spin();
rclcpp::shutdown();
```

Prinsipnya:

- Jangan taruh algoritma besar di `main()`.
- `main()` hanya mengatur lifecycle program.
- Kalau butuh executable baru, buat `xxx_main.cpp`.
- Kalau butuh node baru, buat class node di `src/nodes/` atau `src/core/`.

Di project ini ada beberapa executable:

| Executable | File main | Fungsi |
|------------|-----------|--------|
| `aruco_node` | `src/main/main.cpp` | Deteksi marker utama |
| `calibration_node` | `src/main/calibration_main.cpp` | Kalibrasi kamera |
| `capture_node` | `src/main/capture_main.cpp` | Publish frame webcam |
| `dict_test_node` | `src/main/dict_test_main.cpp` | Validasi semua ID dictionary |

---

## 5. Node Utama: `FiducialDetector`

Class utama ada di:

```text
utils/aruco.h
src/core/aruco.cpp
```

`FiducialDetector` adalah orkestrator. Dia menghubungkan ROS, OpenCV, parameter, dan modul core.

Tanggung jawabnya:

1. Deklarasi dan load parameter ROS.
2. Load camera intrinsics.
3. Inisialisasi dictionary, detector parameter, pose estimator, confidence, alignment, visualizer.
4. Subscribe image topic.
5. Jalankan pipeline deteksi setiap frame.
6. Publish pose, debug image, alignment JSON, FPS, dan rejected candidate count.

Alur utama di callback:

```text
imageCallback()
  ↓
convert ROS Image → cv::Mat
  ↓
runDetection()
  ↓
estimatePoses()
  ↓
computeConfidence()
  ↓
stabilizeDetections()
  ↓
GateAlignmentEngine::update()
  ↓
Visualizer draw overlay
  ↓
publishAll()
```

Ini pola yang bagus untuk project berikutnya: node utama cukup menjadi "pengatur alur", sedangkan detail algoritma ada di class kecil.

---

## 6. Modul Core dan Tanggung Jawabnya

### `DictionaryManager`

File:

```text
utils/dictionary_manager.h
src/core/dictionary_manager.cpp
```

Tanggung jawab:

- Memilih dictionary ArUco.
- Menyimpan metadata dictionary.
- Validasi dictionary saat startup.
- Menyediakan helper `getDictionaryByName()`, `getDictIdByName()`, dan `activeDict()`.

Di project ini dictionary dikunci ke:

```text
DICT_7X7_50
```

Kenapa dipisah?

- Agar semua urusan dictionary tidak tersebar di banyak file.
- Kalau nanti ganti dictionary, cukup ubah manager ini.
- Bisa validasi lebih awal sebelum pipeline jalan.

### `DetectorParametersManager`

File:

```text
utils/detector_parameters.h
src/core/detector_parameters.cpp
```

Tanggung jawab:

- Deklarasi parameter OpenCV ArUco ke ROS.
- Bind nilai parameter dari ROS ke `cv::aruco::DetectorParameters`.
- Menyediakan profile khusus `apply7x7Profile()`.

Keuntungan:

- Tuning detector tidak mengotori node utama.
- Parameter kritis terdokumentasi dalam satu tempat.
- Profile bisa diganti sesuai jenis marker.

### `PoseEstimator`

File:

```text
utils/pose_estimator.h
src/core/pose_estimator.cpp
```

Tanggung jawab:

- Menerima corner marker.
- Membuat object points berdasarkan ukuran marker fisik.
- Menjalankan `cv::solvePnP()`.
- Menghasilkan `rvec`, `tvec`, quaternion, distance.
- Melakukan smoothing pose per marker ID.

Input:

```text
id + 4 corner marker + camera matrix + dist coeffs
```

Output:

```text
PoseResult
```

### `ConfidenceCalculator`

File:

```text
utils/confidence_system.h
src/core/confidence_system.cpp
```

Tanggung jawab:

- Menghitung confidence dari beberapa aspek:
  - Hamming confidence.
  - Reprojection confidence.
  - Pose stability.
  - Contour confidence.
- Menghasilkan nilai aggregate confidence.

Kenapa penting?

- Deteksi marker kadang benar secara ID tetapi corner/pose tidak stabil.
- Confidence membantu sistem downstream memilih hasil yang layak dipakai.

### `GateAlignmentEngine`

File:

```text
utils/gate_alignment.h
src/core/gate_alignment.cpp
```

Tanggung jawab:

- Menghitung error marker terhadap center frame.
- Menyimpan state alignment.
- Mengubah state berdasarkan jumlah frame stabil.

State yang dipakai:

```text
SEARCH → DETECTED → TRACKING/CENTERING → ALIGNED → GATE_READY
```

Output utama:

```text
GateError
```

yang bisa diubah menjadi JSON lewat:

```cpp
GateError::toJson()
```

### `Visualizer`

File:

```text
utils/visualization.h
src/nodes/visualization.cpp
```

Tanggung jawab:

- Draw marker.
- Draw rejected candidate.
- Draw pose axis.
- Draw alignment box.
- Draw HUD/debug overlay.

Meskipun berada di `src/nodes/`, isinya lebih mirip modul visualisasi. Untuk project berikutnya, file seperti ini bisa ditempatkan di `src/core/visualization.cpp` atau `src/visualization/`.

### `FpsMonitor`

File:

```text
utils/fps_monitor.h
src/core/fps_monitor.cpp
```

Tanggung jawab:

- Hitung FPS.
- Hitung latency.
- Dipakai untuk logging dan publish `/fiducial/fps`.

### `MarkerDecoder`, `MarkerGenerator`, `BenchmarkRunner`

Tanggung jawab umum:

- `MarkerDecoder`: decode/analisis bit marker dan debug image.
- `MarkerGenerator`: generate marker untuk testing/cetak.
- `BenchmarkRunner`: pengujian performa atau skenario deteksi.

Pola ini bagus: fitur tambahan tetap dipisah dari node utama.

---

## 7. Alur Data Runtime

```text
Camera / image source
  ↓
ROS topic: /camera/image_raw
  ↓
FiducialDetector::imageCallback()
  ↓
cv_bridge: ROS Image → OpenCV Mat
  ↓
preprocessFrame()
  - grayscale
  - CLAHE optional
  - blur optional
  - sharpen optional
  - CUDA optional jika binary dibuild dengan USE_CUDA
  ↓
detectAruco()
  - OpenCV detectMarkers()
  - splitCombinedBoard()
  - detectTopTabMarkers()
  - full-frame upscale rescue untuk marker kecil
  ↓
estimatePoses()
  - solvePnP
  - smoothing pose
  ↓
computeConfidence()
  - reprojection
  - contour
  - stability
  ↓
stabilizeDetections()
  - smoothing corner
  - hold marker yang hilang sementara
  ↓
GateAlignmentEngine
  - error_x, error_y
  - alignment state
  ↓
Visualizer
  - annotated image
  ↓
ROS publishers
```

Output ROS:

| Topic | Type | Isi |
|-------|------|-----|
| `/fiducial/pose` | `geometry_msgs/msg/PoseStamped` | Pose marker valid pertama |
| `/fiducial/debug_image` | `sensor_msgs/msg/Image` | Frame dengan overlay |
| `/fiducial/alignment` | `std_msgs/msg/String` | JSON error alignment |
| `/fiducial/fps` | `std_msgs/msg/Float32` | FPS runtime |
| `/fiducial/rejected_candidates` | `std_msgs/msg/String` | Jumlah detected/rejected |

---

## 8. Pola Parameter

Parameter didefinisikan di dua tempat:

1. Default di code melalui `declare_parameter()`.
2. Override di YAML `config/detector.yaml`.

Pola ini bagus karena:

- Program tetap bisa jalan tanpa YAML.
- Tuning runtime bisa dilakukan dari launch file.
- Parameter penting mudah dicatat dan dibagikan.

Contoh kelompok parameter:

```yaml
aruco_node:
  ros__parameters:
    marker_size: 0.05
    camera_topic: "/camera/image_raw"
    show_window: false
    alignment_tolerance: 50
    smoothing_alpha: 0.4
    camera_matrix: [...]
    dist_coeffs: [...]
```

Untuk project berikutnya, kelompokkan parameter seperti ini:

```text
Core
Display
Tracking
Camera Intrinsics
Detector Parameters
Preprocessing
Hardware / Capture
```

---

## 9. Pola Launch File

Folder `launch/` memisahkan skenario run:

| Launch file | Kapan dipakai |
|-------------|---------------|
| `webcam.launch.xml` | Kamera USB lokal via OpenCV/V4L2 |
| `realsense.launch.xml` | RealSense driver + detector |
| `ros_topic.launch.xml` | Image topic sudah dipublish node lain |
| `calibration.launch.xml` | Kalibrasi kamera |
| `dict_test.launch.xml` | Validasi dictionary |

Ini pola yang rapi karena setiap skenario punya launch sendiri. Jangan membuat satu launch file terlalu banyak cabang jika skenarionya sudah berbeda jelas.

Template untuk project berikutnya:

```text
<sensor>.launch.xml       → menjalankan sensor + node utama
ros_topic.launch.xml      → hanya subscribe topic existing
calibration.launch.xml    → tool kalibrasi
test.launch.xml           → tool validasi/debug
```

---

## 10. Cara Menambah Fitur Baru dengan Rapi

Misal ingin menambah fitur `MotionPredictor`.

### Langkah 1 - Buat header

```text
utils/motion_predictor.h
```

Isi deklarasi class:

```cpp
#pragma once

namespace fiducial_detector {

class MotionPredictor {
public:
    MotionPredictor();
    void reset();
};

} // namespace fiducial_detector
```

### Langkah 2 - Buat implementasi

```text
src/core/motion_predictor.cpp
```

### Langkah 3 - Tambahkan ke `CMakeLists.txt`

Jika dipakai bersama:

```cmake
set(COMMON_SRCS
  ...
  src/core/motion_predictor.cpp
)
```

### Langkah 4 - Tambahkan dependency ke node utama

Di `utils/aruco.h`:

```cpp
#include "utils/motion_predictor.h"
```

Tambahkan member:

```cpp
std::unique_ptr<MotionPredictor> motion_predictor_;
```

### Langkah 5 - Inisialisasi di `initDetectors()`

```cpp
motion_predictor_ = std::make_unique<MotionPredictor>();
```

### Langkah 6 - Panggil di pipeline

Tambahkan fungsi kecil di `FiducialDetector` jika perlu:

```cpp
void predictMotion(DetectionResult& result);
```

Jangan langsung menaruh semua logic di `imageCallback()`. Callback harus tetap menjadi alur besar yang mudah dibaca.

---

## 11. Template Struktur untuk Project Selanjutnya

Untuk project ROS 2 C++ baru, pakai template ini:

```text
my_robot_vision/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── detector.yaml
├── launch/
│   ├── webcam.launch.xml
│   ├── ros_topic.launch.xml
│   └── test.launch.xml
├── src/
│   ├── main/
│   │   ├── main.cpp
│   │   └── test_main.cpp
│   ├── nodes/
│   │   └── camera_node.cpp
│   └── core/
│       ├── detector.cpp
│       ├── detector_parameters.cpp
│       ├── pose_estimator.cpp
│       ├── confidence_system.cpp
│       ├── alignment_engine.cpp
│       └── visualization.cpp
└── include/my_robot_vision/
    ├── detector.h
    ├── detector_parameters.h
    ├── pose_estimator.h
    ├── confidence_system.h
    ├── alignment_engine.h
    └── visualization.h
```

Jika tetap ingin mengikuti project ini apa adanya, ganti `include/my_robot_vision/` menjadi `utils/`.

---

## 12. Prinsip Desain yang Bisa Ditiru

1. **`main()` harus kecil.** Jangan isi `main()` dengan algoritma.
2. **Node utama menjadi orkestrator.** Dia mengatur urutan proses, bukan menyimpan semua detail.
3. **Satu class satu tanggung jawab.** Dictionary, parameter, pose, confidence, alignment, visualisasi dipisah.
4. **Parameter runtime masuk YAML.** Jangan hardcode semua tuning di tengah pipeline.
5. **Core logic bisa dites tanpa ROS jika memungkinkan.** Class seperti `PoseEstimator` dan `GateAlignmentEngine` tidak harus tergantung topic ROS.
6. **Launch file dipisah per skenario.** Webcam, RealSense, topic existing, calibration, dan test punya file sendiri.
7. **Debug output dibuat eksplisit.** Publish debug image, FPS, rejected count, dan alignment state.
8. **Validasi dibuat sebagai executable sendiri.** `dict_test_node` adalah contoh tool kecil untuk memastikan dictionary valid.
9. **State machine dipisah.** Alignment tidak ditulis acak di callback, tetapi dibungkus dalam `GateAlignmentEngine`.
10. **Rescue logic tetap dibungkus fungsi.** Contoh: `splitCombinedBoard()` dan `detectTopTabMarkers()`.

---

## 13. Hal yang Perlu Dirapikan Jika Dipakai sebagai Template

Beberapa hal di project ini masih bisa dibuat lebih rapi untuk project berikutnya:

1. **Gunakan folder `include/<package_name>/` daripada `utils/`.** Ini lebih umum di ROS 2 C++.
2. **Pindahkan `visualization.cpp` ke `src/core/` atau `src/visualization/`.** Saat ini ia ada di `src/nodes/`, padahal bukan ROS node.
3. **Pisahkan pipeline deteksi dari ROS node.** Misalnya buat class `DetectionPipeline` agar `FiducialDetector` lebih ringan.
4. **Samakan `params.yaml` dengan `detector.yaml`.** `params.yaml` masih punya parameter lama seperti `dictionary_type`, `enable_apriltag`, dan `enable_charuco`.
5. **Gunakan structured JSON library jika output alignment makin kompleks.** Saat ini JSON dibuat manual dengan `snprintf()`.
6. **Kurangi hardcoded profile jika ingin multi-dictionary.** Sekarang desainnya sengaja fokus ke `DICT_7X7_50`.

---

## 14. Checklist Saat Membuat Project Baru

Gunakan checklist ini:

```text
[ ] Buat package ROS 2.
[ ] Tentukan node utama dan nama executable.
[ ] Buat folder config, launch, src/main, src/core, src/nodes, include.
[ ] Pastikan main.cpp hanya init node dan spin.
[ ] Buat class node utama sebagai orkestrator.
[ ] Pisahkan parameter manager.
[ ] Pisahkan algoritma utama ke module core.
[ ] Pisahkan visualisasi/debug.
[ ] Buat launch per skenario run.
[ ] Buat YAML parameter default.
[ ] Buat tool validasi kecil bila ada dictionary/model/calibration.
[ ] Tambahkan install target untuk binary, config, launch, dan header.
[ ] Dokumentasikan topic input/output.
```

---

## 15. Ringkasan Pola Terbaik

Kalau diringkas, struktur rapi project ini adalah:

```text
main.cpp
  hanya menjalankan node

FiducialDetector
  mengatur pipeline dan komunikasi ROS

Core modules
  menyimpan algoritma spesifik

YAML config
  menyimpan tuning runtime

Launch files
  menyimpan cara menjalankan tiap skenario

Rules/docs
  menyimpan problem, solusi, dan keputusan desain
```

Pola ini cocok dipakai ulang untuk project vision lain seperti:

- line detection,
- object tracking,
- landing pad detection,
- obstacle detection,
- pose estimation,
- visual servoing,
- gate traversal.

Kunci utamanya: pisahkan "cara program dijalankan", "komunikasi ROS", dan "algoritma inti". Kalau tiga hal ini tidak dicampur, project akan jauh lebih mudah dipelihara dan dibawa ke project selanjutnya.

