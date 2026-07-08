# fiducial_detector - DICT_7X7_50 Visual Gate Centering

ROS 2 Jazzy workspace untuk deteksi fiducial DICT_7X7_50, pose estimation,
dan gate centering. Paket utama ada di `src/fiducial_detector`.

Sistem mendukung:
- USB webcam/laptop camera via `capture_node`
- Intel RealSense D4xx via `realsense2_camera`
- subscribe ke image topic ROS 2 yang sudah ada
- OpenCV preview window dengan `show_window:=true`
- validasi dictionary `DICT_7X7_50` untuk ID 0-49

Dokumentasi launch lengkap ada di [launch.md](launch.md).
Kalibrasi kamera ada di [CALIBRATION.md](CALIBRATION.md).

---

## Status ROS 2 Jazzy

Workspace ini sudah disesuaikan untuk ROS 2 Jazzy dan OpenCV 4.x. Jika workspace
dipindah folder/device, hapus `build/`, `install/`, dan `log/`, lalu rebuild
karena colcon menyimpan absolute path.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select fiducial_detector \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DUSE_REALSENSE=OFF
source install/setup.bash
```

Validasi:

```bash
ros2 run fiducial_detector dict_test_node
```

Expected: `50/50 PASS`.

---

## Install Dependencies

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-rclcpp \
  ros-jazzy-rclcpp-components \
  ros-jazzy-sensor-msgs \
  ros-jazzy-geometry-msgs \
  ros-jazzy-std-msgs \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport \
  ros-jazzy-tf2 \
  ros-jazzy-tf2-geometry-msgs \
  libopencv-dev \
  libeigen3-dev \
  python3-colcon-common-extensions \
  build-essential \
  cmake \
  v4l-utils
```

Untuk Intel RealSense:

```bash
sudo apt install -y ros-jazzy-realsense2-camera
ros2 pkg executables realsense2_camera
```

Catatan Jetson/Jazzy: paket `ros-jazzy-realsense2-camera` biasanya menarik
`ros-jazzy-librealsense2`. Jangan jadikan `librealsense2-dev` Ubuntu sebagai
syarat utama jika tidak tersedia di repo device.

Build detector tidak perlu `librealsense2-dev` untuk `realsense.launch.xml`,
karena launch tersebut memakai node resmi `realsense2_camera`. Flag
`-DUSE_REALSENSE=ON` hanya untuk mode lama `capture_node source:=realsense`;
di Jetson aman memakai `-DUSE_REALSENSE=OFF`.

---

## Quick Start

Source setiap terminal baru:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Webcam:

```bash
ros2 launch fiducial_detector webcam.launch.xml device_id:=0 show_window:=true
```

RealSense:

```bash
ros2 launch fiducial_detector realsense.launch.xml show_window:=true
```

Default RealSense dibuat aman untuk USB2/Jetson:

```text
424x240 @ 15 FPS, YUYV, color-only
```

Jika RealSense sudah terhubung sebagai USB3/SuperSpeed, profil bisa dinaikkan:

```bash
ros2 launch fiducial_detector realsense.launch.xml \
  width:=640 height:=480 fps_limit:=30 show_window:=true
```

Existing ROS image topic:

```bash
ros2 launch fiducial_detector ros_topic.launch.xml \
  camera_topic:=/camera/camera/color/image_raw show_window:=true
```

---

## RealSense/Jetson Troubleshooting

Jika `show_window:=true` tidak muncul, jangan langsung cek OpenCV dulu. Window
baru dibuat setelah `aruco_node` menerima frame. Pada kasus RealSense, gejala
umumnya:

```text
Incomplete video frame detected
Frame Corrupted
Frames didn't arrived within 5 seconds
Device USB type: 2.1
```

Cek USB speed:

```bash
lsusb -t
```

Ideal untuk D455 adalah `5000M`/SuperSpeed. Jika hanya `480M` atau log ROS
menulis `Device USB type: 2.1`, pakai default ringan:

```bash
ros2 launch fiducial_detector realsense.launch.xml show_window:=true
```

Cek node RGB RealSense dan tes frame langsung:

```bash
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video4 --list-formats-ext

timeout 8s v4l2-ctl -d /dev/video4 \
  --set-fmt-video=width=424,height=240,pixelformat=YUYV \
  --set-parm=15 \
  --stream-mmap --stream-count=5 \
  --stream-to=/tmp/realsense_color.raw

ls -lh /tmp/realsense_color.raw
```

Jika output file `0` byte, masalahnya ada di USB/kamera, bukan di
`show_window`. Coba reset USB:

```bash
# Ganti 1-2 sesuai port RealSense dari lsusb -t.
sudo bash -lc 'echo 0 > /sys/bus/usb/devices/1-2/authorized; sleep 2; echo 1 > /sys/bus/usb/devices/1-2/authorized'
```

Validasi setelah reset:

```bash
ros2 topic echo /camera/camera/color/image_raw --once \
  --qos-reliability reliable --field encoding

ros2 topic echo /fiducial/fps --once
```

Output sehat pada default RealSense:

```text
encoding: yuv422_yuy2
data: 14.x
```

Jika frame masih corrupt:
- pindahkan RealSense ke port USB3 langsung, bukan hub,
- ganti kabel USB3 yang pendek dan bagus,
- jangan hidupkan depth/infra/IMU saat masih USB2,
- tetap pakai `424x240@15` dan `color_format:=YUYV`,
- di Jetson, pastikan power supply cukup.

---

## Topics

Input utama:

| Mode | Topic |
|---|---|
| Webcam launch | `/camera/image_raw` |
| RealSense launch | `/camera/camera/color/image_raw` |
| Existing topic | sesuai arg `camera_topic:=...` |

Output:

| Topic | Isi |
|---|---|
| `/fiducial/pose` | Pose marker |
| `/fiducial/alignment` | State alignment/gate |
| `/fiducial/debug_image` | Annotated image |
| `/fiducial/fps` | FPS deteksi |
| `/fiducial/rejected_candidates` | Statistik rejected candidates |

---

## Executables

| Executable | Deskripsi |
|---|---|
| `aruco_node` | Main fiducial detector |
| `capture_node` | C++ camera publisher untuk webcam/internal capture |
| `calibration_node` | Kalibrasi kamera chessboard |
| `dict_test_node` | Validasi DICT_7X7_50 ID 0-49 |

---

## More Docs

- [launch.md](launch.md): semua perintah launch, argument, dan troubleshooting.
- [CALIBRATION.md](CALIBRATION.md): kalibrasi kamera dan update intrinsics.
- [rules/README.md](rules/README.md): catatan eksperimen dan aturan deteksi marker.
