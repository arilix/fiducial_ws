#pragma once
// ═══════════════════════════════════════════════════════════════════
// CaptureNode — Unified C++ camera publisher (webcam & RealSense)
// Publishes sensor_msgs/Image on /camera/image_raw
// Eliminates dependency on v4l2_camera (Python) and realsense2_camera
// ═══════════════════════════════════════════════════════════════════
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <atomic>
#include <string>

// Optional: librealsense2 (compiled in only if USE_REALSENSE=ON)
#ifdef FIDUCIAL_USE_REALSENSE
#include <librealsense2/rs.hpp>
#endif

namespace fiducial_detector {

class CaptureNode : public rclcpp::Node {
public:
  explicit CaptureNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions());
  ~CaptureNode() override;

private:
  // ── Helpers ────────────────────────────────────────────────────────
  void openWebcam();
  void loopWebcam();

#ifdef FIDUCIAL_USE_REALSENSE
  void openRealSense();
  void loopRealSense();
#endif

  sensor_msgs::msg::Image::SharedPtr matToMsg(
    const cv::Mat& bgr,
    const rclcpp::Time& stamp) const;

  // ── Parameters ─────────────────────────────────────────────────────
  std::string source_;      // "webcam" | "realsense"
  int         device_id_;   // V4L2 index (webcam mode)
  std::string device_path_; // e.g. "/dev/video6" (overrides device_id)
  int         width_, height_;
  double      fps_limit_;
  std::string frame_id_;
  std::string output_topic_;

  // ── ROS ────────────────────────────────────────────────────────────
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr     pub_image_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_info_;

  // ── Webcam runtime ─────────────────────────────────────────────────
  cv::VideoCapture cap_;

#ifdef FIDUCIAL_USE_REALSENSE
  // ── RealSense runtime ──────────────────────────────────────────────
  rs2::pipeline   rs_pipe_;
  rs2::config     rs_cfg_;
#endif

  std::thread      capture_thread_;
  std::atomic<bool> running_{false};
};

} // namespace fiducial_detector
