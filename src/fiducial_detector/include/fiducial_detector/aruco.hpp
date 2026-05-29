#pragma once
#include "fiducial_detector/fps_monitor.hpp"
#include "fiducial_detector/pose_estimator.hpp"
#include "fiducial_detector/detector_parameters.hpp"
#include "fiducial_detector/dictionary_manager.hpp"
#include "fiducial_detector/visualization.hpp"
#include "fiducial_detector/marker_decoder.hpp"
#include "fiducial_detector/charuco_handler.hpp"
#include "fiducial_detector/board_handler.hpp"
#include "fiducial_detector/custom_dictionary.hpp"
#include "fiducial_detector/benchmark_runner.hpp"
#include "fiducial_detector/confidence_system.hpp"
#include "fiducial_detector/board_generator.hpp"
#include "fiducial_detector/hybrid_detector.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
namespace fiducial_detector {
struct DetectedMarker {
  int id{-1};
  MarkerType type{MarkerType::UNKNOWN};
  std::vector<cv::Point2f> corners;
  cv::Point2f center;
  PoseResult  pose;
  DetectionConfidence confidence;
  int hamming{0};
  float decision_margin{0.0f};
  DetectorBackend source{DetectorBackend::OPENCV};
};
struct DetectionResult {
  std::vector<DetectedMarker>              markers;
  std::vector<std::vector<cv::Point2f>>    rejected;
  std::vector<RejectedCandidate>           rejected_with_reasons;
  cv::Size                                 frame_size;
  CharucoResult                            charuco;
  GridBoardResult                          board;
  DiamondResult                            diamonds;
  MarkerDebugImages                        debug_images;
  bool has_debug_images{false};
  DetectorBackend backend_used{DetectorBackend::OPENCV};
};
class FiducialDetector : public rclcpp::Node {
public:
  explicit FiducialDetector(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~FiducialDetector() override;
  bool displayLoop();
private:
  // ── Initialization helpers ───────────────────────────────────────────────
  void declareParameters(); void loadRosParams(); void loadIntrinsics();
  void initDetectors(); void initSubscriber(); void initPublishers();
  // ── Callbacks & timers ──────────────────────────────────────────────────
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  void fpsTimerCallback(); void watchdogCallback();
  // ── Detection pipeline ──────────────────────────────────────────────────
  DetectionResult runDetection(const cv::Mat& frame);
  void detectAruco  (const cv::Mat& gray, DetectionResult& r);
  void detectCharuco(const cv::Mat& gray, DetectionResult& r);
  void detectBoard  (const cv::Mat& gray, DetectionResult& r);
  void detectDiamond(const cv::Mat& gray, DetectionResult& r);
  void estimatePoses(DetectionResult& r);
  void computeConfidence(DetectionResult& r);
  // ── Output & display ────────────────────────────────────────────────────
  void renderAnnotations(cv::Mat& frame, const DetectionResult& r, bool locked);
  void publishAll(const DetectionResult& r, const cv::Mat& ann, const rclcpp::Time& stamp);
  void publishDebugImage(const cv::Mat& img, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& pub, const rclcpp::Time& stamp);
  void reconnectCamera();
  cv::Point2f computeCenter(const std::vector<cv::Point2f>& c);
  cv::Mat preprocessFrame(const cv::Mat& gray);
  void enqueueDisplay(const cv::Mat& frame);
  // ── ROS Infrastructure ───────────────────────────────────────────────────
  image_transport::Subscriber                                   image_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr         pub_debug_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr         pub_debug_cells_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr         pub_debug_thresh_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr         pub_debug_contours_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr           pub_alignment_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr          pub_fps_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr           pub_rejected_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr           pub_cur_dict_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr           pub_dict_score_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr           pub_det_stats_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr           pub_benchmark_;
  rclcpp::TimerBase::SharedPtr fps_timer_, watchdog_timer_;
  // ── Detection Parameters ─────────────────────────────────────────────────
  double      marker_size_{0.05};
  std::string camera_topic_{"/camera/image_raw"};
  std::string dictionary_type_{"DICT_4X4_50"};
  int         alignment_tolerance_{50};
  double      smoothing_alpha_{0.4};
  int         max_missed_frames_{5};
  // ── Feature Flags ────────────────────────────────────────────────────────
  bool        enable_charuco_{true};
  bool        enable_gridboard_{false};
  bool        enable_diamond_{false};
  bool        show_window_{true};
  bool        show_rejected_{true};
  bool        show_corner_labels_{true};
  bool        show_orientation_arrow_{true};
  bool        show_confidence_{true};
  bool        benchmark_mode_{false};
  bool        enable_custom_dict_{false};
  std::string custom_dict_path_{};
  // ── Detection Mode ───────────────────────────────────────────────────────
  DetectionMode detection_mode_{DetectionMode::SINGLE};
  std::string   detection_mode_str_{"SINGLE"};
  // ── ChArUco & GridBoard Board Config ────────────────────────────────────
  int   charuco_cols_{7}, charuco_rows_{5};
  float charuco_sq_{0.035f}, charuco_mk_{0.0175f};
  int   gridboard_cols_{5}, gridboard_rows_{7};
  float gridboard_marker_size_{0.04f}, gridboard_sep_{0.01f};
  // ── Hybrid Detector Config ───────────────────────────────────────────────
  bool        use_opencv_detector_{true};
  bool        use_native_apriltag_{true};
  bool        use_detector_fusion_{true};
  std::string apriltag_family_{"tag36h11"};
  int         apriltag_threads_{4};
  float       apriltag_decimate_{1.0f};
  float       apriltag_blur_{0.0f};
  bool        apriltag_refine_edges_{true};
  double      apriltag_sharpening_{0.25};
  bool        apriltag_debug_{false};
  int         apriltag_max_hamming_{1};
  double      apriltag_min_margin_{40.0};
  // ── Preprocessing Flags ──────────────────────────────────────────────────
  bool        enable_clahe_{true};
  double      clahe_clip_{2.0};
  bool        enable_sharpen_{false};
  bool        enable_blur_{false};
  // ── Camera Intrinsics ────────────────────────────────────────────────────
  struct CameraIntrinsics { cv::Mat K, D; bool valid{false}; } intrinsics_;
  // ── Sub-components (unique_ptr) ──────────────────────────────────────────
  cv::Ptr<cv::aruco::Dictionary>             aruco_dict_;
  std::unique_ptr<DetectorParametersManager> det_params_mgr_;
  std::unique_ptr<PoseEstimator>             pose_estimator_;
  std::unique_ptr<DictionaryManager>         dict_manager_;
  std::unique_ptr<Visualizer>                visualizer_;
  std::unique_ptr<MarkerDecoder>             marker_decoder_;
  std::unique_ptr<CharucoHandler>            charuco_handler_;
  std::unique_ptr<GridBoardHandler>          board_handler_;
  std::unique_ptr<CustomDictionaryManager>   custom_dict_mgr_;
  std::unique_ptr<BenchmarkRunner>           benchmark_runner_;
  std::unique_ptr<ConfidenceCalculator>      confidence_calc_;
  std::unique_ptr<BoardGenerator>            board_generator_;
  std::unique_ptr<HybridDetector>            hybrid_detector_;
  cv::Ptr<cv::CLAHE>                         clahe_;
  FpsMonitor fps_monitor_;
  // ── Runtime State ────────────────────────────────────────────────────────
  std::atomic<bool> cam_connected_{false};
  std::atomic<bool> reconnect_pending_{false};
  rclcpp::Time last_frame_time_;
  uint64_t     frame_count_{0};
  std::atomic<bool> show_cells_window_{false};
  std::atomic<bool> show_thresh_window_{false};
  std::atomic<bool> show_contour_window_{false};
  std::atomic<bool> show_rejected_window_{false};
  std::mutex        debug_mutex_;
  MarkerDebugImages last_debug_;
  // ── Display Thread ───────────────────────────────────────────────────────
  cv::Mat                 display_frame_;
  std::mutex              display_mutex_;
  std::condition_variable display_cv_;
  std::atomic<bool>       display_ready_{false};
};
}

