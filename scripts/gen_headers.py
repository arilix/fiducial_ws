#!/usr/bin/env python3
"""Generate all header files for fiducial_detector package."""
import os
BASE = "/home/arilix/Documents/vtol/vtol aruco/fiducial_ws/src/fiducial_detector/include/fiducial_detector"
os.makedirs(BASE, exist_ok=True)
FPS_MON_H = r"""#pragma once
namespace fiducial_detector {
class FpsMonitor {
public:
  explicit FpsMonitor(std::size_t window_size = 60);
  void tick();
  float getFps() const;
  double getLatencyMs() const;
  void reset();
private:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;
  std::size_t window_size_;
  mutable std::mutex mtx_;
  std::deque<TimePoint> timestamps_;
  double last_latency_ms_{0.0};
};
}
"""
POSE_H = r"""#pragma once
namespace fiducial_detector {
struct PoseResult {
  cv::Vec3d rvec;
  cv::Vec3d tvec;
  Eigen::Quaterniond quaternion;
  double distance{0.0};
  bool valid{false};
};
class PoseEstimator {
public:
  PoseEstimator(const cv::Mat& camera_matrix,
                const cv::Mat& dist_coeffs,
                double marker_size);
  PoseResult estimate(int id, const std::vector<cv::Point2f>& corners);
  std::vector<PoseResult> estimateBatch(
    const std::vector<int>& ids,
    const std::vector<std::vector<cv::Point2f>>& corners_vec);
  void setSmoothingAlpha(double alpha) { alpha_ = alpha; }
  double getSmoothingAlpha() const { return alpha_; }
  const cv::Mat& cameraMatrix() const { return K_; }
  const cv::Mat& distCoeffs()   const { return D_; }
private:
  void applySmoothing(int id, cv::Vec3d& rvec, cv::Vec3d& tvec);
  cv::Mat K_, D_;
  double  marker_size_;
  double  alpha_{0.4};
  struct SmoothState {
    cv::Vec3d rvec{0,0,0};
    cv::Vec3d tvec{0,0,0};
    bool initialised{false};
  };
  std::map<int, SmoothState> smooth_state_;
  std::vector<cv::Point3f> makeObjectPoints() const;
};
}
"""
DET_PARAMS_H = r"""#pragma once
namespace fiducial_detector {
inline const std::map<std::string, int> DICT_NAME_MAP = {
  {"DICT_4X4_50",          cv::aruco::DICT_4X4_50},
  {"DICT_4X4_100",         cv::aruco::DICT_4X4_100},
  {"DICT_4X4_250",         cv::aruco::DICT_4X4_250},
  {"DICT_4X4_1000",        cv::aruco::DICT_4X4_1000},
  {"DICT_5X5_50",          cv::aruco::DICT_5X5_50},
  {"DICT_5X5_100",         cv::aruco::DICT_5X5_100},
  {"DICT_5X5_250",         cv::aruco::DICT_5X5_250},
  {"DICT_5X5_1000",        cv::aruco::DICT_5X5_1000},
  {"DICT_6X6_50",          cv::aruco::DICT_6X6_50},
  {"DICT_6X6_100",         cv::aruco::DICT_6X6_100},
  {"DICT_6X6_250",         cv::aruco::DICT_6X6_250},
  {"DICT_6X6_1000",        cv::aruco::DICT_6X6_1000},
  {"DICT_7X7_50",          cv::aruco::DICT_7X7_50},
  {"DICT_7X7_100",         cv::aruco::DICT_7X7_100},
  {"DICT_7X7_250",         cv::aruco::DICT_7X7_250},
  {"DICT_7X7_1000",        cv::aruco::DICT_7X7_1000},
  {"DICT_ARUCO_ORIGINAL",  cv::aruco::DICT_ARUCO_ORIGINAL},
  {"DICT_APRILTAG_16h5",   cv::aruco::DICT_APRILTAG_16h5},
  {"DICT_APRILTAG_25h9",   cv::aruco::DICT_APRILTAG_25h9},
  {"DICT_APRILTAG_36h10",  cv::aruco::DICT_APRILTAG_36h10},
  {"DICT_APRILTAG_36h11",  cv::aruco::DICT_APRILTAG_36h11},
};
class DetectorParametersManager {
public:
  DetectorParametersManager();
  void declareAll(rclcpp::Node* node);
  void bind(rclcpp::Node* node);
  cv::Ptr<cv::aruco::DetectorParameters> params() const { return params_; }
  static int dictId(const std::string& name);
  static cv::Ptr<cv::aruco::Dictionary> makeDict(const std::string& name);
private:
  cv::Ptr<cv::aruco::DetectorParameters> params_;
};
}
"""
VIZ_H = r"""#pragma once
namespace fiducial_detector {
enum class MarkerType : uint8_t {
  ARUCO    = 0,
  APRILTAG = 1,
  ARTAG    = 2,
  CHARUCO  = 3,
  UNKNOWN  = 99,
};
inline const cv::Scalar CLR_ARUCO      {  0, 220,   0};
inline const cv::Scalar CLR_APRILTAG   {220,   0,   0};
inline const cv::Scalar CLR_CHARUCO    {  0, 220, 220};
inline const cv::Scalar CLR_ARTAG      {  0,   0, 220};
inline const cv::Scalar CLR_REJECTED   { 50,  50, 220};
inline const cv::Scalar CLR_LOCKED     {220, 220,   0};
inline const cv::Scalar CLR_ALIGNED    {  0, 255,   0};
inline const cv::Scalar CLR_UNALIGNED  {  0,   0, 255};
inline const cv::Scalar CLR_WHITE      {255, 255, 255};
inline const cv::Scalar CLR_BLACK      {  0,   0,   0};
inline const cv::Scalar CLR_GRAY       {130, 130, 130};
inline cv::Scalar markerColor(MarkerType t) {
  switch (t) {
    case MarkerType::ARUCO:    return CLR_ARUCO;
    case MarkerType::APRILTAG: return CLR_APRILTAG;
    case MarkerType::CHARUCO:  return CLR_CHARUCO;
    case MarkerType::ARTAG:    return CLR_ARTAG;
    default:                   return CLR_WHITE;
  }
}
inline std::string markerTypeName(MarkerType t) {
  switch (t) {
    case MarkerType::ARUCO:    return "ArUco";
    case MarkerType::APRILTAG: return "AprilTag";
    case MarkerType::CHARUCO:  return "ChArUco";
    case MarkerType::ARTAG:    return "ARTag";
    default:                   return "Unknown";
  }
}
class Visualizer {
public:
  explicit Visualizer(int alignment_tolerance = 50);
  void setAlignmentTolerance(int tol) { alignment_tol_ = tol; }
  int  getAlignmentTolerance() const  { return alignment_tol_; }
  void drawUI(cv::Mat& frame, bool any_marker_locked) const;
  void drawDetectedMarkers(
    cv::Mat& frame,
    const std::vector<std::vector<cv::Point2f>>& corners,
    const std::vector<int>& ids,
    MarkerType type) const;
  void drawRejected(
    cv::Mat& frame,
    const std::vector<std::vector<cv::Point2f>>& rejected) const;
  void drawPoseAxis(
    cv::Mat& frame,
    const PoseResult& pose,
    const cv::Mat& K,
    const cv::Mat& D,
    float axis_length = 0.03f) const;
  void drawMarkerInfo(
    cv::Mat& frame,
    cv::Point2f center,
    int id,
    MarkerType type,
    const PoseResult& pose) const;
  void drawAlignment(
    cv::Mat& frame,
    cv::Point2f marker_center,
    const cv::Size& frame_size) const;
  void drawHUD(
    cv::Mat& frame,
    float fps,
    double latency_ms,
    uint64_t frame_count,
    bool cam_ok) const;
  bool isAligned(cv::Point2f pt, const cv::Size& sz) const;
  bool isLeft   (cv::Point2f pt, const cv::Size& sz) const;
  bool isRight  (cv::Point2f pt, const cv::Size& sz) const;
  bool isUp     (cv::Point2f pt, const cv::Size& sz) const;
  bool isDown   (cv::Point2f pt, const cv::Size& sz) const;
  std::string alignmentString(cv::Point2f pt, const cv::Size& sz) const;
private:
  int alignment_tol_;
  void alphaRect(cv::Mat& frame, cv::Rect rect,
                 cv::Scalar color, double alpha) const;
  void labelText(cv::Mat& frame, const std::string& text,
                 cv::Point origin, cv::Scalar color,
                 double font_scale = 0.55, int thickness = 1) const;
};
}
"""
ARUCO_H = r"""#pragma once
namespace fiducial_detector {
struct DetectedMarker {
  int          id{-1};
  MarkerType   type{MarkerType::UNKNOWN};
  std::vector<cv::Point2f> corners;
  cv::Point2f  center;
  PoseResult   pose;
};
struct DetectionResult {
  std::vector<DetectedMarker>              markers;
  std::vector<std::vector<cv::Point2f>>   rejected;
  cv::Size                                 frame_size;
};
class FiducialDetector : public rclcpp::Node
{
public:
  explicit FiducialDetector(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~FiducialDetector() override;
private:
  void declareParameters();
  void loadIntrinsics();
  void initDetectors();
  void initSubscriber();
  void initPublishers();
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  void fpsTimerCallback();
  void watchdogCallback();
  DetectionResult runDetection(const cv::Mat& frame);
  void detectAruco (const cv::Mat& gray, DetectionResult& result);
  void detectCharuco(const cv::Mat& gray, const cv::Mat& color, DetectionResult& result);
  void estimatePoses(DetectionResult& result);
  void publishAll(const DetectionResult& result,
                  const cv::Mat& annotated,
                  const rclcpp::Time& stamp);
  void reconnectCamera();
  void loadRosParams();
  cv::Point2f computeCenter(const std::vector<cv::Point2f>& corners);
  image_transport::Subscriber                                    image_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  pub_pose_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr          pub_debug_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            pub_alignment_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr           pub_fps_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            pub_rejected_;
  rclcpp::TimerBase::SharedPtr                                   fps_timer_;
  rclcpp::TimerBase::SharedPtr                                   watchdog_timer_;
  double      marker_size_{0.05};
  std::string camera_topic_{"/camera/image_raw"};
  std::string dictionary_type_{"DICT_4X4_50"};
  bool        enable_charuco_{true};
  bool        show_window_{true};
  bool        show_rejected_{true};
  int         alignment_tolerance_{50};
  double      smoothing_alpha_{0.4};
  int         max_missed_frames_{5};
  cv::Ptr<cv::aruco::Dictionary>        aruco_dict_;
  cv::Ptr<cv::aruco::CharucoBoard>      charuco_board_;
  std::unique_ptr<DetectorParametersManager> det_params_mgr_;
  std::unique_ptr<PoseEstimator>             pose_estimator_;
  std::unique_ptr<Visualizer>               visualizer_;
  FpsMonitor                                fps_monitor_;
  struct CameraIntrinsics {
    cv::Mat K;
    cv::Mat D;
    bool valid{false};
  } intrinsics_;
  std::atomic<bool> cam_connected_{false};
  std::atomic<bool> reconnect_pending_{false};
  rclcpp::Time      last_frame_time_;
  uint64_t          frame_count_{0};
  std::mutex        data_mutex_;
};
}
"""
HEADERS = {
    "fps_monitor.hpp":         FPS_MON_H,
    "pose_estimator.hpp":      POSE_H,
    "detector_parameters.hpp": DET_PARAMS_H,
    "visualization.hpp":       VIZ_H,
    "aruco.hpp":               ARUCO_H,
}
for fname, content in HEADERS.items():
    path = os.path.join(BASE, fname)
    with open(path, "w") as f:
        f.write(content.lstrip("\n"))
    print(f"  WROTE {path}  ({content.count(chr(10))} lines)")
print("ALL HEADERS OK")
