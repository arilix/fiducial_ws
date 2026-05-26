#pragma once
#include "fiducial_detector/fps_monitor.hpp"
#include "fiducial_detector/pose_estimator.hpp"
#include "fiducial_detector/detector_parameters.hpp"
#include "fiducial_detector/dictionary_manager.hpp"
#include "fiducial_detector/visualization.hpp"
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
  void enqueueDisplay(const cv::Mat& frame);
  image_transport::Subscriber                                    image_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  pub_pose_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr          pub_debug_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            pub_alignment_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr           pub_fps_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            pub_rejected_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            pub_cur_dict_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            pub_dict_score_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            pub_det_stats_;
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
  DetectionMode detection_mode_{DetectionMode::SINGLE};
  std::string   detection_mode_str_{"SINGLE"};
  bool          benchmark_mode_{false};
  cv::Ptr<cv::aruco::Dictionary>        aruco_dict_;
  cv::Ptr<cv::aruco::CharucoBoard>      charuco_board_;
  std::unique_ptr<DetectorParametersManager> det_params_mgr_;
  std::unique_ptr<PoseEstimator>             pose_estimator_;
  std::unique_ptr<DictionaryManager>        dict_manager_;
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
  cv::Mat                  display_frame_;
  std::mutex               display_mutex_;
  std::condition_variable  display_cv_;
  std::atomic<bool>        display_ready_{false};
public:
  bool displayLoop();
};
}
