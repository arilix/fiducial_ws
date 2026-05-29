#pragma once
#include "fiducial_detector/charuco_handler.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <atomic>
#include <mutex>
#include <string>
namespace fiducial_detector {
enum class CalibMode { CHESSBOARD, CHARUCO };
class CalibrationNode : public rclcpp::Node {
public:
  explicit CalibrationNode(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~CalibrationNode() override;
  bool displayLoop();
private:
  void declareParameters();
  void loadParams();
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  void processChessboard(const cv::Mat& gray, cv::Mat& display);
  void processCharuco(const cv::Mat& gray, cv::Mat& display);
  bool runCalibration();
  void saveResult(const std::string& path);
  void drawStatus(cv::Mat& frame) const;
  void visualizeIntrinsics(const cv::Mat& K, const cv::Mat& D) const;
  image_transport::Subscriber image_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
  std::string camera_topic_;
  std::string output_yaml_;
  CalibMode   calib_mode_{CalibMode::CHARUCO};
  int         chess_cols_{9};
  int         chess_rows_{6};
  float       chess_square_{0.025f};
  int         charuco_cols_{7};
  int         charuco_rows_{5};
  float       charuco_sq_{0.035f};
  float       charuco_mk_{0.0175f};
  std::string dict_name_{"DICT_5X5_50"};
  int         min_frames_{15};
  bool        show_window_{true};
  std::unique_ptr<CharucoHandler> charuco_handler_;
  std::vector<std::vector<cv::Point2f>> chess_corners_all_;
  std::vector<std::vector<cv::Point3f>> chess_obj_points_;
  cv::Size                              image_size_;
  CalibrationResult last_result_;
  bool              capture_requested_{false};
  bool              calibrate_requested_{false};
  std::mutex  frame_mutex_;
  cv::Mat     display_frame_;
  std::atomic<bool> display_ready_{false};
  std::condition_variable display_cv_;
};
} 
