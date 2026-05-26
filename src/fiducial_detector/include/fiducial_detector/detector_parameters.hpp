#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <rclcpp/rclcpp.hpp>
#include <map>
#include <string>
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
