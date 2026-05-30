#pragma once
// NOTE: DICT_NAME_MAP has been consolidated into DictionaryManager (single source of truth).
// Use DictionaryManager::getDictIdByName() and DictionaryManager::getDictionaryByName().
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
namespace fiducial_detector {
class DetectorParametersManager {
public:
  DetectorParametersManager();
  void declareAll(rclcpp::Node* node);
  void bind(rclcpp::Node* node);
  cv::Ptr<cv::aruco::DetectorParameters> params() const { return params_; }
  void applyDictionaryProfile(const std::string& dict_name);
  // Delegates to DictionaryManager::getDictIdByName() — kept for backward compatibility
  static int dictId(const std::string& name);
  // Delegates to DictionaryManager::getDictionaryByName() — kept for backward compatibility
  static cv::Ptr<cv::aruco::Dictionary> makeDict(const std::string& name);
  static bool isAprilTagDict(const std::string& name);
  static bool isMIPDict(const std::string& name);
  static bool is7x7Dict(const std::string& name);
  static int borderBitsForDict(const std::string& name);
private:
  cv::Ptr<cv::aruco::DetectorParameters> params_;
};
}
