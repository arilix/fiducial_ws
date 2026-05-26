#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <string>
#include <vector>
namespace fiducial_detector {
struct CustomDictConfig {
  int  n_markers{50};          
  int  marker_bits_per_side{4};
  int  random_seed{42};        
  std::string name{"CUSTOM"};  
};
class CustomDictionaryManager {
public:
  CustomDictionaryManager() = default;
  cv::Ptr<cv::aruco::Dictionary> generate(
    const CustomDictConfig& cfg,
    cv::Ptr<cv::aruco::Dictionary> base_dict = nullptr);
  cv::Ptr<cv::aruco::Dictionary> extend(
    cv::Ptr<cv::aruco::Dictionary> existing,
    int n_new, int random_seed = 0);
  bool saveYAML(cv::Ptr<cv::aruco::Dictionary> dict,
                const std::string& path,
                const std::string& name = "CUSTOM") const;
  cv::Ptr<cv::aruco::Dictionary> loadYAML(const std::string& path);
  cv::Mat visualizeBytesList(
    cv::Ptr<cv::aruco::Dictionary> dict,
    int marker_size = 64,
    int cols = 8) const;
  cv::Ptr<cv::aruco::Dictionary> getDict() const { return current_dict_; }
  const std::string& getName() const { return current_name_; }
private:
  cv::Ptr<cv::aruco::Dictionary> current_dict_;
  std::string                    current_name_{"CUSTOM"};
};
} 
