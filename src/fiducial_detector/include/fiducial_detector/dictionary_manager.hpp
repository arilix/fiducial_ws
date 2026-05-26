#pragma once
#include <opencv2/aruco.hpp>
#include <map>
#include <mutex>
#include <string>
#include <vector>
#include <functional>
#include <chrono>
#include <future>
#include <atomic>
namespace fiducial_detector {
enum class DetectionMode : uint8_t {
  SINGLE    = 0,
  AUTO      = 1,
  MULTI     = 2,
  BENCHMARK = 3,
};
struct DictionaryInfo {
  std::string                    name;
  int                            dict_id;
  cv::Ptr<cv::aruco::Dictionary> dict;
  std::string                    family;
  int                            marker_bits;
  int                            total_markers;
};
struct DictionaryScore {
  std::string dict_name;
  int         valid_markers{0};
  int         rejected_count{0};
  double      reprojection_error{0.0};
  double      pose_stability{0.0};
  double      latency_ms{0.0};
  double      score{0.0};
  static double compute(int valid, int rejected,
                        double reproj_err, double stability) {
    return static_cast<double>(valid) * 10.0
           - static_cast<double>(rejected) * 0.5
           - reproj_err
           - stability;
  }
};
struct DictDetectionResult {
  std::string                              dict_name;
  std::vector<int>                         ids;
  std::vector<std::vector<cv::Point2f>>   corners;
  std::vector<std::vector<cv::Point2f>>   rejected;
  DictionaryScore                          score;
};
class DictionaryManager {
public:
  DictionaryManager();
  static cv::Ptr<cv::aruco::Dictionary> getDictionaryByName(const std::string& name);
  static int getDictIdByName(const std::string& name);
  static const std::map<std::string, int>& getDictMap();
  static std::vector<std::string> getAllNames();
  bool setActive(const std::string& name);
  std::string activeName() const;
  cv::Ptr<cv::aruco::Dictionary> activeDict() const;
  bool hotReload(const std::string& name);
  std::string autoDetect(
    const cv::Mat& gray,
    const cv::Ptr<cv::aruco::DetectorParameters>& params);
  std::vector<DictDetectionResult> detectAll(
    const cv::Mat& gray,
    const cv::Ptr<cv::aruco::DetectorParameters>& params);
  void benchmark(
    const cv::Mat& gray,
    const cv::Ptr<cv::aruco::DetectorParameters>& params,
    int n_frames = 30);
  const DictionaryInfo& info(const std::string& name) const;
  const std::map<std::string, DictionaryScore>& scores() const { return scores_; }
  DictionaryScore lastScore(const std::string& name) const;
  static const std::vector<std::string> AUTO_SUBSET;
private:
  void initAll();
  DictDetectionResult detectWith(
    const std::string& dict_name,
    const cv::Mat& gray,
    const cv::Ptr<cv::aruco::DetectorParameters>& params) const;
  std::map<std::string, DictionaryInfo> dicts_;
  std::map<std::string, DictionaryScore> scores_;
  std::string active_name_{"DICT_4X4_50"};
  mutable std::mutex mutex_;
};
}
