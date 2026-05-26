#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <map>
#include <string>
#include <vector>
#include <chrono>
namespace fiducial_detector {
struct BenchmarkResult {
  std::string dict_name;
  double fps_avg        {0.0};
  double latency_avg_ms {0.0};
  double latency_min_ms {0.0};
  double latency_max_ms {0.0};
  double latency_std_ms {0.0};
  double cpu_percent    {0.0};
  double memory_mb      {0.0};
  int    markers_found  {0};
  int    false_positives{0};
  double false_pos_rate {0.0};
  double pose_stability {0.0}; 
  std::string toJson() const;
};
struct BenchmarkReport {
  std::vector<BenchmarkResult> results;
  std::string best_fps_dict;
  std::string best_latency_dict;
  std::string best_accuracy_dict;
  double      total_time_s{0.0};
  std::string toJson() const;
  std::string toTable() const;
};
class BenchmarkRunner {
public:
  BenchmarkRunner() = default;
  BenchmarkResult runSingle(
    const cv::Mat& gray,
    cv::Ptr<cv::aruco::Dictionary> dict,
    const cv::Ptr<cv::aruco::DetectorParameters>& params,
    const std::string& dict_name,
    int n_frames = 30);
  BenchmarkReport runAll(
    const cv::Mat& gray,
    const std::map<std::string, cv::Ptr<cv::aruco::Dictionary>>& dicts,
    const cv::Ptr<cv::aruco::DetectorParameters>& params,
    int n_frames = 30);
  double measureFPS(
    const cv::Mat& gray,
    cv::Ptr<cv::aruco::Dictionary> dict,
    const cv::Ptr<cv::aruco::DetectorParameters>& params,
    double duration_s = 2.0);
  static double getCpuUsagePercent();
  static double getMemoryUsageMB();
  static void printReport(const BenchmarkReport& report);
  static bool saveReport(const BenchmarkReport& report,
                         const std::string& path);
  const BenchmarkReport& lastReport() const { return last_report_; }
private:
  BenchmarkReport last_report_;
  static std::vector<double> measureLatencies(
    const cv::Mat& gray,
    cv::Ptr<cv::aruco::Dictionary> dict,
    const cv::Ptr<cv::aruco::DetectorParameters>& params,
    int n_frames);
  static double computeStdDev(const std::vector<double>& values, double mean);
};
} 
