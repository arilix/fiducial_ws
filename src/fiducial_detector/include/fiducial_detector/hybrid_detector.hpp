#pragma once
#include "fiducial_detector/apriltag_backend.hpp"
#include "fiducial_detector/dictionary_manager.hpp"
#include "fiducial_detector/detector_parameters.hpp"
#include "fiducial_detector/pose_estimator.hpp"
#include <opencv2/aruco.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <memory>
#include <chrono>

namespace fiducial_detector {

enum class DetectorBackend : uint8_t {
  OPENCV    = 0,
  APRILTAG3 = 1,
  AUTO      = 2,
  FUSION    = 3,
};

struct HybridMarker {
  int   id{-1};
  std::vector<cv::Point2f> corners;
  cv::Point2f center;
  int   hamming{0};
  float decision_margin{0.0f};
  float confidence{0.0f};
  std::string family;
  DetectorBackend source{DetectorBackend::OPENCV};
};

struct HybridResult {
  std::vector<HybridMarker>                markers;
  std::vector<std::vector<cv::Point2f>>    rejected;
  DetectorBackend                          backend_used{DetectorBackend::OPENCV};
  double                                   latency_ms{0.0};
  int                                      opencv_count{0};
  int                                      apriltag_count{0};
};

struct HybridBenchmark {
  double opencv_fps{0.0};
  double apriltag_fps{0.0};
  int    opencv_detected{0};
  int    apriltag_detected{0};
  double opencv_latency_ms{0.0};
  double apriltag_latency_ms{0.0};
};

class HybridDetector {
public:
  explicit HybridDetector(rclcpp::Logger logger);

  void init(
    const std::string& dict_name,
    const AprilTagConfig& at_cfg,
    cv::Ptr<cv::aruco::Dictionary> opencv_dict,
    cv::Ptr<cv::aruco::DetectorParameters> opencv_params);

  HybridResult detect(
    const cv::Mat& gray,
    const std::string& dict_name,
    DetectorBackend mode);

  HybridBenchmark benchmark(const cv::Mat& gray, int n_frames = 30);

  void setOpenCVDict(cv::Ptr<cv::aruco::Dictionary> dict) { opencv_dict_ = dict; }
  void setOpenCVParams(cv::Ptr<cv::aruco::DetectorParameters> p) { opencv_params_ = p; }
  void setAprilTagConfig(const AprilTagConfig& cfg);

  DetectorBackend autoSelectBackend(const std::string& dict_name);
  static std::string backendName(DetectorBackend b);

  AprilTagBackend& apriltagBackend() { return at_backend_; }
  const AprilTagBackend& apriltagBackend() const { return at_backend_; }

private:
  HybridResult detectOpenCV(const cv::Mat& gray);
  HybridResult detectAprilTag3(const cv::Mat& gray);
  HybridResult detectFusion(const cv::Mat& gray);

  // fusion scoring: opencv weight 0.4, apriltag weight 0.6
  std::vector<HybridMarker> fuseDetections(
    const std::vector<HybridMarker>& opencv,
    const std::vector<HybridMarker>& apriltag);

  float computeMarkerConfidence(const HybridMarker& m, DetectorBackend src);

  AprilTagBackend                        at_backend_;
  cv::Ptr<cv::aruco::Dictionary>         opencv_dict_;
  cv::Ptr<cv::aruco::DetectorParameters> opencv_params_;
  std::string                            current_dict_;
  rclcpp::Logger                         logger_;
};

}
