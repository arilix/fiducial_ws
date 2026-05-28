#pragma once
#include <opencv2/core.hpp>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <functional>

extern "C" {
#include <apriltag/apriltag.h>
}

namespace fiducial_detector {

struct AprilTagConfig {
  std::string family{"tag36h11"};
  bool   load_all_families{true};
  int    nthreads{4};
  float  quad_decimate{1.0f};
  float  quad_sigma{0.0f};
  int    refine_edges{1};
  double decode_sharpening{0.25};
  int    debug{0};
  int    max_hamming{1};
  float  min_decision_margin{40.0f};
};

struct AprilTagDetection {
  int   id{-1};
  int   hamming{0};
  float decision_margin{0.0f};
  std::vector<cv::Point2f> corners;
  cv::Point2f center;
  std::string family_name;
};

struct AprilTagPose {
  cv::Vec3d rvec;
  cv::Vec3d tvec;
  double error{0.0};
  bool valid{false};
};

class AprilTagBackend {
public:
  AprilTagBackend();
  ~AprilTagBackend();

  AprilTagBackend(const AprilTagBackend&) = delete;
  AprilTagBackend& operator=(const AprilTagBackend&) = delete;

  bool configure(const AprilTagConfig& cfg);
  bool setFamily(const std::string& family_name);
  bool loadAllFamilies();
  std::vector<AprilTagDetection> detect(const cv::Mat& gray);
  AprilTagPose estimatePose(
    const AprilTagDetection& det,
    double tagsize, double fx, double fy, double cx, double cy);
  void reconfigure(const AprilTagConfig& cfg);

  bool isValid() const { return td_ != nullptr && !families_.empty(); }
  std::string familyName() const { return current_family_; }
  int familyCount() const;
  int loadedFamilyCount() const { return static_cast<int>(families_.size()); }
  std::vector<std::string> loadedFamilies() const;

  static std::string mapDictToFamily(const std::string& dict_name);
  static bool isAprilTagFamily(const std::string& dict_name);
  static std::vector<std::string> supportedFamilies();

private:
  void destroy();
  apriltag_family_t* createFamily(const std::string& name);
  void destroyFamily(apriltag_family_t* tf, const std::string& name);

  apriltag_detector_t* td_{nullptr};
  std::map<std::string, apriltag_family_t*> families_;
  std::string          current_family_;
  AprilTagConfig       config_;
};

}
