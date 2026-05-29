#include "fiducial_detector/apriltag_backend.hpp"
#include <opencv2/calib3d.hpp>
#include <stdexcept>

extern "C" {
#include <apriltag/tag16h5.h>
#include <apriltag/tag25h9.h>
#include <apriltag/tag36h10.h>
#include <apriltag/tag36h11.h>
#include <apriltag/tagStandard41h12.h>
#include <apriltag/tagStandard52h13.h>
#include <apriltag/tagCircle21h7.h>
#include <apriltag/tagCircle49h12.h>
#include <apriltag/tagCustom48h12.h>
}

namespace fiducial_detector {

AprilTagBackend::AprilTagBackend() = default;

AprilTagBackend::~AprilTagBackend() {
  destroy();
}

void AprilTagBackend::destroy() {
  if (td_) {
    for (auto& kv : families_)
      apriltag_detector_remove_family(td_, kv.second);
    apriltag_detector_destroy(td_);
    td_ = nullptr;
  }
  for (auto& kv : families_)
    destroyFamily(kv.second, kv.first);
  families_.clear();
  current_family_.clear();
}

bool AprilTagBackend::configure(const AprilTagConfig& cfg) {
  config_ = cfg;
  if (cfg.load_all_families)
    return loadAllFamilies();
  return setFamily(cfg.family);
}

bool AprilTagBackend::setFamily(const std::string& family_name) {
  destroy();

  auto tf = createFamily(family_name);
  if (!tf) {
    fprintf(stderr,
      "[AprilTagBackend] ERROR: createFamily('%s') failed. "
      "Check that libapriltag3 is installed: "
      "sudo apt install libapriltag-dev libapriltag3\n",
      family_name.c_str());
    return false;
  }

  td_ = apriltag_detector_create();
  if (!td_) {
    destroyFamily(tf, family_name);
    fprintf(stderr,
      "[AprilTagBackend] FATAL: apriltag_detector_create() returned nullptr. "
      "Ensure libapriltag3 is installed: "
      "sudo apt install libapriltag-dev libapriltag3\n");
    return false;
  }

  td_->nthreads         = config_.nthreads;
  td_->quad_decimate    = config_.quad_decimate;
  td_->quad_sigma       = config_.quad_sigma;
  td_->refine_edges     = config_.refine_edges;
  td_->decode_sharpening = config_.decode_sharpening;
  td_->debug            = config_.debug;

  apriltag_detector_add_family_bits(td_, tf, config_.max_hamming);
  families_[family_name] = tf;
  current_family_ = family_name;
  return true;
}

bool AprilTagBackend::loadAllFamilies() {
  destroy();

  td_ = apriltag_detector_create();
  if (!td_) {
    fprintf(stderr,
      "[AprilTagBackend] FATAL: apriltag_detector_create() returned nullptr. "
      "Ensure libapriltag3 is installed: "
      "sudo apt install libapriltag-dev libapriltag3\n");
    return false;
  }

  td_->nthreads         = config_.nthreads;
  td_->quad_decimate    = config_.quad_decimate;
  td_->quad_sigma       = config_.quad_sigma;
  td_->refine_edges     = config_.refine_edges;
  td_->decode_sharpening = config_.decode_sharpening;
  td_->debug            = config_.debug;

  for (const auto& name : supportedFamilies()) {
    auto tf = createFamily(name);
    if (tf) {
      apriltag_detector_add_family_bits(td_, tf, config_.max_hamming);
      families_[name] = tf;
    }
  }

  current_family_ = "ALL";
  return !families_.empty();
}

void AprilTagBackend::reconfigure(const AprilTagConfig& cfg) {
  config_ = cfg;
  if (td_) {
    td_->nthreads         = cfg.nthreads;
    td_->quad_decimate    = cfg.quad_decimate;
    td_->quad_sigma       = cfg.quad_sigma;
    td_->refine_edges     = cfg.refine_edges;
    td_->decode_sharpening = cfg.decode_sharpening;
    td_->debug            = cfg.debug;
  }
}

std::vector<AprilTagDetection> AprilTagBackend::detect(const cv::Mat& gray) {
  std::vector<AprilTagDetection> results;
  if (!td_ || families_.empty() || gray.empty() || gray.type() != CV_8UC1) {
    if (!td_ || families_.empty()) {
      fprintf(stderr,
        "[AprilTagBackend] WARN: detect() called but backend is not valid "
        "(td_=%s, families=%zu). AprilTag detection skipped.\n",
        td_ ? "ok" : "null", families_.size());
    }
    return results;
  }

  image_u8_t img_header = {
    gray.cols,
    gray.rows,
    static_cast<int32_t>(gray.step[0]),
    gray.data
  };

  zarray_t* detections = apriltag_detector_detect(td_, &img_header);
  if (!detections) return results;

  int n = zarray_size(detections);
  results.reserve(n);

  for (int i = 0; i < n; ++i) {
    apriltag_detection_t* det;
    zarray_get(detections, i, &det);

    if (det->hamming > config_.max_hamming) continue;
    if (det->decision_margin < config_.min_decision_margin) continue;

    AprilTagDetection d;
    d.id = det->id;
    d.hamming = det->hamming;
    d.decision_margin = det->decision_margin;
    d.center = cv::Point2f(static_cast<float>(det->c[0]),
                           static_cast<float>(det->c[1]));

    // resolve family name from the detection's family pointer
    d.family_name = "unknown";
    if (det->family) {
      for (const auto& kv : families_) {
        if (kv.second == det->family) {
          d.family_name = kv.first;
          break;
        }
      }
    }

    d.corners.resize(4);
    for (int j = 0; j < 4; ++j) {
      d.corners[j] = cv::Point2f(static_cast<float>(det->p[j][0]),
                                 static_cast<float>(det->p[j][1]));
    }

    results.push_back(std::move(d));
  }

  apriltag_detections_destroy(detections);
  return results;
}

AprilTagPose AprilTagBackend::estimatePose(
  const AprilTagDetection& det,
  double tagsize, double fx, double fy, double cx, double cy)
{
  AprilTagPose result;
  if (det.corners.size() != 4) return result;

  double half = tagsize / 2.0;
  std::vector<cv::Point3d> obj_pts = {
    {-half,  half, 0}, { half,  half, 0},
    { half, -half, 0}, {-half, -half, 0}
  };
  std::vector<cv::Point2d> img_pts(4);
  for (int i = 0; i < 4; ++i)
    img_pts[i] = cv::Point2d(det.corners[i].x, det.corners[i].y);

  cv::Mat K = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
  cv::Mat D = cv::Mat::zeros(4, 1, CV_64F);
  cv::Mat rvec, tvec;

  bool ok = cv::solvePnP(obj_pts, img_pts, K, D, rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);
  if (ok) {
    result.rvec = cv::Vec3d(rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2));
    result.tvec = cv::Vec3d(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
    result.valid = true;
    result.error = cv::norm(tvec);
  }
  return result;
}

int AprilTagBackend::familyCount() const {
  int total = 0;
  for (const auto& kv : families_)
    if (kv.second) total += static_cast<int>(kv.second->ncodes);
  return total;
}

std::vector<std::string> AprilTagBackend::loadedFamilies() const {
  std::vector<std::string> names;
  for (const auto& kv : families_) names.push_back(kv.first);
  return names;
}

apriltag_family_t* AprilTagBackend::createFamily(const std::string& name) {
  if (name == "tag16h5")          return tag16h5_create();
  if (name == "tag25h9")          return tag25h9_create();
  if (name == "tag36h10")         return tag36h10_create();
  if (name == "tag36h11")         return tag36h11_create();
  if (name == "tagStandard41h12") return tagStandard41h12_create();
  if (name == "tagStandard52h13") return tagStandard52h13_create();
  if (name == "tagCircle21h7")    return tagCircle21h7_create();
  if (name == "tagCircle49h12")   return tagCircle49h12_create();
  if (name == "tagCustom48h12")   return tagCustom48h12_create();
  return nullptr;
}

void AprilTagBackend::destroyFamily(apriltag_family_t* tf, const std::string& name) {
  if (!tf) return;
  if (name == "tag16h5")          tag16h5_destroy(tf);
  else if (name == "tag25h9")     tag25h9_destroy(tf);
  else if (name == "tag36h10")    tag36h10_destroy(tf);
  else if (name == "tag36h11")    tag36h11_destroy(tf);
  else if (name == "tagStandard41h12") tagStandard41h12_destroy(tf);
  else if (name == "tagStandard52h13") tagStandard52h13_destroy(tf);
  else if (name == "tagCircle21h7")    tagCircle21h7_destroy(tf);
  else if (name == "tagCircle49h12")   tagCircle49h12_destroy(tf);
  else if (name == "tagCustom48h12")   tagCustom48h12_destroy(tf);
}

std::string AprilTagBackend::mapDictToFamily(const std::string& dict_name) {
  static const std::map<std::string, std::string> MAP = {
    {"DICT_APRILTAG_16h5",   "tag16h5"},
    {"DICT_APRILTAG_25h9",   "tag25h9"},
    {"DICT_APRILTAG_36h10",  "tag36h10"},
    {"DICT_APRILTAG_36h11",  "tag36h11"},
  };
  auto it = MAP.find(dict_name);
  return (it != MAP.end()) ? it->second : "";
}

bool AprilTagBackend::isAprilTagFamily(const std::string& dict_name) {
  return dict_name.find("APRILTAG") != std::string::npos;
}

std::vector<std::string> AprilTagBackend::supportedFamilies() {
  return {
    "tag16h5", "tag25h9", "tag36h10", "tag36h11",
    "tagStandard41h12", "tagStandard52h13",
    "tagCircle21h7", "tagCircle49h12", "tagCustom48h12"
  };
}

}
