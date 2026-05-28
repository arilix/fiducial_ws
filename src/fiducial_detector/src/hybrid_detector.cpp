#include "fiducial_detector/hybrid_detector.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <numeric>

namespace fiducial_detector {

HybridDetector::HybridDetector(rclcpp::Logger logger)
  : logger_(logger) {}

void HybridDetector::init(
  const std::string& dict_name,
  const AprilTagConfig& at_cfg,
  cv::Ptr<cv::aruco::Dictionary> opencv_dict,
  cv::Ptr<cv::aruco::DetectorParameters> opencv_params)
{
  opencv_dict_   = opencv_dict;
  opencv_params_ = opencv_params;
  current_dict_  = dict_name;

  AprilTagConfig cfg = at_cfg;
  cfg.load_all_families = true;

  if (at_backend_.configure(cfg)) {
    std::string families_str;
    for (const auto& f : at_backend_.loadedFamilies()) {
      if (!families_str.empty()) families_str += ", ";
      families_str += f;
    }
    RCLCPP_INFO(logger_,
      "AprilTag3: %d families loaded [%s] | total_tags=%d | threads=%d | decimate=%.1f",
      at_backend_.loadedFamilyCount(), families_str.c_str(),
      at_backend_.familyCount(), cfg.nthreads, cfg.quad_decimate);
  } else {
    RCLCPP_WARN(logger_, "AprilTag3 backend: failed to load any family");
  }
}

void HybridDetector::setAprilTagConfig(const AprilTagConfig& cfg) {
  at_backend_.reconfigure(cfg);
}

DetectorBackend HybridDetector::autoSelectBackend(const std::string& /*dict_name*/) {
  // All 9 AprilTag families are always loaded — always fuse both backends
  if (at_backend_.isValid()) {
    return DetectorBackend::FUSION;
  }
  return DetectorBackend::OPENCV;
}

std::string HybridDetector::backendName(DetectorBackend b) {
  switch (b) {
    case DetectorBackend::OPENCV:    return "OPENCV";
    case DetectorBackend::APRILTAG3: return "APRILTAG3";
    case DetectorBackend::AUTO:      return "AUTO";
    case DetectorBackend::FUSION:    return "FUSION";
  }
  return "UNKNOWN";
}

HybridResult HybridDetector::detect(
  const cv::Mat& gray,
  const std::string& dict_name,
  DetectorBackend mode)
{
  DetectorBackend effective = mode;
  if (mode == DetectorBackend::AUTO) {
    effective = autoSelectBackend(dict_name);
  }

  auto t0 = std::chrono::steady_clock::now();
  HybridResult result;

  switch (effective) {
    case DetectorBackend::APRILTAG3:
      result = detectAprilTag3(gray);
      break;
    case DetectorBackend::FUSION:
      result = detectFusion(gray);
      break;
    case DetectorBackend::OPENCV:
    default:
      result = detectOpenCV(gray);
      break;
  }

  auto t1 = std::chrono::steady_clock::now();
  result.latency_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  result.backend_used = effective;
  return result;
}

HybridResult HybridDetector::detectOpenCV(const cv::Mat& gray) {
  HybridResult result;
  if (!opencv_dict_ || !opencv_params_) return result;

  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners, rejected;
  cv::aruco::detectMarkers(gray, opencv_dict_, corners, ids, opencv_params_, rejected);

  for (std::size_t i = 0; i < ids.size(); ++i) {
    HybridMarker m;
    m.id      = ids[i];
    m.corners = corners[i];
    m.source  = DetectorBackend::OPENCV;
    m.family  = "aruco";

    cv::Point2f c(0, 0);
    for (auto& p : m.corners) c += p;
    m.center = c * 0.25f;
    m.confidence = computeMarkerConfidence(m, DetectorBackend::OPENCV);

    result.markers.push_back(std::move(m));
  }

  result.rejected = rejected;
  result.opencv_count = static_cast<int>(ids.size());
  return result;
}

HybridResult HybridDetector::detectAprilTag3(const cv::Mat& gray) {
  HybridResult result;
  if (!at_backend_.isValid()) return result;

  auto detections = at_backend_.detect(gray);

  for (auto& d : detections) {
    HybridMarker m;
    m.id              = d.id;
    m.corners         = d.corners;
    m.center          = d.center;
    m.hamming         = d.hamming;
    m.decision_margin = d.decision_margin;
    m.family          = d.family_name;
    m.source          = DetectorBackend::APRILTAG3;
    m.confidence      = computeMarkerConfidence(m, DetectorBackend::APRILTAG3);
    result.markers.push_back(std::move(m));
  }

  result.apriltag_count = static_cast<int>(detections.size());
  return result;
}

HybridResult HybridDetector::detectFusion(const cv::Mat& gray) {
  auto ocv_result = detectOpenCV(gray);
  auto at_result  = detectAprilTag3(gray);

  HybridResult result;
  result.opencv_count   = ocv_result.opencv_count;
  result.apriltag_count = at_result.apriltag_count;
  result.rejected       = ocv_result.rejected;
  result.markers        = fuseDetections(ocv_result.markers, at_result.markers);
  return result;
}

// deduplicate by proximity, prefer higher-confidence detection
std::vector<HybridMarker> HybridDetector::fuseDetections(
  const std::vector<HybridMarker>& opencv,
  const std::vector<HybridMarker>& apriltag)
{
  constexpr float MERGE_DIST = 30.0f;
  constexpr float OPENCV_WEIGHT = 0.4f;
  constexpr float APRILTAG_WEIGHT = 0.6f;

  std::vector<HybridMarker> fused;
  std::vector<bool> at_used(apriltag.size(), false);

  for (const auto& om : opencv) {
    bool merged = false;
    for (std::size_t j = 0; j < apriltag.size(); ++j) {
      if (at_used[j]) continue;
      float dx = om.center.x - apriltag[j].center.x;
      float dy = om.center.y - apriltag[j].center.y;
      float dist = std::sqrt(dx * dx + dy * dy);

      if (dist < MERGE_DIST && om.id == apriltag[j].id) {
        float score_ocv = om.confidence * OPENCV_WEIGHT;
        float score_at  = apriltag[j].confidence * APRILTAG_WEIGHT;

        if (score_at >= score_ocv) {
          HybridMarker m = apriltag[j];
          m.confidence = score_ocv + score_at;
          fused.push_back(std::move(m));
        } else {
          HybridMarker m = om;
          m.confidence = score_ocv + score_at;
          fused.push_back(std::move(m));
        }
        at_used[j] = true;
        merged = true;
        break;
      }
    }
    if (!merged) {
      fused.push_back(om);
    }
  }

  for (std::size_t j = 0; j < apriltag.size(); ++j) {
    if (!at_used[j]) {
      fused.push_back(apriltag[j]);
    }
  }

  return fused;
}

float HybridDetector::computeMarkerConfidence(const HybridMarker& m, DetectorBackend src) {
  float conf = 0.5f;

  if (src == DetectorBackend::APRILTAG3) {
    conf = std::clamp(m.decision_margin / 100.0f, 0.0f, 1.0f);
    if (m.hamming == 0) conf = std::min(conf + 0.2f, 1.0f);
    else if (m.hamming == 1) conf = std::min(conf + 0.1f, 1.0f);
  } else {
    if (m.corners.size() == 4) {
      double side0 = cv::norm(m.corners[0] - m.corners[1]);
      double side1 = cv::norm(m.corners[1] - m.corners[2]);
      double side2 = cv::norm(m.corners[2] - m.corners[3]);
      double side3 = cv::norm(m.corners[3] - m.corners[0]);
      double avg = (side0 + side1 + side2 + side3) / 4.0;
      double dev = (std::abs(side0 - avg) + std::abs(side1 - avg) +
                    std::abs(side2 - avg) + std::abs(side3 - avg)) / 4.0;
      double regularity = 1.0 - std::min(dev / avg, 1.0);
      conf = static_cast<float>(regularity);
    }
  }

  return conf;
}

HybridBenchmark HybridDetector::benchmark(const cv::Mat& gray, int n_frames) {
  HybridBenchmark bm;

  {
    auto t0 = std::chrono::steady_clock::now();
    int total = 0;
    for (int i = 0; i < n_frames; ++i) {
      auto r = detectOpenCV(gray);
      total += r.opencv_count;
    }
    auto t1 = std::chrono::steady_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    bm.opencv_latency_ms = ms / n_frames;
    bm.opencv_fps = (ms > 0) ? (n_frames * 1000.0 / ms) : 0;
    bm.opencv_detected = total / std::max(n_frames, 1);
  }

  if (at_backend_.isValid()) {
    auto t0 = std::chrono::steady_clock::now();
    int total = 0;
    for (int i = 0; i < n_frames; ++i) {
      auto r = detectAprilTag3(gray);
      total += r.apriltag_count;
    }
    auto t1 = std::chrono::steady_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    bm.apriltag_latency_ms = ms / n_frames;
    bm.apriltag_fps = (ms > 0) ? (n_frames * 1000.0 / ms) : 0;
    bm.apriltag_detected = total / std::max(n_frames, 1);
  }

  return bm;
}

}
