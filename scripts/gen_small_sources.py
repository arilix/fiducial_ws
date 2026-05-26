#!/usr/bin/env python3
"""Generate fps_monitor.cpp, pose_estimator.cpp, detector_parameters.cpp."""
import os
SRC = "/home/arilix/Documents/vtol/vtol aruco/fiducial_ws/src/fiducial_detector/src"
os.makedirs(SRC, exist_ok=True)
FPS_CPP = r"""#include "fiducial_detector/fps_monitor.hpp"
namespace fiducial_detector {
FpsMonitor::FpsMonitor(std::size_t window_size)
: window_size_(window_size) {}
void FpsMonitor::tick()
{
  auto now = Clock::now();
  std::lock_guard<std::mutex> lk(mtx_);
  if (!timestamps_.empty()) {
    last_latency_ms_ = std::chrono::duration<double, std::milli>(
      now - timestamps_.back()).count();
  }
  timestamps_.push_back(now);
  while (timestamps_.size() > window_size_) {
    timestamps_.pop_front();
  }
}
float FpsMonitor::getFps() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (timestamps_.size() < 2) return 0.0f;
  double span_s = std::chrono::duration<double>(
    timestamps_.back() - timestamps_.front()).count();
  if (span_s <= 0.0) return 0.0f;
  return static_cast<float>((timestamps_.size() - 1) / span_s);
}
double FpsMonitor::getLatencyMs() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  return last_latency_ms_;
}
void FpsMonitor::reset()
{
  std::lock_guard<std::mutex> lk(mtx_);
  timestamps_.clear();
  last_latency_ms_ = 0.0;
}
}
"""
POSE_CPP = r"""#include "fiducial_detector/pose_estimator.hpp"
namespace fiducial_detector {
PoseEstimator::PoseEstimator(const cv::Mat& K, const cv::Mat& D, double marker_size)
: K_(K.clone()), D_(D.clone()), marker_size_(marker_size)
{}
std::vector<cv::Point3f> PoseEstimator::makeObjectPoints() const
{
  float h = static_cast<float>(marker_size_) * 0.5f;
  return {{-h,  h, 0.f},
          { h,  h, 0.f},
          { h, -h, 0.f},
          {-h, -h, 0.f}};
}
void PoseEstimator::applySmoothing(int id, cv::Vec3d& rvec, cv::Vec3d& tvec)
{
  auto& s = smooth_state_[id];
  if (!s.initialised) {
    s.rvec = rvec;
    s.tvec = tvec;
    s.initialised = true;
    return;
  }
  for (int i = 0; i < 3; ++i) {
    s.rvec[i] = alpha_ * rvec[i] + (1.0 - alpha_) * s.rvec[i];
    s.tvec[i] = alpha_ * tvec[i] + (1.0 - alpha_) * s.tvec[i];
  }
  rvec = s.rvec;
  tvec = s.tvec;
}
PoseResult PoseEstimator::estimate(int id, const std::vector<cv::Point2f>& corners)
{
  PoseResult result;
  if (corners.size() != 4 || K_.empty()) return result;
  auto obj_pts = makeObjectPoints();
  cv::Vec3d rvec, tvec;
  std::vector<std::vector<cv::Point2f>> corners_wrap{corners};
  std::vector<cv::Vec3d> rvecs, tvecs;
  cv::aruco::estimatePoseSingleMarkers(
    corners_wrap, static_cast<float>(marker_size_), K_, D_, rvecs, tvecs);
  if (rvecs.empty()) {
    bool ok = cv::solvePnP(obj_pts, corners, K_, D_, rvec, tvec,
                           false, cv::SOLVEPNP_IPPE_SQUARE);
    if (!ok) return result;
  } else {
    rvec = rvecs[0];
    tvec = tvecs[0];
  }
  applySmoothing(id, rvec, tvec);
  result.rvec     = rvec;
  result.tvec     = tvec;
  result.distance = cv::norm(tvec);
  cv::Mat rot_mat;
  cv::Rodrigues(rvec, rot_mat);
  Eigen::Matrix3d erot;
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      erot(r, c) = rot_mat.at<double>(r, c);
  result.quaternion = Eigen::Quaterniond(erot).normalized();
  result.valid      = true;
  return result;
}
std::vector<PoseResult> PoseEstimator::estimateBatch(
  const std::vector<int>& ids,
  const std::vector<std::vector<cv::Point2f>>& corners_vec)
{
  std::vector<PoseResult> results;
  results.reserve(ids.size());
  for (std::size_t i = 0; i < ids.size(); ++i) {
    results.push_back(estimate(ids[i], corners_vec[i]));
  }
  return results;
}
}
"""
DET_PARAMS_CPP = r"""#include "fiducial_detector/detector_parameters.hpp"
namespace fiducial_detector {
DetectorParametersManager::DetectorParametersManager()
: params_(cv::aruco::DetectorParameters::create())
{}
int DetectorParametersManager::dictId(const std::string& name)
{
  auto it = DICT_NAME_MAP.find(name);
  if (it != DICT_NAME_MAP.end()) return it->second;
  RCLCPP_WARN(rclcpp::get_logger("DetectorParametersManager"),
    "Unknown dictionary '%s', falling back to DICT_4X4_50", name.c_str());
  return cv::aruco::DICT_4X4_50;
}
cv::Ptr<cv::aruco::Dictionary> DetectorParametersManager::makeDict(const std::string& name)
{
  return cv::aruco::getPredefinedDictionary(dictId(name));
}
void DetectorParametersManager::declareAll(rclcpp::Node* node)
{
  node->declare_parameter("adaptiveThreshWinSizeMin",    3);
  node->declare_parameter("adaptiveThreshWinSizeMax",   23);
  node->declare_parameter("adaptiveThreshWinSizeStep",   10);
  node->declare_parameter("adaptiveThreshConstant",     7.0);
  node->declare_parameter("minMarkerPerimeterRate",     0.03);
  node->declare_parameter("maxMarkerPerimeterRate",     4.0);
  node->declare_parameter("polygonalApproxAccuracyRate",0.03);
  node->declare_parameter("minCornerDistanceRate",      0.05);
  node->declare_parameter("minDistanceToBorder",        3);
  node->declare_parameter("minMarkerDistanceRate",      0.05);
  node->declare_parameter("perspectiveRemovePixelPerCell",          8);
  node->declare_parameter("perspectiveRemoveIgnoredMarginPerCell", 0.13);
  node->declare_parameter("maxErroneousBitsInBorderRate", 0.35);
  node->declare_parameter("errorCorrectionRate",          0.6);
  node->declare_parameter("detectInvertedMarker",         false);
  node->declare_parameter("cornerRefinementMethod",       1);
  node->declare_parameter("cornerRefinementWinSize",      5);
  node->declare_parameter("cornerRefinementMaxIterations",30);
  node->declare_parameter("cornerRefinementMinAccuracy",  0.1);
}
void DetectorParametersManager::bind(rclcpp::Node* node)
{
  params_->adaptiveThreshWinSizeMin    = node->get_parameter("adaptiveThreshWinSizeMin").as_int();
  params_->adaptiveThreshWinSizeMax    = node->get_parameter("adaptiveThreshWinSizeMax").as_int();
  params_->adaptiveThreshWinSizeStep   = node->get_parameter("adaptiveThreshWinSizeStep").as_int();
  params_->adaptiveThreshConstant      = node->get_parameter("adaptiveThreshConstant").as_double();
  params_->minMarkerPerimeterRate      = node->get_parameter("minMarkerPerimeterRate").as_double();
  params_->maxMarkerPerimeterRate      = node->get_parameter("maxMarkerPerimeterRate").as_double();
  params_->polygonalApproxAccuracyRate = node->get_parameter("polygonalApproxAccuracyRate").as_double();
  params_->minCornerDistanceRate       = node->get_parameter("minCornerDistanceRate").as_double();
  params_->minDistanceToBorder         = node->get_parameter("minDistanceToBorder").as_int();
  params_->minMarkerDistanceRate       = node->get_parameter("minMarkerDistanceRate").as_double();
  params_->perspectiveRemovePixelPerCell =
    node->get_parameter("perspectiveRemovePixelPerCell").as_int();
  params_->perspectiveRemoveIgnoredMarginPerCell =
    node->get_parameter("perspectiveRemoveIgnoredMarginPerCell").as_double();
  params_->maxErroneousBitsInBorderRate =
    node->get_parameter("maxErroneousBitsInBorderRate").as_double();
  params_->errorCorrectionRate          =
    node->get_parameter("errorCorrectionRate").as_double();
  params_->detectInvertedMarker         =
    node->get_parameter("detectInvertedMarker").as_bool();
  int cr_method = node->get_parameter("cornerRefinementMethod").as_int();
  params_->cornerRefinementMethod = static_cast<cv::aruco::CornerRefineMethod>(cr_method);
  params_->cornerRefinementWinSize =
    node->get_parameter("cornerRefinementWinSize").as_int();
  params_->cornerRefinementMaxIterations =
    node->get_parameter("cornerRefinementMaxIterations").as_int();
  params_->cornerRefinementMinAccuracy =
    node->get_parameter("cornerRefinementMinAccuracy").as_double();
}
}
"""
FILES = {
    "fps_monitor.cpp":         FPS_CPP,
    "pose_estimator.cpp":      POSE_CPP,
    "detector_parameters.cpp": DET_PARAMS_CPP,
}
for fname, code in FILES.items():
    path = os.path.join(SRC, fname)
    with open(path, "w") as f:
        f.write(code.lstrip("\n"))
    print(f"  WROTE {path}  ({code.count(chr(10))} lines)")
print("SMALL SOURCES OK")
