#pragma once
#include "pose_estimator.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <string>
#include <map>
#include <vector>
namespace fiducial_detector {
enum class MarkerType : uint8_t {
  ARUCO    = 0,
  APRILTAG = 1,
  ARTAG    = 2,
  CHARUCO  = 3,
  UNKNOWN  = 99,
};
inline const cv::Scalar CLR_ARUCO      {  0, 220,   0};
inline const cv::Scalar CLR_APRILTAG   {220,   0,   0};
inline const cv::Scalar CLR_CHARUCO    {  0, 220, 220};
inline const cv::Scalar CLR_ARTAG      {  0,   0, 220};
inline const cv::Scalar CLR_REJECTED   { 50,  50, 220};
inline const cv::Scalar CLR_LOCKED     {220, 220,   0};
inline const cv::Scalar CLR_ALIGNED    {  0, 255,   0};
inline const cv::Scalar CLR_UNALIGNED  {  0,   0, 255};
inline const cv::Scalar CLR_WHITE      {255, 255, 255};
inline const cv::Scalar CLR_BLACK      {  0,   0,   0};
inline const cv::Scalar CLR_GRAY       {130, 130, 130};
inline cv::Scalar markerColor(MarkerType t) {
  switch (t) {
    case MarkerType::ARUCO:    return CLR_ARUCO;
    case MarkerType::APRILTAG: return CLR_APRILTAG;
    case MarkerType::CHARUCO:  return CLR_CHARUCO;
    case MarkerType::ARTAG:    return CLR_ARTAG;
    default:                   return CLR_WHITE;
  }
}
inline std::string markerTypeName(MarkerType t) {
  switch (t) {
    case MarkerType::ARUCO:    return "ArUco";
    case MarkerType::APRILTAG: return "AprilTag";
    case MarkerType::CHARUCO:  return "ChArUco";
    case MarkerType::ARTAG:    return "ARTag";
    default:                   return "Unknown";
  }
}
class Visualizer {
public:
  explicit Visualizer(int alignment_tolerance = 50);
  void setAlignmentTolerance(int tol) { alignment_tol_ = tol; }
  int  getAlignmentTolerance() const  { return alignment_tol_; }
  void drawUI(cv::Mat& frame, bool any_marker_locked) const;
  void drawDetectedMarkers(
    cv::Mat& frame,
    const std::vector<std::vector<cv::Point2f>>& corners,
    const std::vector<int>& ids,
    MarkerType type) const;
  void drawRejected(
    cv::Mat& frame,
    const std::vector<std::vector<cv::Point2f>>& rejected) const;
  void drawPoseAxis(
    cv::Mat& frame,
    const PoseResult& pose,
    const cv::Mat& K,
    const cv::Mat& D,
    float axis_length = 0.03f) const;
  void drawMarkerInfo(
    cv::Mat& frame,
    cv::Point2f center,
    int id,
    MarkerType type,
    const PoseResult& pose) const;
  void drawAlignment(
    cv::Mat& frame,
    cv::Point2f marker_center,
    const cv::Size& frame_size) const;
  void drawDictionaryPanel(
    cv::Mat& frame,
    const std::string& active_dict,
    const std::string& mode,
    const std::map<std::string,double>& scores) const;
  void drawHUD(
    cv::Mat& frame,
    float fps,
    double latency_ms,
    uint64_t frame_count,
    bool cam_ok) const;
  bool isAligned(cv::Point2f pt, const cv::Size& sz) const;
  bool isLeft   (cv::Point2f pt, const cv::Size& sz) const;
  bool isRight  (cv::Point2f pt, const cv::Size& sz) const;
  bool isUp     (cv::Point2f pt, const cv::Size& sz) const;
  bool isDown   (cv::Point2f pt, const cv::Size& sz) const;
  std::string alignmentString(cv::Point2f pt, const cv::Size& sz) const;
private:
  int alignment_tol_;
  void alphaRect(cv::Mat& frame, cv::Rect rect,
                 cv::Scalar color, double alpha) const;
  void labelText(cv::Mat& frame, const std::string& text,
                 cv::Point origin, cv::Scalar color,
                 double font_scale = 0.55, int thickness = 1) const;
};
}
