#!/usr/bin/env python3
"""Generate visualization.cpp"""
import os
SRC = "/home/arilix/Documents/vtol/vtol aruco/fiducial_ws/src/fiducial_detector/src"
VIZ_CPP = r"""#include "fiducial_detector/visualization.hpp"
namespace fiducial_detector {
Visualizer::Visualizer(int alignment_tolerance)
: alignment_tol_(alignment_tolerance) {}
void Visualizer::alphaRect(cv::Mat& frame, cv::Rect rect,
                           cv::Scalar color, double alpha) const
{
  cv::Mat roi = frame(rect & cv::Rect(0,0,frame.cols,frame.rows));
  cv::Mat overlay = roi.clone();
  cv::rectangle(overlay, cv::Rect(0,0,roi.cols,roi.rows), color, -1);
  cv::addWeighted(overlay, alpha, roi, 1.0-alpha, 0, roi);
}
void Visualizer::labelText(cv::Mat& frame, const std::string& text,
                           cv::Point origin, cv::Scalar color,
                           double fs, int th) const
{
  int baseline = 0;
  auto sz = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, fs, th, &baseline);
  cv::Rect bg(origin.x-2, origin.y-sz.height-2, sz.width+4, sz.height+baseline+4);
  bg &= cv::Rect(0,0,frame.cols,frame.rows);
  cv::rectangle(frame, bg, CLR_BLACK, -1);
  cv::putText(frame, text, origin, cv::FONT_HERSHEY_SIMPLEX, fs, color, th, cv::LINE_AA);
}
bool Visualizer::isAligned(cv::Point2f pt, const cv::Size& sz) const {
  float cx = sz.width * 0.5f, cy = sz.height * 0.5f;
  return std::abs(pt.x-cx) <= alignment_tol_ && std::abs(pt.y-cy) <= alignment_tol_;
}
bool Visualizer::isLeft (cv::Point2f pt, const cv::Size& sz) const {
  return pt.x < sz.width*0.5f - alignment_tol_;
}
bool Visualizer::isRight(cv::Point2f pt, const cv::Size& sz) const {
  return pt.x > sz.width*0.5f + alignment_tol_;
}
bool Visualizer::isUp   (cv::Point2f pt, const cv::Size& sz) const {
  return pt.y < sz.height*0.5f - alignment_tol_;
}
bool Visualizer::isDown (cv::Point2f pt, const cv::Size& sz) const {
  return pt.y > sz.height*0.5f + alignment_tol_;
}
std::string Visualizer::alignmentString(cv::Point2f pt, const cv::Size& sz) const
{
  if (isAligned(pt, sz))     return "TARGET_LOCK";
  std::string h, v;
  if      (isLeft(pt, sz))   h = "GESER_KIRI";
  else if (isRight(pt, sz))  h = "GESER_KANAN";
  if      (isUp(pt, sz))     v = "NAIK";
  else if (isDown(pt, sz))   v = "TURUN";
  if (h.empty()) return v;
  if (v.empty()) return h;
  return h + "|" + v;
}
void Visualizer::drawUI(cv::Mat& frame, bool any_locked) const
{
  int W = frame.cols, H = frame.rows;
  int cx = W/2, cy = H/2, tol = alignment_tol_;
  cv::Scalar oc = any_locked ? CLR_LOCKED : CLR_UNALIGNED;
  cv::Mat guide = frame.clone();
  cv::line(guide, {0, cy}, {W, cy}, CLR_GRAY, 1, cv::LINE_AA);
  cv::line(guide, {cx, 0}, {cx, H}, CLR_GRAY, 1, cv::LINE_AA);
  cv::addWeighted(guide, 0.5, frame, 0.5, 0, frame);
  int arm = 20;
  cv::line(frame, {cx-arm, cy}, {cx+arm, cy}, CLR_WHITE, 2, cv::LINE_AA);
  cv::line(frame, {cx, cy-arm}, {cx, cy+arm}, CLR_WHITE, 2, cv::LINE_AA);
  cv::Rect box(cx-tol, cy-tol, 2*tol, 2*tol);
  box &= cv::Rect(0,0,W,H);
  alphaRect(frame, box, oc, 0.12);
  cv::rectangle(frame, box, oc, 2, cv::LINE_AA);
  int tick = 12;
  auto ctick = [&](cv::Point p, int dx, int dy){
    cv::line(frame, p, {p.x+dx*tick, p.y}, oc, 2, cv::LINE_AA);
    cv::line(frame, p, {p.x, p.y+dy*tick}, oc, 2, cv::LINE_AA);
  };
  ctick({cx-tol,cy-tol},  1, 1);
  ctick({cx+tol,cy-tol}, -1, 1);
  ctick({cx+tol,cy+tol}, -1,-1);
  ctick({cx-tol,cy+tol},  1,-1);
  cv::circle(frame, {cx,cy}, 4, any_locked ? CLR_LOCKED : CLR_WHITE, -1, cv::LINE_AA);
}
void Visualizer::drawDetectedMarkers(
  cv::Mat& frame,
  const std::vector<std::vector<cv::Point2f>>& corners,
  const std::vector<int>& ids,
  MarkerType type) const
{
  if (corners.empty()) return;
  cv::Scalar color = markerColor(type);
  cv::aruco::drawDetectedMarkers(frame, corners, ids, color);
  for (std::size_t i = 0; i < corners.size(); ++i) {
    cv::Point2f c(0,0);
    for (const auto& p : corners[i]) c += p;
    c *= 0.25f;
    cv::circle(frame, c, 6, color, -1, cv::LINE_AA);
    std::string badge = markerTypeName(type) + " #" + std::to_string(ids[i]);
    labelText(frame, badge,
      cv::Point((int)c.x+8, (int)c.y-8), color, 0.45, 1);
  }
}
void Visualizer::drawRejected(
  cv::Mat& frame,
  const std::vector<std::vector<cv::Point2f>>& rejected) const
{
  if (rejected.empty()) return;
  std::vector<int> empty_ids;
  cv::aruco::drawDetectedMarkers(frame, rejected, empty_ids, CLR_REJECTED);
}
void Visualizer::drawPoseAxis(
  cv::Mat& frame,
  const PoseResult& pose,
  const cv::Mat& K,
  const cv::Mat& D,
  float axis_length) const
{
  if (!pose.valid || K.empty()) return;
  cv::aruco::drawAxis(frame, K, D, pose.rvec, pose.tvec, axis_length);
}
void Visualizer::drawMarkerInfo(
  cv::Mat& frame,
  cv::Point2f center,
  int id,
  MarkerType type,
  const PoseResult& pose) const
{
  cv::Scalar color = markerColor(type);
  int x = (int)center.x + 12, y = (int)center.y + 12;
  auto line = [&](const std::string& s){
    labelText(frame, s, {x, y}, color, 0.44, 1);
    y += 15;
  };
  line("ID: " + std::to_string(id) + "  [" + markerTypeName(type) + "]");
  line("Center: (" + std::to_string((int)center.x) + "," + std::to_string((int)center.y) + ")");
  if (pose.valid) {
    char buf[80];
    std::snprintf(buf, sizeof(buf), "Pose: (%.3f, %.3f, %.3f) m",
      pose.tvec[0], pose.tvec[1], pose.tvec[2]);
    line(buf);
    std::snprintf(buf, sizeof(buf), "Dist: %.3f m", pose.distance);
    line(buf);
    std::snprintf(buf, sizeof(buf), "Q: (%.2f,%.2f,%.2f,%.2f)",
      pose.quaternion.x(), pose.quaternion.y(),
      pose.quaternion.z(), pose.quaternion.w());
    line(buf);
  }
}
void Visualizer::drawAlignment(
  cv::Mat& frame,
  cv::Point2f marker_center,
  const cv::Size& sz) const
{
  int W = sz.width, H = sz.height;
  int cx = W/2, cy = H/2, tol = alignment_tol_;
  int font = cv::FONT_HERSHEY_SIMPLEX;
  double fs = 0.75;
  int th = 2;
  if (isAligned(marker_center, sz)) {
    labelText(frame, "TARGET LOCK",
      {cx-75, cy-tol-22}, CLR_LOCKED, 0.85, 2);
    labelText(frame, "POSISI CENTER",
      {cx-85, cy+tol+40}, CLR_ALIGNED, 0.78, 2);
    return;
  }
  if (isLeft(marker_center, sz))
    cv::putText(frame, "GESER KIRI",  {18, cy}, font, fs, CLR_UNALIGNED, th, cv::LINE_AA);
  else if (isRight(marker_center, sz))
    cv::putText(frame, "GESER KANAN", {W-175, cy}, font, fs, CLR_UNALIGNED, th, cv::LINE_AA);
  if (isUp(marker_center, sz))
    cv::putText(frame, "NAIK",  {cx-35, 38}, font, fs, CLR_UNALIGNED, th, cv::LINE_AA);
  else if (isDown(marker_center, sz))
    cv::putText(frame, "TURUN", {cx-42, H-22}, font, fs, CLR_UNALIGNED, th, cv::LINE_AA);
}
void Visualizer::drawHUD(
  cv::Mat& frame,
  float fps,
  double latency_ms,
  uint64_t frame_count,
  bool cam_ok) const
{
  char buf[128];
  int y = 25;
  auto hline = [&](const std::string& s, cv::Scalar c){
    cv::putText(frame, s, {10, y}, cv::FONT_HERSHEY_SIMPLEX, 0.58, c, 2, cv::LINE_AA);
    y += 22;
  };
  std::snprintf(buf, sizeof(buf), "FPS: %.1f", fps);
  hline(buf, CLR_WHITE);
  std::snprintf(buf, sizeof(buf), "Latency: %.1f ms", latency_ms);
  hline(buf, CLR_GRAY);
  std::snprintf(buf, sizeof(buf), "Frame: %llu", (unsigned long long)frame_count);
  hline(buf, CLR_GRAY);
  std::snprintf(buf, sizeof(buf), "CAM: %s", cam_ok ? "OK" : "RECONNECTING...");
  hline(buf, cam_ok ? CLR_ALIGNED : CLR_UNALIGNED);
}
}
"""
path = os.path.join(SRC, "visualization.cpp")
with open(path, "w") as f:
    f.write(VIZ_CPP.lstrip("\n"))
print(f"WROTE {path}  ({VIZ_CPP.count(chr(10))} lines)")
print("VIZ OK")
