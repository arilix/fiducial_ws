#!/usr/bin/env python3
"""Append remaining sections to aruco.cpp"""
import os
TARGET = "/home/arilix/Documents/vtol/vtol aruco/fiducial_ws/src/fiducial_detector/src/aruco.cpp"
DETECT_SECTION = r"""
void FiducialDetector::detectAruco(const cv::Mat & frame,
                                   std::vector<DetectedMarker> & out)
{
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners, rejected;
  cv::aruco::detectMarkers(frame, aruco_dict_, corners, ids, aruco_params_, rejected);
  for (size_t i = 0; i < ids.size(); ++i) {
    DetectedMarker m;
    m.id      = ids[i];
    m.type    = MarkerType::ARUCO;
    m.corners = corners[i];
    m.color   = COLOR_ARUCO;
    cv::Point2f c(0,0);
    for (const auto& pt : m.corners) c += pt;
    m.center = c * 0.25f;
    out.push_back(std::move(m));
  }
}
void FiducialDetector::detectARTag(const cv::Mat & frame,
                                   std::vector<DetectedMarker> & out)
{
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners, rejected;
  cv::aruco::detectMarkers(frame, artag_dict_, corners, ids, aruco_params_, rejected);
  for (size_t i = 0; i < ids.size(); ++i) {
    DetectedMarker m;
    m.id      = ids[i];
    m.type    = MarkerType::ARTAG;
    m.corners = corners[i];
    m.color   = COLOR_ARTAG;
    cv::Point2f c(0,0);
    for (const auto& pt : m.corners) c += pt;
    m.center = c * 0.25f;
    out.push_back(std::move(m));
  }
}
void FiducialDetector::detectAprilTag(const cv::Mat & frame,
                                      std::vector<DetectedMarker> & out)
{
  if (!tag_detector_) return;
  cv::Mat gray;
  cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
  image_u8_t img { gray.cols, gray.rows, gray.cols, gray.data };
  zarray_t* dets = apriltag_detector_detect(tag_detector_, &img);
  for (int i = 0; i < zarray_size(dets); ++i) {
    apriltag_detection_t* d;
    zarray_get(dets, i, &d);
    DetectedMarker m;
    m.id    = d->id;
    m.type  = MarkerType::APRILTAG;
    m.color = COLOR_APRILTAG;
    m.corners.resize(4);
    m.corners[0] = cv::Point2f((float)d->p[3][0], (float)d->p[3][1]);
    m.corners[1] = cv::Point2f((float)d->p[2][0], (float)d->p[2][1]);
    m.corners[2] = cv::Point2f((float)d->p[1][0], (float)d->p[1][1]);
    m.corners[3] = cv::Point2f((float)d->p[0][0], (float)d->p[0][1]);
    m.center = cv::Point2f((float)d->c[0], (float)d->c[1]);
    out.push_back(std::move(m));
  }
  apriltag_detections_destroy(dets);
}
void FiducialDetector::detectCharuco(const cv::Mat & frame,
                                     std::vector<DetectedMarker> & out)
{
  std::vector<int> mids;
  std::vector<std::vector<cv::Point2f>> mcorners;
  cv::aruco::detectMarkers(frame, aruco_dict_, mcorners, mids, aruco_params_);
  if (mids.empty()) return;
  std::vector<cv::Point2f> cc;
  std::vector<int> ci;
  cv::aruco::interpolateCornersCharuco(mcorners, mids, frame, charuco_board_,
    cc, ci, intrinsics_.cameraMatrix, intrinsics_.distCoeffs);
  if (ci.empty()) return;
  cv::Point2f c(0,0);
  for (const auto& pt : cc) c += pt;
  c *= (1.0f / (float)cc.size());
  DetectedMarker m;
  m.id = 0; m.type = MarkerType::CHARUCO; m.color = COLOR_CHARUCO; m.center = c;
  auto br = cv::boundingRect(cc);
  m.corners = {
    cv::Point2f((float)br.x,            (float)br.y),
    cv::Point2f((float)(br.x+br.width), (float)br.y),
    cv::Point2f((float)(br.x+br.width), (float)(br.y+br.height)),
    cv::Point2f((float)br.x,            (float)(br.y+br.height))
  };
  out.push_back(std::move(m));
}
void FiducialDetector::estimatePose(DetectedMarker & m)
{
  if (!intrinsics_.valid || m.corners.size() != 4) return;
  float half = (float)marker_size_ * 0.5f;
  std::vector<cv::Point3f> obj = {
    {-half,  half, 0.f},
    { half,  half, 0.f},
    { half, -half, 0.f},
    {-half, -half, 0.f}
  };
  bool ok = cv::solvePnP(obj, m.corners,
    intrinsics_.cameraMatrix, intrinsics_.distCoeffs,
    m.rvec, m.tvec, false, cv::SOLVEPNP_IPPE_SQUARE);
  if (ok) {
    m.distance = cv::norm(m.tvec);
    m.poseValid = true;
  } else {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("FiducialDetector"),
      *rclcpp::Clock::make_shared(), 2000, "solvePnP failed for marker %d", m.id);
  }
}
void FiducialDetector::smoothMarkers(std::vector<DetectedMarker> & dets)
{
  std::lock_guard<std::mutex> lock(markers_mutex_);
  for (auto & s : tracked_markers_) s.missedFrames++;
  for (auto & d : dets) {
    bool found = false;
    for (auto & s : tracked_markers_) {
      if (s.id == d.id && s.type == d.type) {
        float a = (float)smoothing_alpha_;
        s.center.x  = a * d.center.x  + (1-a) * s.center.x;
        s.center.y  = a * d.center.y  + (1-a) * s.center.y;
        s.tvec[0]   = a * d.tvec[0]   + (1-a) * s.tvec[0];
        s.tvec[1]   = a * d.tvec[1]   + (1-a) * s.tvec[1];
        s.tvec[2]   = a * d.tvec[2]   + (1-a) * s.tvec[2];
        s.distance  = a * d.distance  + (1-a) * s.distance;
        s.missedFrames = 0;
        d.center   = s.center;
        d.tvec     = s.tvec;
        d.distance = s.distance;
        found = true;
        break;
      }
    }
    if (!found) {
      SmoothedMarker s;
      s.id = d.id; s.type = d.type;
      s.center = d.center; s.tvec = d.tvec; s.distance = d.distance;
      tracked_markers_.push_back(s);
    }
  }
  tracked_markers_.erase(
    std::remove_if(tracked_markers_.begin(), tracked_markers_.end(),
      [this](const SmoothedMarker& s){ return s.missedFrames > max_missed_frames_; }),
    tracked_markers_.end());
}
"""
ALIGN_SECTION = r"""
HorizontalAlign FiducialDetector::getHorizAlign(const cv::Point2f& pt,
                                                 const cv::Size& sz) const
{
  float cx = sz.width * 0.5f;
  if (pt.x < cx - alignment_tolerance_) return HorizontalAlign::LEFT;
  if (pt.x > cx + alignment_tolerance_) return HorizontalAlign::RIGHT;
  return HorizontalAlign::CENTER;
}
VerticalAlign FiducialDetector::getVertAlign(const cv::Point2f& pt,
                                              const cv::Size& sz) const
{
  float cy = sz.height * 0.5f;
  if (pt.y < cy - alignment_tolerance_) return VerticalAlign::UP;
  if (pt.y > cy + alignment_tolerance_) return VerticalAlign::DOWN;
  return VerticalAlign::CENTER;
}
bool FiducialDetector::isAligned(const cv::Point2f& pt, const cv::Size& sz) const
{
  float cx = sz.width  * 0.5f;
  float cy = sz.height * 0.5f;
  return (std::abs(pt.x - cx) <= alignment_tolerance_ &&
          std::abs(pt.y - cy) <= alignment_tolerance_);
}
std::string FiducialDetector::buildAlignmentString(
    const std::vector<DetectedMarker>& markers, const cv::Size& sz) const
{
  if (markers.empty()) return "NO_MARKER";
  const auto& m = markers[0];
  if (isAligned(m.center, sz)) return "TARGET_LOCK";
  std::string h, v;
  switch (getHorizAlign(m.center, sz)) {
    case HorizontalAlign::LEFT:  h = "GESER_KIRI";  break;
    case HorizontalAlign::RIGHT: h = "GESER_KANAN"; break;
    default:                     h = "CENTER_H";    break;
  }
  switch (getVertAlign(m.center, sz)) {
    case VerticalAlign::UP:   v = "NAIK";  break;
    case VerticalAlign::DOWN: v = "TURUN"; break;
    default:                  v = "CENTER_V"; break;
  }
  if (h == "CENTER_H") return v;
  if (v == "CENTER_V") return h;
  return h + "|" + v;
}
"""
UI_SECTION = r"""
void FiducialDetector::drawUI(cv::Mat & frame,
                              const std::vector<DetectedMarker> & markers)
{
  int W = frame.cols, H = frame.rows;
  int cx = W / 2, cy = H / 2;
  int tol = alignment_tolerance_;
  bool locked = !markers.empty() && isAligned(markers[0].center, frame.size());
  cv::Scalar overlay_color = locked ? COLOR_ALIGNED : COLOR_UNALIGNED;
  cv::Mat overlay = frame.clone();
  cv::rectangle(overlay,
    cv::Point(cx - tol, cy - tol),
    cv::Point(cx + tol, cy + tol),
    overlay_color, 2);
  int tick = 15;
  auto drawCornerTick = [&](cv::Point corner, int dx, int dy) {
    cv::line(overlay, corner, {corner.x + dx*tick, corner.y}, overlay_color, 2);
    cv::line(overlay, corner, {corner.x, corner.y + dy*tick}, overlay_color, 2);
  };
  drawCornerTick({cx-tol, cy-tol},  1,  1);
  drawCornerTick({cx+tol, cy-tol}, -1,  1);
  drawCornerTick({cx+tol, cy+tol}, -1, -1);
  drawCornerTick({cx-tol, cy+tol},  1, -1);
  cv::addWeighted(overlay, 0.7, frame, 0.3, 0, frame);
  cv::line(frame, {0, cy}, {W, cy}, COLOR_WHITE, 1, cv::LINE_AA);
  cv::line(frame, {cx, 0}, {cx, H}, COLOR_WHITE, 1, cv::LINE_AA);
  cv::circle(frame, {cx, cy}, 5, overlay_color, -1, cv::LINE_AA);
  if (markers.empty()) return;
  const auto& m = markers[0];
  auto h = getHorizAlign(m.center, frame.size());
  auto v = getVertAlign(m.center,  frame.size());
  int font = cv::FONT_HERSHEY_SIMPLEX;
  double fs = 0.7;
  int thickness = 2;
  if (locked) {
    cv::putText(frame, "TARGET LOCK",
      {cx - 80, cy - tol - 20}, font, 0.9, COLOR_ALIGNED, thickness, cv::LINE_AA);
    cv::putText(frame, "POSISI CENTER",
      {cx - 90, cy + tol + 40}, font, 0.8, COLOR_ALIGNED, 2, cv::LINE_AA);
  } else {
    if (h == HorizontalAlign::LEFT)
      cv::putText(frame, "GESER KIRI",  {30, cy}, font, fs, COLOR_UNALIGNED, thickness, cv::LINE_AA);
    else if (h == HorizontalAlign::RIGHT)
      cv::putText(frame, "GESER KANAN", {W-160, cy}, font, fs, COLOR_UNALIGNED, thickness, cv::LINE_AA);
    if (v == VerticalAlign::UP)
      cv::putText(frame, "NAIK",  {cx-30, 40}, font, fs, COLOR_UNALIGNED, thickness, cv::LINE_AA);
    else if (v == VerticalAlign::DOWN)
      cv::putText(frame, "TURUN", {cx-40, H-20}, font, fs, COLOR_UNALIGNED, thickness, cv::LINE_AA);
  }
}
void FiducialDetector::drawMarker(cv::Mat & frame, const DetectedMarker & m)
{
  if (m.corners.size() != 4) return;
  std::vector<cv::Point> pts;
  for (const auto& p : m.corners) pts.push_back({(int)p.x, (int)p.y});
  const cv::Point* ptsArr = pts.data();
  int nPts = (int)pts.size();
  cv::polylines(frame, &ptsArr, &nPts, 1, true, m.color, 2, cv::LINE_AA);
  cv::Point2i ci((int)m.center.x, (int)m.center.y);
  cv::circle(frame, ci, 5, m.color, -1, cv::LINE_AA);
  std::string label = markerTypeString(m.type) + " #" + std::to_string(m.id);
  int baseline = 0;
  auto ts = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
  cv::Point tl(ci.x + 10, ci.y - 10);
  cv::rectangle(frame, tl - cv::Point(2,ts.height+2),
    tl + cv::Point(ts.width+2, baseline), COLOR_BLACK, -1);
  cv::putText(frame, label, tl, cv::FONT_HERSHEY_SIMPLEX, 0.5, m.color, 1, cv::LINE_AA);
  int y = ci.y + 20;
  auto infoText = [&](const std::string& s) {
    cv::putText(frame, s, {ci.x + 10, y}, cv::FONT_HERSHEY_SIMPLEX, 0.45,
                COLOR_WHITE, 1, cv::LINE_AA);
    y += 16;
  };
  infoText("center: (" + std::to_string((int)m.center.x) + ","
                       + std::to_string((int)m.center.y) + ")");
  if (m.poseValid) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "pose: (%.2f,%.2f,%.2f)m",
      m.tvec[0], m.tvec[1], m.tvec[2]);
    infoText(buf);
    std::snprintf(buf, sizeof(buf), "dist: %.3f m", m.distance);
    infoText(buf);
  }
}
void FiducialDetector::drawHUD(cv::Mat & frame)
{
  char buf[128];
  std::snprintf(buf, sizeof(buf), "FPS: %.1f", current_fps_);
  cv::putText(frame, buf, {10, 25}, cv::FONT_HERSHEY_SIMPLEX,
              0.7, COLOR_WHITE, 2, cv::LINE_AA);
  std::snprintf(buf, sizeof(buf), "Frame: %llu", (unsigned long long)frame_count_);
  cv::putText(frame, buf, {10, 50}, cv::FONT_HERSHEY_SIMPLEX,
              0.5, cv::Scalar(180,180,180), 1, cv::LINE_AA);
  std::string status = camera_connected_ ? "CAM: OK" : "CAM: RECONNECTING...";
  cv::Scalar sc = camera_connected_ ? COLOR_ALIGNED : COLOR_UNALIGNED;
  cv::putText(frame, status, {10, 75}, cv::FONT_HERSHEY_SIMPLEX, 0.5, sc, 1, cv::LINE_AA);
}
"""
PUBLISH_SECTION = r"""
void FiducialDetector::publishResults(
    const std::vector<DetectedMarker> & markers,
    const cv::Mat & frame,
    const rclcpp::Time & stamp,
    const cv::Size & frame_size)
{
  {
    auto img_msg = cv_bridge::CvImage(
      std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    img_msg->header.stamp = stamp;
    img_msg->header.frame_id = "camera";
    pub_debug_->publish(*img_msg);
  }
  {
    auto msg = std_msgs::msg::String();
    msg.data = buildAlignmentString(markers, frame_size);
    pub_alignment_->publish(msg);
  }
  if (!markers.empty()) {
    const auto & m = markers[0];
    if (m.poseValid) {
      auto msg = geometry_msgs::msg::PoseStamped();
      msg.header.stamp = stamp;
      msg.header.frame_id = "camera";
      msg.pose.position.x = m.tvec[0];
      msg.pose.position.y = m.tvec[1];
      msg.pose.position.z = m.tvec[2];
      cv::Mat rot;
      cv::Rodrigues(m.rvec, rot);
      Eigen::Matrix3d erot;
      for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
          erot(r, c) = rot.at<double>(r, c);
      Eigen::Quaterniond q(erot);
      msg.pose.orientation.x = q.x();
      msg.pose.orientation.y = q.y();
      msg.pose.orientation.z = q.z();
      msg.pose.orientation.w = q.w();
      pub_pose_->publish(msg);
    }
  }
}
}
"""
with open(TARGET, "a") as f:
    f.write(DETECT_SECTION)
    f.write(ALIGN_SECTION)
    f.write(UI_SECTION)
    f.write(PUBLISH_SECTION)
print("ALL SECTIONS APPENDED OK")
