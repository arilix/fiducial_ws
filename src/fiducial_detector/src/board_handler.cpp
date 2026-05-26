#include "fiducial_detector/board_handler.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <cmath>
#include <sstream>
namespace fiducial_detector {
GridBoardHandler::GridBoardHandler(
  const GridBoardConfig& cfg,
  cv::Ptr<cv::aruco::Dictionary> dict)
: dict_(dict), cfg_(cfg)
{
  board_ = cv::aruco::GridBoard::create(
    cfg.markers_x, cfg.markers_y,
    cfg.marker_size, cfg.marker_sep, dict);
}
void GridBoardHandler::reconfigure(
  const GridBoardConfig& cfg,
  cv::Ptr<cv::aruco::Dictionary> dict)
{
  cfg_   = cfg;
  dict_  = dict;
  board_ = cv::aruco::GridBoard::create(
    cfg.markers_x, cfg.markers_y,
    cfg.marker_size, cfg.marker_sep, dict);
}
GridBoardResult GridBoardHandler::detect(
  const cv::Mat& gray,
  const cv::Ptr<cv::aruco::DetectorParameters>& params,
  const cv::Mat& K, const cv::Mat& D)
{
  GridBoardResult result;
  cv::aruco::detectMarkers(gray, dict_, result.corners, result.ids, params);
  if (result.ids.empty()) return result;
  if (!K.empty()) {
    std::vector<std::vector<cv::Point2f>> rejected;
    cv::aruco::refineDetectedMarkers(
      gray, board_, result.corners, result.ids, rejected, K, D, 10.f, 3.f, true,
      cv::noArray(), params);
  }
  result.n_markers_used = (int)result.ids.size();
  if (!K.empty() && result.n_markers_used >= 1) {
    cv::Vec3d rvec, tvec;
    int n_used = cv::aruco::estimatePoseBoard(
      result.corners, result.ids, board_, K, D, rvec, tvec);
    if (n_used > 0) {
      result.pose.rvec     = rvec;
      result.pose.tvec     = tvec;
      result.pose.distance = cv::norm(tvec);
      cv::Mat rot;
      cv::Rodrigues(rvec, rot);
      Eigen::Matrix3d erot;
      for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
          erot(r, c) = rot.at<double>(r, c);
      result.pose.quaternion = Eigen::Quaterniond(erot).normalized();
      result.pose.valid      = true;
      result.valid           = true;
      result.reprojection_error = calcReprojectionError(result, K, D);
    }
  } else if (result.n_markers_used >= 1) {
    result.valid = true;
  }
  return result;
}
double GridBoardHandler::calcReprojectionError(
  const GridBoardResult& result,
  const cv::Mat& K, const cv::Mat& D) const
{
  if (!result.pose.valid || K.empty()) return -1.0;
  double total_err = 0.0;
  int    total_pts = 0;
  for (std::size_t i = 0; i < result.corners.size(); ++i) {
    float h = cfg_.marker_size * 0.5f;
    std::vector<cv::Point3f> obj = {{-h,h,0},{h,h,0},{h,-h,0},{-h,-h,0}};
    std::vector<cv::Point2f> prj;
    cv::projectPoints(obj, result.pose.rvec, result.pose.tvec, K, D, prj);
    for (std::size_t j = 0; j < 4 && j < result.corners[i].size(); ++j) {
      double dx = prj[j].x - result.corners[i][j].x;
      double dy = prj[j].y - result.corners[i][j].y;
      total_err += std::sqrt(dx*dx + dy*dy);
      ++total_pts;
    }
  }
  return (total_pts > 0) ? total_err / total_pts : -1.0;
}
void GridBoardHandler::draw(
  cv::Mat& frame,
  const GridBoardResult& result,
  const cv::Mat& K, const cv::Mat& D,
  float axis_len) const
{
  if (result.ids.empty()) return;
  cv::aruco::drawDetectedMarkers(frame, result.corners, result.ids,
                                 {0, 220, 160});
  if (result.pose.valid && !K.empty()) {
    cv::aruco::drawAxis(frame, K, D,
      result.pose.rvec, result.pose.tvec, axis_len);
  }
  char buf[128];
  std::snprintf(buf, sizeof(buf),
    "GridBoard: %d/%d markers  Reproj=%.2fpx",
    result.n_markers_used,
    cfg_.markers_x * cfg_.markers_y,
    result.reprojection_error);
  cv::putText(frame, buf, {10, frame.rows - 15},
    cv::FONT_HERSHEY_SIMPLEX, 0.55, {0, 220, 160}, 2, cv::LINE_AA);
}
cv::Mat GridBoardHandler::generateImage(cv::Size img_size, int margin) const
{
  cv::Mat board_img;
  board_->draw(img_size, board_img, margin, 1);
  return board_img;
}
} 
