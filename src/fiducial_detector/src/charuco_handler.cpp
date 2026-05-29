#include "fiducial_detector/charuco_handler.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <cmath>
#include <sstream>
namespace fiducial_detector {
CharucoHandler::CharucoHandler(
  int cols, int rows, float square_size, float marker_size,
  cv::Ptr<cv::aruco::Dictionary> dict)
: dict_(dict), cols_(cols), rows_(rows),
  square_size_(square_size), marker_size_(marker_size)
{
  board_ = cv::aruco::CharucoBoard::create(cols, rows, square_size, marker_size, dict);
}
void CharucoHandler::reconfigure(
  int cols, int rows, float sq_size, float mk_size,
  cv::Ptr<cv::aruco::Dictionary> dict)
{
  cols_        = cols;
  rows_        = rows;
  square_size_ = sq_size;
  marker_size_ = mk_size;
  dict_        = dict;
  board_       = cv::aruco::CharucoBoard::create(cols, rows, sq_size, mk_size, dict);
  calib_frames_.clear();
}
CharucoResult CharucoHandler::detect(
  const cv::Mat& gray,
  const cv::Ptr<cv::aruco::DetectorParameters>& params,
  const cv::Mat& K, const cv::Mat& D,
  int min_corners)
{
  CharucoResult result;
  cv::aruco::detectMarkers(gray, dict_,
    result.marker_corners, result.marker_ids, params);
  if (result.marker_ids.empty()) return result;
  result.n_corners = cv::aruco::interpolateCornersCharuco(
    result.marker_corners, result.marker_ids, gray, board_,
    result.charuco_corners, result.charuco_ids,
    K.empty() ? cv::noArray() : K,
    D.empty() ? cv::noArray() : D);
  if (result.n_corners < min_corners) return result;
  if (!K.empty() && result.n_corners >= 4) {
    result.pose = estimatePoseBoard(result, K, D);
    result.pose_valid = result.pose.valid;
  }
  return result;
}
DiamondResult CharucoHandler::detectDiamond(
  const cv::Mat& gray,
  const cv::Ptr<cv::aruco::DetectorParameters>& params,
  float sq_len, float mk_len,
  const cv::Mat& K, const cv::Mat& D)
{
  DiamondResult result;
  std::vector<std::vector<cv::Point2f>> marker_corners;
  std::vector<int> marker_ids;
  cv::aruco::detectMarkers(gray, dict_, marker_corners, marker_ids, params);
  if (marker_ids.empty()) return result;
  cv::aruco::detectCharucoDiamond(
    gray,
    marker_corners,
    marker_ids,
    sq_len / mk_len,
    result.diamond_corners,
    result.diamond_ids,
    K.empty() ? cv::noArray() : K,
    D.empty() ? cv::noArray() : D);
  if (result.diamond_ids.empty()) return result;
  result.valid = true;
  if (!K.empty()) {
    result.poses.resize(result.diamond_ids.size());
    for (std::size_t i = 0; i < result.diamond_ids.size(); ++i) {
      cv::Vec3d rvec, tvec;
      std::vector<cv::Vec3d> rvecs, tvecs;
      cv::aruco::estimatePoseSingleMarkers(
        {result.diamond_corners[i]}, sq_len, K, D, rvecs, tvecs);
      if (!rvecs.empty()) {
        PoseResult pr;
        pr.rvec = rvecs[0];
        pr.tvec = tvecs[0];
        pr.distance = cv::norm(tvecs[0]);
        cv::Mat rot;
        cv::Rodrigues(pr.rvec, rot);
        Eigen::Matrix3d erot;
        for (int r = 0; r < 3; ++r)
          for (int c = 0; c < 3; ++c)
            erot(r, c) = rot.at<double>(r, c);
        pr.quaternion = Eigen::Quaterniond(erot).normalized();
        pr.valid = true;
        result.poses[i] = pr;
      }
    }
  }
  return result;
}
PoseResult CharucoHandler::estimatePoseBoard(
  const CharucoResult& detection,
  const cv::Mat& K, const cv::Mat& D)
{
  PoseResult result;
  if (detection.charuco_corners.size() < 4 || K.empty()) return result;
  cv::Vec3d rvec, tvec;
  bool ok = cv::aruco::estimatePoseCharucoBoard(
    detection.charuco_corners,
    detection.charuco_ids,
    board_,
    K, D,
    rvec, tvec);
  if (!ok) return result;
  result.rvec     = rvec;
  result.tvec     = tvec;
  result.distance = cv::norm(tvec);
  cv::Mat rot;
  cv::Rodrigues(rvec, rot);
  Eigen::Matrix3d erot;
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      erot(r, c) = rot.at<double>(r, c);
  result.quaternion = Eigen::Quaterniond(erot).normalized();
  result.valid      = true;
  return result;
}
bool CharucoHandler::addCalibrationFrame(
  const CharucoResult& detection, cv::Size image_size)
{
  if (detection.charuco_corners.size() < 4) return false;
  CalibrationFrame frame;
  frame.charuco_corners = detection.charuco_corners;
  frame.charuco_ids     = detection.charuco_ids;
  frame.image_size      = image_size;
  calib_frames_.push_back(std::move(frame));
  return true;
}
CalibrationResult CharucoHandler::calibrate(int min_frames)
{
  CalibrationResult result;
  if ((int)calib_frames_.size() < min_frames) return result;
  std::vector<std::vector<cv::Point2f>> all_corners;
  std::vector<std::vector<int>>         all_ids;
  cv::Size image_size = calib_frames_[0].image_size;
  for (const auto& f : calib_frames_) {
    all_corners.push_back(f.charuco_corners);
    all_ids.push_back(f.charuco_ids);
    image_size = f.image_size;
  }
  std::vector<cv::Mat> rvecs, tvecs;
  double reproj = cv::aruco::calibrateCameraCharuco(
    all_corners, all_ids, board_,
    image_size,
    result.camera_matrix, result.dist_coeffs,
    rvecs, tvecs);
  result.reprojection_error = reproj;
  result.n_frames_used      = (int)calib_frames_.size();
  result.valid              = (reproj < 5.0); 
  return result;
}
bool CharucoHandler::saveCalibrationYAML(
  const CalibrationResult& cal, const std::string& path) const
{
  if (!cal.valid) return false;
  cv::FileStorage fs(path, cv::FileStorage::WRITE);
  if (!fs.isOpened()) return false;
  fs << "calibration_date" << "2026";
  fs << "image_width"  << 0;
  fs << "image_height" << 0;
  fs << "camera_matrix" << cal.camera_matrix;
  fs << "distortion_coefficients" << cal.dist_coeffs;
  fs << "reprojection_error" << cal.reprojection_error;
  fs << "n_frames" << cal.n_frames_used;
  fs.release();
  return true;
}
bool CharucoHandler::loadCalibrationYAML(
  const std::string& path, cv::Mat& K, cv::Mat& D) const
{
  cv::FileStorage fs(path, cv::FileStorage::READ);
  if (!fs.isOpened()) return false;
  fs["camera_matrix"] >> K;
  if (K.empty()) { fs["M"] >> K; }
  fs["distortion_coefficients"] >> D;
  if (D.empty()) { fs["D"] >> D; }
  if (D.empty()) { fs["dist_coeffs"] >> D; }
  fs.release();
  return (!K.empty() && !D.empty());
}
void CharucoHandler::drawCorners(
  cv::Mat& frame,
  const CharucoResult& result,
  cv::Scalar color) const
{
  if (result.charuco_corners.empty()) return;
  if (!result.marker_ids.empty()) {
    cv::aruco::drawDetectedMarkers(frame, result.marker_corners, result.marker_ids);
  }
  cv::aruco::drawDetectedCornersCharuco(
    frame, result.charuco_corners, result.charuco_ids, color);
}
void CharucoHandler::drawDiamonds(
  cv::Mat& frame,
  const DiamondResult& result) const
{
  if (result.diamond_corners.empty()) return;
  cv::aruco::drawDetectedDiamonds(frame, result.diamond_corners, result.diamond_ids);
}
cv::Mat CharucoHandler::generateBoardImage(cv::Size board_size, int margin) const
{
  cv::Mat board_img;
  board_->draw(board_size, board_img, margin, 1);
  return board_img;
}
} 
