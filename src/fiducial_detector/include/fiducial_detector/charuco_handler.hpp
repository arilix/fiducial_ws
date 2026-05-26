#pragma once
#include "fiducial_detector/pose_estimator.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <string>
#include <vector>
namespace fiducial_detector {
struct CharucoResult {
  std::vector<cv::Point2f>              charuco_corners;
  std::vector<int>                      charuco_ids;
  std::vector<std::vector<cv::Point2f>> marker_corners;
  std::vector<int>                      marker_ids;
  int       n_corners{0};
  PoseResult pose;
  double    reprojection_error{-1.0};
  bool      pose_valid{false};
};
struct DiamondResult {
  std::vector<std::vector<cv::Point2f>> diamond_corners;
  std::vector<cv::Vec4i>               diamond_ids;
  std::vector<PoseResult>              poses;
  bool valid{false};
};
struct CalibrationFrame {
  std::vector<cv::Point2f> charuco_corners;
  std::vector<int>         charuco_ids;
  cv::Size                 image_size;
};
struct CalibrationResult {
  cv::Mat camera_matrix;
  cv::Mat dist_coeffs;
  double  reprojection_error{0.0};
  int     n_frames_used{0};
  bool    valid{false};
};
class CharucoHandler {
public:
  CharucoHandler(int cols, int rows, float square_size, float marker_size,
                 cv::Ptr<cv::aruco::Dictionary> dict);
  cv::Ptr<cv::aruco::CharucoBoard> board() const { return board_; }
  void reconfigure(int cols, int rows, float sq_size, float mk_size,
                   cv::Ptr<cv::aruco::Dictionary> dict);
  CharucoResult detect(
    const cv::Mat& gray,
    const cv::Ptr<cv::aruco::DetectorParameters>& params,
    const cv::Mat& K = cv::Mat(),
    const cv::Mat& D = cv::Mat(),
    int min_corners = 4);
  DiamondResult detectDiamond(
    const cv::Mat& gray,
    const cv::Ptr<cv::aruco::DetectorParameters>& params,
    float sq_len, float mk_len,
    const cv::Mat& K = cv::Mat(),
    const cv::Mat& D = cv::Mat());
  PoseResult estimatePoseBoard(
    const CharucoResult& detection,
    const cv::Mat& K, const cv::Mat& D);
  bool addCalibrationFrame(const CharucoResult& detection, cv::Size image_size);
  CalibrationResult calibrate(int min_frames = 5);
  bool saveCalibrationYAML(const CalibrationResult& cal, const std::string& path) const;
  bool loadCalibrationYAML(const std::string& path, cv::Mat& K, cv::Mat& D) const;
  void clearCalibrationFrames() { calib_frames_.clear(); }
  int  calibrationFrameCount() const { return (int)calib_frames_.size(); }
  void drawCorners(cv::Mat& frame, const CharucoResult& result,
                   cv::Scalar color = {0, 220, 220}) const;
  void drawDiamonds(cv::Mat& frame, const DiamondResult& result) const;
  cv::Mat generateBoardImage(cv::Size board_size = {800, 600}, int margin = 20) const;
  int   cols()       const { return cols_; }
  int   rows()       const { return rows_; }
  float squareSize() const { return square_size_; }
  float markerSize() const { return marker_size_; }
private:
  cv::Ptr<cv::aruco::CharucoBoard> board_;
  cv::Ptr<cv::aruco::Dictionary>   dict_;
  std::vector<CalibrationFrame>    calib_frames_;
  int   cols_, rows_;
  float square_size_, marker_size_;
};
} 
