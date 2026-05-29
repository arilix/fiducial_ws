#pragma once
#include "fiducial_detector/pose_estimator.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <vector>
#include <string>
#include <memory>
namespace fiducial_detector {
struct GridBoardConfig {
  int   markers_x{5};
  int   markers_y{7};
  float marker_size{0.04f};   
  float marker_sep{0.01f};    
};
struct GridBoardResult {
  std::vector<std::vector<cv::Point2f>> corners;
  std::vector<int>                      ids;
  PoseResult                            pose;
  double                                reprojection_error{-1.0};
  int                                   n_markers_used{0};
  bool                                  valid{false};
};
class GridBoardHandler {
public:
  explicit GridBoardHandler(
    const GridBoardConfig& cfg,
    cv::Ptr<cv::aruco::Dictionary> dict);
  void reconfigure(const GridBoardConfig& cfg,
                   cv::Ptr<cv::aruco::Dictionary> dict);
  cv::Ptr<cv::aruco::GridBoard> board() const { return board_; }
  const GridBoardConfig& config() const { return cfg_; }
  GridBoardResult detect(
    const cv::Mat& gray,
    const cv::Ptr<cv::aruco::DetectorParameters>& params,
    const cv::Mat& K = cv::Mat(),
    const cv::Mat& D = cv::Mat());
  double calcReprojectionError(
    const GridBoardResult& result,
    const cv::Mat& K, const cv::Mat& D) const;
  void draw(cv::Mat& frame,
            const GridBoardResult& result,
            const cv::Mat& K = cv::Mat(),
            const cv::Mat& D = cv::Mat(),
            float axis_len   = 0.05f) const;
  cv::Mat generateImage(cv::Size img_size = {800, 1000},
                        int margin = 10) const;
private:
  cv::Ptr<cv::aruco::GridBoard>  board_;
  cv::Ptr<cv::aruco::Dictionary> dict_;
  GridBoardConfig                cfg_;
};
} 
