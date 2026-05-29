#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <string>
#include <vector>
namespace fiducial_detector {
enum class BoardType { GRID, CHARUCO, DIAMOND };
struct BoardGenConfig {
  BoardType type       {BoardType::GRID};
  int  markers_x       {5};
  int  markers_y       {7};
  float marker_size    {0.04f};   
  float marker_sep     {0.01f};   
  float square_size    {0.05f};   
  int  dpi             {150};     
  int  margin_px       {20};      
  bool add_border      {true};    
  bool add_id_labels   {false};   
  std::string title    {};        
};
class BoardGenerator {
public:
  BoardGenerator() = default;
  cv::Mat generateGridBoard(
    const BoardGenConfig& cfg,
    cv::Ptr<cv::aruco::Dictionary> dict) const;
  cv::Mat generateCharucoBoard(
    const BoardGenConfig& cfg,
    cv::Ptr<cv::aruco::Dictionary> dict) const;
  cv::Mat generateDiamondBoard(
    const BoardGenConfig& cfg,
    cv::Ptr<cv::aruco::Dictionary> dict,
    cv::Vec4i diamond_ids = {0, 1, 2, 3}) const;
  cv::Mat generateMarker(
    int id,
    cv::Ptr<cv::aruco::Dictionary> dict,
    int px_size    = 200,
    int border_bits = 1) const;
  bool savePNG(const cv::Mat& image, const std::string& path) const;
  bool saveSVG(const cv::Mat& image, const std::string& path,
               float width_mm = 210.0f) const;
  bool savePrintPNG(const cv::Mat& image, const std::string& path,
                    float width_mm = 210.0f, int dpi = 300) const;
private:
  cv::Mat addAnnotations(const cv::Mat& board_img,
                         const BoardGenConfig& cfg,
                         const std::string& extra_label = "") const;
};
} 
