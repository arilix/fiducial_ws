#include "fiducial_detector/custom_dictionary.hpp"
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <cmath>
namespace fiducial_detector {
cv::Ptr<cv::aruco::Dictionary> CustomDictionaryManager::generate(
  const CustomDictConfig& cfg,
  cv::Ptr<cv::aruco::Dictionary> base_dict)
{
  cv::Ptr<cv::aruco::Dictionary> result;
  if (base_dict) {
    result = cv::aruco::generateCustomDictionary(
      cfg.n_markers,
      cfg.marker_bits_per_side,
      base_dict);
  } else {
    result = cv::aruco::generateCustomDictionary(
      cfg.n_markers,
      cfg.marker_bits_per_side);
  }
  if (result.empty()) {
    throw std::runtime_error("generateCustomDictionary returned empty dict");
  }
  current_dict_ = result;
  current_name_ = cfg.name;
  return result;
}
cv::Ptr<cv::aruco::Dictionary> CustomDictionaryManager::extend(
  cv::Ptr<cv::aruco::Dictionary> existing,
  int n_new, int )
{
  if (!existing || n_new <= 0) return existing;
  int total = existing->bytesList.rows + n_new;
  cv::Ptr<cv::aruco::Dictionary> result =
    cv::aruco::generateCustomDictionary(total, existing->markerSize, existing);
  current_dict_ = result;
  return result;
}
bool CustomDictionaryManager::saveYAML(
  cv::Ptr<cv::aruco::Dictionary> dict,
  const std::string& path,
  const std::string& name) const
{
  if (!dict) return false;
  cv::FileStorage fs(path, cv::FileStorage::WRITE);
  if (!fs.isOpened()) return false;
  fs << "name"       << name;
  fs << "nMarkers"   << dict->bytesList.rows;
  fs << "markerSize" << dict->markerSize;
  fs << "maxCorrectionBits" << dict->maxCorrectionBits;
  fs << "bytesList"  << dict->bytesList;
  fs.release();
  return true;
}
cv::Ptr<cv::aruco::Dictionary> CustomDictionaryManager::loadYAML(
  const std::string& path)
{
  cv::FileStorage fs(path, cv::FileStorage::READ);
  if (!fs.isOpened()) return nullptr;
  auto dict = cv::makePtr<cv::aruco::Dictionary>();
  cv::FileNode n_markers_node = fs["nMarkers"];
  cv::FileNode marker_size_node = fs["markerSize"];
  cv::FileNode bytes_node = fs["bytesList"];
  cv::FileNode max_corr_node = fs["maxCorrectionBits"];
  if (bytes_node.empty() || marker_size_node.empty()) {
    fs.release();
    return cv::aruco::getPredefinedDictionary(0); 
  }
  dict->markerSize = (int)marker_size_node;
  if (!max_corr_node.empty())
    dict->maxCorrectionBits = (int)max_corr_node;
  bytes_node >> dict->bytesList;
  fs.release();
  if (dict->bytesList.empty()) return nullptr;
  current_dict_ = dict;
  return dict;
}
cv::Mat CustomDictionaryManager::visualizeBytesList(
  cv::Ptr<cv::aruco::Dictionary> dict,
  int marker_size, int cols) const
{
  if (!dict || dict->bytesList.rows == 0) {
    return cv::Mat::zeros(100, 100, CV_8UC3);
  }
  int n_markers = dict->bytesList.rows;
  int n_bits    = dict->markerSize;
  int rows_grid = (n_markers + cols - 1) / cols;
  int pad   = 4;
  int cell  = marker_size + pad;
  int W     = cols * cell + pad;
  int H     = rows_grid * cell + pad + 20;
  cv::Mat canvas(H, W, CV_8UC3, cv::Scalar(30, 30, 30));
  for (int idx = 0; idx < n_markers; ++idx) {
    int gr = idx / cols;
    int gc = idx % cols;
    int x0 = gc * cell + pad;
    int y0 = gr * cell + pad;
    cv::Mat mk_img;
    cv::aruco::drawMarker(dict, idx, marker_size, mk_img, 1);
    if (mk_img.channels() == 1) {
      cv::cvtColor(mk_img, mk_img, cv::COLOR_GRAY2BGR);
    }
    if (x0 + marker_size <= W && y0 + marker_size <= H - 20) {
      mk_img.copyTo(canvas(cv::Rect(x0, y0, marker_size, marker_size)));
    }
  }
  char title[64];
  std::snprintf(title, sizeof(title),
    "Custom Dict: %d markers, %dx%d bits", n_markers, n_bits, n_bits);
  cv::putText(canvas, title, {pad, H - 6},
    cv::FONT_HERSHEY_SIMPLEX, 0.45, {200, 200, 200}, 1, cv::LINE_AA);
  return canvas;
}
} 
