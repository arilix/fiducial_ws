#include "fiducial_detector/board_generator.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <fstream>
#include <sstream>
#include <cmath>
#include <vector>
static std::string base64_encode(const std::vector<uchar>& buf) {
  static const char table[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  int val = 0, bits = -6;
  for (uchar c : buf) {
    val = (val << 8) | c;
    bits += 8;
    while (bits >= 0) {
      out.push_back(table[(val >> bits) & 63]);
      bits -= 6;
    }
  }
  if (bits > -6) out.push_back(table[((val << 8) >> (bits + 8)) & 63]);
  while (out.size() % 4) out.push_back('=');
  return out;
}
namespace fiducial_detector {
cv::Mat BoardGenerator::generateGridBoard(
  const BoardGenConfig& cfg,
  cv::Ptr<cv::aruco::Dictionary> dict) const
{
  int pixels_per_m = cfg.dpi * 39; 
  float total_w = cfg.markers_x * cfg.marker_size
                + (cfg.markers_x - 1) * cfg.marker_sep;
  float total_h = cfg.markers_y * cfg.marker_size
                + (cfg.markers_y - 1) * cfg.marker_sep;
  int px_w = (int)(total_w * pixels_per_m) + 2 * cfg.margin_px;
  int px_h = (int)(total_h * pixels_per_m) + 2 * cfg.margin_px;
  px_w = std::max(400, std::min(px_w, 4000));
  px_h = std::max(400, std::min(px_h, 4000));
  auto board = cv::aruco::GridBoard::create(
    cfg.markers_x, cfg.markers_y,
    cfg.marker_size, cfg.marker_sep, dict);
  cv::Mat board_img;
  board->draw({px_w, px_h}, board_img, cfg.margin_px, 1);
  return addAnnotations(board_img, cfg,
    "GridBoard " + std::to_string(cfg.markers_x) + "x"
    + std::to_string(cfg.markers_y));
}
cv::Mat BoardGenerator::generateCharucoBoard(
  const BoardGenConfig& cfg,
  cv::Ptr<cv::aruco::Dictionary> dict) const
{
  int pixels_per_m = cfg.dpi * 39;
  float total_w = cfg.markers_x * cfg.square_size;
  float total_h = cfg.markers_y * cfg.square_size;
  int px_w = (int)(total_w * pixels_per_m) + 2 * cfg.margin_px;
  int px_h = (int)(total_h * pixels_per_m) + 2 * cfg.margin_px;
  px_w = std::max(400, std::min(px_w, 4000));
  px_h = std::max(400, std::min(px_h, 4000));
  float mk_size = cfg.marker_size < cfg.square_size
                ? cfg.marker_size
                : cfg.square_size * 0.7f;
  auto board = cv::aruco::CharucoBoard::create(
    cfg.markers_x, cfg.markers_y,
    cfg.square_size, mk_size, dict);
  cv::Mat board_img;
  board->draw({px_w, px_h}, board_img, cfg.margin_px, 1);
  return addAnnotations(board_img, cfg,
    "ChArUco " + std::to_string(cfg.markers_x) + "x"
    + std::to_string(cfg.markers_y));
}
cv::Mat BoardGenerator::generateDiamondBoard(
  const BoardGenConfig& cfg,
  cv::Ptr<cv::aruco::Dictionary> dict,
  cv::Vec4i diamond_ids) const
{
  int px_size = std::max(400, cfg.dpi * 4); 
  float sq = cfg.square_size;
  float mk = cfg.marker_size < sq ? cfg.marker_size : sq * 0.7f;
  auto board = cv::aruco::CharucoBoard::create(3, 3, sq, mk, dict);
  cv::Mat board_img;
  board->draw({px_size, px_size}, board_img, cfg.margin_px, 1);
  char id_label[64];
  std::snprintf(id_label, sizeof(id_label),
    "Diamond IDs: [%d, %d, %d, %d]",
    diamond_ids[0], diamond_ids[1], diamond_ids[2], diamond_ids[3]);
  return addAnnotations(board_img, cfg, id_label);
}
cv::Mat BoardGenerator::generateMarker(
  int id, cv::Ptr<cv::aruco::Dictionary> dict,
  int px_size, int border_bits) const
{
  cv::Mat mk;
  cv::aruco::drawMarker(dict, id, px_size, mk, border_bits);
  cv::Mat canvas(px_size + 30, px_size, CV_8U, cv::Scalar(255));
  mk.copyTo(canvas(cv::Rect(0, 0, px_size, px_size)));
  char label[32];
  std::snprintf(label, sizeof(label), "ID=%d", id);
  cv::putText(canvas, label, {4, px_size + 20},
    cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0), 1, cv::LINE_AA);
  cv::Mat color;
  cv::cvtColor(canvas, color, cv::COLOR_GRAY2BGR);
  return color;
}
bool BoardGenerator::savePNG(const cv::Mat& image, const std::string& path) const {
  if (image.empty()) return false;
  return cv::imwrite(path, image);
}
bool BoardGenerator::saveSVG(
  const cv::Mat& image, const std::string& path, float width_mm) const
{
  if (image.empty()) return false;
  std::vector<uchar> png_buf;
  std::vector<int> params = {cv::IMWRITE_PNG_COMPRESSION, 9};
  if (!cv::imencode(".png", image, png_buf, params)) return false;
  std::string b64 = base64_encode(png_buf);
  float aspect = (float)image.rows / (float)image.cols;
  float height_mm = width_mm * aspect;
  std::ofstream f(path);
  if (!f.is_open()) return false;
  f << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
  f << "<svg xmlns=\"http://www.w3.org/2000/svg\" "
    << "xmlns:xlink=\"http://www.w3.org/1999/xlink\" "
    << "width=\"" << width_mm << "mm\" "
    << "height=\"" << height_mm << "mm\" "
    << "viewBox=\"0 0 " << image.cols << " " << image.rows << "\">\n";
  f << "  <image width=\"" << image.cols << "\" height=\"" << image.rows << "\" "
    << "xlink:href=\"data:image/png;base64," << b64 << "\"/>\n";
  f << "</svg>\n";
  return true;
}
bool BoardGenerator::savePrintPNG(
  const cv::Mat& image, const std::string& path,
  float width_mm, int dpi) const
{
  if (image.empty()) return false;
  float width_inch = width_mm / 25.4f;
  int target_w = (int)(width_inch * dpi);
  float aspect = (float)image.rows / (float)image.cols;
  int target_h = (int)(target_w * aspect);
  cv::Mat resized;
  cv::resize(image, resized, {target_w, target_h}, 0, 0, cv::INTER_AREA);
  std::vector<int> params = {cv::IMWRITE_PNG_COMPRESSION, 6};
  return cv::imwrite(path, resized, params);
}
cv::Mat BoardGenerator::addAnnotations(
  const cv::Mat& board_img,
  const BoardGenConfig& cfg,
  const std::string& extra_label) const
{
  if (board_img.empty()) return board_img;
  cv::Mat out;
  if (board_img.channels() == 1) {
    cv::cvtColor(board_img, out, cv::COLOR_GRAY2BGR);
  } else {
    out = board_img.clone();
  }
  if (!cfg.title.empty() || !extra_label.empty()) {
    int strip_h = 40;
    cv::Mat strip(strip_h, out.cols, CV_8UC3, cv::Scalar(240, 240, 240));
    std::string text = cfg.title.empty() ? extra_label : cfg.title;
    if (!cfg.title.empty() && !extra_label.empty())
      text = cfg.title + " | " + extra_label;
    char info[80];
    std::snprintf(info, sizeof(info),
      "  marker=%.0fmm  sep=%.0fmm  %s",
      cfg.marker_size * 1000.f, cfg.marker_sep * 1000.f, text.c_str());
    cv::putText(strip, info, {4, 26},
      cv::FONT_HERSHEY_SIMPLEX, 0.5, {30, 30, 30}, 1, cv::LINE_AA);
    cv::Mat combined(out.rows + strip_h, out.cols, CV_8UC3);
    out.copyTo(combined(cv::Rect(0, 0, out.cols, out.rows)));
    strip.copyTo(combined(cv::Rect(0, out.rows, strip.cols, strip_h)));
    return combined;
  }
  return out;
}
} 
