#include "fiducial_detector/marker_generator.hpp"
#include "fiducial_detector/dictionary_manager.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <sys/stat.h>
namespace fiducial_detector {

static bool ensureDir(const std::string& path) {
  struct stat st{};
  if (stat(path.c_str(), &st) == 0) return S_ISDIR(st.st_mode);
  // Recursively create parent directories
  std::size_t pos = path.rfind('/');
  if (pos != std::string::npos && pos > 0) {
    if (!ensureDir(path.substr(0, pos))) return false;
  }
  return mkdir(path.c_str(), 0755) == 0 || errno == EEXIST;
}

int MarkerGenerator::generateDictionarySet(
  const std::string& dict_name,
  const std::string& output_dir,
  int marker_size_px,
  int border_bits)
{
  cv::Ptr<cv::aruco::Dictionary> dict;
  int total = 0;
  try {
    dict = DictionaryManager::getDictionaryByName(dict_name);
    total = DictionaryManager::getDictMap().count(dict_name)
      ? 0 : 0;  // will use dict->bytesList.rows
    total = dict->bytesList.rows;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "[MarkerGenerator] Unknown dictionary: %s (%s)\n",
      dict_name.c_str(), e.what());
    return -1;
  }

  if (!ensureDir(output_dir)) {
    std::fprintf(stderr, "[MarkerGenerator] Cannot create directory: %s\n", output_dir.c_str());
    return -1;
  }

  int written = 0;
  for (int id = 0; id < total; ++id) {
    cv::Mat img;
    cv::aruco::drawMarker(dict, id, marker_size_px, img, border_bits);

    // Add white padding so the black border is clearly visible on screen/print
    int pad = marker_size_px / 8;
    cv::Mat padded(img.rows + 2 * pad, img.cols + 2 * pad, CV_8UC1,
                   cv::Scalar(255));
    img.copyTo(padded(cv::Rect(pad, pad, img.cols, img.rows)));

    char fname[512];
    std::snprintf(fname, sizeof(fname), "%s/marker_%d.png",
      output_dir.c_str(), id);

    if (cv::imwrite(fname, padded)) {
      ++written;
    } else {
      std::fprintf(stderr, "[MarkerGenerator] Failed to write: %s\n", fname);
    }
  }
  return written;
}

bool MarkerGenerator::validateGeneratedDictionary(
  const std::string& dict_name,
  const std::string& marker_dir,
  std::vector<MarkerValidationResult>& results)
{
  cv::Ptr<cv::aruco::Dictionary> dict;
  int total = 0;
  try {
    dict  = DictionaryManager::getDictionaryByName(dict_name);
    total = dict->bytesList.rows;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "[MarkerGenerator] Unknown dictionary: %s (%s)\n",
      dict_name.c_str(), e.what());
    return false;
  }

  auto dp = cv::aruco::DetectorParameters::create();
  // Lenient parameters so even tiny generated images are detected cleanly
  dp->adaptiveThreshWinSizeMin  = 3;
  dp->adaptiveThreshWinSizeMax  = 23;
  dp->adaptiveThreshWinSizeStep = 10;
  dp->minMarkerPerimeterRate    = 0.02;
  dp->perspectiveRemovePixelPerCell = (dict_name.find("7X7") != std::string::npos) ? 10 : 8;
  dp->perspectiveRemoveIgnoredMarginPerCell = 0.10;
  dp->errorCorrectionRate       = 0.6;
  dp->cornerRefinementMethod    = cv::aruco::CORNER_REFINE_SUBPIX;

  results.clear();
  results.reserve(total);

  bool all_pass = true;
  for (int id = 0; id < total; ++id) {
    char fname[512];
    std::snprintf(fname, sizeof(fname), "%s/marker_%d.png",
      marker_dir.c_str(), id);

    MarkerValidationResult r;
    r.id = id;

    cv::Mat img = cv::imread(fname, cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
      r.pass   = false;
      r.reason = "file_not_found";
      results.push_back(r);
      all_pass = false;
      continue;
    }

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners, rejected;
    cv::aruco::detectMarkers(img, dict, corners, ids, dp, rejected);

    if (ids.empty()) {
      r.pass   = false;
      r.reason = "not_detected";
      all_pass = false;
    } else if (ids[0] != id) {
      r.pass   = false;
      r.hamming = std::abs(ids[0] - id);
      r.reason = "id_mismatch(got=" + std::to_string(ids[0]) + ")";
      all_pass = false;
    } else {
      r.pass    = true;
      r.hamming = 0;
      r.reason  = "ok";
    }
    results.push_back(r);
  }
  return all_pass;
}

void MarkerGenerator::printValidationReport(
  const std::string& dict_name,
  const std::vector<MarkerValidationResult>& results)
{
  const std::string sep(54, '=');
  const std::string dash(54, '-');
  std::printf("\n%s\n", sep.c_str());
  std::printf("  DICTIONARY TEST MODE: %s\n", dict_name.c_str());
  std::printf("%s\n", dash.c_str());

  int pass_count = 0, fail_count = 0;
  for (const auto& r : results) {
    if (r.pass) {
      std::printf("  ID %3d -> PASS  (Hamming=%d)\n", r.id, r.hamming);
      ++pass_count;
    } else {
      std::printf("  ID %3d -> FAIL  [%s]\n", r.id, r.reason.c_str());
      ++fail_count;
    }
  }

  std::printf("%s\n", dash.c_str());
  if (fail_count == 0) {
    std::printf("  RESULT: %d/%d PASS  (0 FAIL) ✓\n",
      pass_count, (int)results.size());
  } else {
    std::printf("  RESULT: %d/%d PASS  (%d FAIL) ✗\n",
      pass_count, (int)results.size(), fail_count);
  }
  std::printf("%s\n\n", sep.c_str());
}

} // namespace fiducial_detector
