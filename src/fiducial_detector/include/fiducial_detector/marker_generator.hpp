#pragma once
#include <opencv2/aruco.hpp>
#include <string>
#include <vector>
namespace fiducial_detector {

struct MarkerValidationResult {
  int         id{-1};
  bool        pass{false};
  int         hamming{std::numeric_limits<int>::max()};
  std::string reason;
};

class MarkerGenerator {
public:
  // Generate all markers for a dictionary to output_dir as marker_N.png.
  // Returns number of images written, or -1 on error.
  static int generateDictionarySet(
    const std::string& dict_name,
    const std::string& output_dir,
    int marker_size_px = 200,
    int border_bits = 1);

  // Load each marker_N.png from marker_dir, run cv::aruco::detectMarkers,
  // verify detected ID matches file index. Fills results for all N markers.
  static bool validateGeneratedDictionary(
    const std::string& dict_name,
    const std::string& marker_dir,
    std::vector<MarkerValidationResult>& results);

  // Print PASS/FAIL table to stdout.
  static void printValidationReport(
    const std::string& dict_name,
    const std::vector<MarkerValidationResult>& results);
};

} // namespace fiducial_detector
