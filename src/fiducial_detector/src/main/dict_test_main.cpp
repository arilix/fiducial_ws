#include <opencv2/opencv.hpp>
#include "utils/aruco.h"
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

static const int DICT_ID    = cv::aruco::DICT_7X7_50;
static const int NUM_MARKERS = 50;
static const int IMG_SIZE    = 400;
static const int BORDER_BITS = 1;

static cv::Mat generateMarkerImage(
    cv::Ptr<cv::aruco::Dictionary> dict, int id)
{
    cv::Mat img;
    cv::aruco::drawMarker(dict, id, IMG_SIZE - 40, img, BORDER_BITS);
    // Add white border for clean detection
    cv::Mat bordered;
    cv::copyMakeBorder(img, bordered, 20, 20, 20, 20,
                       cv::BORDER_CONSTANT, cv::Scalar(255));
    return bordered;
}

static cv::Ptr<cv::aruco::DetectorParameters> make7x7Params() {
    auto p = cv::aruco::DetectorParameters::create();
    // Perspective normalization optimized for 7×7 grid resolution
    p->perspectiveRemovePixelPerCell          = 10;
    p->perspectiveRemoveIgnoredMarginPerCell  = 0.10;
    p->markerBorderBits                       = BORDER_BITS;
    // Adaptive threshold range for small/distant markers
    p->adaptiveThreshWinSizeMin               = 3;
    p->adaptiveThreshWinSizeMax               = 33;
    p->adaptiveThreshWinSizeStep              = 10;
    p->adaptiveThreshConstant                 = 7.0;
    p->minMarkerPerimeterRate                 = 0.015;
    p->maxMarkerPerimeterRate                 = 4.0;
    p->polygonalApproxAccuracyRate            = 0.03;
    p->minCornerDistanceRate                  = 0.05;
    p->minDistanceToBorder                    = 3;
    p->maxErroneousBitsInBorderRate           = 0.35;
    p->errorCorrectionRate                    = 0.6;
    p->detectInvertedMarker                   = true;
    // Subpixel refinement for stable corner localization
    p->cornerRefinementMethod                 = cv::aruco::CORNER_REFINE_SUBPIX;
    p->cornerRefinementWinSize                = 5;
    p->cornerRefinementMaxIterations          = 50;
    p->cornerRefinementMinAccuracy            = 0.01;
    return p;
}

int main(int /*argc*/, char** /*argv*/) {
    const std::string sep(54, '=');
    const std::string dash(54, '-');

    std::printf("\n%s\n", sep.c_str());
    std::printf("  DICT_7X7_50 Validation — ID 0 to %d\n", NUM_MARKERS - 1);
    std::printf("%s\n", dash.c_str());

    auto dict   = cv::aruco::getPredefinedDictionary(DICT_ID);
    auto params = make7x7Params();

    // Validate dictionary metadata before running per-ID tests
    if (!dict || dict->bytesList.empty()) {
        std::printf("  FAILED: dictionary not loaded\n%s\n\n", sep.c_str());
        return 1;
    }
    if (dict->bytesList.rows != NUM_MARKERS) {
        std::printf("  FAILED: expected %d markers, got %d\n%s\n\n",
            NUM_MARKERS, dict->bytesList.rows, sep.c_str());
        return 1;
    }
    std::printf("  Dictionary: DICT_7X7_50 | Markers: %d | BorderBits: %d\n",
        dict->bytesList.rows, BORDER_BITS);
    std::printf("%s\n", dash.c_str());

    int pass_count = 0;
    std::vector<int> failed_ids;

    for (int id = 0; id < NUM_MARKERS; ++id) {
        cv::Mat marker_img = generateMarkerImage(dict, id);
        // drawMarker returns a grayscale (1-channel) image — no conversion needed
        const cv::Mat& gray = marker_img;

        std::vector<int> detected_ids;
        std::vector<std::vector<cv::Point2f>> corners, rejected;
        cv::aruco::detectMarkers(gray, dict, corners, detected_ids, params, rejected);

        bool pass = false;
        if (detected_ids.size() == 1 && detected_ids[0] == id) {
            pass = true;
            ++pass_count;
        } else {
            failed_ids.push_back(id);
        }

        std::printf("  ID %2d → %s\n", id, pass ? "PASS" : "FAIL");
    }

    std::printf("%s\n", dash.c_str());
    if (pass_count == NUM_MARKERS) {
        std::printf("  Result: %d/%d PASS  ✓\n", pass_count, NUM_MARKERS);
        std::printf("%s\n\n", sep.c_str());
        return 0;
    } else {
        std::printf("  Result: %d/%d PASS — FAILED IDs:", pass_count, NUM_MARKERS);
        for (int fid : failed_ids) std::printf(" %d", fid);
        std::printf("\n%s\n\n", sep.c_str());
        return 1;
    }
}
