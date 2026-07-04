#include "utils/detector_parameters.h"

namespace fiducial_detector {

DetectorParametersManager::DetectorParametersManager()
    : params_(cv::aruco::DetectorParameters::create())
{}

void DetectorParametersManager::declareAll(rclcpp::Node* node) {
    node->declare_parameter("adaptiveThreshWinSizeMin",             3);
    node->declare_parameter("adaptiveThreshWinSizeMax",            33);
    node->declare_parameter("adaptiveThreshWinSizeStep",           10);
    node->declare_parameter("adaptiveThreshConstant",             7.0);
    node->declare_parameter("minMarkerPerimeterRate",            0.015);
    node->declare_parameter("maxMarkerPerimeterRate",              4.0);
    node->declare_parameter("polygonalApproxAccuracyRate",        0.03);
    node->declare_parameter("minCornerDistanceRate",              0.05);
    node->declare_parameter("minDistanceToBorder",                   3);
    node->declare_parameter("minMarkerDistanceRate",              0.05);
    node->declare_parameter("markerBorderBits",                      1);
    node->declare_parameter("perspectiveRemovePixelPerCell",         10);
    node->declare_parameter("perspectiveRemoveIgnoredMarginPerCell", 0.10);
    node->declare_parameter("maxErroneousBitsInBorderRate",        0.35);
    node->declare_parameter("errorCorrectionRate",                  0.6);
    node->declare_parameter("detectInvertedMarker",                true);
    node->declare_parameter("cornerRefinementMethod",                1);  // SUBPIX
    node->declare_parameter("cornerRefinementWinSize",               5);
    node->declare_parameter("cornerRefinementMaxIterations",        50);
    node->declare_parameter("cornerRefinementMinAccuracy",         0.01);
}

void DetectorParametersManager::bind(rclcpp::Node* node) {
    params_->adaptiveThreshWinSizeMin    = node->get_parameter("adaptiveThreshWinSizeMin").as_int();
    params_->adaptiveThreshWinSizeMax    = node->get_parameter("adaptiveThreshWinSizeMax").as_int();
    params_->adaptiveThreshWinSizeStep   = node->get_parameter("adaptiveThreshWinSizeStep").as_int();
    params_->adaptiveThreshConstant      = node->get_parameter("adaptiveThreshConstant").as_double();
    params_->minMarkerPerimeterRate      = node->get_parameter("minMarkerPerimeterRate").as_double();
    params_->maxMarkerPerimeterRate      = node->get_parameter("maxMarkerPerimeterRate").as_double();
    params_->polygonalApproxAccuracyRate = node->get_parameter("polygonalApproxAccuracyRate").as_double();
    params_->minCornerDistanceRate       = node->get_parameter("minCornerDistanceRate").as_double();
    params_->minDistanceToBorder         = node->get_parameter("minDistanceToBorder").as_int();
    params_->minMarkerDistanceRate       = node->get_parameter("minMarkerDistanceRate").as_double();
    params_->markerBorderBits            = node->get_parameter("markerBorderBits").as_int();
    params_->perspectiveRemovePixelPerCell =
        node->get_parameter("perspectiveRemovePixelPerCell").as_int();
    params_->perspectiveRemoveIgnoredMarginPerCell =
        node->get_parameter("perspectiveRemoveIgnoredMarginPerCell").as_double();
    params_->maxErroneousBitsInBorderRate =
        node->get_parameter("maxErroneousBitsInBorderRate").as_double();
    params_->errorCorrectionRate =
        node->get_parameter("errorCorrectionRate").as_double();
    params_->detectInvertedMarker =
        node->get_parameter("detectInvertedMarker").as_bool();
    int cr = node->get_parameter("cornerRefinementMethod").as_int();
    params_->cornerRefinementMethod =
        static_cast<cv::aruco::CornerRefineMethod>(cr);
    params_->cornerRefinementWinSize =
        node->get_parameter("cornerRefinementWinSize").as_int();
    params_->cornerRefinementMaxIterations =
        node->get_parameter("cornerRefinementMaxIterations").as_int();
    params_->cornerRefinementMinAccuracy =
        node->get_parameter("cornerRefinementMinAccuracy").as_double();
}

void DetectorParametersManager::apply7x7Profile() {
    // Perspective normalization: 10 px/cell and 10% margin for 7×7 grid resolution
    params_->perspectiveRemovePixelPerCell          = 10;
    params_->perspectiveRemoveIgnoredMarginPerCell  = 0.10;
    params_->markerBorderBits                       = 1;

    // Wide adaptive threshold window for small/distant markers
    params_->adaptiveThreshWinSizeMin               = 3;
    params_->adaptiveThreshWinSizeMax               = 33;
    params_->adaptiveThreshWinSizeStep              = 10;
    params_->adaptiveThreshConstant                 = 7.0;

    // Permissive size filter enables detection of small markers
    params_->minMarkerPerimeterRate                 = 0.015;
    params_->maxMarkerPerimeterRate                 = 4.0;
    params_->polygonalApproxAccuracyRate            = 0.03;
    params_->minCornerDistanceRate                  = 0.05;
    params_->minDistanceToBorder                    = 3;
    params_->maxErroneousBitsInBorderRate           = 0.35;
    params_->errorCorrectionRate                    = 0.6;
    params_->detectInvertedMarker                   = true;

    // Subpixel corner refinement for stable multi-pose estimation
    params_->cornerRefinementMethod                 = cv::aruco::CORNER_REFINE_SUBPIX;
    params_->cornerRefinementWinSize                = 5;
    params_->cornerRefinementMaxIterations          = 50;
    params_->cornerRefinementMinAccuracy            = 0.01;
}

} // namespace fiducial_detector
