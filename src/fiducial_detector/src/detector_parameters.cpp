#include "fiducial_detector/detector_parameters.hpp"
#include "fiducial_detector/dictionary_manager.hpp"
namespace fiducial_detector {
DetectorParametersManager::DetectorParametersManager()
: params_(cv::aruco::DetectorParameters::create())
{}
int DetectorParametersManager::dictId(const std::string& name)
{
  try {
    return DictionaryManager::getDictIdByName(name);
  } catch (const std::invalid_argument&) {
    RCLCPP_WARN(rclcpp::get_logger("DetectorParametersManager"),
      "Unknown dictionary '%s', falling back to DICT_4X4_50", name.c_str());
    return cv::aruco::DICT_4X4_50;
  }
}
cv::Ptr<cv::aruco::Dictionary> DetectorParametersManager::makeDict(const std::string& name)
{
  try {
    return DictionaryManager::getDictionaryByName(name);
  } catch (const std::invalid_argument&) {
    return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  }
}
bool DetectorParametersManager::isAprilTagDict(const std::string& name) {
  return name.find("APRILTAG") != std::string::npos;
}
bool DetectorParametersManager::isMIPDict(const std::string& name) {
  return name.find("MIP") != std::string::npos;
}
bool DetectorParametersManager::is7x7Dict(const std::string& name) {
  return name.find("7X7") != std::string::npos;
}
int DetectorParametersManager::borderBitsForDict(const std::string& name) {
  if (isAprilTagDict(name)) return 2;
  return 1;
}
void DetectorParametersManager::declareAll(rclcpp::Node* node)
{
  node->declare_parameter("adaptiveThreshWinSizeMin",    3);
  node->declare_parameter("adaptiveThreshWinSizeMax",   23);
  node->declare_parameter("adaptiveThreshWinSizeStep",   10);
  node->declare_parameter("adaptiveThreshConstant",     7.0);
  node->declare_parameter("minMarkerPerimeterRate",     0.02);
  node->declare_parameter("maxMarkerPerimeterRate",     4.0);
  node->declare_parameter("polygonalApproxAccuracyRate",0.03);
  node->declare_parameter("minCornerDistanceRate",      0.05);
  node->declare_parameter("minDistanceToBorder",        3);
  node->declare_parameter("minMarkerDistanceRate",      0.05);
  node->declare_parameter("markerBorderBits",           1);
  node->declare_parameter("perspectiveRemovePixelPerCell",          8);
  node->declare_parameter("perspectiveRemoveIgnoredMarginPerCell", 0.13);
  node->declare_parameter("maxErroneousBitsInBorderRate", 0.35);
  node->declare_parameter("errorCorrectionRate",          0.6);
  node->declare_parameter("detectInvertedMarker",         true);
  node->declare_parameter("cornerRefinementMethod",       1);
  node->declare_parameter("cornerRefinementWinSize",      5);
  node->declare_parameter("cornerRefinementMaxIterations",50);
  node->declare_parameter("cornerRefinementMinAccuracy",  0.01);
}
void DetectorParametersManager::bind(rclcpp::Node* node)
{
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
  params_->errorCorrectionRate          =
    node->get_parameter("errorCorrectionRate").as_double();
  params_->detectInvertedMarker         =
    node->get_parameter("detectInvertedMarker").as_bool();
  int cr_method = node->get_parameter("cornerRefinementMethod").as_int();
  params_->cornerRefinementMethod = static_cast<cv::aruco::CornerRefineMethod>(cr_method);
  params_->cornerRefinementWinSize =
    node->get_parameter("cornerRefinementWinSize").as_int();
  params_->cornerRefinementMaxIterations =
    node->get_parameter("cornerRefinementMaxIterations").as_int();
  params_->cornerRefinementMinAccuracy =
    node->get_parameter("cornerRefinementMinAccuracy").as_double();
}
void DetectorParametersManager::applyDictionaryProfile(const std::string& dict_name)
{
  auto logger = rclcpp::get_logger("DetectorParametersManager");

  if (isAprilTagDict(dict_name)) {
    params_->markerBorderBits                      = 2;
    params_->cornerRefinementMethod                = cv::aruco::CORNER_REFINE_APRILTAG;
    params_->adaptiveThreshConstant                = 7.0;
    params_->minCornerDistanceRate                 = 0.02;
    params_->minDistanceToBorder                   = 3;
    params_->minMarkerPerimeterRate                = 0.02;
    params_->maxMarkerPerimeterRate                = 4.0;
    params_->polygonalApproxAccuracyRate           = 0.03;
    params_->perspectiveRemovePixelPerCell         = 8;
    params_->perspectiveRemoveIgnoredMarginPerCell = 0.13;
    params_->maxErroneousBitsInBorderRate          = 0.5;
    params_->errorCorrectionRate                   = 0.6;
    params_->detectInvertedMarker                  = true;
    params_->cornerRefinementWinSize               = 5;
    params_->cornerRefinementMaxIterations         = 50;
    params_->cornerRefinementMinAccuracy           = 0.01;
    RCLCPP_INFO(logger, "Detector profile: AprilTag | borderBits=2 cornerRefine=APRILTAG");
  } else if (isMIPDict(dict_name)) {
    params_->markerBorderBits                      = 1;
    params_->cornerRefinementMethod                = cv::aruco::CORNER_REFINE_SUBPIX;
    params_->minMarkerPerimeterRate                = 0.02;
    params_->maxMarkerPerimeterRate                = 4.0;
    params_->maxErroneousBitsInBorderRate          = 0.5;
    params_->errorCorrectionRate                   = 0.8;
    params_->detectInvertedMarker                  = true;
    params_->cornerRefinementMaxIterations         = 50;
    params_->cornerRefinementMinAccuracy           = 0.01;
    RCLCPP_INFO(logger, "Detector profile: MIP | borderBits=1 errorRate=0.8");
  } else if (is7x7Dict(dict_name)) {
    // 7×7 grid needs higher resolution per cell and wider adaptive threshold
    // windows to reliably read the denser bit pattern, especially at range.
    params_->markerBorderBits                      = 1;
    params_->perspectiveRemovePixelPerCell         = 10;
    params_->perspectiveRemoveIgnoredMarginPerCell = 0.10;
    params_->adaptiveThreshWinSizeMin              = 3;
    params_->adaptiveThreshWinSizeMax              = 33;
    params_->adaptiveThreshWinSizeStep             = 10;
    params_->adaptiveThreshConstant                = 7.0;
    params_->minMarkerPerimeterRate                = 0.015;
    params_->maxMarkerPerimeterRate                = 4.0;
    params_->polygonalApproxAccuracyRate           = 0.03;
    params_->minCornerDistanceRate                 = 0.05;
    params_->minDistanceToBorder                   = 3;
    params_->maxErroneousBitsInBorderRate          = 0.35;
    params_->errorCorrectionRate                   = 0.6;
    params_->detectInvertedMarker                  = true;
    // CORNER_REFINE_CONTOUR is robust for 7x7 and avoids the SUBPIX
    // stack-overflow / memory-corruption seen in OpenCV 4.x with tight
    // minAccuracy + high maxIterations on dense bit patterns.
    params_->cornerRefinementMethod                = cv::aruco::CORNER_REFINE_CONTOUR;
    params_->cornerRefinementWinSize               = 5;
    params_->cornerRefinementMaxIterations         = 30;
    params_->cornerRefinementMinAccuracy           = 0.1;
    RCLCPP_INFO(logger,
      "Detector profile: 7×7 | pixPerCell=10 margin=0.10 threshMax=33 cornerRefine=CONTOUR");
  } else {
    RCLCPP_DEBUG(logger, "Detector profile: standard ArUco | dict=%s", dict_name.c_str());
  }
}
}
