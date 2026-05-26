#include "fiducial_detector/detector_parameters.hpp"
namespace fiducial_detector {
DetectorParametersManager::DetectorParametersManager()
: params_(cv::aruco::DetectorParameters::create())
{}
int DetectorParametersManager::dictId(const std::string& name)
{
  auto it = DICT_NAME_MAP.find(name);
  if (it != DICT_NAME_MAP.end()) return it->second;
  RCLCPP_WARN(rclcpp::get_logger("DetectorParametersManager"),
    "Unknown dictionary '%s', falling back to DICT_4X4_50", name.c_str());
  return cv::aruco::DICT_4X4_50;
}
cv::Ptr<cv::aruco::Dictionary> DetectorParametersManager::makeDict(const std::string& name)
{
  return cv::aruco::getPredefinedDictionary(dictId(name));
}
void DetectorParametersManager::declareAll(rclcpp::Node* node)
{
  node->declare_parameter("adaptiveThreshWinSizeMin",    3);
  node->declare_parameter("adaptiveThreshWinSizeMax",   23);
  node->declare_parameter("adaptiveThreshWinSizeStep",   10);
  node->declare_parameter("adaptiveThreshConstant",     7.0);
  node->declare_parameter("minMarkerPerimeterRate",     0.03);
  node->declare_parameter("maxMarkerPerimeterRate",     4.0);
  node->declare_parameter("polygonalApproxAccuracyRate",0.03);
  node->declare_parameter("minCornerDistanceRate",      0.05);
  node->declare_parameter("minDistanceToBorder",        3);
  node->declare_parameter("minMarkerDistanceRate",      0.05);
  node->declare_parameter("perspectiveRemovePixelPerCell",          8);
  node->declare_parameter("perspectiveRemoveIgnoredMarginPerCell", 0.13);
  node->declare_parameter("maxErroneousBitsInBorderRate", 0.35);
  node->declare_parameter("errorCorrectionRate",          0.6);
  node->declare_parameter("detectInvertedMarker",         false);
  node->declare_parameter("cornerRefinementMethod",       1);
  node->declare_parameter("cornerRefinementWinSize",      5);
  node->declare_parameter("cornerRefinementMaxIterations",30);
  node->declare_parameter("cornerRefinementMinAccuracy",  0.1);
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
}
