#include "fiducial_detector/aruco.hpp"
#include "fiducial_detector/marker_decoder.hpp"
#include "fiducial_detector/charuco_handler.hpp"
#include "fiducial_detector/board_handler.hpp"
#include "fiducial_detector/custom_dictionary.hpp"
#include "fiducial_detector/benchmark_runner.hpp"
#include "fiducial_detector/board_generator.hpp"
#include <opencv2/imgproc.hpp>
#include <tf2/LinearMath/Quaternion.h>
namespace fiducial_detector {
FiducialDetector::FiducialDetector(const rclcpp::NodeOptions& options)
: rclcpp::Node("aruco_node", options),
  fps_monitor_(60)
{
  cv::setUseOptimized(true);
  declareParameters();
  loadRosParams();
  loadIntrinsics();
  initDetectors();
  initPublishers();
  initSubscriber();
  fps_timer_ = create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&FiducialDetector::fpsTimerCallback, this));
  watchdog_timer_ = create_wall_timer(
    std::chrono::seconds(2),
    std::bind(&FiducialDetector::watchdogCallback, this));
  RCLCPP_INFO(get_logger(), "═══════════════════════════════════════");
  RCLCPP_INFO(get_logger(), " FiducialDetector — ROS 2 Humble");
  RCLCPP_INFO(get_logger(), "═══════════════════════════════════════");
  RCLCPP_INFO(get_logger(), "  camera_topic    : %s", camera_topic_.c_str());
  RCLCPP_INFO(get_logger(), "  marker_size     : %.3f m", marker_size_);
  RCLCPP_INFO(get_logger(), "  dictionary      : %s", dictionary_type_.c_str());
  RCLCPP_INFO(get_logger(), "  enable_charuco  : %s", enable_charuco_ ? "true":"false");
  RCLCPP_INFO(get_logger(), "  alignment_tol   : %d px", alignment_tolerance_);
}
FiducialDetector::~FiducialDetector()
{
  if (show_window_) cv::destroyAllWindows();
}
void FiducialDetector::declareParameters()
{
  declare_parameter("marker_size",         0.05);
  declare_parameter("camera_topic",        "/camera/image_raw");
  declare_parameter("dictionary_type",     "DICT_4X4_50");
  declare_parameter("enable_charuco",      true);
  declare_parameter("show_window",         true);
  declare_parameter("show_rejected",       true);
  declare_parameter("alignment_tolerance", 50);
  declare_parameter("smoothing_alpha",     0.4);
  declare_parameter("max_missed_frames",   5);
  declare_parameter("camera_matrix",  std::vector<double>{
    640.0, 0.0, 320.0,  0.0, 640.0, 240.0,  0.0, 0.0, 1.0});
  declare_parameter("dist_coeffs",   std::vector<double>{0,0,0,0,0});
  declare_parameter("detection_mode", "SINGLE");
  declare_parameter("benchmark_on_start", false);
  declare_parameter("enable_gridboard", false);
  declare_parameter("enable_diamond",   false);
  declare_parameter("enable_custom_dict", false);
  declare_parameter("custom_dict_path",   std::string(""));
  declare_parameter("show_corner_labels",     true);
  declare_parameter("show_orientation_arrow", true);
  declare_parameter("show_confidence",        true);
  declare_parameter("charuco_cols", 7);
  declare_parameter("charuco_rows", 5);
  declare_parameter("charuco_sq",   0.035);
  declare_parameter("charuco_mk",   0.0175);
  declare_parameter("gridboard_cols", 5);
  declare_parameter("gridboard_rows", 7);
  declare_parameter("gridboard_marker_size", 0.04);
  declare_parameter("gridboard_sep",         0.01);
  det_params_mgr_ = std::make_unique<DetectorParametersManager>();
  det_params_mgr_->declareAll(this);
}
void FiducialDetector::loadRosParams()
{
  marker_size_           = get_parameter("marker_size").as_double();
  camera_topic_          = get_parameter("camera_topic").as_string();
  dictionary_type_       = get_parameter("dictionary_type").as_string();
  enable_charuco_        = get_parameter("enable_charuco").as_bool();
  show_window_           = get_parameter("show_window").as_bool();
  show_rejected_         = get_parameter("show_rejected").as_bool();
  alignment_tolerance_   = get_parameter("alignment_tolerance").as_int();
  smoothing_alpha_       = get_parameter("smoothing_alpha").as_double();
  max_missed_frames_     = get_parameter("max_missed_frames").as_int();
  enable_gridboard_      = get_parameter("enable_gridboard").as_bool();
  enable_diamond_        = get_parameter("enable_diamond").as_bool();
  enable_custom_dict_    = get_parameter("enable_custom_dict").as_bool();
  custom_dict_path_      = get_parameter("custom_dict_path").as_string();
  show_corner_labels_    = get_parameter("show_corner_labels").as_bool();
  show_orientation_arrow_= get_parameter("show_orientation_arrow").as_bool();
  show_confidence_       = get_parameter("show_confidence").as_bool();
  charuco_cols_          = get_parameter("charuco_cols").as_int();
  charuco_rows_          = get_parameter("charuco_rows").as_int();
  charuco_sq_            = (float)get_parameter("charuco_sq").as_double();
  charuco_mk_            = (float)get_parameter("charuco_mk").as_double();
  gridboard_cols_        = get_parameter("gridboard_cols").as_int();
  gridboard_rows_        = get_parameter("gridboard_rows").as_int();
  gridboard_marker_size_ = (float)get_parameter("gridboard_marker_size").as_double();
  gridboard_sep_         = (float)get_parameter("gridboard_sep").as_double();
  detection_mode_str_ = get_parameter("detection_mode").as_string();
  if      (detection_mode_str_ == "AUTO")      detection_mode_ = DetectionMode::AUTO;
  else if (detection_mode_str_ == "MULTI")     detection_mode_ = DetectionMode::MULTI;
  else if (detection_mode_str_ == "BENCHMARK") detection_mode_ = DetectionMode::BENCHMARK;
  else                                         detection_mode_ = DetectionMode::SINGLE;
  RCLCPP_INFO(get_logger(), "Detection mode: %s", detection_mode_str_.c_str());
}
void FiducialDetector::loadIntrinsics()
{
  auto cam_vec  = get_parameter("camera_matrix").as_double_array();
  auto dist_vec = get_parameter("dist_coeffs").as_double_array();
  if (cam_vec.size() == 9) {
    intrinsics_.K = cv::Mat(3, 3, CV_64F, cam_vec.data()).clone();
  } else {
    intrinsics_.K = (cv::Mat_<double>(3,3) <<
      640, 0, 320,  0, 640, 240,  0, 0, 1);
    RCLCPP_WARN(get_logger(), "camera_matrix invalid — using placeholder. "
      "Run camera calibration and update config/detector.yaml!");
  }
  if (!dist_vec.empty()) {
    intrinsics_.D = cv::Mat(1, (int)dist_vec.size(), CV_64F, dist_vec.data()).clone();
  } else {
    intrinsics_.D = cv::Mat::zeros(1, 5, CV_64F);
  }
  intrinsics_.valid = true;
  RCLCPP_INFO(get_logger(), "Camera intrinsics loaded (valid=%s)",
    intrinsics_.valid ? "true":"false");
}
void FiducialDetector::initDetectors()
{
  dict_manager_ = std::make_unique<DictionaryManager>();
  dict_manager_->setActive(dictionary_type_);
  aruco_dict_ = dict_manager_->activeDict();
  RCLCPP_INFO(get_logger(), "ArUco dictionary: %s", dictionary_type_.c_str());
  det_params_mgr_->bind(this);
  charuco_handler_ = std::make_unique<CharucoHandler>(
    charuco_cols_, charuco_rows_, charuco_sq_, charuco_mk_, aruco_dict_);
  if (enable_gridboard_) {
    GridBoardConfig bcfg;
    bcfg.markers_x   = gridboard_cols_;
    bcfg.markers_y   = gridboard_rows_;
    bcfg.marker_size = gridboard_marker_size_;
    bcfg.marker_sep  = gridboard_sep_;
    board_handler_ = std::make_unique<GridBoardHandler>(bcfg, aruco_dict_);
    RCLCPP_INFO(get_logger(), "GridBoard %dx%d enabled", gridboard_cols_, gridboard_rows_);
  }
  if (enable_custom_dict_ && !custom_dict_path_.empty()) {
    custom_dict_mgr_ = std::make_unique<CustomDictionaryManager>();
    auto cdict = custom_dict_mgr_->loadYAML(custom_dict_path_);
    if (cdict) { aruco_dict_ = cdict; RCLCPP_INFO(get_logger(), "Custom dict loaded"); }
  }
  marker_decoder_   = std::make_unique<MarkerDecoder>(aruco_dict_);
  confidence_calc_  = std::make_unique<ConfidenceCalculator>();
  benchmark_runner_ = std::make_unique<BenchmarkRunner>();
  board_generator_  = std::make_unique<BoardGenerator>();
  pose_estimator_   = std::make_unique<PoseEstimator>(
    intrinsics_.K, intrinsics_.D, marker_size_);
  pose_estimator_->setSmoothingAlpha(smoothing_alpha_);
  visualizer_ = std::make_unique<Visualizer>(alignment_tolerance_);
  RCLCPP_INFO(get_logger(), "All detectors initialized");
}
void FiducialDetector::initPublishers()
{
  pub_pose_      = create_publisher<geometry_msgs::msg::PoseStamped>("/fiducial/pose", 10);
  pub_debug_     = create_publisher<sensor_msgs::msg::Image>("/fiducial/debug_image", 10);
  pub_alignment_ = create_publisher<std_msgs::msg::String>("/fiducial/alignment", 10);
  pub_fps_       = create_publisher<std_msgs::msg::Float32>("/fiducial/fps", 10);
  pub_rejected_  = create_publisher<std_msgs::msg::String>("/fiducial/rejected_candidates", 10);
  RCLCPP_INFO(get_logger(), "Publishers created on /fiducial/{{pose,debug_image,alignment,fps,rejected_candidates}}");
}
void FiducialDetector::initSubscriber()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
  image_sub_ = image_transport::create_subscription(
    this, camera_topic_,
    std::bind(&FiducialDetector::imageCallback, this, std::placeholders::_1),
    "raw", qos.get_rmw_qos_profile());
  cam_connected_ = true;
  RCLCPP_INFO(get_logger(), "Subscribed to '%s'", camera_topic_.c_str());
}
void FiducialDetector::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  cam_connected_ = true;
  last_frame_time_ = msg->header.stamp;
  cv::Mat frame;
  try {
    frame = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
  } catch (const cv_bridge::Exception& e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what());
    return;
  }
  if (frame.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Empty frame received");
    return;
  }
  fps_monitor_.tick();
  ++frame_count_;
  DetectionResult result = runDetection(frame);
  estimatePoses(result);
  cv::Mat annotated = frame.clone();
  bool any_locked = false;
  for (const auto& m : result.markers) {
    std::vector<std::vector<cv::Point2f>> c_wrap{m.corners};
    std::vector<int> id_wrap{m.id};
    visualizer_->drawDetectedMarkers(annotated, c_wrap, id_wrap, m.type);
    if (m.pose.valid) {
      visualizer_->drawPoseAxis(annotated, m.pose,
        intrinsics_.K, intrinsics_.D, (float)marker_size_ * 0.6f);
    }
    if (visualizer_->isAligned(m.center, result.frame_size)) any_locked = true;
  }
  if (show_rejected_) visualizer_->drawRejected(annotated, result.rejected);
  visualizer_->drawUI(annotated, any_locked);
  // visualizer_->drawHUD(annotated,
  //   fps_monitor_.getFps(), fps_monitor_.getLatencyMs(),
  //   frame_count_, cam_connected_.load());
  if (show_window_) enqueueDisplay(annotated);
  publishAll(result, annotated, msg->header.stamp);
}
void FiducialDetector::fpsTimerCallback()
{
  auto msg = std_msgs::msg::Float32();
  msg.data = fps_monitor_.getFps();
  pub_fps_->publish(msg);
  RCLCPP_DEBUG(get_logger(), "FPS: %.1f", msg.data);
}
void FiducialDetector::watchdogCallback()
{
  if (!cam_connected_) return;
  if (frame_count_ == 0) return;
  auto diff = (now() - last_frame_time_).seconds();
  if (diff > 3.0) {
    RCLCPP_WARN(get_logger(),
      "No frame received for %.1f s — camera disconnected", diff);
    cam_connected_ = false;
    reconnectCamera();
  }
}
DetectionResult FiducialDetector::runDetection(const cv::Mat& frame)
{
  DetectionResult result;
  result.frame_size = frame.size();
  cv::Mat gray;
  cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
  switch (detection_mode_) {
    case DetectionMode::AUTO: {
      auto dp = det_params_mgr_->params();
      std::string best = dict_manager_->autoDetect(gray, dp);
      if (best != dict_manager_->activeName()) {
        RCLCPP_INFO(get_logger(), "AUTO: switched to %s", best.c_str());
      }
      aruco_dict_ = dict_manager_->activeDict();
      detectAruco(gray, result);
      break;
    }
    case DetectionMode::MULTI: {
      auto dp = det_params_mgr_->params();
      auto all_results = dict_manager_->detectAll(gray, dp);
      for (const auto& dr : all_results) {
        for (std::size_t i = 0; i < dr.ids.size(); ++i) {
          DetectedMarker m;
          m.id      = dr.ids[i];
          m.type    = MarkerType::ARUCO;
          m.corners = dr.corners[i];
          m.center  = computeCenter(dr.corners[i]);
          result.markers.push_back(std::move(m));
        }
        result.rejected.insert(result.rejected.end(),
          dr.rejected.begin(), dr.rejected.end());
      }
      RCLCPP_DEBUG(get_logger(), "MULTI: %zu total markers from %zu dicts",
        result.markers.size(), all_results.size());
      break;
    }
    case DetectionMode::SINGLE:
    default:
      detectAruco(gray, result);
      break;
  }
  if (enable_charuco_) detectCharuco(gray, result);
  if (enable_gridboard_ && board_handler_) detectBoard(gray, result);
  if (enable_diamond_ && charuco_handler_) detectDiamond(gray, result);
  return result;
}
void FiducialDetector::detectAruco(const cv::Mat& gray, DetectionResult& result)
{
  auto dp = det_params_mgr_->params();
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners, rejected;
  cv::aruco::detectMarkers(gray, aruco_dict_, corners, ids, dp, rejected);
  for (std::size_t i = 0; i < ids.size(); ++i) {
    DetectedMarker m;
    m.id      = ids[i];
    m.type    = MarkerType::ARUCO;
    m.corners = corners[i];
    m.center  = computeCenter(corners[i]);
    result.markers.push_back(std::move(m));
  }
  result.rejected.insert(result.rejected.end(), rejected.begin(), rejected.end());
  RCLCPP_DEBUG(get_logger(), "ArUco [%s]: %zu detected, %zu rejected",
    dict_manager_->activeName().c_str(), ids.size(), rejected.size());
}
void FiducialDetector::detectCharuco(
  const cv::Mat& gray, DetectionResult& result)
{
  if (!charuco_handler_) return;
  auto params = det_params_mgr_->params();
  auto cr = charuco_handler_->detect(gray, params,
    intrinsics_.K, intrinsics_.D);
  result.charuco = cr;
  if (!cr.charuco_corners.empty()) {
    cv::Point2f c(0,0);
    for (auto& p:cr.charuco_corners) c+=p;
    c *= 1.f/(float)cr.charuco_corners.size();
    auto br = cv::boundingRect(cr.charuco_corners);
    DetectedMarker m;
    m.id=0; m.type=MarkerType::CHARUCO;
    m.corners={{(float)br.x,(float)br.y},{(float)(br.x+br.width),(float)br.y},
               {(float)(br.x+br.width),(float)(br.y+br.height)},{(float)br.x,(float)(br.y+br.height)}};
    m.center=c; result.markers.push_back(std::move(m));
  }
}
void FiducialDetector::detectBoard(const cv::Mat& gray, DetectionResult& result)
{
  if (!board_handler_) return;
  auto dp = det_params_mgr_->params();
  result.board = board_handler_->detect(gray, dp, intrinsics_.K, intrinsics_.D);
}
void FiducialDetector::detectDiamond(const cv::Mat& gray, DetectionResult& result)
{
  if (!charuco_handler_) return;
  auto dp = det_params_mgr_->params();
  result.diamonds = charuco_handler_->detectDiamond(
    gray, dp, charuco_sq_, charuco_mk_, intrinsics_.K, intrinsics_.D);
}
void FiducialDetector::estimatePoses(DetectionResult& result)
{
  if (!intrinsics_.valid || !pose_estimator_) return;
  for (auto& m : result.markers) {
    if (m.corners.size() == 4) {
      m.pose = pose_estimator_->estimate(m.id, m.corners);
    }
  }
}
void FiducialDetector::publishAll(
  const DetectionResult& result,
  const cv::Mat& annotated,
  const rclcpp::Time& stamp)
{
  auto img_msg = cv_bridge::CvImage(
    std_msgs::msg::Header(), "bgr8", annotated).toImageMsg();
  img_msg->header.stamp    = stamp;
  img_msg->header.frame_id = "camera";
  pub_debug_->publish(*img_msg);
  {
    auto msg = std_msgs::msg::String();
    if (!result.markers.empty()) {
      msg.data = visualizer_->alignmentString(
        result.markers[0].center, result.frame_size);
    } else {
      msg.data = "NO_MARKER";
    }
    pub_alignment_->publish(msg);
    RCLCPP_INFO(get_logger(), "FPS: %.1f | Alignment: %s", fps_monitor_.getFps(), msg.data.c_str());
  }
  for (const auto& m : result.markers) {
    if (m.pose.valid) {
      auto msg = geometry_msgs::msg::PoseStamped();
      msg.header.stamp    = stamp;
      msg.header.frame_id = "camera";
      msg.pose.position.x = m.pose.tvec[0];
      msg.pose.position.y = m.pose.tvec[1];
      msg.pose.position.z = m.pose.tvec[2];
      msg.pose.orientation.x = m.pose.quaternion.x();
      msg.pose.orientation.y = m.pose.quaternion.y();
      msg.pose.orientation.z = m.pose.quaternion.z();
      msg.pose.orientation.w = m.pose.quaternion.w();
      pub_pose_->publish(msg);
      break;
    }
  }
  {
    auto msg = std_msgs::msg::String();
    msg.data = "{\"rejected_count\":" + std::to_string(result.rejected.size()) +
               ",\"detected_count\":" + std::to_string(result.markers.size()) + "}";
    pub_rejected_->publish(msg);
  }
}
void FiducialDetector::reconnectCamera()
{
  if (reconnect_pending_.exchange(true)) return;
  std::thread([this]() {
    std::this_thread::sleep_for(std::chrono::seconds(2));
    try {
      initSubscriber();
      RCLCPP_INFO(get_logger(), "Camera reconnected");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Reconnect failed: %s", e.what());
    }
    reconnect_pending_ = false;
  }).detach();
}
cv::Point2f FiducialDetector::computeCenter(const std::vector<cv::Point2f>& corners)
{
  cv::Point2f c(0.f, 0.f);
  for (const auto& p : corners) c += p;
  return c * (1.0f / (float)corners.size());
}
void FiducialDetector::enqueueDisplay(const cv::Mat& frame)
{
  { std::lock_guard<std::mutex> lk(display_mutex_); display_frame_ = frame.clone(); display_ready_ = true; }
  display_cv_.notify_one();
}
bool FiducialDetector::displayLoop()
{
  if (!show_window_) { std::this_thread::sleep_for(std::chrono::milliseconds(50)); return rclcpp::ok(); }
  cv::Mat frame;
  {
    std::unique_lock<std::mutex> lk(display_mutex_);
    display_cv_.wait_for(lk, std::chrono::milliseconds(100), [this]{ return display_ready_.load(); });
    if (!display_ready_) return rclcpp::ok();
    frame = display_frame_.clone(); display_ready_ = false;
  }
  cv::imshow("Fiducial Detector", frame);
  if (show_cells_window_ || show_thresh_window_ || show_contour_window_ || show_rejected_window_) {
    std::lock_guard<std::mutex> lk(debug_mutex_);
    if (show_cells_window_  && last_debug_.valid && !last_debug_.cell_grid_image.empty())
      cv::imshow("Marker Cells",last_debug_.cell_grid_image);
    if (show_thresh_window_ && last_debug_.valid && !last_debug_.threshold_image.empty())
      cv::imshow("Threshold",  last_debug_.threshold_image);
    if (show_contour_window_&& last_debug_.valid && !last_debug_.contour_image.empty())
      cv::imshow("Contours",   last_debug_.contour_image);
    if (show_rejected_window_&&last_debug_.valid && !last_debug_.rejected_image.empty())
      cv::imshow("Rejected",   last_debug_.rejected_image);
  }
  int key = cv::waitKey(1);
  if      (key == 27) { rclcpp::shutdown(); return false; }
  else if (key == 'd') { bool v=!show_cells_window_; show_cells_window_=v;show_thresh_window_=v;show_contour_window_=v;show_rejected_window_=v; }
  else if (key == 'c') show_cells_window_   = !show_cells_window_;
  else if (key == 't') show_thresh_window_  = !show_thresh_window_;
  else if (key == 'n') show_contour_window_ = !show_contour_window_;
  else if (key == 'r') show_rejected_window_= !show_rejected_window_;
  return rclcpp::ok();
}
void FiducialDetector::publishDebugImage(
  const cv::Mat& img,
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& pub,
  const rclcpp::Time& stamp)
{
  if (img.empty() || !pub) return;
  cv::Mat out; if (img.channels()==1) cv::cvtColor(img,out,cv::COLOR_GRAY2BGR); else out=img;
  auto msg = cv_bridge::CvImage(std_msgs::msg::Header(),"bgr8",out).toImageMsg();
  msg->header.stamp=stamp; msg->header.frame_id="camera"; pub->publish(*msg);
}
void FiducialDetector::computeConfidence(DetectionResult& result)
{
  if (!confidence_calc_) return;
  for (auto& m : result.markers) {
    if (m.corners.size()!=4) continue;
    m.confidence = confidence_calc_->compute(
      m.corners, m.id, m.pose.rvec, m.pose.tvec,
      intrinsics_.K, intrinsics_.D, 0, 4, 0, marker_size_);
  }
}
}
