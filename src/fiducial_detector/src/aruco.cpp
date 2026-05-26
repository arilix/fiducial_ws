#include "fiducial_detector/aruco.hpp"
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
  RCLCPP_INFO(get_logger(), "FiducialDetector started");
}
FiducialDetector::~FiducialDetector()
{
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
  det_params_mgr_ = std::make_unique<DetectorParametersManager>();
  det_params_mgr_->declareAll(this);
}
void FiducialDetector::loadRosParams()
{
  marker_size_         = get_parameter("marker_size").as_double();
  camera_topic_        = get_parameter("camera_topic").as_string();
  dictionary_type_     = get_parameter("dictionary_type").as_string();
  enable_charuco_      = get_parameter("enable_charuco").as_bool();
  show_window_         = get_parameter("show_window").as_bool();
  show_rejected_       = get_parameter("show_rejected").as_bool();
  alignment_tolerance_ = get_parameter("alignment_tolerance").as_int();
  smoothing_alpha_     = get_parameter("smoothing_alpha").as_double();
  max_missed_frames_   = get_parameter("max_missed_frames").as_int();
  detection_mode_str_  = get_parameter("detection_mode").as_string();
  benchmark_mode_      = get_parameter("benchmark_on_start").as_bool();
  if      (detection_mode_str_ == "AUTO")      detection_mode_ = DetectionMode::AUTO;
  else if (detection_mode_str_ == "MULTI")     detection_mode_ = DetectionMode::MULTI;
  else if (detection_mode_str_ == "BENCHMARK") detection_mode_ = DetectionMode::BENCHMARK;
  else                                         detection_mode_ = DetectionMode::SINGLE;
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
  if (enable_charuco_) {
    charuco_board_ = cv::aruco::CharucoBoard::create(
      7, 5, 0.035f, 0.0175f, aruco_dict_);
    RCLCPP_INFO(get_logger(), "ChArUco board initialised (7x5)");
  }
  pose_estimator_ = std::make_unique<PoseEstimator>(
    intrinsics_.K, intrinsics_.D, marker_size_);
  pose_estimator_->setSmoothingAlpha(smoothing_alpha_);
  visualizer_ = std::make_unique<Visualizer>(alignment_tolerance_);
}
void FiducialDetector::initPublishers()
{
  pub_pose_      = create_publisher<geometry_msgs::msg::PoseStamped>("/fiducial/pose", 10);
  pub_debug_     = create_publisher<sensor_msgs::msg::Image>("/fiducial/debug_image", 10);
  pub_alignment_ = create_publisher<std_msgs::msg::String>("/fiducial/alignment", 10);
  pub_fps_       = create_publisher<std_msgs::msg::Float32>("/fiducial/fps", 10);
  pub_rejected_  = create_publisher<std_msgs::msg::String>("/fiducial/rejected_candidates", 10);
  pub_cur_dict_  = create_publisher<std_msgs::msg::String>("/fiducial/current_dictionary", 10);
  pub_dict_score_= create_publisher<std_msgs::msg::String>("/fiducial/dictionary_score", 10);
  pub_det_stats_ = create_publisher<std_msgs::msg::String>("/fiducial/detection_stats", 10);
  RCLCPP_INFO(get_logger(), "Publishers created on /fiducial/{{pose,debug_image,alignment,fps,rejected_candidates,current_dictionary,dictionary_score,detection_stats}}");
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
      cv::drawMarker(annotated, m.center, cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 20, 2, cv::LINE_AA);
      if (visualizer_->isAligned(m.center, result.frame_size)) {
        any_locked = true;
        RCLCPP_INFO(get_logger(), "Posisi Centering: (%.1f, %.1f)", m.center.x, m.center.y);
      }
    }
  }
  visualizer_->drawUI(annotated, any_locked);
  if (show_rejected_) visualizer_->drawRejected(annotated, result.rejected);
  if (show_window_) {
    enqueueDisplay(annotated);
  }
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
  if (detection_mode_ == DetectionMode::AUTO) {
    std::string best = dict_manager_->autoDetect(gray, det_params_mgr_->params());
    aruco_dict_ = dict_manager_->activeDict();
    dictionary_type_ = best;
    RCLCPP_DEBUG(get_logger(), "AUTO: selected %s", best.c_str());
  } else if (detection_mode_ == DetectionMode::MULTI) {
    auto all = dict_manager_->detectAll(gray, det_params_mgr_->params());
    for (const auto& dr : all) {
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
    if (enable_charuco_) detectCharuco(gray, frame, result);
    return result;
  } else if (detection_mode_ == DetectionMode::BENCHMARK) {
    dict_manager_->benchmark(gray, det_params_mgr_->params(), 10);
  }
  detectAruco(gray, result);
  if (enable_charuco_) {
    detectCharuco(gray, frame, result);
  }
  return result;
}
void FiducialDetector::detectAruco(const cv::Mat& gray, DetectionResult& result)
{
  auto dp = det_params_mgr_->params();
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners, rejected;
  cv::aruco::detectMarkers(gray, dict_manager_->activeDict(), corners, ids, dp, rejected);
  if (!ids.empty()) {
    cv::aruco::refineDetectedMarkers(
      gray, charuco_board_ ? cv::Ptr<cv::aruco::Board>(charuco_board_)
                           : cv::makePtr<cv::aruco::Board>(),
      corners, ids, rejected,
      intrinsics_.K, intrinsics_.D,
      10.f, 3.f, true, cv::noArray(), dp);
  }
  for (std::size_t i = 0; i < ids.size(); ++i) {
    DetectedMarker m;
    m.id      = ids[i];
    m.type    = MarkerType::ARUCO;
    m.corners = corners[i];
    m.center  = computeCenter(corners[i]);
    result.markers.push_back(std::move(m));
  }
  result.rejected.insert(result.rejected.end(), rejected.begin(), rejected.end());
  RCLCPP_DEBUG(get_logger(), "ArUco: %zu detected, %zu rejected",
    ids.size(), rejected.size());
}
void FiducialDetector::detectCharuco(
  const cv::Mat& gray, [[maybe_unused]] const cv::Mat& color, DetectionResult& result)
{
  if (!charuco_board_) return;
  std::vector<int> marker_ids;
  std::vector<std::vector<cv::Point2f>> marker_corners;
  cv::aruco::detectMarkers(gray, aruco_dict_, marker_corners, marker_ids,
                           det_params_mgr_->params());
  if (marker_ids.empty()) return;
  std::vector<cv::Point2f> charuco_corners;
  std::vector<int>         charuco_ids;
  int n = cv::aruco::interpolateCornersCharuco(
    marker_corners, marker_ids, gray, charuco_board_,
    charuco_corners, charuco_ids,
    intrinsics_.K, intrinsics_.D);
  if (n < 4) return;
  cv::Point2f c(0,0);
  for (const auto& p : charuco_corners) c += p;
  c *= (1.0f / (float)charuco_corners.size());
  auto br = cv::boundingRect(charuco_corners);
  DetectedMarker m;
  m.id   = 0;
  m.type = MarkerType::CHARUCO;
  m.corners = {
    cv::Point2f((float)br.x,            (float)br.y),
    cv::Point2f((float)(br.x+br.width), (float)br.y),
    cv::Point2f((float)(br.x+br.width), (float)(br.y+br.height)),
    cv::Point2f((float)br.x,            (float)(br.y+br.height))
  };
  m.center = c;
  result.markers.push_back(std::move(m));
  RCLCPP_DEBUG(get_logger(), "ChArUco: %d corners interpolated", n);
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
    msg.data = dict_manager_->activeName();
    pub_cur_dict_->publish(msg);
  }
  {
    auto s = dict_manager_->lastScore(dict_manager_->activeName());
    char buf[256];
    std::snprintf(buf, sizeof(buf),
      "{\"dict\":\"%s\",\"valid\":%d,\"rejected\":%d,\"latency_ms\":%.2f,\"score\":%.1f}",
      s.dict_name.c_str(), s.valid_markers, s.rejected_count, s.latency_ms, s.score);
    auto msg = std_msgs::msg::String();
    msg.data = buf;
    pub_dict_score_->publish(msg);
  }
  {
    char buf[256];
    std::snprintf(buf, sizeof(buf),
      "{\"mode\":\"%s\",\"fps\":%.1f,\"markers\":%zu,\"frame\":%llu}",
      detection_mode_str_.c_str(), fps_monitor_.getFps(),
      result.markers.size(), (unsigned long long)frame_count_);
    auto msg = std_msgs::msg::String();
    msg.data = buf;
    pub_det_stats_->publish(msg);
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
  {
    std::lock_guard<std::mutex> lk(display_mutex_);
    display_frame_ = frame.clone();
    display_ready_ = true;
  }
  display_cv_.notify_one();
}
bool FiducialDetector::displayLoop()
{
  if (!show_window_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    return rclcpp::ok();
  }
  cv::Mat frame_to_show;
  {
    std::unique_lock<std::mutex> lk(display_mutex_);
    display_cv_.wait_for(lk, std::chrono::milliseconds(100),
      [this]{ return display_ready_.load(); });
    if (!display_ready_) return rclcpp::ok();
    frame_to_show = display_frame_.clone();
    display_ready_ = false;
  }
  cv::imshow("Fiducial Detector", frame_to_show);
  int key = cv::waitKey(1);
  if (key == 27) {
    RCLCPP_INFO(get_logger(), "ESC pressed — shutting down");
    rclcpp::shutdown();
    return false;
  }
  return rclcpp::ok();
}
}
