#include "utils/aruco.h"
#include "utils/marker_decoder.h"
#include <opencv2/imgproc.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

namespace fiducial_detector {

FiducialDetector::FiducialDetector(const rclcpp::NodeOptions& options)
    : rclcpp::Node("aruco_node", options)
    , fps_monitor_(60)
{
    cv::setUseOptimized(true);
    declareParameters();
    loadRosParams();
    loadIntrinsics();
    initDetectors();
    initPublishers();
    initSubscriber();

    if (capture_internal_) {
        initInternalCapture();
        capture_start_timer_ = create_wall_timer(
            std::chrono::milliseconds(200),
            [this]() {
                capture_start_timer_->cancel();
                capture_running_ = true;
                capture_thread_  = std::thread(&FiducialDetector::loopInternalCapture, this);
            });
    }

    fps_timer_ = create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&FiducialDetector::fpsTimerCallback, this));
    watchdog_timer_ = create_wall_timer(
        std::chrono::seconds(2),
        std::bind(&FiducialDetector::watchdogCallback, this));

    RCLCPP_INFO(get_logger(), "═══════════════════════════════════════");
    RCLCPP_INFO(get_logger(), " DICT_7X7_50 Gate Centering — ROS2 Humble");
    RCLCPP_INFO(get_logger(), "═══════════════════════════════════════");
    RCLCPP_INFO(get_logger(), "  camera_topic : %s", camera_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  marker_size  : %.3f m", marker_size_);
    RCLCPP_INFO(get_logger(), "  align_tol    : %d px", alignment_tolerance_);
    RCLCPP_INFO(get_logger(), "  stable_frames: %d", alignment_stable_frames_);
    if (capture_internal_) {
        RCLCPP_INFO(get_logger(), "  capture_internal: ON (device=%s)",
            capture_device_path_.empty()
                ? ("/dev/video" + std::to_string(capture_device_id_)).c_str()
                : capture_device_path_.c_str());
    }
}

FiducialDetector::~FiducialDetector() {
    if (capture_internal_) {
        capture_running_ = false;
        if (capture_thread_.joinable()) capture_thread_.join();
        if (cap_.isOpened()) cap_.release();
    }
    if (show_window_) {
        try { cv::destroyAllWindows(); } catch (...) {}
    }
}

void FiducialDetector::declareParameters() {
    declare_parameter("marker_size",          0.05);
    declare_parameter("camera_topic",         "/camera/image_raw");
    declare_parameter("show_window",          false);
    declare_parameter("show_rejected",        true);
    declare_parameter("alignment_tolerance",  50);
    declare_parameter("smoothing_alpha",      0.4);
    declare_parameter("max_missed_frames",    5);
    declare_parameter("alignment_stable_frames", 10);
    declare_parameter("camera_matrix",  std::vector<double>{
        640.0, 0.0, 320.0,  0.0, 640.0, 240.0,  0.0, 0.0, 1.0});
    declare_parameter("dist_coeffs",   std::vector<double>{0,0,0,0,0});
    declare_parameter("show_corner_labels",     true);
    declare_parameter("show_orientation_arrow", true);
    declare_parameter("show_confidence",        true);
    declare_parameter("enable_clahe",           true);
    declare_parameter("clahe_clip_limit",       2.0);
    declare_parameter("enable_sharpen",         false);
    declare_parameter("enable_blur",            false);
    declare_parameter("cuda",                   false);          // true/false
    declare_parameter("capture_internal",       false);
    declare_parameter("capture_device_id",      0);
    declare_parameter("capture_device_path",    std::string(""));
    declare_parameter("capture_width",          640);
    declare_parameter("capture_height",         480);
    declare_parameter("capture_fps",            30.0);
    det_params_mgr_ = std::make_unique<DetectorParametersManager>();
    det_params_mgr_->declareAll(this);
}

void FiducialDetector::loadRosParams() {
    marker_size_             = get_parameter("marker_size").as_double();
    camera_topic_            = get_parameter("camera_topic").as_string();
    show_window_             = get_parameter("show_window").as_bool();
    show_rejected_           = get_parameter("show_rejected").as_bool();
    alignment_tolerance_     = get_parameter("alignment_tolerance").as_int();
    smoothing_alpha_         = get_parameter("smoothing_alpha").as_double();
    max_missed_frames_       = get_parameter("max_missed_frames").as_int();
    alignment_stable_frames_ = get_parameter("alignment_stable_frames").as_int();
    show_corner_labels_      = get_parameter("show_corner_labels").as_bool();
    show_orientation_arrow_  = get_parameter("show_orientation_arrow").as_bool();
    show_confidence_         = get_parameter("show_confidence").as_bool();
    enable_clahe_            = get_parameter("enable_clahe").as_bool();
    clahe_clip_              = get_parameter("clahe_clip_limit").as_double();
    enable_sharpen_          = get_parameter("enable_sharpen").as_bool();
    enable_blur_             = get_parameter("enable_blur").as_bool();
    {
        use_cuda_enabled_ = get_parameter("cuda").as_bool();
    }
    capture_internal_        = get_parameter("capture_internal").as_bool();
    capture_device_id_       = get_parameter("capture_device_id").as_int();
    capture_device_path_     = get_parameter("capture_device_path").as_string();
    capture_width_           = get_parameter("capture_width").as_int();
    capture_height_          = get_parameter("capture_height").as_int();
    capture_fps_             = get_parameter("capture_fps").as_double();
}

void FiducialDetector::loadIntrinsics() {
    auto cam_vec  = get_parameter("camera_matrix").as_double_array();
    auto dist_vec = get_parameter("dist_coeffs").as_double_array();
    if (cam_vec.size() == 9) {
        intrinsics_.K = cv::Mat(3, 3, CV_64F, cam_vec.data()).clone();
    } else {
        intrinsics_.K = (cv::Mat_<double>(3,3) << 640,0,320, 0,640,240, 0,0,1);
        RCLCPP_WARN(get_logger(), "camera_matrix invalid — using placeholder. Run calibration!");
    }
    if (!dist_vec.empty()) {
        intrinsics_.D = cv::Mat(1, static_cast<int>(dist_vec.size()),
                                CV_64F, dist_vec.data()).clone();
    } else {
        intrinsics_.D = cv::Mat::zeros(1, 5, CV_64F);
    }
    intrinsics_.valid = true;
    RCLCPP_INFO(get_logger(), "Camera intrinsics loaded");
}

void FiducialDetector::initDetectors() {
    dict_manager_ = std::make_unique<DictionaryManager>();
    dict_manager_->setActive("DICT_7X7_50");
    aruco_dict_ = dict_manager_->activeDict();
    dict_manager_->printStartupValidation("DICT_7X7_50");

    det_params_mgr_->bind(this);
    det_params_mgr_->apply7x7Profile();

    if (!dict_manager_->validateDictionary("DICT_7X7_50")) {
        RCLCPP_WARN(get_logger(), "DICT_7X7_50 validation failed");
    }

    if (enable_clahe_) {
        clahe_ = cv::createCLAHE(clahe_clip_, cv::Size(8, 8));
    }

#ifdef FIDUCIAL_USE_CUDA
    // Probe CUDA device availability at startup
    if (use_cuda_enabled_) {
        int cuda_devs = cv::cuda::getCudaEnabledDeviceCount();
        if (cuda_devs > 0) {
            cv::cuda::setDevice(0);
            cv::cuda::DeviceInfo di(0);
            cuda_ok_ = true;
            if (enable_clahe_) {
                // CLAHE on GPU: same clip limit, 8×8 tile grid
                cuda_clahe_ = cv::cuda::createCLAHE(clahe_clip_, cv::Size(8, 8));
            }
            RCLCPP_INFO(get_logger(),
                "CUDA preprocessing: ENABLED — %s (%d MB free)",
                di.name(), static_cast<int>(di.freeMemory() / 1024 / 1024));
        } else {
            RCLCPP_WARN(get_logger(),
                "CUDA: no GPU detected — falling back to CPU preprocessing");
        }
    } else {
        RCLCPP_INFO(get_logger(), "CUDA preprocessing: OFF (launch with cuda:=on to enable)");
    }
#else
    if (use_cuda_enabled_) {
        RCLCPP_WARN(get_logger(),
            "cuda:=on requested but binary not compiled with USE_CUDA=ON — "
            "rebuild: colcon build --cmake-args -DUSE_CUDA=ON");
    }
#endif

    pose_estimator_ = std::make_unique<PoseEstimator>(
        intrinsics_.K, intrinsics_.D, marker_size_);
    pose_estimator_->setSmoothingAlpha(smoothing_alpha_);

    confidence_calc_  = std::make_unique<ConfidenceCalculator>();
    benchmark_runner_ = std::make_unique<BenchmarkRunner>();

    gate_alignment_ = std::make_unique<GateAlignmentEngine>(
        alignment_tolerance_,
        3,
        alignment_stable_frames_,
        max_missed_frames_);

    visualizer_ = std::make_unique<Visualizer>(alignment_tolerance_);

    RCLCPP_INFO(get_logger(), "DICT_7X7_50 detector initialized (ID 0-49)");
}

void FiducialDetector::initPublishers() {
    pub_pose_      = create_publisher<geometry_msgs::msg::PoseStamped>("/fiducial/pose", 10);
    pub_debug_     = create_publisher<sensor_msgs::msg::Image>("/fiducial/debug_image", 10);
    pub_alignment_ = create_publisher<std_msgs::msg::String>("/fiducial/alignment", 10);
    pub_fps_       = create_publisher<std_msgs::msg::Float32>("/fiducial/fps", 10);
    pub_rejected_  = create_publisher<std_msgs::msg::String>("/fiducial/rejected_candidates", 10);
    RCLCPP_INFO(get_logger(),
        "Publishers: /fiducial/{pose,debug_image,alignment,fps,rejected_candidates}");
}

void FiducialDetector::initSubscriber() {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
    image_sub_ = image_transport::create_subscription(
        this, camera_topic_,
        std::bind(&FiducialDetector::imageCallback, this, std::placeholders::_1),
        "raw", qos.get_rmw_qos_profile());
    cam_connected_ = true;
    RCLCPP_INFO(get_logger(), "Subscribed to '%s'", camera_topic_.c_str());
}

void FiducialDetector::imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
    // Serialize callback — no parallel detection on same frame
    std::lock_guard<std::mutex> cb_lock(callback_mutex_);
    cam_connected_   = true;
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
    computeConfidence(result);

    GateError gate_err;
    if (!result.markers.empty()) {
        const auto& primary = result.markers[0];
        float dist = primary.pose.valid
            ? static_cast<float>(primary.pose.distance) : 0.f;
        gate_err = gate_alignment_->update(
            primary.center, result.frame_size, dist, primary.id);
    } else {
        gate_err = gate_alignment_->update(
            {0.f, 0.f}, result.frame_size, 0.f, -1);
    }

    cv::Mat annotated = frame.clone();
    bool any_locked = false;

    for (const auto& m : result.markers) {
        std::vector<std::vector<cv::Point2f>> c_wrap{m.corners};
        std::vector<int> id_wrap{m.id};
        visualizer_->drawDetectedMarkers(annotated, c_wrap, id_wrap, m.type);
        if (m.pose.valid) {
            visualizer_->drawPoseAxis(annotated, m.pose,
                intrinsics_.K, intrinsics_.D,
                static_cast<float>(marker_size_) * 0.6f);
        }
        if (visualizer_->isAligned(m.center, result.frame_size)) any_locked = true;
    }

    if (show_rejected_) visualizer_->drawRejected(annotated, result.rejected);
    visualizer_->drawUI(annotated, any_locked);

    if (show_window_) enqueueDisplay(annotated);
    publishAll(result, annotated, gate_err, msg->header.stamp);
}

void FiducialDetector::fpsTimerCallback() {
    auto msg  = std_msgs::msg::Float32();
    msg.data  = fps_monitor_.getFps();
    pub_fps_->publish(msg);
}

void FiducialDetector::watchdogCallback() {
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

cv::Mat FiducialDetector::preprocessFrame(const cv::Mat& gray) {
    cv::Mat processed = gray;

#ifdef FIDUCIAL_USE_CUDA
    if (use_cuda_enabled_ && cuda_ok_) {
        try {
            cv::cuda::GpuMat gpu;
            gpu.upload(processed);

            if (enable_clahe_ && cuda_clahe_) {
                // CLAHE on GPU: equalizes local contrast for low-light/glare scenes
                cv::cuda::GpuMat gpu_eq;
                cuda_clahe_->apply(gpu, gpu_eq);
                gpu = gpu_eq;
            }
            if (enable_blur_) {
                auto blur_f = cv::cuda::createGaussianFilter(
                    gpu.type(), gpu.type(), cv::Size(3, 3), 0);
                cv::cuda::GpuMat gpu_blurred;
                blur_f->apply(gpu, gpu_blurred);
                gpu = gpu_blurred;
            }
            if (enable_sharpen_) {
                // Unsharp mask: sharpened = 1.5*orig - 0.5*blurred
                auto blur_f = cv::cuda::createGaussianFilter(
                    gpu.type(), gpu.type(), cv::Size(0, 0), 3);
                cv::cuda::GpuMat gpu_blurred;
                blur_f->apply(gpu, gpu_blurred);
                cv::cuda::GpuMat gpu_sharp;
                cv::cuda::addWeighted(gpu, 1.5, gpu_blurred, -0.5, 0, gpu_sharp);
                gpu = gpu_sharp;
            }
            gpu.download(processed);
            return processed;
        } catch (const cv::Exception& e) {
            RCLCPP_WARN_ONCE(get_logger(),
                "CUDA preprocessing error: %s — falling back to CPU", e.what());
        }
    }
#endif

    // CPU fallback path
    if (enable_clahe_ && clahe_) {
        cv::Mat enhanced;
        // CLAHE equalizes local contrast — critical for low-light and high-glare scenes
        clahe_->apply(processed, enhanced);
        processed = enhanced;
    }
    if (enable_blur_) {
        cv::GaussianBlur(processed, processed, cv::Size(3, 3), 0);
    }
    if (enable_sharpen_) {
        cv::Mat blurred;
        cv::GaussianBlur(processed, blurred, cv::Size(0, 0), 3);
        cv::addWeighted(processed, 1.5, blurred, -0.5, 0, processed);
    }
    return processed;
}

DetectionResult FiducialDetector::runDetection(const cv::Mat& frame) {
    DetectionResult result;
    result.frame_size = frame.size();
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    gray = preprocessFrame(gray);
    detectAruco(gray, result);
    return result;
}

void FiducialDetector::detectAruco(const cv::Mat& gray, DetectionResult& result) {
    auto dp = det_params_mgr_->params();
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners, rejected;

    cv::aruco::detectMarkers(gray, aruco_dict_, corners, ids, dp, rejected);

    result.markers.reserve(ids.size());
    for (std::size_t i = 0; i < ids.size(); ++i) {
        DetectedMarker m;
        m.id      = ids[i];
        m.type    = MarkerType::ARUCO;
        m.corners = corners[i];
        m.center  = computeCenter(corners[i]);
        result.markers.push_back(std::move(m));
    }
    result.rejected.insert(result.rejected.end(), rejected.begin(), rejected.end());
}

void FiducialDetector::estimatePoses(DetectionResult& result) {
    if (!intrinsics_.valid || !pose_estimator_) return;
    for (auto& m : result.markers) {
        if (m.corners.size() == 4) {
            m.pose = pose_estimator_->estimate(m.id, m.corners);
        }
    }
}

void FiducialDetector::computeConfidence(DetectionResult& result) {
    if (!confidence_calc_) return;
    for (auto& m : result.markers) {
        if (m.corners.size() != 4) continue;
        // Reprojection-based confidence: reprojects estimated pose back to image
        // and measures pixel deviation from detected corners
        m.confidence = confidence_calc_->compute(
            m.corners, m.id, m.pose.rvec, m.pose.tvec,
            intrinsics_.K, intrinsics_.D, 0, 7, 0, marker_size_);
    }
}

void FiducialDetector::logDetectedMarkers(
    const DetectionResult& result,
    const GateError& gate_err) const
{
    if (result.markers.empty()) return;

    float  fps_now = fps_monitor_.getFps();
    double lat_ms  = fps_monitor_.getLatencyMs();

    for (const auto& m : result.markers) {
        float conf_pct = m.confidence.aggregate * 100.f;
        std::printf(
            "\n[Detector]\n"
            "  Family=DICT_7X7_50  ID=%d  Confidence=%.0f%%  FPS=%.1f  Latency=%.1fms\n",
            m.id, conf_pct, fps_now, lat_ms);
        if (m.pose.valid) {
            std::printf(
                "[Pose]\n"
                "  XYZ=(%.3f, %.3f, %.3f)m  Distance=%.3fm\n",
                m.pose.tvec[0], m.pose.tvec[1], m.pose.tvec[2],
                m.pose.distance);
        }
    }

    // Print state machine + alignment error once per frame (for primary marker)
    std::printf(
        "[Tracking]\n"
        "  State=%s\n"
        "[Alignment]\n"
        "  State=%s  ErrorX=%+.0f  ErrorY=%+.0f\n"
        "[Gate]\n"
        "  State=%s  Distance=%.3fm\n",
        gate_err.stateName().c_str(),
        gate_err.stateName().c_str(),
        gate_err.error_x, gate_err.error_y,
        gate_err.stateName().c_str(),
        gate_err.distance);
    std::fflush(stdout);
}

void FiducialDetector::publishAll(
    const DetectionResult& result,
    const cv::Mat& annotated,
    const GateError& gate_err,
    const rclcpp::Time& stamp)
{
    logDetectedMarkers(result, gate_err);

    auto img_msg = cv_bridge::CvImage(
        std_msgs::msg::Header(), "bgr8", annotated).toImageMsg();
    img_msg->header.stamp    = stamp;
    img_msg->header.frame_id = "camera";
    pub_debug_->publish(*img_msg);

    {
        auto msg   = std_msgs::msg::String();
        msg.data   = gate_err.toJson();
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
        auto msg  = std_msgs::msg::String();
        msg.data  = "{\"rejected_count\":"
                  + std::to_string(result.rejected.size())
                  + ",\"detected_count\":"
                  + std::to_string(result.markers.size()) + "}";
        pub_rejected_->publish(msg);
    }
}

void FiducialDetector::reconnectCamera() {
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

cv::Point2f FiducialDetector::computeCenter(
    const std::vector<cv::Point2f>& corners)
{
    cv::Point2f c(0.f, 0.f);
    for (const auto& p : corners) c += p;
    return c * (1.0f / static_cast<float>(corners.size()));
}

void FiducialDetector::enqueueDisplay(const cv::Mat& frame) {
    // Lock guards display_frame_ before notify — prevents race with displayLoop
    { std::lock_guard<std::mutex> lk(display_mutex_);
      display_frame_ = frame.clone();
      display_ready_ = true; }
    display_cv_.notify_one();
}

bool FiducialDetector::displayLoop() {
    if (!show_window_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        return rclcpp::ok();
    }
    try {
        // Wait for a new display frame with 50ms timeout
        cv::Mat local_frame;
        {
            std::unique_lock<std::mutex> lk(display_mutex_);
            display_cv_.wait_for(lk, std::chrono::milliseconds(50),
                                 [this] { return display_ready_.load(); });
            if (!display_ready_) return rclcpp::ok();
            local_frame    = display_frame_.clone();
            display_ready_ = false;
        }
        cv::imshow("Fiducial Detector", local_frame);
        int key = cv::waitKey(1);
        if (key == 'q' || key == 27) return false;
        if (show_cells_window_ || show_thresh_window_
                || show_contour_window_ || show_rejected_window_) {
            std::lock_guard<std::mutex> lk(debug_mutex_);
            if (show_cells_window_  && last_debug_.valid && !last_debug_.cell_grid_image.empty())
                cv::imshow("Marker Cells", last_debug_.cell_grid_image);
            if (show_thresh_window_ && last_debug_.valid && !last_debug_.threshold_image.empty())
                cv::imshow("Threshold",   last_debug_.threshold_image);
            if (show_contour_window_ && last_debug_.valid && !last_debug_.contour_image.empty())
                cv::imshow("Contours",    last_debug_.contour_image);
            if (show_rejected_window_ && last_debug_.valid && !last_debug_.rejected_image.empty())
                cv::imshow("Rejected",    last_debug_.rejected_image);
            int key2 = cv::waitKey(1);
            if      (key2 == 'd') { bool v = !show_cells_window_; show_cells_window_ = v; show_thresh_window_ = v; show_contour_window_ = v; show_rejected_window_ = v; }
            else if (key2 == 'c') show_cells_window_    = !show_cells_window_;
            else if (key2 == 't') show_thresh_window_   = !show_thresh_window_;
            else if (key2 == 'n') show_contour_window_  = !show_contour_window_;
            else if (key2 == 'r') show_rejected_window_ = !show_rejected_window_;
        } else {
            int key2 = cv::waitKey(1);
            if      (key2 == 'd') { show_cells_window_ = true; show_thresh_window_ = true; show_contour_window_ = true; show_rejected_window_ = true; }
            else if (key2 == 'c') show_cells_window_    = true;
            else if (key2 == 't') show_thresh_window_   = true;
            else if (key2 == 'n') show_contour_window_  = true;
            else if (key2 == 'r') show_rejected_window_ = true;
        }
    } catch (const cv::Exception& e) {
        RCLCPP_WARN_ONCE(get_logger(),
            "OpenCV display error (no GUI?): %s — disabling show_window", e.what());
        show_window_ = false;
    } catch (...) {
        RCLCPP_WARN_ONCE(get_logger(), "Display loop exception — disabling show_window");
        show_window_ = false;
    }
    return rclcpp::ok();
}

void FiducialDetector::initInternalCapture() {
    std::string dev = capture_device_path_.empty()
        ? std::to_string(capture_device_id_)
        : capture_device_path_;

    if (capture_device_path_.empty())
        cap_.open(capture_device_id_, cv::CAP_V4L2);
    else
        cap_.open(capture_device_path_, cv::CAP_V4L2);

    if (!cap_.isOpened()) {
        RCLCPP_ERROR(get_logger(),
            "capture_internal: failed to open '%s' — waiting for image topic", dev.c_str());
        capture_internal_ = false;
        return;
    }

    cap_.set(cv::CAP_PROP_FRAME_WIDTH,  capture_width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, capture_height_);
    cap_.set(cv::CAP_PROP_FPS,          capture_fps_);

    RCLCPP_INFO(get_logger(),
        "capture_internal: opened %s | %.0fx%.0f @ %.0ffps",
        dev.c_str(),
        cap_.get(cv::CAP_PROP_FRAME_WIDTH),
        cap_.get(cv::CAP_PROP_FRAME_HEIGHT),
        cap_.get(cv::CAP_PROP_FPS));

    auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
    pub_internal_cam_ = create_publisher<sensor_msgs::msg::Image>(camera_topic_, qos);
}

void FiducialDetector::loopInternalCapture() {
    const double min_interval_s = (capture_fps_ > 0) ? 1.0 / capture_fps_ : 0.0;
    cv::Mat frame;

    while (capture_running_ && rclcpp::ok()) {
        auto t_start = std::chrono::steady_clock::now();

        if (!cap_.read(frame) || frame.empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "capture_internal: camera read failed — retrying");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if (capture_device_path_.empty())
                cap_.open(capture_device_id_, cv::CAP_V4L2);
            else
                cap_.open(capture_device_path_, cv::CAP_V4L2);
            continue;
        }

        auto msg = std::make_shared<sensor_msgs::msg::Image>();
        msg->header.stamp    = now();
        msg->header.frame_id = "camera_optical_frame";
        msg->height   = frame.rows;
        msg->width    = frame.cols;
        msg->encoding = "bgr8";
        msg->step     = frame.step;
        msg->data.assign(frame.datastart, frame.dataend);
        pub_internal_cam_->publish(std::move(*msg));

        // Rate limiting to match capture_fps_
        auto elapsed = std::chrono::steady_clock::now() - t_start;
        double elapsed_s = std::chrono::duration<double>(elapsed).count();
        if (elapsed_s < min_interval_s) {
            std::this_thread::sleep_for(
                std::chrono::duration<double>(min_interval_s - elapsed_s));
        }
    }
}

} // namespace fiducial_detector
