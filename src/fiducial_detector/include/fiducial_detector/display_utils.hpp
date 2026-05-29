#pragma once
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <opencv2/opencv.hpp>
namespace fiducial_detector {
/**
 * @brief Shared display loop used by both FiducialDetector and CalibrationNode.
 *
 * Waits (up to 100 ms) for a new frame to be enqueued, then renders it via
 * cv::imshow and handles ESC to trigger rclcpp::shutdown().
 *
 * @param display_ready   Atomic flag set true when a new frame is available.
 * @param display_mutex   Mutex protecting display_frame.
 * @param display_cv      Condition variable to wake the display thread.
 * @param display_frame   Frame buffer written by the callback thread.
 * @param window_name     OpenCV window title.
 * @return true while rclcpp::ok(), false if ESC was pressed or node shut down.
 */
bool runDisplayLoop(
  std::atomic<bool>&      display_ready,
  std::mutex&             display_mutex,
  std::condition_variable& display_cv,
  cv::Mat&                display_frame,
  const std::string&      window_name);
} // namespace fiducial_detector
