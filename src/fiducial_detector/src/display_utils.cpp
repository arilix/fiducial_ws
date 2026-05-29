#include "fiducial_detector/display_utils.hpp"
#include <chrono>
#include <thread>
#include <rclcpp/rclcpp.hpp>
namespace fiducial_detector {
bool runDisplayLoop(
  std::atomic<bool>&       display_ready,
  std::mutex&              display_mutex,
  std::condition_variable& display_cv,
  cv::Mat&                 display_frame,
  const std::string&       window_name)
{
  cv::Mat frame;
  {
    std::unique_lock<std::mutex> lk(display_mutex);
    display_cv.wait_for(lk, std::chrono::milliseconds(100),
      [&display_ready]{ return display_ready.load(); });
    if (!display_ready) return rclcpp::ok();
    frame = display_frame.clone();
    display_ready = false;
  }
  cv::imshow(window_name, frame);
  int key = cv::waitKey(1);
  if (key == 27) {        // ESC
    rclcpp::shutdown();
    return false;
  }
  return rclcpp::ok();
}
} // namespace fiducial_detector
