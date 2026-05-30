#include "fiducial_detector/aruco.hpp"
#include <thread>
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // Use SingleThreadedExecutor: the capture_thread_ handles concurrent image
  // acquisition; ROS callbacks (imageCallback, timers) must be serialized to
  // prevent race conditions in cv::aruco::detectMarkers and autoDetect.
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto node = std::make_shared<fiducial_detector::FiducialDetector>();
  executor->add_node(node);
  RCLCPP_INFO(node->get_logger(),
    "Spinning on SingleThreadedExecutor");
  std::thread spin_thread([&executor]() {
    executor->spin();
  });
  while (rclcpp::ok()) {
    if (!node->displayLoop()) break;
  }
  rclcpp::shutdown();
  if (spin_thread.joinable()) spin_thread.join();
  return 0;
}
