#include "fiducial_detector/aruco.hpp"
#include <rclcpp/rclcpp.hpp>
#include <thread>
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
    rclcpp::ExecutorOptions(), 4);
  auto node = std::make_shared<fiducial_detector::FiducialDetector>();
  executor->add_node(node);
  RCLCPP_INFO(node->get_logger(), "Spinning on MultiThreadedExecutor (4 threads)");
  std::thread spin_thread([&executor]() {
    executor->spin();
  });
  while (node->displayLoop()) { }
  rclcpp::shutdown();
  executor->cancel();
  if (spin_thread.joinable()) spin_thread.join();
  cv::destroyAllWindows();
  return 0;
}
