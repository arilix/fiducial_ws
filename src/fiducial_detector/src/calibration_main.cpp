#include "fiducial_detector/calibration_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <thread>
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<fiducial_detector::CalibrationNode>();
  std::thread spin_t([&node](){ rclcpp::spin(node); });
  while (node->displayLoop()) {}
  rclcpp::shutdown();
  if (spin_t.joinable()) spin_t.join();
  cv::destroyAllWindows();
  return 0;
}
