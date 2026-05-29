#include <rclcpp/rclcpp.hpp>
#include "fiducial_detector/capture_node.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<fiducial_detector::CaptureNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
