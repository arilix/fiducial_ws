#include <rclcpp/rclcpp.hpp>
#include "utils/capture_node.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<fiducial_detector::CaptureNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
