#include "landing_mission_controller/landing_mission_controller.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<landing_mission_controller::LandingMissionController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
