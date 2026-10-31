#include "landing_coordinator/landing_coordinator.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<landing_coordinator::LandingCoordinator>());
  rclcpp::shutdown();
  return 0;
}
