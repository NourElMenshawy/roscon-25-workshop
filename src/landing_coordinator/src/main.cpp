#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "landing_coordinator/landing_coordinator.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<landing_coordinator::LandingCoordinator>());
  rclcpp::shutdown();
  return 0;
}
