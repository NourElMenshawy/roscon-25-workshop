#include "landing_coordinator/srv/request_landing.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <thread>

using namespace std::chrono_literals;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("landing_request_demo_client");
  const auto drone_id = node->declare_parameter<std::string>("drone_id", "");
  const auto delay_ms = node->declare_parameter<int>("delay_ms", 0);
  if (drone_id.empty() || delay_ms < 0) {
    RCLCPP_ERROR(node->get_logger(), "Set drone_id and nonnegative delay_ms");
    rclcpp::shutdown();
    return 2;
  }

  using RequestLanding = landing_coordinator::srv::RequestLanding;
  auto client = node->create_client<RequestLanding>(
      "/landing_coordinator/request_landing");
  if (!client->wait_for_service(10s)) {
    RCLCPP_ERROR(node->get_logger(), "LandingCoordinator service unavailable");
    rclcpp::shutdown();
    return 3;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
  auto request = std::make_shared<RequestLanding::Request>();
  request->drone_id = drone_id;
  auto pending = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, pending, 5s) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    client->remove_pending_request(pending);
    RCLCPP_ERROR(node->get_logger(), "Landing request timed out or failed");
    rclcpp::shutdown();
    return 4;
  }

  const auto response = pending.get();
  RCLCPP_INFO(node->get_logger(),
              "drone=%s granted=%s owner=%s mission=%llu reason=%s",
              drone_id.c_str(), response->granted ? "true" : "false",
              response->owner_id.c_str(),
              static_cast<unsigned long long>(response->mission_id),
              response->reason.c_str());
  rclcpp::shutdown();
  return 0;
}
