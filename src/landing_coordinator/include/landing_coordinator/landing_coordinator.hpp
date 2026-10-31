#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "landing_coordinator/landing_arbitrator.hpp"
#include "landing_coordinator/srv/request_landing.hpp"

namespace landing_coordinator {

class LandingCoordinator final : public rclcpp::Node {
 public:
  explicit LandingCoordinator(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  using RequestLanding = landing_coordinator::srv::RequestLanding;

  void handleRequest(const RequestLanding::Request& request, RequestLanding::Response& response);

  LandingArbitrator arbitrator_;
  rclcpp::Service<RequestLanding>::SharedPtr service_;
};

}  // namespace landing_coordinator
