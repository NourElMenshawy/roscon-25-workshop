#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "landing_coordinator/landing_arbitrator.hpp"
#include "landing_coordinator/msg/landing_status.hpp"
#include "landing_coordinator/srv/report_landing_complete.hpp"
#include "landing_coordinator/srv/request_landing.hpp"

namespace landing_coordinator {

class LandingCoordinator final : public rclcpp::Node {
 public:
  explicit LandingCoordinator(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  using RequestLanding = landing_coordinator::srv::RequestLanding;
  using ReportLandingComplete = landing_coordinator::srv::ReportLandingComplete;
  using LandingStatus = landing_coordinator::msg::LandingStatus;

  void handleRequest(const RequestLanding::Request& request, RequestLanding::Response& response);

  void handleCompletion(const ReportLandingComplete::Request& request,
                        ReportLandingComplete::Response& response);

  LandingArbitrator arbitrator_;
  rclcpp::Service<RequestLanding>::SharedPtr service_;
  rclcpp::Service<ReportLandingComplete>::SharedPtr completion_service_;
  rclcpp::Publisher<LandingStatus>::SharedPtr status_publisher_;
};

}  // namespace landing_coordinator
