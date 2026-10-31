#include "landing_coordinator/landing_coordinator.hpp"

#include <memory>

namespace landing_coordinator {

LandingCoordinator::LandingCoordinator(const rclcpp::NodeOptions& options)
    : rclcpp::Node("landing_coordinator", options)
{
  status_publisher_ =
      create_publisher<LandingStatus>("~/status", rclcpp::QoS(1).reliable().transient_local());

  LandingStatus available;
  available.mission_id = 0;
  available.owner_id = "";
  available.state = LandingStatus::AVAILABLE;
  status_publisher_->publish(available);

  service_ = create_service<RequestLanding>(
      "~/request_landing",
      [this](std::shared_ptr<RequestLanding::Request>
                 request,  // NOLINT(performance-unnecessary-value-param)
             std::shared_ptr<RequestLanding::Response>
                 response) {  // NOLINT(performance-unnecessary-value-param)
        handleRequest(*request, *response);
      });

  completion_service_ = create_service<ReportLandingComplete>(
      "~/report_landing_complete",
      [this](std::shared_ptr<ReportLandingComplete::Request> request,
             std::shared_ptr<ReportLandingComplete::Response> response) {
        handleCompletion(*request, *response);
      });

  RCLCPP_INFO(get_logger(), "Coordinator ready: %s; completion: %s", service_->get_service_name(),
              completion_service_->get_service_name());
}

void LandingCoordinator::handleRequest(const RequestLanding::Request& request,
                                       RequestLanding::Response& response)
{
  const auto decision = arbitrator_.requestLanding(request.drone_id);
  response.granted = decision.granted;
  response.owner_id = decision.owner_id;
  response.mission_id = decision.mission_id;
  response.reason = decision.reason;

  // Exactly one new reservation is published in this initial single-mission demo.
  // Invalid requests, retries, and denied requests cannot change the status.
  if (decision.reason == "granted") {
    LandingStatus status;
    status.mission_id = decision.mission_id;
    status.owner_id = decision.owner_id;
    status.state = LandingStatus::RESERVED;
    status_publisher_->publish(status);
  }

  RCLCPP_INFO(get_logger(), "requester='%s' result=%s owner='%s' mission=%llu",
              request.drone_id.c_str(), decision.reason.c_str(), decision.owner_id.c_str(),
              static_cast<unsigned long long>(decision.mission_id));
}

void LandingCoordinator::handleCompletion(const ReportLandingComplete::Request& request,
                                          ReportLandingComplete::Response& response)
{
  const auto decision =
      arbitrator_.reportLandingComplete(request.drone_id, request.mission_id, request.success);
  response.accepted = decision.accepted;
  response.completed = decision.completed;
  response.reason = decision.reason;

  if (decision.newly_completed) {
    LandingStatus status;
    status.mission_id = request.mission_id;
    status.owner_id = request.drone_id;
    status.state = LandingStatus::COMPLETED;
    // Preserve the owner: this physical landing slot remains occupied.
    status_publisher_->publish(status);
  }

  RCLCPP_INFO(get_logger(),
              "landing completion requester='%s' mission=%llu success=%s "
              "accepted=%s reason='%s'",
              request.drone_id.c_str(), static_cast<unsigned long long>(request.mission_id),
              request.success ? "true" : "false", decision.accepted ? "true" : "false",
              decision.reason.c_str());
}

}  // namespace landing_coordinator
