#include "landing_coordinator/landing_coordinator.hpp"

#include <memory>

namespace landing_coordinator {

LandingCoordinator::LandingCoordinator(const rclcpp::NodeOptions& options)
    : rclcpp::Node("landing_coordinator", options)
{
  // Private name resolves to /landing_coordinator/request_landing by default.
  service_ = create_service<RequestLanding>(
      "~/request_landing",
      // rclcpp requires its shared_ptr callback arguments by value.
      // Keep this API signature and avoid copies inside our own handler.
      [this](std::shared_ptr<RequestLanding::Request>
                 request,  // NOLINT(performance-unnecessary-value-param)
             std::shared_ptr<RequestLanding::Response>
                 response) {  // NOLINT(performance-unnecessary-value-param)
        handleRequest(*request, *response);
      });

  RCLCPP_INFO(get_logger(), "Coordinator ready: %s", service_->get_service_name());
}

void LandingCoordinator::handleRequest(const RequestLanding::Request& request,
                                       RequestLanding::Response& response)
{
  const auto decision = arbitrator_.requestLanding(request.drone_id);
  response.granted = decision.granted;
  response.owner_id = decision.owner_id;
  response.mission_id = decision.mission_id;
  response.reason = decision.reason;

  RCLCPP_INFO(get_logger(), "requester='%s' result=%s owner='%s' mission=%llu",
              request.drone_id.c_str(), decision.reason.c_str(), decision.owner_id.c_str(),
              static_cast<unsigned long long>(decision.mission_id));
}

}  // namespace landing_coordinator
