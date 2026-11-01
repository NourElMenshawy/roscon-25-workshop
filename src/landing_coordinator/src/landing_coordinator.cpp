#include "landing_coordinator/landing_coordinator.hpp"

#include <cmath>
#include <memory>
#include <stdexcept>

namespace landing_coordinator {

LandingCoordinator::LandingCoordinator(const rclcpp::NodeOptions& options)
    : rclcpp::Node("landing_coordinator", options) {
  // A late-joining subscriber sees the most recent authoritative status.
  status_publisher_ = create_publisher<LandingStatus>(
      "~/status", rclcpp::QoS(1).reliable().transient_local());
  sites_publisher_ = create_publisher<LandingSiteMap>(
      "~/landing_sites", rclcpp::QoS(1).reliable().transient_local());

  // Site coordinates are operator-verified in the common TF map frame.
  // Both sites default to unverified: incomplete configuration cannot authorize
  // a POSITION_LANDING selection. Occupancy is set only upon mission completion.
  configured_sites_.frame_id = declare_parameter<std::string>(
      "site_frame_id", "map");
  landing_coordinator::msg::LandingSite primary{};
  primary.site_id = "aruco_primary";
  primary.position.x = declare_parameter<double>("site_a_x", 0.0);
  primary.position.y = declare_parameter<double>("site_a_y", 0.0);
  primary.position.z = declare_parameter<double>("site_a_z", 0.0);
  primary.clearance_radius_m = declare_parameter<double>("site_a_radius_m", 0.75);
  primary.verified = declare_parameter<bool>("site_a_verified", false);
  primary.occupied = true;

  landing_coordinator::msg::LandingSite secondary{};
  secondary.site_id = "secondary_position";
  secondary.position.x = declare_parameter<double>("site_b_x", 0.0);
  secondary.position.y = declare_parameter<double>("site_b_y", 0.0);
  secondary.position.z = declare_parameter<double>("site_b_z", 0.0);
  secondary.clearance_radius_m = declare_parameter<double>("site_b_radius_m", 0.75);
  secondary.verified = declare_parameter<bool>("site_b_verified", false);
  secondary.occupied = false;

  const auto finite_site = [](const landing_coordinator::msg::LandingSite& site) {
    return std::isfinite(site.position.x) && std::isfinite(site.position.y) &&
           std::isfinite(site.position.z) &&
           std::isfinite(site.clearance_radius_m) &&
           site.clearance_radius_m > 0.0;
  };
  if (configured_sites_.frame_id.empty() || !finite_site(primary) ||
      !finite_site(secondary)) {
    throw std::invalid_argument("Invalid configured landing-site geometry");
  }
  configured_sites_.sites = {primary, secondary};

  LandingStatus available;
  available.mission_id = 0;
  available.owner_id = "";
  available.state = LandingStatus::AVAILABLE;
  status_publisher_->publish(available);

  // Private name resolves to /landing_coordinator/request_landing by default.
  service_ = create_service<RequestLanding>(
      "~/request_landing",
      // rclcpp requires its shared_ptr callback arguments by value.
      // Keep this API signature and avoid copies inside our own handler.
      [this](
          std::shared_ptr<RequestLanding::Request> request,  // NOLINT(performance-unnecessary-value-param)
          std::shared_ptr<RequestLanding::Response> response) {  // NOLINT(performance-unnecessary-value-param)
        handleRequest(*request, *response);
      });

  completion_service_ = create_service<ReportLandingComplete>(
      "~/report_landing_complete",
      [this](std::shared_ptr<ReportLandingComplete::Request> request,
             std::shared_ptr<ReportLandingComplete::Response> response) {
        handleCompletion(*request, *response);
      });

  RCLCPP_INFO(get_logger(), "Coordinator ready: %s; completion: %s",
              service_->get_service_name(), completion_service_->get_service_name());
}

void LandingCoordinator::handleRequest(
    const RequestLanding::Request& request,
    RequestLanding::Response& response) {
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
              request.drone_id.c_str(), decision.reason.c_str(),
              decision.owner_id.c_str(),
              static_cast<unsigned long long>(decision.mission_id));
}

void LandingCoordinator::handleCompletion(
    const ReportLandingComplete::Request& request,
    ReportLandingComplete::Response& response) {
  const auto decision = arbitrator_.reportLandingComplete(
      request.drone_id, request.mission_id, request.success);
  response.accepted = decision.accepted;
  response.completed = decision.completed;
  response.reason = decision.reason;

  if (decision.newly_completed) {
    LandingStatus status;
    status.mission_id = request.mission_id;
    status.owner_id = request.drone_id;
    status.state = LandingStatus::COMPLETED;
    // Publish an atomic site catalogue with this mission's authoritative
    // identity. Both site centres still require independent verification.
    auto sites = configured_sites_;
    sites.mission_id = request.mission_id;
    sites.owner_id = request.drone_id;
    sites_publisher_->publish(sites);

    // Preserve the owner: the primary landing slot remains occupied.
    status_publisher_->publish(status);
  }

  RCLCPP_INFO(get_logger(),
              "landing completion requester='%s' mission=%llu success=%s "
              "accepted=%s reason='%s'",
              request.drone_id.c_str(),
              static_cast<unsigned long long>(request.mission_id),
              request.success ? "true" : "false",
              decision.accepted ? "true" : "false", decision.reason.c_str());
}

}  // namespace landing_coordinator
