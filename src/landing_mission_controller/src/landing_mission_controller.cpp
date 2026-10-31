#include "landing_mission_controller/landing_mission_controller.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

using namespace std::chrono_literals;

namespace landing_mission_controller {

LandingMissionController::LandingMissionController(const rclcpp::NodeOptions& options)
    : Node("landing_mission_controller", options),
      drone_id_(declare_parameter<std::string>("drone_id", "")),
      target_topic_(declare_parameter<std::string>("target_topic", "target_pose")),
      coordinator_service_(declare_parameter<std::string>(
          "coordinator_service", "/landing_coordinator/request_landing")),
      request_delay_(declare_parameter<int>("request_delay_ms", 100)),
      observation_timeout_(declare_parameter<int>("observation_timeout_ms", 1500)),
      response_timeout_(declare_parameter<int>("response_timeout_ms", 3000)) {
  if (drone_id_.empty() || target_topic_.empty() || coordinator_service_.empty() ||
      request_delay_.count() < 0 || observation_timeout_.count() <= 0 ||
      response_timeout_.count() <= 0) {
    throw std::invalid_argument("Invalid drone_id/topic/service or timing parameters");
  }

  client_ = create_client<RequestLanding>(coordinator_service_);
  target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      target_topic_, rclcpp::QoS(1).best_effort(),
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr observation) {
        onObservation(std::move(observation));
      });
  timer_ = create_wall_timer(20ms, [this] { onTimer(); });

  RCLCPP_INFO(get_logger(), "Drone '%s' listening on '%s'; service='%s'; delay=%lld ms",
              drone_id_.c_str(), target_topic_.c_str(), coordinator_service_.c_str(),
              static_cast<long long>(request_delay_.count()));
}

bool LandingMissionController::isValidObservation(
    const geometry_msgs::msg::PoseStamped& observation) {
  const auto& p = observation.pose.position;
  const auto& q = observation.pose.orientation;
  const double qnorm2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  // The ArUco tracker publishes a camera-optical-frame pose, so Z must be forward.
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && p.z > 0.0 &&
         std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) &&
         std::isfinite(q.w) && std::isfinite(qnorm2) && qnorm2 > 1e-10;
}

void LandingMissionController::onObservation(
    geometry_msgs::msg::PoseStamped::ConstSharedPtr observation) {
  if (!isValidObservation(*observation)) {
    return;
  }
  const auto before = policy_.state();
  policy_.observeTag(Clock::now());
  logTransition(before);
}

void LandingMissionController::onTimer() {
  const auto now = Clock::now();
  const auto before = policy_.state();
  policy_.expireObservation(now, observation_timeout_);
  logTransition(before);

  if (policy_.state() == MissionState::REQUESTING &&
      now - request_sent_at_ > response_timeout_) {
    // A late response from this attempt must not change the state.
    client_->remove_pending_request(pending_request_id_);
    pending_request_id_ = -1;
    const auto previous = policy_.state();
    policy_.requestTimedOut();
    RCLCPP_WARN(get_logger(), "Landing request timed out; waiting for a fresh marker");
    logTransition(previous);
    return;
  }

  if (policy_.readyToRequest(now, request_delay_, observation_timeout_) &&
      client_->service_is_ready()) {
    requestLanding();
  }
}

void LandingMissionController::requestLanding() {
  const auto request = std::make_shared<RequestLanding::Request>();
  request->drone_id = drone_id_;

  const auto before = policy_.state();
  policy_.requestSent();
  request_sent_at_ = Clock::now();
  logTransition(before);

  try {
    auto pending = client_->async_send_request(
        request, [this](rclcpp::Client<RequestLanding>::SharedFuture future) {
          onResponse(std::move(future));
        });
    pending_request_id_ = pending.request_id;
    RCLCPP_INFO(get_logger(), "Requesting landing ownership: drone='%s'",
                drone_id_.c_str());
  } catch (const std::exception& error) {
    const auto previous = policy_.state();
    policy_.requestTimedOut();
    RCLCPP_ERROR(get_logger(), "Failed to send landing request: %s", error.what());
    logTransition(previous);
  }
}

void LandingMissionController::onResponse(
    rclcpp::Client<RequestLanding>::SharedFuture future) {
  if (policy_.state() != MissionState::REQUESTING) {
    return;
  }
  pending_request_id_ = -1;
  const auto response = future.get();
  const auto before = policy_.state();
  policy_.receiveDecision(response->granted, response->owner_id, drone_id_,
                          response->mission_id);
  RCLCPP_INFO(get_logger(), "Landing response: granted=%s owner='%s' mission=%llu reason='%s'",
              response->granted ? "true" : "false", response->owner_id.c_str(),
              static_cast<unsigned long long>(response->mission_id),
              response->reason.c_str());
  logTransition(before);
}

void LandingMissionController::logTransition(MissionState before) const {
  if (before != policy_.state()) {
    RCLCPP_INFO(get_logger(), "Mission state: %s -> %s", stateName(before),
                stateName(policy_.state()));
  }
}

}  // namespace landing_mission_controller
