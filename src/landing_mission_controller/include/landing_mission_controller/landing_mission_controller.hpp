#pragma once

#include "landing_mission_controller/mission_policy.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <landing_coordinator/msg/landing_status.hpp>
#include <landing_coordinator/srv/request_landing.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

namespace landing_mission_controller {

class LandingMissionController final : public rclcpp::Node {
public:
  explicit LandingMissionController(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  [[nodiscard]] MissionState state() const noexcept { return policy_.state(); }
  [[nodiscard]] std::uint64_t missionId() const noexcept { return policy_.missionId(); }

private:
  using LandingStatus = landing_coordinator::msg::LandingStatus;
  using RequestLanding = landing_coordinator::srv::RequestLanding;
  using Clock = std::chrono::steady_clock;

  static bool isValidObservation(const geometry_msgs::msg::PoseStamped& observation);
  void onObservation(geometry_msgs::msg::PoseStamped::ConstSharedPtr observation);
  void onStatus(LandingStatus::ConstSharedPtr status);
  void onTimer();
  void requestLanding();
  void onResponse(rclcpp::Client<RequestLanding>::SharedFuture future);
  void logTransition(MissionState before) const;

  const std::string drone_id_;
  const std::string target_topic_;
  const std::string coordinator_service_;
  const std::string status_topic_;
  const std::chrono::milliseconds request_delay_;
  const bool require_approach_ready_;
  bool approach_ready_{false};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr approach_sub_;
  const std::chrono::milliseconds observation_timeout_;
  const std::chrono::milliseconds response_timeout_;

  MissionPolicy policy_;
  std::int64_t pending_request_id_{-1};
  Clock::time_point request_sent_at_{};

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Subscription<LandingStatus>::SharedPtr status_sub_;
  rclcpp::Client<RequestLanding>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace landing_mission_controller
