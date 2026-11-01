#pragma once

#include <chrono>
#include <cstdint>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <landing_coordinator/msg/landing_status.hpp>
#include <landing_coordinator/srv/report_landing_complete.hpp>
#include <optional>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

#include "CustomMode_v2.hpp"
#include "precision_land_executor/follower_strategy_selector.hpp"

class CustomModeExecutor : public px4_ros2::ModeExecutorBase {
 public:
  CustomModeExecutor(px4_ros2::ModeBase& owned_mode, px4_ros2::ModeBase& second_mode);

  void onActivate() override;
  void onDeactivate(DeactivateReason reason) override;

 private:
  using LandingStatus = landing_coordinator::msg::LandingStatus;
  using ReportLandingComplete = landing_coordinator::srv::ReportLandingComplete;
  using VehicleLandDetected = px4_msgs::msg::VehicleLandDetected;
  using VehicleStatus = px4_msgs::msg::VehicleStatus;
  using VehicleCommand = px4_msgs::msg::VehicleCommand;
  using VehicleCommandAck = px4_msgs::msg::VehicleCommandAck;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using Clock = std::chrono::steady_clock;

  enum class State { Idle, PrecisionLand, WaitUntilDisarmed };

  void onLandingStatus(LandingStatus::ConstSharedPtr status);
  void onTargetPose(PoseStamped::ConstSharedPtr observation);
  void onVehicleCommandAck(VehicleCommandAck::ConstSharedPtr ack);
  void onVehicleLandDetected(VehicleLandDetected::ConstSharedPtr event);
  void onVehicleStatus(VehicleStatus::ConstSharedPtr status);
  void tryReportCompletion();
  void onCompletionResponse(rclcpp::Client<ReportLandingComplete>::SharedFuture future);
  void tryActivatePrimary();
  void startFollowerSelection();
  void pollFollowerSelection();
  void switchToState(State state, px4_ros2::Result previous_result);

  px4_ros2::ModeBase& _second_mode;

  std::string _drone_id;
  int _px4_system_id{1};
  bool _enable_auto_switch{false};
  int _target_freshness_ms{1500};
  std::uint64_t _mission_id{0};
  std::string _current_owner;
  bool _authorized_primary{false};
  bool _activation_requested{false};
  bool _precision_scheduled{false};
  bool _precision_started{false};
  bool _precision_succeeded{false};
  bool _observed_airborne{false};
  bool _touchdown_confirmed{false};
  bool _observed_armed{false};
  bool _landing_disarm_confirmed{false};
  bool _failsafe_observed{false};
  bool _vehicle_failsafe_active{false};
  // PX4 monotonic microsecond timestamps avoid accepting out-of-order status.
  std::uint64_t _last_vehicle_status_us{0};
  std::uint64_t _activation_status_floor_us{0};
  bool _completion_acknowledged{false};
  bool _mission_completed_observed{false};
  bool _completion_in_flight{false};
  std::int64_t _completion_request_id{-1};
  std::optional<Clock::time_point> _completion_sent_at;
  int _completion_response_timeout_ms{3000};
  State _state{State::Idle};
  std::optional<Clock::time_point> _last_valid_target;

  rclcpp::Subscription<LandingStatus>::SharedPtr _status_sub;
  rclcpp::Subscription<PoseStamped>::SharedPtr _target_sub;
  rclcpp::Subscription<VehicleCommandAck>::SharedPtr _command_ack_sub;
  rclcpp::Publisher<VehicleCommand>::SharedPtr _command_pub;
  rclcpp::Subscription<VehicleLandDetected>::SharedPtr _land_detected_sub;
  rclcpp::Subscription<VehicleStatus>::SharedPtr _vehicle_status_sub;
  rclcpp::Client<ReportLandingComplete>::SharedPtr _completion_client;
  rclcpp::TimerBase::SharedPtr _activation_timer;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _strategy_pub;
  precision_land_executor::FollowerStrategySelector _follower_selector;
};
