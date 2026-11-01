#pragma once

#include "CustomMode_v2.hpp"
#include "precision_land_executor/follower_strategy_selector.hpp"
#include "precision_land_executor/shared_landing_site.hpp"
#include "precision_land_executor/tf_site_resolver.hpp"

#include <landing_coordinator/msg/landing_status.hpp>
#include <landing_coordinator/msg/landing_site_map.hpp>
#include <landing_coordinator/srv/report_landing_complete.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <std_msgs/msg/bool.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

class CustomModeExecutor : public px4_ros2::ModeExecutorBase {
public:
  CustomModeExecutor(px4_ros2::ModeBase& owned_mode,
                     px4_ros2::ModeBase& second_mode);

  void onActivate() override;
  void onDeactivate(DeactivateReason reason) override;

private:
  using LandingStatus = landing_coordinator::msg::LandingStatus;
  using LandingSiteMap = landing_coordinator::msg::LandingSiteMap;
  using ReportLandingComplete = landing_coordinator::srv::ReportLandingComplete;
  using VehicleLandDetected = px4_msgs::msg::VehicleLandDetected;
  using VehicleStatus = px4_msgs::msg::VehicleStatus;
  using VehicleCommand = px4_msgs::msg::VehicleCommand;
  using VehicleCommandAck = px4_msgs::msg::VehicleCommandAck;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using Clock = std::chrono::steady_clock;

  enum class State { Idle, PrecisionLand, WaitUntilDisarmed };

  void onLandingStatus(LandingStatus::ConstSharedPtr status);
  void onLandingSites(LandingSiteMap::ConstSharedPtr sites);
  void onTargetPose(PoseStamped::ConstSharedPtr observation);
  void onVehicleCommandAck(VehicleCommandAck::ConstSharedPtr ack);
  void onVehicleLandDetected(VehicleLandDetected::ConstSharedPtr event);
  void onVehicleStatus(VehicleStatus::ConstSharedPtr status);
  void tryReportCompletion();
  void onCompletionResponse(rclcpp::Client<ReportLandingComplete>::SharedFuture future);
  void tryActivatePrimary();
  void tryActivateApproach();
  void beginPrecisionLanding();
  void onApproachReady(std_msgs::msg::Bool::ConstSharedPtr ready);
  void startFollowerSelection();
  void maybeStartFollowerSelection();
  void pollFollowerSelection();
  void beginSecondaryPositionLanding(const precision_land_executor::SiteEvaluation& site);
  void tryReportSecondaryLanding();
  precision_land_executor::SiteEvaluation evaluateFollowerSites();
  void switchToState(State state, px4_ros2::Result previous_result);

  px4_ros2::ModeBase& _second_mode;

  std::string _drone_id;
  int _px4_system_id{1};
  bool _enable_auto_switch{false};
  bool _approach_enabled{false};
  bool _enable_position_landing{false};
  bool _secondary_started{false};
  bool _secondary_reported{false};
  bool _approach_activation_attempted{false};
  bool _route_airborne{false};
  bool _approach_ready{false};
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
  std::optional<Clock::time_point> _completion_seen_at;
  bool _follower_selection_started{false};
  bool _tf_lookup_pending{false};
  std::optional<Clock::time_point> _last_tf_retry;
  std::optional<precision_land_executor::SiteCatalog> _site_catalog;
  std::string _site_frame_id;
  std::string _local_frame_id;
  double _site_separation_margin_m{2.0};

  rclcpp::Subscription<LandingStatus>::SharedPtr _status_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _approach_ready_sub;
  rclcpp::Subscription<LandingSiteMap>::SharedPtr _sites_sub;
  rclcpp::Subscription<PoseStamped>::SharedPtr _target_sub;
  rclcpp::Subscription<VehicleCommandAck>::SharedPtr _command_ack_sub;
  rclcpp::Publisher<VehicleCommand>::SharedPtr _command_pub;
  rclcpp::Subscription<VehicleLandDetected>::SharedPtr _land_detected_sub;
  rclcpp::Subscription<VehicleStatus>::SharedPtr _vehicle_status_sub;
  rclcpp::Client<ReportLandingComplete>::SharedPtr _completion_client;
  rclcpp::TimerBase::SharedPtr _activation_timer;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _strategy_pub;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr _secondary_landed_pub;
  precision_land_executor::FollowerStrategySelector _follower_selector;
  std::shared_ptr<tf2_ros::Buffer> _tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> _tf_listener;
};
