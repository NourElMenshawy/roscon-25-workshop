#include "CustomModeExecutor_v2.hpp"

#include <cmath>
#include <memory>
#include <stdexcept>

using CustomModeWithExecutor =
    px4_ros2::NodeWithModeExecutor<CustomModeExecutor, CustomWaypoints, PrecisionLand>;

static constexpr bool kEnableDebugOutput = true;

CustomModeExecutor::CustomModeExecutor(px4_ros2::ModeBase& owned_mode,
                                       px4_ros2::ModeBase& second_mode)
    : ModeExecutorBase(Settings{}, owned_mode), _second_mode(second_mode)
{
  auto& ros_node = node();

  _drone_id = ros_node.declare_parameter<std::string>("drone_id", "");
  _px4_system_id = ros_node.declare_parameter<int>("px4_system_id", 1);
  _enable_auto_switch = ros_node.declare_parameter<bool>("enable_auto_switch", false);
  _target_freshness_ms = ros_node.declare_parameter<int>("target_freshness_ms", 1500);
  _completion_response_timeout_ms =
      ros_node.declare_parameter<int>("completion_response_timeout_ms", 3000);

  if (_drone_id.empty() || _px4_system_id < 1 || _px4_system_id > 255 ||
      _target_freshness_ms <= 0 || _completion_response_timeout_ms <= 0) {
    throw std::invalid_argument("Invalid drone_id, px4_system_id, or target_freshness_ms");
  }

  // PX4 topics are RELATIVE to the node namespace: /px4_0 or /px4_1.
  _command_pub =
      ros_node.create_publisher<VehicleCommand>("fmu/in/vehicle_command", rclcpp::QoS(10));

  _command_ack_sub = ros_node.create_subscription<VehicleCommandAck>(
      "fmu/out/vehicle_command_ack", rclcpp::QoS(10).best_effort(),
      [this](VehicleCommandAck::ConstSharedPtr ack) { onVehicleCommandAck(ack); });

  _land_detected_sub = ros_node.create_subscription<VehicleLandDetected>(
      "fmu/out/vehicle_land_detected", rclcpp::QoS(1).best_effort(),
      [this](VehicleLandDetected::ConstSharedPtr event) { onVehicleLandDetected(event); });

  // PX4 1.18 publishes VehicleStatus MESSAGE_VERSION=4 on this topic.
  // This subscription remains active even when PX4 deactivates our mode executor.
  _vehicle_status_sub = ros_node.create_subscription<VehicleStatus>(
      "fmu/out/vehicle_status_v4", rclcpp::QoS(5).best_effort(),
      [this](VehicleStatus::ConstSharedPtr status) { onVehicleStatus(status); });

  // Diagnostic-only output: this topic is NOT consumed as a PX4 mode command.
  _strategy_pub = ros_node.create_publisher<std_msgs::msg::String>(
      "landing_strategy/decision", rclcpp::QoS(1).reliable().transient_local());

  _completion_client =
      ros_node.create_client<ReportLandingComplete>("/landing_coordinator/report_landing_complete");

  _target_sub = ros_node.create_subscription<PoseStamped>(
      "target_pose", rclcpp::QoS(10).best_effort(),
      [this](PoseStamped::ConstSharedPtr observation) { onTargetPose(observation); });

  _status_sub = ros_node.create_subscription<LandingStatus>(
      "/landing_coordinator/status", rclcpp::QoS(1).reliable().transient_local(),
      [this](LandingStatus::ConstSharedPtr status) { onLandingStatus(status); });

  // The library is registered by NodeWithModeExecutor before callbacks run.
  // A timer allows status to arrive before QGC arms the vehicle.
  _activation_timer = ros_node.create_wall_timer(std::chrono::milliseconds(200), [this] {
    tryActivatePrimary();
    tryReportCompletion();
    pollFollowerSelection();
  });

  RCLCPP_INFO(ros_node.get_logger(),
              "PX4 mission executor ready drone=%s namespace=%s auto_switch=%s", _drone_id.c_str(),
              ros_node.get_namespace(), _enable_auto_switch ? "true" : "false");
}

void CustomModeExecutor::onLandingStatus(LandingStatus::ConstSharedPtr status)
{
  if (status->state == LandingStatus::AVAILABLE) {
    if (_mission_id != 0) {
      RCLCPP_WARN(node().get_logger(),
                  "Coordinator returned AVAILABLE during mission %llu; "
                  "automatic authority is disabled (in-flight recovery not implemented)",
                  static_cast<unsigned long long>(_mission_id));
    }
    _authorized_primary = false;
    _follower_selector.cancel();
    return;
  }

  if ((status->state != LandingStatus::RESERVED && status->state != LandingStatus::COMPLETED) ||
      status->owner_id.empty() || status->mission_id == 0) {
    RCLCPP_WARN(node().get_logger(), "Ignoring invalid landing status");
    return;
  }

  // In this demo mission IDs are monotonic only while the coordinator runs.
  if (_mission_id > status->mission_id ||
      (_mission_id == status->mission_id && !_current_owner.empty() &&
       _current_owner != status->owner_id)) {
    RCLCPP_WARN(node().get_logger(), "Ignoring stale/conflicting landing owner");
    return;
  }

  if (status->mission_id != _mission_id) {
    _follower_selector.cancel();
    _activation_requested = false;
    _completion_acknowledged = false;
    _mission_completed_observed = false;
    // Never carry landing evidence across mission identifiers.
    _precision_started = false;
    _precision_succeeded = false;
    _observed_airborne = false;
    _touchdown_confirmed = false;
    _observed_armed = false;
    _landing_disarm_confirmed = false;
    _failsafe_observed = false;
    _mission_id = status->mission_id;
  }
  _current_owner = status->owner_id;

  if (status->state == LandingStatus::COMPLETED) {
    const bool first_completion_notice = !_mission_completed_observed;
    _mission_completed_observed = true;
    _authorized_primary = false;
    _completion_acknowledged = _current_owner == _drone_id;
    RCLCPP_INFO(node().get_logger(), "Mission %llu completed by %s; no further activation",
                static_cast<unsigned long long>(_mission_id), _current_owner.c_str());
    if (first_completion_notice && _current_owner != _drone_id) {
      startFollowerSelection();
    }
    return;
  }

  if (_mission_completed_observed) {
    // Do not regress from COMPLETED to a delayed RESERVED sample.
    return;
  }
  _authorized_primary = _current_owner == _drone_id;
  RCLCPP_INFO(node().get_logger(), "Mission %llu owner=%s role=%s",
              static_cast<unsigned long long>(_mission_id), _current_owner.c_str(),
              _authorized_primary ? "PRIMARY" : "FOLLOWER");
}

void CustomModeExecutor::onTargetPose(PoseStamped::ConstSharedPtr observation)
{
  const auto& p = observation->pose.position;
  const auto& q = observation->pose.orientation;
  const double quat_norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && std::isfinite(quat_norm) &&
      quat_norm > 1e-8) {
    _last_valid_target = Clock::now();
  }
}

void CustomModeExecutor::tryActivatePrimary()
{
  if (!_enable_auto_switch || !_authorized_primary || _activation_requested || !isArmed() ||
      isInCharge() || !_last_valid_target || _command_pub->get_subscription_count() == 0) {
    return;
  }

  if (Clock::now() - *_last_valid_target > std::chrono::milliseconds(_target_freshness_ms)) {
    return;
  }

  if (ownedMode().id() == px4_ros2::ModeBase::kModeIDInvalid) {
    return;
  }

  VehicleCommand cmd{};
  cmd.timestamp = 0;
  cmd.command = VehicleCommand::VEHICLE_CMD_SET_NAV_STATE;
  cmd.param1 = static_cast<float>(ownedMode().id());
  cmd.target_system = static_cast<std::uint8_t>(_px4_system_id);
  cmd.target_component = 1;
  cmd.source_system = 200;
  cmd.source_component = 1;
  cmd.from_external = true;

  // Request a switch into the executor's OWNED mode. The PX4 interface
  // library then calls onActivate(), where PrecisionLand is scheduled.
  // Deliberately a single request per mission: if PX4 rejects it or a GCS
  // overrides it, do not repeatedly seize control.
  _activation_requested = true;
  RCLCPP_INFO(node().get_logger(), "Requesting PX4 mode activation mission=%llu nav_state=%u",
              static_cast<unsigned long long>(_mission_id),
              static_cast<unsigned>(ownedMode().id()));
  _command_pub->publish(cmd);
}

void CustomModeExecutor::onVehicleCommandAck(VehicleCommandAck::ConstSharedPtr ack)
{
  if (!_activation_requested || ack->command != VehicleCommand::VEHICLE_CMD_SET_NAV_STATE ||
      ack->target_component != 1) {
    return;
  }
  RCLCPP_INFO(node().get_logger(), "PX4 activation command ACK result=%u",
              static_cast<unsigned>(ack->result));
}

void CustomModeExecutor::onActivate()
{
  RCLCPP_INFO(node().get_logger(), "CustomModeExecutor activated");

  // Fail closed: even if someone manually selects the owned mode,
  // do not begin landing without coordinator authorization.
  if (!_authorized_primary || _mission_id == 0 || !isArmed()) {
    RCLCPP_WARN(node().get_logger(),
                "No current landing grant for this drone; requesting PX4 Hold");
    scheduleMode(px4_ros2::ModeBase::kModeIDLoiter, [this](px4_ros2::Result result) {
      RCLCPP_INFO(node().get_logger(), "Hold result: %s", resultToString(result));
    });
    return;
  }

  if (_precision_scheduled) {
    return;
  }
  _precision_scheduled = true;
  _precision_started = true;
  _precision_succeeded = false;
  _observed_airborne = false;
  _touchdown_confirmed = false;
  _observed_armed = false;
  _landing_disarm_confirmed = false;
  _failsafe_observed = false;
  _activation_status_floor_us = _last_vehicle_status_us;
  switchToState(State::PrecisionLand, px4_ros2::Result::Success);
}

void CustomModeExecutor::onDeactivate(DeactivateReason reason)
{
  // PX4 may deactivate this executor as part of a normal landing/disarm.
  // Preserve mission evidence: VehicleStatus may arrive after this callback.
  if (_precision_started && reason == DeactivateReason::FailsafeActivated) {
    _failsafe_observed = true;
  }
  _precision_scheduled = false;
  _state = State::Idle;
  RCLCPP_WARN(node().get_logger(), "CustomModeExecutor deactivated (%s)",
              reason == DeactivateReason::FailsafeActivated ? "failsafe" : "other");
}

void CustomModeExecutor::switchToState(State state, px4_ros2::Result previous_result)
{
  if (previous_result != px4_ros2::Result::Success) {
    RCLCPP_ERROR(node().get_logger(), "Scheduled mode failed: %s; leaving control to PX4",
                 resultToString(previous_result));
    _precision_scheduled = false;
    _state = State::Idle;
    // If the custom mode failed and our executor still controls PX4,
    // request Hold rather than leave it executing a failed landing mode.
    // Never override a manual QGC/failsafe mode switch.
    if (isInCharge() && isArmed()) {
      scheduleMode(px4_ros2::ModeBase::kModeIDLoiter, [this](px4_ros2::Result hold_result) {
        RCLCPP_WARN(node().get_logger(), "Recovery Hold result: %s", resultToString(hold_result));
      });
    }
    return;
  }

  _state = state;
  switch (_state) {
    case State::PrecisionLand:
      RCLCPP_INFO(node().get_logger(), "PRIMARY: scheduling PrecisionLandCustom");
      scheduleMode(_second_mode.id(), [this](px4_ros2::Result result) {
        switchToState(State::WaitUntilDisarmed, result);
      });
      break;
    case State::WaitUntilDisarmed:
      _precision_succeeded = true;  // The precision mode itself returned success.
      waitUntilDisarmed([this](px4_ros2::Result result) {
        // 'Deactivated' is possible during normal PX4 auto-disarming, so
        // do not infer success or failure from this callback alone.
        RCLCPP_INFO(node().get_logger(),
                    "Precision landing disarm wait: %s (touchdown=%s); "
                    "awaiting independent PX4 landing-disarm confirmation",
                    resultToString(result), _touchdown_confirmed ? "true" : "false");
        // The wall timer reports only after mode success, observed touchdown,
        // and VehicleStatus confirms disarming by landing.
      });
      break;
    case State::Idle:
      break;
  }
}

void CustomModeExecutor::onVehicleLandDetected(VehicleLandDetected::ConstSharedPtr event)
{
  if (!_precision_started || _completion_acknowledged) {
    return;
  }
  // Require an airborne->landed observation during this particular activation;
  // a retained stale 'landed' event is not sufficient.
  if (!event->landed) {
    _observed_airborne = true;
  } else if (_observed_airborne) {
    _touchdown_confirmed = true;
  }
}

void CustomModeExecutor::onVehicleStatus(VehicleStatus::ConstSharedPtr status)
{
  // PX4 timestamps are monotonic within one SITL instance. Ignore samples
  // received out of order (or duplicate samples).
  if (status->timestamp == 0 || status->timestamp <= _last_vehicle_status_us) {
    return;
  }
  _last_vehicle_status_us = status->timestamp;
  _vehicle_failsafe_active = status->failsafe;
  if (_vehicle_failsafe_active) {
    _follower_selector.cancel();
  }

  if (!_precision_started || _completion_acknowledged ||
      status->timestamp <= _activation_status_floor_us) {
    return;
  }

  if (status->failsafe) {
    _failsafe_observed = true;
  }

  if (status->arming_state == VehicleStatus::ARMING_STATE_ARMED) {
    _observed_armed = true;
  } else if (status->arming_state == VehicleStatus::ARMING_STATE_DISARMED) {
    // Require the transition edge. A later status message with an updated
    // reason must not turn an unrelated disarm into a landing confirmation.
    if (_observed_armed &&
        status->latest_disarming_reason == VehicleStatus::ARM_DISARM_REASON_LANDING &&
        !_landing_disarm_confirmed) {
      _landing_disarm_confirmed = true;
      RCLCPP_INFO(node().get_logger(), "PX4 confirmed armed -> disarmed by landing (mission=%llu)",
                  static_cast<unsigned long long>(_mission_id));
    }
    _observed_armed = false;
  }
}

void CustomModeExecutor::tryReportCompletion()
{
  // A normal landing can deactivate ModeExecutorBase before waitUntilDisarmed
  // returns Success. Use independent PX4 state evidence instead.
  if (!_precision_started || !_precision_succeeded || !_touchdown_confirmed ||
      !_landing_disarm_confirmed || _failsafe_observed || _mission_id == 0 ||
      _current_owner != _drone_id || _completion_acknowledged) {
    return;
  }

  if (_completion_in_flight) {
    if (_completion_sent_at && Clock::now() - *_completion_sent_at >
                                   std::chrono::milliseconds(_completion_response_timeout_ms)) {
      _completion_client->remove_pending_request(_completion_request_id);
      _completion_request_id = -1;
      _completion_in_flight = false;
      _completion_sent_at.reset();
      RCLCPP_WARN(node().get_logger(), "Landing completion response timed out; will retry");
    }
    return;
  }

  if (!_completion_client->service_is_ready()) {
    return;
  }

  auto request = std::make_shared<ReportLandingComplete::Request>();
  request->mission_id = _mission_id;
  request->drone_id = _drone_id;
  request->success = true;
  try {
    _completion_in_flight = true;
    _completion_sent_at = Clock::now();
    auto pending = _completion_client->async_send_request(
        request, [this](rclcpp::Client<ReportLandingComplete>::SharedFuture future) {
          onCompletionResponse(future);
        });
    _completion_request_id = pending.request_id;
    RCLCPP_INFO(node().get_logger(), "Reporting confirmed touchdown + disarm mission=%llu drone=%s",
                static_cast<unsigned long long>(_mission_id), _drone_id.c_str());
  } catch (const std::exception& e) {
    _completion_in_flight = false;
    _completion_request_id = -1;
    _completion_sent_at.reset();
    RCLCPP_ERROR(node().get_logger(), "Cannot report landing completion: %s", e.what());
  }
}

void CustomModeExecutor::onCompletionResponse(
    rclcpp::Client<ReportLandingComplete>::SharedFuture future)
{
  _completion_in_flight = false;
  _completion_request_id = -1;
  _completion_sent_at.reset();
  try {
    const auto response = future.get();
    if (response->accepted && response->completed) {
      _completion_acknowledged = true;
      _mission_completed_observed = true;
      _authorized_primary = false;
      RCLCPP_INFO(node().get_logger(), "Landing completion recorded: %s", response->reason.c_str());
    } else {
      // Fail closed: a rejected report must not repeatedly flood the service.
      // The coordinator is authoritative; no second landing is unlocked.
      _completion_acknowledged = true;
      RCLCPP_ERROR(node().get_logger(), "Landing completion rejected: %s",
                   response->reason.c_str());
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node().get_logger(), "Landing completion response failed: %s", e.what());
    // Transient errors retry on the next timer tick; requests are idempotent.
  }
}

void CustomModeExecutor::startFollowerSelection()
{
  // The coordinator confirms the primary completed, but does NOT grant a
  // second landing site. Live capabilities therefore fail closed.
  precision_land_executor::StrategySnapshot snapshot{};
  snapshot.mission_id = _mission_id;
  snapshot.aruco_fresh = _last_valid_target && Clock::now() - *_last_valid_target <=
                                                   std::chrono::milliseconds(_target_freshness_ms);
  snapshot.vehicle_failsafe = _vehicle_failsafe_active;

  // Deliberately false until implemented and independently verified:
  //  - no shared position transformed into this PX4 vehicle's local frame,
  //  - no unoccupied secondary landing site has been authorized,
  //  - no separate visual landing estimator has been validated.
  snapshot.shared_position_available = false;
  snapshot.shared_position_frame_valid = false;
  snapshot.position_site_verified = false;
  snapshot.aruco_site_verified = false;
  snapshot.visual_estimator_ready = false;
  snapshot.visual_site_verified = false;

  if (!_follower_selector.start(snapshot)) {
    RCLCPP_ERROR(node().get_logger(), "Cannot schedule follower selection mission=%llu; SAFE_HOLD",
                 static_cast<unsigned long long>(_mission_id));
  } else {
    RCLCPP_INFO(node().get_logger(),
                "Follower selection scheduled with stdexec::when_all mission=%llu",
                static_cast<unsigned long long>(_mission_id));
  }
}

void CustomModeExecutor::pollFollowerSelection()
{
  auto result = _follower_selector.takeResult();
  if (!result) {
    return;
  }

  // Results are applied only on the ROS executor thread. Ignore work from an
  // old mission, an obsolete role, or a subsequent PX4 failsafe.
  if (!_mission_completed_observed || _mission_id != result->mission_id ||
      _current_owner == _drone_id || _vehicle_failsafe_active) {
    RCLCPP_WARN(node().get_logger(), "Discarding obsolete follower decision");
    return;
  }

  std_msgs::msg::String message;
  message.data = std::string{"mission="} + std::to_string(result->mission_id) +
                 " strategy=" + precision_land_executor::toString(result->strategy);
  _strategy_pub->publish(message);

  RCLCPP_INFO(node().get_logger(),
              "Follower decision mission=%llu selected=%s outcome=%s "
              "position=%s aruco=%s visual=%s (decision-only; PX4 unchanged)",
              static_cast<unsigned long long>(result->mission_id),
              precision_land_executor::toString(result->strategy),
              result->outcome == precision_land_executor::SelectionOutcome::Selected
                  ? "selected"
                  : "error_safe_hold",
              precision_land_executor::toString(result->position.reason),
              precision_land_executor::toString(result->aruco.reason),
              precision_land_executor::toString(result->visual.reason));
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  try {
    auto node_with_mode =
        std::make_shared<CustomModeWithExecutor>("CustomModeDemo", kEnableDebugOutput);
    rclcpp::spin(node_with_mode);
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("CustomModeDemo"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
