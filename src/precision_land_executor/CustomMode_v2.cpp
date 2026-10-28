// ============================================================================
// ORIGINAL VERSION - CustomMode_v2.cpp
// ============================================================================
#include "CustomMode_v2.hpp"

#include <px4_ros2/components/node_with_mode.hpp>

static const std::string kModeNameCustomWaypoints = "CustomWaypoints";
static const std::string kModeNamePrecisionLandCustom = "PrecisionLandCustom";

CustomWaypoints::CustomWaypoints(rclcpp::Node &node)
    : px4_ros2::ModeBase(node, kModeNameCustomWaypoints), _node(node) {
  loadParameters();

  _trajectory_setpoint =
      std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
  _local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);

  RCLCPP_INFO(node.get_logger(), "CustomWaypoints mode initialized.");
}

void CustomWaypoints::loadParameters() {
  // Load parameters specific to the CustomWaypoints mode
}

void CustomWaypoints::onActivate() {
  // Initialize waypoints
  _trajectory_waypoints.emplace_back(1.0f, 2.5f, -3.0f);
  _trajectory_waypoints.emplace_back(14.0f, 2.5f, -3.0f);
  _trajectory_waypoints.emplace_back(14.0f, 8.0f, -3.0f);
  _trajectory_waypoints.emplace_back(0.0f, 0.0f, -3.0f);

  _current_waypoint_index = 0; // Start at the first waypoint
  RCLCPP_INFO(_node.get_logger(), "CustomWaypoints mode activated");
  // Set initial trajectory setpoint
}
void CustomWaypoints::onDeactivate() {
  RCLCPP_INFO(_node.get_logger(), "CustomWaypoints mode deactivated");
  // Reset trajectory setpoint
}
void CustomWaypoints::updateSetpoint([[maybe_unused]] float dt_s) {
  if (_current_waypoint_index < _trajectory_waypoints.size()) {
    // Set the trajectory setpoint to the current waypoint
    auto current_waypoint = _trajectory_waypoints[_current_waypoint_index];
    _trajectory_setpoint->updatePosition(current_waypoint);

    // Check if we reached the current waypoint
    if ((_local_position->positionNed() - current_waypoint).norm() < 0.5f) {
      _current_waypoint_index++; // Move to the next waypoint
    }
  } else {
    // All waypoints completed, reset or stop
    RCLCPP_INFO(_node.get_logger(), "All waypoints completed.");
    completed(px4_ros2::Result::Success);
    return; // Exit the update loop
  }
}

PrecisionLand::PrecisionLand(rclcpp::Node &node)
    : ModeBase(node, kModeNamePrecisionLandCustom), _node(node) {
  _trajectory_setpoint =
      std::make_shared<px4_ros2::TrajectorySetpointType>(*this);

  _vehicle_local_position =
      std::make_shared<px4_ros2::OdometryLocalPosition>(*this);

  _vehicle_attitude = std::make_shared<px4_ros2::OdometryAttitude>(*this);

  _target_pose_sub = _node.create_subscription<geometry_msgs::msg::PoseStamped>(
      "/target_pose", rclcpp::QoS(1).best_effort(),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        targetPoseCallback(msg);
      });

  _vehicle_land_detected_sub =
      _node.create_subscription<px4_msgs::msg::VehicleLandDetected>(
          "/px4_0/fmu/out/vehicle_land_detected", rclcpp::QoS(1).best_effort(),
          [this](const px4_msgs::msg::VehicleLandDetected::SharedPtr msg) {
            vehicleLandDetectedCallback(msg);
          });

  loadParameters();

  const std::string mode =
      _node.has_parameter("processing_mode")
          ? _node.get_parameter("processing_mode").as_string()
          : _node.declare_parameter<std::string>("processing_mode", "stdexec");

  const std::int64_t delay_ms =
      _node.has_parameter("processing_delay_ms")
          ? _node.get_parameter("processing_delay_ms").as_int()
          : _node.declare_parameter<std::int64_t>("processing_delay_ms", 0);

  if (mode == "sync") {
    _processing_mode = ProcessingMode::Sync;
  } else if (mode == "stdexec") {
    _processing_mode = ProcessingMode::Stdexec;
  } else {
    throw std::invalid_argument("processing_mode must be 'sync' or 'stdexec'");
  }

  if (delay_ms < 0 || delay_ms > 500) {
    throw std::invalid_argument(
        "processing_delay_ms must be between 0 and 500");
  }

  _processing_delay = std::chrono::milliseconds{delay_ms};

  RCLCPP_INFO(_node.get_logger(), "Pose processing: backend=%s, delay=%ld ms",
              mode.c_str(), static_cast<long>(delay_ms));
}

PrecisionLand::~PrecisionLand() {
  _target_scope.request_stop();

  // Wait until no worker can access this object.
  (void)stdexec::sync_wait(_target_scope.on_empty());
}
void PrecisionLand::loadParameters() {
  _node.declare_parameter<float>("descent_vel", 1.0);
  _node.declare_parameter<float>("vel_p_gain", 1.5);
  _node.declare_parameter<float>("vel_i_gain", 0.0);
  _node.declare_parameter<float>("max_velocity", 3.0);
  _node.declare_parameter<float>("target_timeout", 3.0);
  _node.declare_parameter<float>("delta_position", 0.25);
  _node.declare_parameter<float>("delta_velocity", 0.25);

  _node.get_parameter("descent_vel", _param_descent_vel);
  _node.get_parameter("vel_p_gain", _param_vel_p_gain);
  _node.get_parameter("vel_i_gain", _param_vel_i_gain);
  _node.get_parameter("max_velocity", _param_max_velocity);
  _node.get_parameter("target_timeout", _param_target_timeout);
  _node.get_parameter("delta_position", _param_delta_position);
  _node.get_parameter("delta_velocity", _param_delta_velocity);

  RCLCPP_INFO(_node.get_logger(), "descent_vel: %f", _param_descent_vel);
  RCLCPP_INFO(_node.get_logger(), "vel_i_gain: %f", _param_vel_i_gain);
}

void PrecisionLand::vehicleLandDetectedCallback(
    const px4_msgs::msg::VehicleLandDetected::SharedPtr &msg) {
  _land_detected = msg->landed;
}

void PrecisionLand::targetPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr &msg) {
  if (!_search_started) {
    return;
  }

  using Clock = std::chrono::steady_clock;
  const auto callback_started = Clock::now();

  ++_frames_received;

  const auto received_at = _node.now();

  const rclcpp::Time image_stamp(msg->header.stamp,
                                 _node.get_clock()->get_clock_type());

  const std::optional<rclcpp::Time> source_stamp =
      image_stamp.nanoseconds() > 0 ? std::optional<rclcpp::Time>{image_stamp}
                                    : std::nullopt;

  // Capture all input state before asynchronous processing.
  const PoseSnapshot snapshot{
      .tag_camera =
          ArucoTag{
              .position =
                  Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y,
                                  msg->pose.position.z),
              .orientation = Eigen::Quaterniond(
                  msg->pose.orientation.w, msg->pose.orientation.x,
                  msg->pose.orientation.y, msg->pose.orientation.z),
              .timestamp = received_at,
          },
      .vehicle_position_ned =
          _vehicle_local_position->positionNed().cast<double>(),
      .vehicle_attitude_ned = _vehicle_attitude->attitude().cast<double>(),
  };

  std::uint64_t sequence;
  std::uint64_t generation;

  {
    std::scoped_lock lock(_target_mutex);
    sequence = ++_next_observation_sequence;
    generation = _generation;
  }
  auto record_callback_duration = [&]() {
    _last_callback_us = std::chrono::duration_cast<std::chrono::microseconds>(
                            Clock::now() - callback_started)
                            .count();
  };

  // ---------------------------------------------------------
  // Backend 1: synchronous processing
  // ---------------------------------------------------------
  if (_processing_mode == ProcessingMode::Sync) {
    std::this_thread::sleep_for(_processing_delay);

    auto world_tag = getTagWorld(snapshot);

    {
      std::scoped_lock lock(_target_mutex);

      _pending_target = PendingTarget{.world_tag = std::move(world_tag),
                                      .image_stamp = source_stamp,
                                      .generation = _generation,
                                      .sequence = sequence};
    }

    ++_frames_completed;

    _last_pipeline_us = std::chrono::duration_cast<std::chrono::microseconds>(
                            Clock::now() - callback_started)
                            .count();

    record_callback_duration();
    return;
  }

  // ---------------------------------------------------------
  // Backend 2: stdexec with latest-observation-wins mailbox
  // ---------------------------------------------------------
  {
    std::scoped_lock lock(_target_mutex);

    if (_waiting_observation) {
      ++_pending_replaced;
    }

    _waiting_observation = PendingObservation{
        .snapshot = snapshot,
        .image_stamp = source_stamp,
        .sequence = sequence,
        .generation = generation,
        .enqueued_at = callback_started,
    };
  }

  record_callback_duration();
}

void PrecisionLand::dispatchWaitingObservation() {
  using Clock = std::chrono::steady_clock;

  if (_processing_mode != ProcessingMode::Stdexec || !_search_started) {
    return;
  }

  std::optional<PendingObservation> next;

  {
    std::scoped_lock lock(_target_mutex);

    if (_processing_in_flight || !_waiting_observation) {
      return;
    }

    if (_waiting_observation->generation != _generation) {
      _waiting_observation.reset();
      return;
    }

    next = std::move(_waiting_observation);
    _waiting_observation.reset();

    // Reserve the worker slot before unlocking.
    _processing_in_flight = true;
  }

  auto observation = std::move(*next);

  const auto scheduled_at = Clock::now();

  _last_queue_wait_us = std::chrono::duration_cast<std::chrono::microseconds>(
                            scheduled_at - observation.enqueued_at)
                            .count();

  try {
    auto pipeline =
        stdexec::schedule(_target_pool.get_scheduler())

        | stdexec::then([snapshot = std::move(observation.snapshot),
                         delay = _processing_delay]() {
            std::this_thread::sleep_for(delay);
            return getTagWorld(snapshot);
          })

        |
        stdexec::then([this, generation = observation.generation,
                       sequence = observation.sequence,
                       stamp = observation.image_stamp,
                       enqueued_at = observation.enqueued_at](ArucoTag result) {
          const auto pipeline_us =
              std::chrono::duration_cast<std::chrono::microseconds>(
                  Clock::now() - enqueued_at)
                  .count();

          std::scoped_lock lock(_target_mutex);

          const bool current_generation = generation == _generation;

          const bool newer_than_applied = sequence > _last_applied_sequence;

          const bool newer_than_pending =
              !_pending_target || sequence > _pending_target->sequence;

          if (current_generation && newer_than_applied && newer_than_pending) {
            _pending_target = PendingTarget{
                .world_tag = std::move(result),
                .image_stamp = stamp,
                .generation = generation,
                .sequence = sequence,
            };

            ++_frames_completed;
            _last_pipeline_us = pipeline_us;
          } else {
            ++_results_rejected_stale;
          }

          _processing_in_flight = false;
        })

        | stdexec::upon_error([this](auto &&) noexcept {
            std::scoped_lock lock(_target_mutex);
            ++_processing_errors;
            _processing_in_flight = false;
          })

        | stdexec::upon_stopped([this]() noexcept {
            std::scoped_lock lock(_target_mutex);
            _processing_in_flight = false;
          });

    _target_scope.spawn(std::move(pipeline));

  } catch (const std::exception &e) {
    {
      std::scoped_lock lock(_target_mutex);
      _processing_in_flight = false;
    }

    ++_processing_errors;

    RCLCPP_ERROR(_node.get_logger(), "Failed to dispatch pose processing: %s",
                 e.what());
  }
}
PrecisionLand::ArucoTag
PrecisionLand::getTagWorld(const PoseSnapshot &snapshot) {
  const ArucoTag &tag = snapshot.tag_camera;

  // Existing workshop camera rotation.
  Eigen::Matrix3d r;
  r << 0, -1, 0, 1, 0, 0, 0, 0, 1;

  const Eigen::Quaterniond quat_ned(r);

  // World <- Drone
  const Eigen::Affine3d drone_transform =
      Eigen::Translation3d(snapshot.vehicle_position_ned) *
      snapshot.vehicle_attitude_ned;

  // Drone <- Camera
  const Eigen::Affine3d camera_transform =
      Eigen::Translation3d(0, 0, 0) * quat_ned;

  // Camera <- Tag
  const Eigen::Affine3d tag_transform =
      Eigen::Translation3d(tag.position) * tag.orientation;

  // World <- Tag
  const Eigen::Affine3d tag_world_transform =
      drone_transform * camera_transform * tag_transform;

  return ArucoTag{
      .position = tag_world_transform.translation(),
      .orientation = Eigen::Quaterniond(tag_world_transform.rotation()),
      .timestamp = tag.timestamp,
  };
}

void PrecisionLand::consumePendingTarget() {
  std::optional<PendingTarget> pending;

  {
    std::scoped_lock lock(_target_mutex);

    if (!_pending_target) {
      return;
    }

    auto candidate = std::move(*_pending_target);
    _pending_target.reset();

    if (candidate.generation == _generation &&
        candidate.sequence > _last_applied_sequence) {
      _last_applied_sequence = candidate.sequence;
      pending = std::move(candidate);
    } else {
      ++_results_rejected_stale;
    }
  }

  if (!pending) {
    return;
  }

  // Only the serialized controller path changes landing state.
  _tag = std::move(pending->world_tag);
  _last_image_stamp = pending->image_stamp;

  ++_frames_applied;
}

void PrecisionLand::onActivate() {

  _frames_received = 0;
  _frames_completed = 0;
  _frames_applied = 0;
  _frames_dropped_busy = 0;
  _processing_errors = 0;

  _last_callback_us = 0;
  _last_pipeline_us = 0;
  _max_update_dt_ms = 0.0;
  {
    std::scoped_lock lock(_target_mutex);
    ++_generation;
    _waiting_observation.reset();
    _pending_target.reset();
    _last_applied_sequence = 0;
  }

  _pending_replaced = 0;
  _results_rejected_stale = 0;
  _last_queue_wait_us = 0;

  _last_control_tick.reset();
  _max_control_wall_ms = 0.0;
  _control_gaps_over_30ms = 0;
  // Reset data from the previous activation.
  _search_started = false;
  _tag = ArucoTag();
  _last_image_stamp.reset();

  _search_waypoint_index = 0;
  _vel_x_integral = 0.0f;
  _vel_y_integral = 0.0f;

  _land_detected = false;
  _target_lost_prev = true;

  generateSearchWaypoints();

  _search_started = true;
  switchToState(State::Search);
}
void PrecisionLand::onDeactivate() {
  _search_started = false;

  std::scoped_lock lock(_target_mutex);
  _waiting_observation.reset();
  ++_generation;
  _pending_target.reset();
}

void PrecisionLand::updateSetpoint([[maybe_unused]] float dt_s) {
  // Integrate the latest finished asynchronous operation.

  const auto control_now = std::chrono::steady_clock::now();

  if (_last_control_tick) {
    const double wall_dt_ms = std::chrono::duration<double, std::milli>(
                                  control_now - *_last_control_tick)
                                  .count();

    _max_control_wall_ms = std::max(_max_control_wall_ms, wall_dt_ms);

    if (wall_dt_ms > 30.0) {
      ++_control_gaps_over_30ms;
    }
  }

  _last_control_tick = control_now;

  // Consume completed results before starting another job.
  consumePendingTarget();

  // Start the latest waiting job if the worker is free.
  dispatchWaitingObservation();

  // Continue with your existing state machine.
  _max_update_dt_ms =
      std::max(_max_update_dt_ms, static_cast<double>(dt_s) * 1000.0);

  RCLCPP_INFO_THROTTLE(
      _node.get_logger(), *_node.get_clock(), 3000,
      "BENCH backend=%s delay=%lldms "
      "rx=%llu completed=%llu applied=%llu "
      "busy_drop=%llu errors=%llu "
      "callback=%.3fms pipeline=%.3fms "
      "max_control_dt=%.2fms "
      "pending_replaced=%llu stale_rejected=%llu "
      "queue_wait=%.3fms max_wall_dt=%.2fms "
      "wall_gaps_gt30=%llu",
      _processing_mode == ProcessingMode::Sync ? "sync" : "stdexec",
      static_cast<long long>(_processing_delay.count()),
      static_cast<unsigned long long>(_frames_received.load()),
      static_cast<unsigned long long>(_frames_completed.load()),
      static_cast<unsigned long long>(_frames_applied.load()),
      static_cast<unsigned long long>(_frames_dropped_busy.load()),
      static_cast<unsigned long long>(_processing_errors.load()),
      _last_callback_us.load() / 1000.0, _last_pipeline_us.load() / 1000.0,
      _max_update_dt_ms,
      static_cast<unsigned long long>(_pending_replaced.load()),
      static_cast<unsigned long long>(_results_rejected_stale.load()),
      _last_queue_wait_us.load() / 1000.0, _max_control_wall_ms,
      static_cast<unsigned long long>(_control_gaps_over_30ms));

  bool const target_lost = checkTargetTimeout();
  if (!target_lost && _last_image_stamp.has_value() &&
      (_state == State::Approach || _state == State::Descend)) {

    const double target_age_ms =
        (_node.now() - *_last_image_stamp).seconds() * 1000.0;

    RCLCPP_INFO_THROTTLE(_node.get_logger(), *_node.get_clock(), 1000,
                         "Controller target age: %.2f ms", target_age_ms);
  }
  if (target_lost && !_target_lost_prev) {
    RCLCPP_INFO(_node.get_logger(), "Target lost: State %s",
                stateName(_state).c_str());

  } else if (!target_lost && _target_lost_prev) {
    RCLCPP_INFO(_node.get_logger(), "Target acquired");
  }

  _target_lost_prev = target_lost;

  // State machine
  switch (_state) {
  case State::Idle: {
    // No-op -- just spin
    break;
  }

  case State::Search: {
    if (!target_lost) {
      _approach_altitude = _vehicle_local_position->positionNed().z();
      switchToState(State::Approach);
      break;
    }

    auto waypoint_position = _search_waypoints[_search_waypoint_index];

    _trajectory_setpoint->updatePosition(waypoint_position);

    if (positionReached(waypoint_position)) {
      _search_waypoint_index++;

      // If we have searched all waypoints, start over
      if (_search_waypoint_index >=
          static_cast<int>(_search_waypoints.size())) {
        _search_waypoint_index = 0;
      }
    }

    break;
  }

  case State::Approach: {
    if (target_lost) {
      RCLCPP_INFO(_node.get_logger(), "Failed! Target lost during %s",
                  stateName(_state).c_str());
      ModeBase::completed(px4_ros2::Result::ModeFailureOther);
      switchToState(State::Idle);
      return;
    }

    // Approach using position setpoints
    auto target_position = Eigen::Vector3f(_tag.position.x(), _tag.position.y(),
                                           _approach_altitude);

    _trajectory_setpoint->updatePosition(target_position);

    if (positionReached(target_position)) {
      switchToState(State::Descend);
    }

    break;
  }

  case State::Descend: {
    if (target_lost) {
      RCLCPP_INFO(_node.get_logger(), "Failed! Target lost during %s",
                  stateName(_state).c_str());
      ModeBase::completed(px4_ros2::Result::ModeFailureOther);
      switchToState(State::Idle);
      return;
    }

    // Descend using velocity setpoints and P velocity controller for XY
    Eigen::Vector2f vel = calculateVelocitySetpointXY();
    _trajectory_setpoint->update(
        Eigen::Vector3f(vel.x(), vel.y(), _param_descent_vel), std::nullopt,
        px4_ros2::quaternionToYaw(_tag.orientation));

    if (_land_detected) {
      switchToState(State::Finished);
    }

    break;
  }

  case State::Finished: {
    ModeBase::completed(px4_ros2::Result::Success);
    break;
  }
  } // end switch/case
}

Eigen::Vector2f PrecisionLand::calculateVelocitySetpointXY() {
  float const p_gain = _param_vel_p_gain;
  float const i_gain = _param_vel_i_gain;

  // P component
  float const delta_pos_x =
      _vehicle_local_position->positionNed().x() - _tag.position.x();
  float const delta_pos_y =
      _vehicle_local_position->positionNed().y() - _tag.position.y();

  // I component
  _vel_x_integral += delta_pos_x;
  _vel_y_integral += delta_pos_y;
  float const max_integral = _param_max_velocity;
  _vel_x_integral =
      std::clamp(_vel_x_integral, -1.f * max_integral, max_integral);
  _vel_y_integral =
      std::clamp(_vel_y_integral, -1.f * max_integral, max_integral);

  float const xp = delta_pos_x * p_gain;
  float const xi = _vel_x_integral * i_gain;
  float const yp = delta_pos_y * p_gain;
  float const yi = _vel_y_integral * i_gain;

  // Sum P and I gains
  float vx = -1.f * (xp + xi);
  float vy = -1.f * (yp + yi);

  // 0.1m/s min vel and 3m/s max vel
  vx = std::clamp(vx, -1.f * _param_max_velocity, _param_max_velocity);
  vy = std::clamp(vy, -1.f * _param_max_velocity, _param_max_velocity);

  return Eigen::Vector2f(vx, vy);
}

bool PrecisionLand::checkTargetTimeout() {
  if (!_tag.valid()) {
    return true;
  }

  if (_node.now().seconds() - _tag.timestamp.seconds() >
      _param_target_timeout) {
    return true;
  }

  return false;
}

void PrecisionLand::generateSearchWaypoints() {
  // Generate spiral search waypoints
  // The search waypoints are generated in the NED frame
  // Parameters for the search pattern
  double const start_x = _vehicle_local_position->positionNed().x();
  double const start_y = _vehicle_local_position->positionNed().y();
  double current_z = _vehicle_local_position->positionNed().z();
  auto min_z = -1.0;

  double const max_radius = 1.0;
  double const layer_spacing = 0.5;
  int const points_per_layer = 16;
  std::vector<Eigen::Vector3f> waypoints;

  // Generate waypoints
  // Calculate the number of layers needed
  int const num_layers =
      (static_cast<int>((min_z - current_z) / layer_spacing) / 2) < 1
          ? 1
          : (static_cast<int>((min_z - current_z) / layer_spacing) / 2);

  // Generate waypoints
  for (int layer = 0; layer < num_layers; ++layer) {
    std::vector<Eigen::Vector3f> layer_waypoints;

    // Spiral out to max radius
    double radius = 0.0;

    for (int point = 0; point < points_per_layer + 1; ++point) {
      double const angle = 2.0 * M_PI * point / points_per_layer;
      double const x = start_x + radius * cos(angle);
      double const y = start_y + radius * sin(angle);
      double const z = current_z;

      layer_waypoints.emplace_back(x, y, z);
      radius += max_radius / points_per_layer;
    }

    // Push the spiral out waypoints to the main waypoints vector
    waypoints.insert(waypoints.end(), layer_waypoints.begin(),
                     layer_waypoints.end());

    // Decrease the altitude for the inward spiral
    current_z += layer_spacing;

    // Reverse the layer waypoints for spiral in
    std::reverse(layer_waypoints.begin(), layer_waypoints.end());

    // Adjust the z-coordinate for the inward spiral
    for (auto &waypoint : layer_waypoints) {
      waypoint.z() = current_z;
    }

    // Push the reversed waypoints to the main waypoints vector
    waypoints.insert(waypoints.end(), layer_waypoints.begin(),
                     layer_waypoints.end());

    // Decrease the altitude for the next outward spiral
    current_z += layer_spacing;
  }

  _search_waypoints = waypoints;
}

bool PrecisionLand::positionReached(const Eigen::Vector3f &target) const {
  auto position = _vehicle_local_position->positionNed();
  auto velocity = _vehicle_local_position->velocityNed();

  const auto delta_pos = target - position;
  // NOTE: this does NOT handle a moving target!
  return (delta_pos.norm() < _param_delta_position) &&
         (velocity.norm() < _param_delta_velocity);
}

std::string PrecisionLand::stateName(State state) {
  switch (state) {
  case State::Idle:
    return "Idle";

  case State::Search:
    return "Search";

  case State::Approach:
    return "Approach";

  case State::Descend:
    return "Descend";

  case State::Finished:
    return "Finished";

  default:
    return "Unknown";
  }
}

void PrecisionLand::switchToState(State state) {
  RCLCPP_INFO(_node.get_logger(), "Switching to %s", stateName(state).c_str());
  _state = state;
}
