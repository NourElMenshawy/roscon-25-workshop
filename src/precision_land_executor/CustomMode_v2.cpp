// ============================================================================
// ORIGINAL VERSION - CustomMode_v2.cpp
// ============================================================================
#include "CustomMode_v2.hpp"

#include <exception>
#include <stdexcept>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/exceptions.hpp>

#include <px4_ros2/components/node_with_mode.hpp>

static const std::string kModeNameCustomWaypoints = "CustomWaypoints";
static const std::string kModeNamePrecisionLandCustom = "PrecisionLandCustom";

CustomWaypoints::CustomWaypoints(rclcpp::Node &node)
    : px4_ros2::ModeBase(node, kModeNameCustomWaypoints), _node(node)
{
  loadParameters();
  _trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
  _local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);

  if (_approach_enabled) {
    _ready_pub = _node.create_publisher<std_msgs::msg::Bool>(
        "approach/ready", rclcpp::QoS(1).reliable().transient_local());
    _coordinator_sub = _node.create_subscription<landing_coordinator::msg::LandingStatus>(
        "/landing_coordinator/status", rclcpp::QoS(1).reliable().transient_local(),
        [this](landing_coordinator::msg::LandingStatus::ConstSharedPtr status) {
          onCoordinatorStatus(status);
        });
    _tf_buffer = std::make_shared<tf2_ros::Buffer>(_node.get_clock());
    _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);
  }
  RCLCPP_INFO(node.get_logger(), "CustomWaypoints initialized (competition=%s)",
              _approach_enabled ? "true" : "false");
}

void CustomWaypoints::loadParameters()
{
  // NodeWithModeExecutor constructs the owned mode before its executor.
  _approach_enabled = _node.declare_parameter<bool>("approach_enabled", false);
  _drone_id = _node.declare_parameter<std::string>("drone_id", "");
  _world_frame = _node.declare_parameter<std::string>("approach_world_frame", "map");
  _local_frame = _node.declare_parameter<std::string>(
      "approach_local_frame", _node.get_namespace());
  if (!_local_frame.empty() && _local_frame.front() == '/') {
    _local_frame.erase(0, 1);
  }
  if (_local_frame.find("/odom_ned") == std::string::npos) {
    _local_frame += "/odom_ned";
  }
  _arrival_radius_m = _node.declare_parameter<double>(
      "approach_arrival_radius_m", 0.75);
  _route_world = _node.declare_parameter<std::vector<double>>(
      "approach_waypoints_map", std::vector<double>{});

  if (_approach_enabled &&
      (_drone_id.empty() || _world_frame.empty() || _local_frame.empty() ||
       !std::isfinite(_arrival_radius_m) || _arrival_radius_m <= 0.0 ||
       !precision_land_executor::validApproachRoute(_route_world))) {
    throw std::invalid_argument("Invalid competition approach route or parameters");
  }
}

bool CustomWaypoints::resolveWorldWaypoints()
{
  if (_route_transforms_resolved) {
    return true;
  }
  try {
    const auto transform = _tf_buffer->lookupTransform(
        _local_frame, _world_frame, rclcpp::Time(0));
    std::vector<Eigen::Vector3f> resolved;
    for (std::size_t i = 0; i < _route_world.size(); i += 3) {
      geometry_msgs::msg::PointStamped source;
      source.header.frame_id = _world_frame;
      source.point.x = _route_world[i];
      source.point.y = _route_world[i + 1];
      source.point.z = _route_world[i + 2];
      geometry_msgs::msg::PointStamped target;
      tf2::doTransform(source, target, transform);
      resolved.emplace_back(
          static_cast<float>(target.point.x),
          static_cast<float>(target.point.y),
          static_cast<float>(target.point.z));
    }
    _trajectory_waypoints = std::move(resolved);
    _route_transforms_resolved = true;
    RCLCPP_INFO(_node.get_logger(),
                "Approach route transformed %zu map waypoints into %s",
                _trajectory_waypoints.size(), _local_frame.c_str());
    return true;
  } catch (const tf2::TransformException& error) {
    RCLCPP_WARN_THROTTLE(_node.get_logger(), *_node.get_clock(), 2000,
                         "Waiting for approach transform: %s", error.what());
    return false;
  }
}

void CustomWaypoints::onCoordinatorStatus(
    landing_coordinator::msg::LandingStatus::ConstSharedPtr status)
{
  if (!_approach_enabled || !_route_active || _route_frozen ||
      status->state == landing_coordinator::msg::LandingStatus::AVAILABLE) {
    return;
  }
  if (status->owner_id != _drone_id) {
    // Freeze immediately on losing arbitration, even before the executor's
    // Hold-mode transition. No more route setpoints will be issued.
    _frozen_position = _local_position->positionNed();
    _route_frozen = true;
    RCLCPP_INFO(_node.get_logger(),
                "Follower abandoning approach; holding current position");
  }
}

void CustomWaypoints::onActivate()
{
  _current_waypoint_index = 0;
  _route_active = true;
  _route_ready = false;
  _route_frozen = false;
  _route_transforms_resolved = false;
  _trajectory_waypoints.clear();
  if (_approach_enabled) {
    std_msgs::msg::Bool ready;
    ready.data = false;
    _ready_pub->publish(ready);
    RCLCPP_INFO(_node.get_logger(), "Starting competition waypoint approach");
    return;
  }
  // Preserve the original optional standalone waypoint mode.
  _trajectory_waypoints.emplace_back(1.0f, 2.5f, -3.0f);
  _trajectory_waypoints.emplace_back(14.0f, 2.5f, -3.0f);
  _trajectory_waypoints.emplace_back(14.0f, 8.0f, -3.0f);
  _trajectory_waypoints.emplace_back(0.0f, 0.0f, -3.0f);
  RCLCPP_INFO(_node.get_logger(), "CustomWaypoints mode activated");
}

void CustomWaypoints::onDeactivate()
{
  _route_active = false;
  if (_approach_enabled && _ready_pub) {
    // A stale Transient Local ready=true must not authorize a new landing
    // request after QGC or PX4 has deactivated waypoint navigation.
    std_msgs::msg::Bool ready;
    ready.data = false;
    _ready_pub->publish(ready);
  }
  RCLCPP_INFO(_node.get_logger(), "CustomWaypoints mode deactivated");
}

void CustomWaypoints::updateSetpoint([[maybe_unused]] float dt_s)
{
  if (_approach_enabled) {
    if (_route_frozen) {
      _trajectory_setpoint->updatePosition(_frozen_position);
      return;
    }
    if (!resolveWorldWaypoints()) {
      // A missing transform must not cause uncontrolled navigation.
      _trajectory_setpoint->updatePosition(_local_position->positionNed());
      return;
    }
    if (_current_waypoint_index >= _trajectory_waypoints.size()) {
      // Keep the last staging setpoint active while awaiting ownership.
      _trajectory_setpoint->updatePosition(_trajectory_waypoints.back());
      if (!_route_ready) {
        _route_ready = true;
        std_msgs::msg::Bool ready;
        ready.data = true;
        _ready_pub->publish(ready);
        RCLCPP_INFO(_node.get_logger(),
                    "Approach ready: staging waypoint reached; arbitration enabled");
      }
      return;
    }
  }

  if (_current_waypoint_index < _trajectory_waypoints.size()) {
    const auto waypoint = _trajectory_waypoints[_current_waypoint_index];
    _trajectory_setpoint->updatePosition(waypoint);
    const float tolerance = _approach_enabled
                                ? static_cast<float>(_arrival_radius_m) : 0.5f;
    if ((_local_position->positionNed() - waypoint).norm() < tolerance) {
      ++_current_waypoint_index;
    }
  } else if (!_approach_enabled) {
    completed(px4_ros2::Result::Success);
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
      "target_pose", rclcpp::QoS(1).best_effort(),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        targetPoseCallback(msg);
      });

  _vehicle_land_detected_sub =
      _node.create_subscription<px4_msgs::msg::VehicleLandDetected>(
          "fmu/out/vehicle_land_detected", rclcpp::QoS(1).best_effort(),
          [this](const px4_msgs::msg::VehicleLandDetected::SharedPtr msg) {
            vehicleLandDetectedCallback(msg);
          });

  loadParameters();

  RCLCPP_INFO(_node.get_logger(), "ArUco pose processing: stdexec");
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

  // Snapshot PX4 state on the ROS callback, before scheduling worker computation.
  PoseSnapshot snapshot{
      .tag_camera =
          ArucoTag{
              .position = Eigen::Vector3d(msg->pose.position.x,
                                          msg->pose.position.y,
                                          msg->pose.position.z),
              .orientation = Eigen::Quaterniond(
                  msg->pose.orientation.w, msg->pose.orientation.x,
                  msg->pose.orientation.y, msg->pose.orientation.z),
              .timestamp = _node.now(),
          },
      .vehicle_position_ned =
          _vehicle_local_position->positionNed().cast<double>(),
      .vehicle_attitude_ned = _vehicle_attitude->attitude().cast<double>(),
  };

  std::scoped_lock lock(_target_mutex);
  _waiting_observation = PendingObservation{
      .snapshot = std::move(snapshot),
      .sequence = ++_next_observation_sequence,
      .generation = _generation,
  };
}

void PrecisionLand::dispatchWaitingObservation() {
  if (!_search_started) {
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
    _processing_in_flight = true;
  }

  auto observation = std::move(*next);
  try {
    auto pipeline =
        stdexec::schedule(_target_pool.get_scheduler())
        | stdexec::then([snapshot = std::move(observation.snapshot)]() {
            return getTagWorld(snapshot);
          })
        | stdexec::then([this, generation = observation.generation,
                         sequence = observation.sequence](ArucoTag result) {
            std::scoped_lock lock(_target_mutex);
            const bool current_generation = generation == _generation;
            const bool newer_than_applied = sequence > _last_applied_sequence;
            const bool newer_than_pending =
                !_pending_target || sequence > _pending_target->sequence;

            if (current_generation && newer_than_applied && newer_than_pending) {
              _pending_target = PendingTarget{
                  .world_tag = std::move(result),
                  .generation = generation,
                  .sequence = sequence,
              };
            }
            _processing_in_flight = false;
          })
        | stdexec::upon_error([this](auto &&) noexcept {
            {
              std::scoped_lock lock(_target_mutex);
              _processing_in_flight = false;
            }
            RCLCPP_ERROR(_node.get_logger(), "stdexec ArUco processing failed");
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
    RCLCPP_ERROR(_node.get_logger(), "Failed to schedule ArUco processing: %s",
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
    }
  }

  if (pending) {
    // Only the serialized controller path changes the landing target.
    _tag = std::move(pending->world_tag);
  }
}

void PrecisionLand::onActivate() {
  {
    std::scoped_lock lock(_target_mutex);
    ++_generation;
    _waiting_observation.reset();
    _pending_target.reset();
    _last_applied_sequence = 0;
  }

  _search_started = false;
  _tag = ArucoTag();
  _search_waypoint_index = 0;
  _vel_x_integral = 0.0f;
  _vel_y_integral = 0.0f;
  _land_detected = false;
  _mode_completion_sent = false;
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
  consumePendingTarget();
  dispatchWaitingObservation();

  const bool target_lost = checkTargetTimeout();
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
    if (!_mode_completion_sent) {
      _mode_completion_sent = true;
      ModeBase::completed(px4_ros2::Result::Success);
    }
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
