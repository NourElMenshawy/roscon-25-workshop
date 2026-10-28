// ============================================================================
// ORIGINAL VERSION - CustomMode_v2.hpp
// ============================================================================
#pragma once

// PX4 Interface Library
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/odometry/angular_velocity.hpp>
#include <px4_ros2/odometry/attitude.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/utils/geometry.hpp>

// ROS 2 Core
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <exec/async_scope.hpp>
#include <exec/static_thread_pool.hpp>
#include <stdexec/execution.hpp>

#include <cstdint>
#include <mutex>
#include <optional>
// C++ Std
#include <Eigen/Eigen>
#include <atomic>
#include <chrono>
#include <cmath> // for M_PI
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

class CustomWaypoints : public px4_ros2::ModeBase {
public:
  explicit CustomWaypoints(rclcpp::Node &node);

  // See ModeBase
  void onActivate() override;
  void onDeactivate() override;
  void updateSetpoint([[maybe_unused]] float dt_s) override;

private:
  void loadParameters();
  // ROS 2
  rclcpp::Node &_node;

  // px4_ros2_cpp
  std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;
  std::shared_ptr<px4_ros2::OdometryLocalPosition> _local_position;

  std::vector<Eigen::Vector3f>
      _trajectory_waypoints;      // Vector to hold waypoints
  size_t _current_waypoint_index; // Index of the current waypoint
};
// PrecisionLand
class PrecisionLand : public px4_ros2::ModeBase {
public:
  explicit PrecisionLand(rclcpp::Node &node);
  ~PrecisionLand() override;

  void
  targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr &msg);
  void vehicleLandDetectedCallback(
      const px4_msgs::msg::VehicleLandDetected::SharedPtr &msg);

  // See ModeBase
  void onActivate() override;
  void onDeactivate() override;
  void updateSetpoint(float dt_s) override;

private:
  struct ArucoTag {
    // Initialize position with NaN values directly in the struct
    Eigen::Vector3d position =
        Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Quaterniond orientation;
    rclcpp::Time timestamp;

    bool valid() { return timestamp.nanoseconds() > 0; };
  };

  struct PoseSnapshot {
    ArucoTag tag_camera;

    Eigen::Vector3d vehicle_position_ned;
    Eigen::Quaterniond vehicle_attitude_ned;
  };
  void loadParameters();

  static ArucoTag getTagWorld(const PoseSnapshot &snapshot);

  Eigen::Vector2f calculateVelocitySetpointXY();
  bool checkTargetTimeout();
  bool positionReached(const Eigen::Vector3f &target) const;
  struct PendingTarget {
    ArucoTag world_tag;
    std::optional<rclcpp::Time> image_stamp;
    std::uint64_t generation;
    std::uint64_t sequence;
  };

  // Shared only between the ROS callbacks and worker.
  std::mutex _target_mutex;
  std::optional<PendingTarget> _pending_target;

  bool _processing_in_flight = false;
  std::uint64_t _generation = 0;

  // The pool must outlive operations tracked by the scope.
  exec::static_thread_pool _target_pool{1};
  exec::async_scope _target_scope;

  void consumePendingTarget();

  enum class State {
    Idle,
    Search,   // Searches for target using a search pattern
    Approach, // Positioning over landing target while maintaining altitude
    Descend,  // Stay over landing target while descending
    Finished
  };

  enum class ProcessingMode { Sync, Stdexec };

  ProcessingMode _processing_mode{ProcessingMode::Stdexec};
  std::chrono::milliseconds _processing_delay{0};

  struct PendingObservation {
    PoseSnapshot snapshot;
    std::optional<rclcpp::Time> image_stamp;
    std::uint64_t sequence;
    std::uint64_t generation;
    std::chrono::steady_clock::time_point enqueued_at;
  };
  // Benchmark counters.
  std::atomic<std::uint64_t> _frames_received{0};
  std::atomic<std::uint64_t> _frames_completed{0};
  std::atomic<std::uint64_t> _frames_applied{0};
  std::atomic<std::uint64_t> _frames_dropped_busy{0};
  std::atomic<std::uint64_t> _processing_errors{0};

  // All durations are microseconds.
  std::atomic<std::int64_t> _last_callback_us{0};
  std::atomic<std::int64_t> _last_pipeline_us{0};

  std::optional<PendingObservation> _waiting_observation;

  std::uint64_t _next_observation_sequence{0};
  std::uint64_t _last_applied_sequence{0};

  std::atomic<std::uint64_t> _pending_replaced{0};
  std::atomic<std::uint64_t> _results_rejected_stale{0};
  std::atomic<std::int64_t> _last_queue_wait_us{0};

  std::optional<std::chrono::steady_clock::time_point> _last_control_tick;

  double _max_control_wall_ms{0.0};
  std::uint64_t _control_gaps_over_30ms{0};

  void dispatchWaitingObservation();
  // Accessed only by the serialized mode-control path.
  double _max_update_dt_ms{0.0};
  void switchToState(State state);
  std::string stateName(State state);

  // ros2
  rclcpp::Node &_node;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      _target_pose_sub;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr
      _vehicle_land_detected_sub;

  // px4_ros2_cpp
  std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
  std::shared_ptr<px4_ros2::OdometryAttitude> _vehicle_attitude;
  std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;

  // Data
  State _state = State::Search;
  bool _search_started = false;

  ArucoTag _tag;
  std::optional<rclcpp::Time> _last_image_stamp;
  float _approach_altitude = {};

  // Land detection
  bool _land_detected = false;
  bool _target_lost_prev = true;

  // Waypoints for Search pattern
  std::vector<Eigen::Vector3f> _search_waypoints;
  // Search pattern generation
  void generateSearchWaypoints();
  // Search pattern index
  int _search_waypoint_index = 0;

  // Parameters
  float _param_descent_vel = {};
  float _param_vel_p_gain = {};
  float _param_vel_i_gain = {};
  float _param_max_velocity = {};
  float _param_target_timeout = {};
  float _param_delta_position = {};
  float _param_delta_velocity = {};

  float _vel_x_integral{};
  float _vel_y_integral{};
};
