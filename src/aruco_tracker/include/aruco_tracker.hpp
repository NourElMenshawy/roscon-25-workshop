#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "aruco_detector.hpp"

namespace aruco_tracker {

class ArucoTrackerNode final : public rclcpp::Node {
public:
  ArucoTrackerNode();

private:
  struct CalibrationState {
    ArucoDetector::CameraCalibration calibration;
    std::string frame_id;
  };

  void handle_camera_info(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);

  void handle_image(sensor_msgs::msg::Image::ConstSharedPtr msg);

  [[nodiscard]]
  std::optional<CalibrationState> calibration_snapshot() const;

  [[nodiscard]]
  geometry_msgs::msg::PoseStamped
  make_pose_message(const ArucoDetector::MarkerObservation &observation,
                    const std_msgs::msg::Header &image_header,
                    const std::string &fallback_frame) const;

  std::unique_ptr<ArucoDetector> detector_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
      camera_info_subscription_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      target_pose_publisher_;

  mutable std::mutex calibration_mutex_;

  std::optional<CalibrationState> calibration_;
};

} // namespace aruco_tracker
