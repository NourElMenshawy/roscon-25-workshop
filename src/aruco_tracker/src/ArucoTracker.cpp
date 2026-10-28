#include "aruco_tracker.hpp"

#include <cstdlib>
#include <exception>
#include <memory>
#include <utility>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

namespace aruco_tracker {

ArucoTrackerNode::ArucoTrackerNode() : Node{"aruco_tracker"} {
  const auto marker_id = declare_parameter<int>("marker_id", 0);

  const auto marker_size_m = declare_parameter<double>("marker_size_m", 0.5);

  const auto dictionary_id = declare_parameter<int>(
      "dictionary_id", static_cast<int>(cv::aruco::DICT_4X4_250));

  if (marker_id < 0) {
    throw std::invalid_argument("marker_id must be >= 0");
  }

  if (marker_size_m <= 0.0) {
    throw std::invalid_argument("marker_size_m must be > 0");
  }

  detector_ = std::make_unique<ArucoDetector>(
      marker_id, marker_size_m,
      static_cast<cv::aruco::PredefinedDictionaryType>(dictionary_id));

  target_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "target_pose", rclcpp::QoS{10});

  camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "camera_info", rclcpp::SensorDataQoS{},
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        handle_camera_info(std::move(msg));
      });

  image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      "camera", rclcpp::SensorDataQoS{},
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        handle_image(std::move(msg));
      });

  RCLCPP_INFO(get_logger(),
              "ArUco tracker started: "
              "marker_id=%d marker_size=%.3f m "
              "dictionary=%d",
              marker_id, marker_size_m, dictionary_id);
}

void ArucoTrackerNode::handle_camera_info(
    sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
  //
  // K:
  //
  // [ fx  0 cx ]
  // [  0 fy cy ]
  // [  0  0  1 ]
  //
  if (msg->k[0] <= 0.0 || msg->k[4] <= 0.0) {
    RCLCPP_WARN(get_logger(), "Received invalid CameraInfo: "
                              "fx or fy is <= 0");

    return;
  }

  ArucoDetector::CameraCalibration calibration;

  calibration.camera_matrix = cv::Matx33d{msg->k[0], msg->k[1], msg->k[2],

                                          msg->k[3], msg->k[4], msg->k[5],

                                          msg->k[6], msg->k[7], msg->k[8]};

  calibration.distortion_coefficients = msg->d;

  CalibrationState state{
      .calibration = std::move(calibration),
      .frame_id = msg->header.frame_id,
  };

  {
    std::scoped_lock lock{calibration_mutex_};

    calibration_ = std::move(state);
  }
}

void ArucoTrackerNode::handle_image(
    sensor_msgs::msg::Image::ConstSharedPtr msg) {
  const auto calibration = calibration_snapshot();

  if (!calibration) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Waiting for CameraInfo before "
                         "processing images");

    return;
  }

  cv_bridge::CvImageConstPtr cv_image;

  try {
    cv_image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
  } catch (const cv_bridge::Exception &exception) {
    RCLCPP_ERROR(get_logger(), "cv_bridge conversion failed: %s",
                 exception.what());

    return;
  }

  const auto observation =
      detector_->detect(cv_image->image, calibration->calibration);

  if (!observation) {
    return;
  }

  auto pose =
      make_pose_message(*observation, msg->header, calibration->frame_id);

  target_pose_publisher_->publish(std::move(pose));
}

std::optional<ArucoTrackerNode::CalibrationState>
ArucoTrackerNode::calibration_snapshot() const {
  std::scoped_lock lock{calibration_mutex_};

  return calibration_;
}

geometry_msgs::msg::PoseStamped ArucoTrackerNode::make_pose_message(
    const ArucoDetector::MarkerObservation &observation,
    const std_msgs::msg::Header &image_header,
    const std::string &fallback_frame) const {
  geometry_msgs::msg::PoseStamped pose;

  //
  // Preserve the image timestamp.
  //
  // The pose corresponds to what the camera saw
  // at this particular instant, not "now".
  //
  pose.header.stamp = image_header.stamp;

  if (!image_header.frame_id.empty()) {
    pose.header.frame_id = image_header.frame_id;
  } else {
    pose.header.frame_id = fallback_frame;
  }

  pose.pose.position.x = observation.translation[0];

  pose.pose.position.y = observation.translation[1];

  pose.pose.position.z = observation.translation[2];

  const auto &r = observation.rotation;

  tf2::Matrix3x3 rotation{r(0, 0), r(0, 1), r(0, 2), r(1, 0), r(1, 1),
                          r(1, 2), r(2, 0), r(2, 1), r(2, 2)};

  tf2::Quaternion quaternion;

  rotation.getRotation(quaternion);

  quaternion.normalize();

  pose.pose.orientation.x = quaternion.x();

  pose.pose.orientation.y = quaternion.y();

  pose.pose.orientation.z = quaternion.z();

  pose.pose.orientation.w = quaternion.w();

  return pose;
}

} // namespace aruco_tracker

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<aruco_tracker::ArucoTrackerNode>();

    rclcpp::spin(node);
  } catch (const std::exception &exception) {
    RCLCPP_FATAL(rclcpp::get_logger("aruco_tracker"),
                 "Aruco tracker failed: %s", exception.what());

    rclcpp::shutdown();

    return EXIT_FAILURE;
  }

  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
