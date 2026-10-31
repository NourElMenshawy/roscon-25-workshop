#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

using namespace std::chrono_literals;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("mock_target_publisher");
  const auto topic = node->declare_parameter<std::string>("target_topic", "target_pose");
  const auto enabled = node->declare_parameter<bool>("enabled", true);
  if (topic.empty()) {
    throw std::invalid_argument("target_topic must not be empty");
  }
  auto publisher = node->create_publisher<geometry_msgs::msg::PoseStamped>(
      topic, rclcpp::QoS(1).best_effort());
  auto timer = node->create_wall_timer(100ms, [node, publisher, enabled] {
    if (!enabled) {
      return;
    }
    geometry_msgs::msg::PoseStamped message;
    message.header.stamp = node->now();
    message.header.frame_id = "camera_optical_frame";
    message.pose.position.x = 0.2;
    message.pose.position.y = -0.1;
    message.pose.position.z = 2.0;
    message.pose.orientation.w = 1.0;
    publisher->publish(message);
  });
  (void)timer;
  RCLCPP_INFO(node->get_logger(), "Publishing mock ArUco observations on '%s'",
              topic.c_str());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
