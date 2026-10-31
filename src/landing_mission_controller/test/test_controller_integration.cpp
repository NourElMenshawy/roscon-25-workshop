#include "landing_mission_controller/landing_mission_controller.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <landing_coordinator/srv/request_landing.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>

using landing_mission_controller::LandingMissionController;
using landing_mission_controller::MissionState;
using RequestLanding = landing_coordinator::srv::RequestLanding;
using namespace std::chrono_literals;

class ControllerIntegrationTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

namespace {

rclcpp::NodeOptions optionsFor(const std::string& id,
                               const std::string& topic, int delay_ms) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("drone_id", id),
      rclcpp::Parameter("target_topic", topic),
      rclcpp::Parameter("coordinator_service",
                        "/test_landing_mission_coordinator/request_landing"),
      rclcpp::Parameter("request_delay_ms", delay_ms),
      rclcpp::Parameter("observation_timeout_ms", 1500),
      rclcpp::Parameter("response_timeout_ms", 3000),
  });
  return options;
}

void runArbitrationScenario(int a_delay_ms, int b_delay_ms,
                            const std::string& expected_owner) {
  auto server = std::make_shared<rclcpp::Node>("fake_coordinator");
  std::string owner;
  int requests = 0;
  auto service = server->create_service<RequestLanding>(
      "/test_landing_mission_coordinator/request_landing",
      [&owner, &requests](std::shared_ptr<RequestLanding::Request> request,
                          std::shared_ptr<RequestLanding::Response> response) {
        ++requests;
        if (owner.empty()) {
          owner = request->drone_id;
        }
        response->granted = owner == request->drone_id;
        response->owner_id = owner;
        response->mission_id = 1;
        response->reason = response->granted ? "granted" : "reserved_by_other";
      });
  (void)service;

  auto a = std::make_shared<LandingMissionController>(
      optionsFor("drone_a", "/test/drone_a/target_pose", a_delay_ms));
  auto b = std::make_shared<LandingMissionController>(
      optionsFor("drone_b", "/test/drone_b/target_pose", b_delay_ms));
  auto pub = std::make_shared<rclcpp::Node>("mock_observations");
  auto pub_a = pub->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/test/drone_a/target_pose", rclcpp::QoS(1).best_effort());
  auto pub_b = pub->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/test/drone_b/target_pose", rclcpp::QoS(1).best_effort());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server);
  executor.add_node(a);
  executor.add_node(b);
  executor.add_node(pub);

  geometry_msgs::msg::PoseStamped observation;
  observation.pose.position.z = 2.0;
  observation.pose.orientation.w = 1.0;

  const auto end = std::chrono::steady_clock::now() + 4s;
  while (std::chrono::steady_clock::now() < end &&
         (a->state() == MissionState::SEARCHING ||
          a->state() == MissionState::WAITING_TO_REQUEST ||
          a->state() == MissionState::REQUESTING ||
          b->state() == MissionState::SEARCHING ||
          b->state() == MissionState::WAITING_TO_REQUEST ||
          b->state() == MissionState::REQUESTING)) {
    observation.header.stamp = pub->now();
    pub_a->publish(observation);
    pub_b->publish(observation);
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }

  const auto a_expected = expected_owner == "drone_a" ? MissionState::PRIMARY
                                                      : MissionState::FOLLOWER;
  const auto b_expected = expected_owner == "drone_b" ? MissionState::PRIMARY
                                                      : MissionState::FOLLOWER;
  EXPECT_EQ(a->state(), a_expected);
  EXPECT_EQ(b->state(), b_expected);
  EXPECT_EQ(a->missionId(), 1u);
  EXPECT_EQ(b->missionId(), 1u);
  EXPECT_EQ(owner, expected_owner);
  EXPECT_EQ(requests, 2);
}

}  // namespace

TEST_F(ControllerIntegrationTest, DroneAWins) {
  runArbitrationScenario(50, 800, "drone_a");
}

TEST_F(ControllerIntegrationTest, DroneBWins) {
  runArbitrationScenario(800, 50, "drone_b");
}

TEST_F(ControllerIntegrationTest, InvalidMarkerDoesNotRequest) {
  auto node = std::make_shared<LandingMissionController>(
      optionsFor("drone_b", "/test/invalid_marker", 10));
  auto publisher_node = std::make_shared<rclcpp::Node>("publish_bad_marker");
  auto publisher = publisher_node->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/test/invalid_marker", rclcpp::QoS(1).best_effort());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(publisher_node);
  geometry_msgs::msg::PoseStamped bad_marker;
  bad_marker.pose.position.z = 0.0;
  bad_marker.pose.orientation.w = 1.0;
  for (int i = 0; i < 30; ++i) {
    publisher->publish(bad_marker);
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_EQ(node->state(), MissionState::SEARCHING);
}
