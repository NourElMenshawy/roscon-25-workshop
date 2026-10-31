#include "landing_coordinator/landing_coordinator.hpp"
#include "landing_coordinator/srv/request_landing.hpp"

#include <gtest/gtest.h>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <thread>

using namespace std::chrono_literals;

class LandingServiceTest : public ::testing::Test {
 protected:
  using Service = landing_coordinator::srv::RequestLanding;
  using Client = rclcpp::Client<Service>;

  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    coordinator_ = std::make_shared<landing_coordinator::LandingCoordinator>();
    client_node_ = std::make_shared<rclcpp::Node>("landing_coordinator_test_client");
    client_ = client_node_->create_client<Service>(
        "/landing_coordinator/request_landing");

    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(coordinator_);
    executor_->add_node(client_node_);
    spin_thread_ = std::thread([this] { executor_->spin(); });

    ASSERT_TRUE(client_->wait_for_service(5s));
  }

  void TearDown() override {
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    executor_->remove_node(client_node_);
    executor_->remove_node(coordinator_);
    client_.reset();
    client_node_.reset();
    coordinator_.reset();
    executor_.reset();
  }

  Client::FutureAndRequestId send(const std::string& drone_id) {
    auto request = std::make_shared<Service::Request>();
    request->drone_id = drone_id;
    return client_->async_send_request(request);
  }

  std::shared_ptr<Service::Response> receive(Client::FutureAndRequestId& pending) {
    if (pending.wait_for(5s) != std::future_status::ready) {
      client_->remove_pending_request(pending);
      return nullptr;
    }
    return pending.get();
  }

  std::shared_ptr<landing_coordinator::LandingCoordinator> coordinator_;
  rclcpp::Node::SharedPtr client_node_;
  Client::SharedPtr client_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
};

TEST_F(LandingServiceTest, FirstWinsAndRetryIsIdempotent) {
  auto first_pending = send("drone_a");
  auto first = receive(first_pending);
  ASSERT_NE(first, nullptr);
  EXPECT_TRUE(first->granted);
  EXPECT_EQ(first->owner_id, "drone_a");
  EXPECT_EQ(first->reason, "granted");

  auto second_pending = send("drone_b");
  auto second = receive(second_pending);
  ASSERT_NE(second, nullptr);
  EXPECT_FALSE(second->granted);
  EXPECT_EQ(second->owner_id, "drone_a");
  EXPECT_EQ(second->mission_id, first->mission_id);

  auto retry_pending = send("drone_a");
  auto retry = receive(retry_pending);
  ASSERT_NE(retry, nullptr);
  EXPECT_TRUE(retry->granted);
  EXPECT_EQ(retry->reason, "already_owner");
  EXPECT_EQ(retry->mission_id, first->mission_id);
}

TEST_F(LandingServiceTest, InvalidRequestDoesNotReserve) {
  auto invalid_pending = send("  ");
  auto invalid = receive(invalid_pending);
  ASSERT_NE(invalid, nullptr);
  EXPECT_FALSE(invalid->granted);
  EXPECT_EQ(invalid->mission_id, 0u);

  auto valid_pending = send("drone_b");
  auto valid = receive(valid_pending);
  ASSERT_NE(valid, nullptr);
  EXPECT_TRUE(valid->granted);
  EXPECT_EQ(valid->owner_id, "drone_b");
}

TEST_F(LandingServiceTest, TwoOutstandingRequestsHaveOneWinner) {
  // Enqueue both requests before waiting for either response.
  auto a_pending = send("drone_a");
  auto b_pending = send("drone_b");
  auto a = receive(a_pending);
  auto b = receive(b_pending);
  ASSERT_NE(a, nullptr);
  ASSERT_NE(b, nullptr);

  EXPECT_NE(a->granted, b->granted);
  EXPECT_EQ(a->owner_id, b->owner_id);
  EXPECT_EQ(a->mission_id, b->mission_id);
  EXPECT_EQ(a->mission_id, 1u);
}
