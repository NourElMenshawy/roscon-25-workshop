#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <vector>

#include "landing_coordinator/landing_coordinator.hpp"
#include "landing_coordinator/msg/landing_site_map.hpp"
#include "landing_coordinator/msg/landing_status.hpp"
#include "landing_coordinator/srv/request_landing.hpp"

using namespace std::chrono_literals;

class LandingServiceTest : public ::testing::Test {
 protected:
  using Service = landing_coordinator::srv::RequestLanding;
  using Client = rclcpp::Client<Service>;

  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    coordinator_ = std::make_shared<landing_coordinator::LandingCoordinator>();
    client_node_ = std::make_shared<rclcpp::Node>("landing_coordinator_test_client");
    client_ = client_node_->create_client<Service>("/landing_coordinator/request_landing");

    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(coordinator_);
    executor_->add_node(client_node_);
    spin_thread_ = std::thread([this] { executor_->spin(); });

    ASSERT_TRUE(client_->wait_for_service(5s));
  }

  void TearDown() override
  {
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

  Client::FutureAndRequestId send(const std::string& drone_id)
  {
    auto request = std::make_shared<Service::Request>();
    request->drone_id = drone_id;
    return client_->async_send_request(request);
  }

  std::shared_ptr<Service::Response> receive(Client::FutureAndRequestId& pending)
  {
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

TEST_F(LandingServiceTest, FirstWinsAndRetryIsIdempotent)
{
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
  EXPECT_EQ(second->reason, "reserved_by_other");
  EXPECT_EQ(second->mission_id, first->mission_id);

  auto retry_pending = send("drone_a");
  auto retry = receive(retry_pending);
  ASSERT_NE(retry, nullptr);
  EXPECT_TRUE(retry->granted);
  EXPECT_EQ(retry->reason, "already_owner");
  EXPECT_EQ(retry->mission_id, first->mission_id);
}

TEST_F(LandingServiceTest, InvalidRequestDoesNotReserve)
{
  auto invalid_pending = send("  ");
  auto invalid = receive(invalid_pending);
  ASSERT_NE(invalid, nullptr);
  EXPECT_FALSE(invalid->granted);
  EXPECT_EQ(invalid->mission_id, 0u);
  EXPECT_EQ(invalid->reason, "invalid_drone_id");

  auto valid_pending = send("drone_b");
  auto valid = receive(valid_pending);
  ASSERT_NE(valid, nullptr);
  EXPECT_TRUE(valid->granted);
  EXPECT_EQ(valid->owner_id, "drone_b");
}

TEST_F(LandingServiceTest, TwoOutstandingRequestsHaveOneWinner)
{
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

TEST_F(LandingServiceTest, BroadcastsStatusAndReplaysReservationToLateJoiner)
{
  using Status = landing_coordinator::msg::LandingStatus;

  struct Observed {
    std::mutex mutex;
    std::condition_variable condition;
    std::vector<Status> messages;
  };

  const auto make_listener = [this](const std::shared_ptr<Observed>& observed) {
    return client_node_->create_subscription<Status>(
        "/landing_coordinator/status", rclcpp::QoS(1).reliable().transient_local(),
        [observed](Status::ConstSharedPtr message) {
          {
            std::lock_guard<std::mutex> lock(observed->mutex);
            observed->messages.push_back(*message);
          }
          observed->condition.notify_all();
        });
  };

  const auto wait_for = [](const std::shared_ptr<Observed>& observed, std::uint8_t state,
                           const std::string& owner, std::uint64_t mission_id) {
    std::unique_lock<std::mutex> lock(observed->mutex);
    return observed->condition.wait_for(lock, 3s, [&] {
      for (const auto& status : observed->messages) {
        if (status.state == state && status.owner_id == owner && status.mission_id == mission_id) {
          return true;
        }
      }
      return false;
    });
  };

  auto initial_messages = std::make_shared<Observed>();
  auto first_listener = make_listener(initial_messages);
  ASSERT_TRUE(wait_for(initial_messages, Status::AVAILABLE, "", 0));

  auto first_pending = send("drone_a");
  auto first = receive(first_pending);
  ASSERT_NE(first, nullptr);
  ASSERT_TRUE(first->granted);
  ASSERT_TRUE(wait_for(initial_messages, Status::RESERVED, "drone_a", 1));

  // The current reservation is retained by the transient-local publisher.
  auto late_messages = std::make_shared<Observed>();
  auto late_listener = make_listener(late_messages);
  ASSERT_TRUE(wait_for(late_messages, Status::RESERVED, "drone_a", 1));

  // Suppress unused-variable warnings; the subscriptions own the callbacks.
  (void)first_listener;
  (void)late_listener;
}

TEST_F(LandingServiceTest, CompletionServicePublishesAndRetainsCompletedStatus)
{
  using Completion = landing_coordinator::srv::ReportLandingComplete;
  using Status = landing_coordinator::msg::LandingStatus;

  auto complete_client =
      client_node_->create_client<Completion>("/landing_coordinator/report_landing_complete");
  ASSERT_TRUE(complete_client->wait_for_service(5s));

  auto first_pending = send("drone_a");
  auto first = receive(first_pending);
  ASSERT_NE(first, nullptr);
  ASSERT_TRUE(first->granted);

  auto report = [&complete_client](const std::string& drone_id, std::uint64_t mission_id,
                                   bool success) {
    auto request = std::make_shared<Completion::Request>();
    request->drone_id = drone_id;
    request->mission_id = mission_id;
    request->success = success;
    auto pending = complete_client->async_send_request(request);
    if (pending.wait_for(5s) != std::future_status::ready) {
      complete_client->remove_pending_request(pending);
      return std::shared_ptr<Completion::Response>{};
    }
    return pending.get();
  };

  auto wrong_owner = report("drone_b", first->mission_id, true);
  ASSERT_NE(wrong_owner, nullptr);
  EXPECT_FALSE(wrong_owner->accepted);
  EXPECT_EQ(wrong_owner->reason, "not_owner");

  auto wrong_mission = report("drone_a", first->mission_id + 1, true);
  ASSERT_NE(wrong_mission, nullptr);
  EXPECT_FALSE(wrong_mission->accepted);
  EXPECT_EQ(wrong_mission->reason, "wrong_mission_id");

  auto unsuccessful = report("drone_a", first->mission_id, false);
  ASSERT_NE(unsuccessful, nullptr);
  EXPECT_FALSE(unsuccessful->accepted);
  EXPECT_EQ(unsuccessful->reason, "unsuccessful_result");

  auto completed = report("drone_a", first->mission_id, true);
  ASSERT_NE(completed, nullptr);
  EXPECT_TRUE(completed->accepted);
  EXPECT_TRUE(completed->completed);
  EXPECT_EQ(completed->reason, "completed");

  auto again = report("drone_a", first->mission_id, true);
  ASSERT_NE(again, nullptr);
  EXPECT_TRUE(again->accepted);
  EXPECT_EQ(again->reason, "already_completed");

  // Completion also retains a single, mission-scoped site catalogue. The
  // default geometry is NOT marked verified, preventing an accidental second
  // landing on unexamined Gazebo coordinates.
  using SiteMap = landing_coordinator::msg::LandingSiteMap;
  std::mutex sites_mutex;
  std::condition_variable sites_condition;
  bool received_sites = false;
  auto sites_sub = client_node_->create_subscription<SiteMap>(
      "/landing_coordinator/landing_sites", rclcpp::QoS(1).reliable().transient_local(),
      [&](SiteMap::ConstSharedPtr msg) {
        if (msg->mission_id == first->mission_id && msg->owner_id == "drone_a" &&
            msg->frame_id == "map" && msg->sites.size() == 2 && msg->sites[0].occupied &&
            !msg->sites[1].occupied && !msg->sites[0].verified && !msg->sites[1].verified) {
          {
            std::lock_guard<std::mutex> guard(sites_mutex);
            received_sites = true;
          }
          sites_condition.notify_all();
        }
      });
  {
    std::unique_lock<std::mutex> site_lock(sites_mutex);
    EXPECT_TRUE(sites_condition.wait_for(site_lock, 4s, [&] { return received_sites; }));
  }
  (void)sites_sub;

  // Late-joining subscriptions receive the last status, not RESERVED.
  std::mutex mutex;
  std::condition_variable condition;
  bool got_completed = false;
  auto late_sub = client_node_->create_subscription<Status>(
      "/landing_coordinator/status", rclcpp::QoS(1).reliable().transient_local(),
      [&](Status::ConstSharedPtr msg) {
        if (msg->state == Status::COMPLETED && msg->owner_id == "drone_a" &&
            msg->mission_id == first->mission_id) {
          {
            std::lock_guard<std::mutex> lock(mutex);
            got_completed = true;
          }
          condition.notify_all();
        }
      });
  std::unique_lock<std::mutex> lock(mutex);
  EXPECT_TRUE(condition.wait_for(lock, 4s, [&] { return got_completed; }));
  lock.unlock();
  (void)late_sub;

  // The landing site stays reserved, even after COMPLETED.
  auto b_pending = send("drone_b");
  auto b = receive(b_pending);
  ASSERT_NE(b, nullptr);
  EXPECT_FALSE(b->granted);
  EXPECT_EQ(b->owner_id, "drone_a");
}
