#include "landing_mission_controller/mission_policy.hpp"

#include <gtest/gtest.h>

#include <chrono>

using landing_mission_controller::MissionPolicy;
using landing_mission_controller::MissionState;
using namespace std::chrono_literals;

TEST(MissionPolicyTest, RequiresTagBeforeRequest) {
  MissionPolicy policy;
  const auto now = MissionPolicy::TimePoint{};
  EXPECT_FALSE(policy.readyToRequest(now + 200ms, 100ms, 1500ms));
  policy.observeTag(now);
  EXPECT_EQ(policy.state(), MissionState::WAITING_TO_REQUEST);
  EXPECT_FALSE(policy.readyToRequest(now + 99ms, 100ms, 1500ms));
  EXPECT_TRUE(policy.readyToRequest(now + 100ms, 100ms, 1500ms));
}

TEST(MissionPolicyTest, StaleObservationResetsSearch) {
  MissionPolicy policy;
  const auto now = MissionPolicy::TimePoint{};
  policy.observeTag(now);
  policy.expireObservation(now + 1501ms, 1500ms);
  EXPECT_EQ(policy.state(), MissionState::SEARCHING);
  EXPECT_FALSE(policy.readyToRequest(now + 1501ms, 100ms, 1500ms));
}

TEST(MissionPolicyTest, AcceptedRequestBecomesPrimary) {
  MissionPolicy policy;
  policy.observeTag(MissionPolicy::TimePoint{});
  policy.requestSent();
  policy.receiveDecision(true, "drone_a", "drone_a", 42);
  EXPECT_EQ(policy.state(), MissionState::PRIMARY);
  EXPECT_EQ(policy.missionId(), 42u);
}

TEST(MissionPolicyTest, RejectedRequestBecomesFollower) {
  MissionPolicy policy;
  policy.observeTag(MissionPolicy::TimePoint{});
  policy.requestSent();
  policy.receiveDecision(false, "drone_b", "drone_a", 42);
  EXPECT_EQ(policy.state(), MissionState::FOLLOWER);
  EXPECT_EQ(policy.missionId(), 42u);
}

TEST(MissionPolicyTest, InvalidDecisionCannotAssignRole) {
  MissionPolicy policy;
  policy.observeTag(MissionPolicy::TimePoint{});
  policy.requestSent();
  policy.receiveDecision(true, "drone_b", "drone_a", 42);
  EXPECT_EQ(policy.state(), MissionState::SEARCHING);
}

TEST(MissionPolicyTest, LateDecisionAfterTimeoutIsIgnored) {
  MissionPolicy policy;
  policy.observeTag(MissionPolicy::TimePoint{});
  policy.requestSent();
  policy.requestTimedOut();
  policy.receiveDecision(true, "drone_a", "drone_a", 42);
  EXPECT_EQ(policy.state(), MissionState::SEARCHING);
}
