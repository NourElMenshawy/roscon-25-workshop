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

TEST(MissionPolicyTest, StatusAssignsFollowerWithoutTagDetection) {
  MissionPolicy policy;
  EXPECT_TRUE(policy.receiveStatus(1, "drone_a", "drone_b", 17));
  EXPECT_EQ(policy.state(), MissionState::FOLLOWER);
  EXPECT_EQ(policy.missionId(), 17u);
}

TEST(MissionPolicyTest, StatusAssignsPrimaryWithoutServiceResponse) {
  MissionPolicy policy;
  EXPECT_TRUE(policy.receiveStatus(1, "drone_b", "drone_b", 17));
  EXPECT_EQ(policy.state(), MissionState::PRIMARY);
  EXPECT_EQ(policy.missionId(), 17u);
}

TEST(MissionPolicyTest, ReservedStatusOverridesOutstandingRequest) {
  MissionPolicy policy;
  policy.observeTag(MissionPolicy::TimePoint{});
  policy.requestSent();
  EXPECT_TRUE(policy.receiveStatus(1, "drone_a", "drone_b", 2));
  EXPECT_EQ(policy.state(), MissionState::FOLLOWER);
  // A late service result from the old pending request must not override status.
  policy.receiveDecision(true, "drone_b", "drone_b", 2);
  EXPECT_EQ(policy.state(), MissionState::FOLLOWER);
  EXPECT_EQ(policy.missionId(), 2u);
}

TEST(MissionPolicyTest, InitialAvailableCannotUndoReservation) {
  MissionPolicy policy;
  EXPECT_TRUE(policy.receiveStatus(0, "", "drone_b", 0));
  EXPECT_EQ(policy.state(), MissionState::SEARCHING);
  EXPECT_TRUE(policy.receiveStatus(1, "drone_a", "drone_b", 1));
  EXPECT_FALSE(policy.receiveStatus(0, "", "drone_b", 0));
  EXPECT_EQ(policy.state(), MissionState::FOLLOWER);
}

TEST(MissionPolicyTest, ConflictingAndStaleStatusIgnored) {
  MissionPolicy policy;
  EXPECT_TRUE(policy.receiveStatus(1, "drone_a", "drone_b", 10));
  EXPECT_FALSE(policy.receiveStatus(1, "drone_b", "drone_b", 10));
  EXPECT_FALSE(policy.receiveStatus(1, "drone_b", "drone_b", 9));
  EXPECT_EQ(policy.state(), MissionState::FOLLOWER);
  EXPECT_EQ(policy.missionId(), 10u);
}

TEST(MissionPolicyTest, MalformedStatusIgnored) {
  MissionPolicy policy;
  EXPECT_FALSE(policy.receiveStatus(1, "", "drone_b", 1));
  EXPECT_FALSE(policy.receiveStatus(1, "drone_a", "drone_b", 0));
  EXPECT_FALSE(policy.receiveStatus(23, "drone_a", "drone_b", 1));
  EXPECT_FALSE(policy.receiveStatus(0, "drone_a", "drone_b", 0));
  EXPECT_EQ(policy.state(), MissionState::SEARCHING);
  EXPECT_EQ(policy.missionId(), 0u);
}

TEST(MissionPolicyTest, CompletedStatusAdvancesFollowerWithoutArUco) {
  MissionPolicy follower;
  EXPECT_TRUE(follower.receiveStatus(1, "drone_a", "drone_b", 7));
  EXPECT_EQ(follower.state(), MissionState::FOLLOWER);
  EXPECT_TRUE(follower.receiveStatus(2, "drone_a", "drone_b", 7));
  EXPECT_EQ(follower.state(), MissionState::READY_FOR_SELECTION);
  EXPECT_EQ(follower.missionId(), 7u);
}

TEST(MissionPolicyTest, CompletedStatusMarksPrimaryLanded) {
  MissionPolicy primary;
  EXPECT_TRUE(primary.receiveStatus(2, "drone_a", "drone_a", 1));
  EXPECT_EQ(primary.state(), MissionState::LANDED);
  EXPECT_TRUE(primary.receiveStatus(2, "drone_a", "drone_a", 1));
  EXPECT_EQ(primary.state(), MissionState::LANDED);
}

TEST(MissionPolicyTest, DelayedReservedCannotUndoCompletion) {
  MissionPolicy follower;
  ASSERT_TRUE(follower.receiveStatus(1, "drone_a", "drone_b", 1));
  ASSERT_TRUE(follower.receiveStatus(2, "drone_a", "drone_b", 1));
  EXPECT_FALSE(follower.receiveStatus(1, "drone_a", "drone_b", 1));
  EXPECT_FALSE(follower.receiveStatus(2, "drone_c", "drone_b", 1));
  EXPECT_FALSE(follower.receiveStatus(2, "drone_a", "drone_b", 0));
  EXPECT_EQ(follower.state(), MissionState::READY_FOR_SELECTION);
}

TEST(MissionPolicyTest, SecondaryLandingRequiresCurrentCompletedMission) {
  MissionPolicy policy;
  EXPECT_FALSE(policy.secondaryLandingComplete(5));
  EXPECT_TRUE(policy.receiveStatus(1, "drone_b", "drone_a", 5));
  EXPECT_FALSE(policy.secondaryLandingComplete(5));
  EXPECT_TRUE(policy.receiveStatus(2, "drone_b", "drone_a", 5));
  EXPECT_EQ(policy.state(), MissionState::READY_FOR_SELECTION);
  EXPECT_FALSE(policy.secondaryLandingComplete(4));
  EXPECT_TRUE(policy.secondaryLandingComplete(5));
  EXPECT_EQ(policy.state(), MissionState::LANDED);
  EXPECT_FALSE(policy.secondaryLandingComplete(5));
  EXPECT_TRUE(policy.receiveStatus(2, "drone_b", "drone_a", 5));
  EXPECT_EQ(policy.state(), MissionState::LANDED);
}

TEST(MissionPolicyTest, SecondaryLandingCannotOverridePrimaryOrNewMission) {
  MissionPolicy policy;
  EXPECT_TRUE(policy.receiveStatus(2, "drone_a", "drone_a", 1));
  EXPECT_FALSE(policy.secondaryLandingComplete(1));
  EXPECT_TRUE(policy.receiveStatus(2, "drone_b", "drone_a", 2));
  EXPECT_EQ(policy.state(), MissionState::READY_FOR_SELECTION);
  EXPECT_FALSE(policy.secondaryLandingComplete(1));
  EXPECT_EQ(policy.state(), MissionState::READY_FOR_SELECTION);
}
