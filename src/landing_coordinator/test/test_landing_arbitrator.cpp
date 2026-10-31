#include "landing_coordinator/landing_arbitrator.hpp"

#include <gtest/gtest.h>

#include <future>
#include <string>
#include <vector>

using landing_coordinator::LandingArbitrator;
using landing_coordinator::LandingDecision;

TEST(LandingArbitratorTest, FirstRequestWins) {
  LandingArbitrator arbitrator;
  const auto result = arbitrator.requestLanding("drone_a");
  EXPECT_TRUE(result.granted);
  EXPECT_EQ(result.owner_id, "drone_a");
  EXPECT_EQ(result.mission_id, 1u);
  EXPECT_EQ(result.reason, "granted");
}

TEST(LandingArbitratorTest, DifferentDroneCannotTakeOwnership) {
  LandingArbitrator arbitrator;
  const auto first = arbitrator.requestLanding("drone_a");
  const auto second = arbitrator.requestLanding("drone_b");
  EXPECT_TRUE(first.granted);
  EXPECT_FALSE(second.granted);
  EXPECT_EQ(second.owner_id, "drone_a");
  EXPECT_EQ(second.mission_id, first.mission_id);
  EXPECT_EQ(second.reason, "reserved_by_other");
}

TEST(LandingArbitratorTest, RetryByOwnerIsIdempotent) {
  LandingArbitrator arbitrator;
  const auto first = arbitrator.requestLanding("drone_b");
  const auto retry = arbitrator.requestLanding("drone_b");
  EXPECT_TRUE(retry.granted);
  EXPECT_EQ(retry.reason, "already_owner");
  EXPECT_EQ(retry.mission_id, first.mission_id);
  EXPECT_EQ(retry.owner_id, "drone_b");
}

TEST(LandingArbitratorTest, BlankIdentityCannotClaimResource) {
  LandingArbitrator arbitrator;
  const auto invalid = arbitrator.requestLanding(" \t\n ");
  EXPECT_FALSE(invalid.granted);
  EXPECT_EQ(invalid.owner_id, "");
  EXPECT_EQ(invalid.mission_id, 0u);
  EXPECT_EQ(invalid.reason, "invalid_drone_id");

  EXPECT_TRUE(arbitrator.requestLanding("drone_a").granted);
}

TEST(LandingArbitratorTest, ConcurrentRequestsHaveExactlyOneWinner) {
  LandingArbitrator arbitrator;
  constexpr int kCount = 32;
  std::promise<void> start_promise;
  auto start = start_promise.get_future().share();
  std::vector<std::future<LandingDecision>> results;
  results.reserve(kCount);

  for (int i = 0; i < kCount; ++i) {
    results.push_back(std::async(std::launch::async, [&, i] {
      start.wait();
      return arbitrator.requestLanding("drone_" + std::to_string(i));
    }));
  }
  start_promise.set_value();

  int winners = 0;
  std::string chosen_owner;
  std::vector<LandingDecision> decisions;
  for (auto& future : results) {
    auto decision = future.get();
    if (decision.granted) {
      ++winners;
      chosen_owner = decision.owner_id;
    }
    decisions.push_back(std::move(decision));
  }
  ASSERT_EQ(winners, 1);
  ASSERT_FALSE(chosen_owner.empty());
  for (const auto& decision : decisions) {
    EXPECT_EQ(decision.owner_id, chosen_owner);
    EXPECT_EQ(decision.mission_id, 1u);
  }
}
