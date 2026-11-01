#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <optional>
#include <stdexcept>
#include <thread>

#include "precision_land_executor/follower_strategy_selector.hpp"

using namespace precision_land_executor;
using namespace std::chrono_literals;

namespace {

std::optional<SelectionResult> waitForResult(FollowerStrategySelector& selector)
{
  for (int i = 0; i < 400; ++i) {
    if (auto result = selector.takeResult()) {
      return result;
    }
    std::this_thread::sleep_for(5ms);
  }
  return std::nullopt;
}

Eligibility throwOnCheck(const StrategySnapshot&)
{
  throw std::runtime_error("injected check failure");
}

std::atomic<bool> slow_check_started{false};
std::atomic<bool> allow_slow_check_to_finish{false};

Eligibility slowCheck(const StrategySnapshot&)
{
  slow_check_started.store(true);
  while (!allow_slow_check_to_finish.load()) {
    std::this_thread::yield();
  }
  return {true, EligibilityReason::Eligible};
}

}  // namespace

TEST(FollowerStrategySelector, ComposesThreeChecksAndSelectsHold)
{
  FollowerStrategySelector selector;
  ASSERT_TRUE(selector.start(StrategySnapshot{.mission_id = 1, .aruco_fresh = true}));
  const auto result = waitForResult(selector);
  ASSERT_TRUE(result);
  EXPECT_EQ(result->mission_id, 1U);
  EXPECT_EQ(result->strategy, LandingStrategy::SafeHold);
  EXPECT_EQ(result->outcome, SelectionOutcome::Selected);
}

TEST(FollowerStrategySelector, SelectsAvailablePosition)
{
  FollowerStrategySelector selector;
  StrategySnapshot snapshot{.mission_id = 11,
                            .shared_position_available = true,
                            .shared_position_frame_valid = true,
                            .position_site_verified = true};
  ASSERT_TRUE(selector.start(snapshot));
  const auto result = waitForResult(selector);
  ASSERT_TRUE(result);
  EXPECT_EQ(result->strategy, LandingStrategy::Position);
}

TEST(FollowerStrategySelector, CheckErrorFailsClosed)
{
  FollowerStrategySelector selector(FollowerStrategySelector::Checks{
      .position = &throwOnCheck,
      .aruco = &evaluateAruco,
      .visual = &evaluateVisual,
  });
  ASSERT_TRUE(selector.start(StrategySnapshot{.mission_id = 12}));
  const auto result = waitForResult(selector);
  ASSERT_TRUE(result);
  EXPECT_EQ(result->strategy, LandingStrategy::SafeHold);
  EXPECT_EQ(result->outcome, SelectionOutcome::Error);
}

TEST(FollowerStrategySelector, CancellationSuppressesInFlightResult)
{
  slow_check_started.store(false);
  allow_slow_check_to_finish.store(false);
  FollowerStrategySelector selector(FollowerStrategySelector::Checks{
      .position = &slowCheck,
      .aruco = &evaluateAruco,
      .visual = &evaluateVisual,
  });

  ASSERT_TRUE(selector.start(StrategySnapshot{.mission_id = 13}));
  for (int i = 0; i < 400 && !slow_check_started.load(); ++i) {
    std::this_thread::sleep_for(5ms);
  }
  if (!slow_check_started.load()) {
    // Release the gate even on test failure to avoid blocking scope teardown.
    allow_slow_check_to_finish.store(true);
    FAIL() << "slow selection check did not start";
  }

  selector.cancel();
  allow_slow_check_to_finish.store(true);
  std::this_thread::sleep_for(30ms);
  EXPECT_FALSE(selector.takeResult());
}

TEST(FollowerStrategySelector, RestartInvalidatesEarlierMission)
{
  slow_check_started.store(false);
  allow_slow_check_to_finish.store(false);
  FollowerStrategySelector selector(FollowerStrategySelector::Checks{
      .position = &slowCheck,
      .aruco = &evaluateAruco,
      .visual = &evaluateVisual,
  });
  ASSERT_TRUE(selector.start(StrategySnapshot{.mission_id = 14}));
  for (int i = 0; i < 400 && !slow_check_started.load(); ++i) {
    std::this_thread::sleep_for(5ms);
  }
  if (!slow_check_started.load()) {
    allow_slow_check_to_finish.store(true);
    FAIL() << "slow selection check did not start";
  }
  selector.cancel();
  allow_slow_check_to_finish.store(true);
  // The next selection supersedes the old one, even if both complete later.
  ASSERT_TRUE(selector.start(StrategySnapshot{.mission_id = 15}));
  const auto result = waitForResult(selector);
  ASSERT_TRUE(result);
  EXPECT_EQ(result->mission_id, 15U);
}
