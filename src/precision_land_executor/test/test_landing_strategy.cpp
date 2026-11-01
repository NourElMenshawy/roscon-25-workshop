#include <gtest/gtest.h>

#include "precision_land_executor/landing_strategy.hpp"

using namespace precision_land_executor;

TEST(LandingStrategyPolicy, DefaultsToSafeHoldWithoutVerifiedSites)
{
  StrategySnapshot snapshot{.mission_id = 1, .aruco_fresh = true};
  const auto result = selectStrategy(snapshot, evaluatePosition(snapshot), evaluateAruco(snapshot),
                                     evaluateVisual(snapshot));
  EXPECT_EQ(result.strategy, LandingStrategy::SafeHold);
  EXPECT_EQ(result.position.reason, EligibilityReason::MissingTarget);
  EXPECT_EQ(result.aruco.reason, EligibilityReason::UnverifiedSite);
  EXPECT_EQ(result.visual.reason, EligibilityReason::NotImplemented);
}

TEST(LandingStrategyPolicy, PositionHasPriorityWhenAllAreSafe)
{
  StrategySnapshot snapshot{.mission_id = 4,
                            .shared_position_available = true,
                            .shared_position_frame_valid = true,
                            .position_site_verified = true,
                            .aruco_fresh = true,
                            .aruco_site_verified = true,
                            .visual_estimator_ready = true,
                            .visual_site_verified = true};
  const auto result = selectStrategy(snapshot, evaluatePosition(snapshot), evaluateAruco(snapshot),
                                     evaluateVisual(snapshot));
  EXPECT_EQ(result.strategy, LandingStrategy::Position);
}

TEST(LandingStrategyPolicy, ArucoIsChosenWhenPositionFrameIsInvalid)
{
  StrategySnapshot snapshot{.mission_id = 5,
                            .shared_position_available = true,
                            .position_site_verified = true,
                            .aruco_fresh = true,
                            .aruco_site_verified = true};
  const auto result = selectStrategy(snapshot, evaluatePosition(snapshot), evaluateAruco(snapshot),
                                     evaluateVisual(snapshot));
  EXPECT_EQ(result.position.reason, EligibilityReason::InvalidFrame);
  EXPECT_EQ(result.strategy, LandingStrategy::Aruco);
}

TEST(LandingStrategyPolicy, VisualIsSelectedWhenOthersUnavailable)
{
  StrategySnapshot snapshot{
      .mission_id = 6, .visual_estimator_ready = true, .visual_site_verified = true};
  const auto result = selectStrategy(snapshot, evaluatePosition(snapshot), evaluateAruco(snapshot),
                                     evaluateVisual(snapshot));
  EXPECT_EQ(result.strategy, LandingStrategy::Visual);
}

TEST(LandingStrategyPolicy, FailsafeAlwaysSelectsHold)
{
  StrategySnapshot snapshot{.mission_id = 7,
                            .shared_position_available = true,
                            .shared_position_frame_valid = true,
                            .position_site_verified = true,
                            .aruco_fresh = true,
                            .aruco_site_verified = true,
                            .vehicle_failsafe = true};
  const auto result = selectStrategy(snapshot, evaluatePosition(snapshot), evaluateAruco(snapshot),
                                     evaluateVisual(snapshot));
  EXPECT_EQ(result.strategy, LandingStrategy::SafeHold);
  EXPECT_EQ(result.position.reason, EligibilityReason::Failsafe);
}

TEST(LandingStrategyPolicy, SelectionGuardsAgainstUnsafeCheckResult)
{
  StrategySnapshot snapshot{.mission_id = 8, .vehicle_failsafe = true};
  const auto eligible = Eligibility{true, EligibilityReason::Eligible};
  EXPECT_EQ(selectStrategy(snapshot, eligible, eligible, eligible).strategy,
            LandingStrategy::SafeHold);
}
