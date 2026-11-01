#include "precision_land_executor/shared_landing_site.hpp"
#include "precision_land_executor/landing_strategy.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <optional>

using namespace precision_land_executor;

namespace {

SiteCatalog catalog() {
  SiteCatalog result{};
  result.mission_id = 1;
  result.owner_id = "drone_a";
  result.frame_id = "map";
  result.sites = {
      Site{.id = "aruco_primary",
           .center = {.x = 0.0, .y = 0.0, .z = 0.0},
           .clearance_radius_m = 0.75,
           .verified = true,
           .occupied = true},
      Site{.id = "secondary_position",
           .center = {.x = 5.0, .y = 1.0, .z = 0.0},
           .clearance_radius_m = 0.75,
           .verified = true,
           .occupied = false},
  };
  return result;
}

ResolvedSite localSite() {
  return ResolvedSite{.mission_id = 1,
                      .site_id = "secondary_position",
                      .source_frame = "map",
                      .target_frame = "px4_1/odom_ned",
                      .local_position = {.x = 1.0, .y = 2.0, .z = 0.0}};
}

SiteEvaluation evaluate(const SiteCatalog* input,
                        const std::optional<ResolvedSite>& resolved) {
  return evaluateSites(input, 1, "drone_a", "map", "px4_1/odom_ned",
                       resolved, 2.0);
}

}  // namespace

TEST(SharedLandingSites, AcceptsTfResolvedCandidateWithCorrectMissionAndFrames) {
  const auto input = catalog();
  const auto result = evaluate(&input, localSite());
  ASSERT_TRUE(result.position_available);
  ASSERT_TRUE(result.frame_valid);
  ASSERT_TRUE(result.site_verified);
  EXPECT_EQ(result.reason, SiteRejection::Eligible);
  EXPECT_EQ(result.selected_site_id, "secondary_position");
  EXPECT_DOUBLE_EQ(result.local_position.x, 1.0);
  EXPECT_DOUBLE_EQ(result.local_position.y, 2.0);
}

TEST(SharedLandingSites, MissingTfFailsClosed) {
  const auto input = catalog();
  const auto result = evaluate(&input, std::nullopt);
  EXPECT_TRUE(result.position_available);
  EXPECT_FALSE(result.frame_valid);
  EXPECT_FALSE(result.site_verified);
  EXPECT_EQ(result.reason, SiteRejection::InvalidFrame);
}

TEST(SharedLandingSites, DefaultsFailClosedWithoutCatalogue) {
  const auto result = evaluate(nullptr, localSite());
  EXPECT_FALSE(result.position_available);
  EXPECT_EQ(result.reason, SiteRejection::MissingCatalog);
}

TEST(SharedLandingSites, WrongMissionOrOwnerRejected) {
  auto input = catalog();
  input.mission_id = 2;
  EXPECT_EQ(evaluate(&input, localSite()).reason, SiteRejection::WrongMission);
  input.mission_id = 1;
  input.owner_id = "drone_b";
  EXPECT_EQ(evaluate(&input, localSite()).reason, SiteRejection::WrongMission);
}

TEST(SharedLandingSites, UnverifiedSourceOrTargetRejected) {
  auto input = catalog();
  input.sites[0].verified = false;
  EXPECT_EQ(evaluate(&input, localSite()).reason, SiteRejection::UnverifiedSite);
  input.sites[0].verified = true;
  input.sites[1].verified = false;
  EXPECT_EQ(evaluate(&input, localSite()).reason, SiteRejection::UnverifiedSite);
}

TEST(SharedLandingSites, OccupiedOrMissingSecondaryRejected) {
  auto input = catalog();
  input.sites[1].occupied = true;
  EXPECT_EQ(evaluate(&input, localSite()).reason, SiteRejection::InvalidGeometry);
  input.sites.erase(input.sites.begin() + 1);
  EXPECT_EQ(evaluate(&input, localSite()).reason, SiteRejection::MissingCandidate);
}

TEST(SharedLandingSites, OverlapAndMarginRejected) {
  auto input = catalog();
  input.sites[1].center = {.x = 1.0, .y = 0.0, .z = 0.0};
  EXPECT_EQ(evaluate(&input, localSite()).reason, SiteRejection::InsufficientSeparation);
  input.sites[1].center = {.x = 3.5, .y = 0.0, .z = 0.0};
  EXPECT_EQ(evaluate(&input, localSite()).reason, SiteRejection::Eligible);
}

TEST(SharedLandingSites, RejectsWrongResolvedMissionSiteOrFrame) {
  const auto input = catalog();
  auto resolved = localSite();
  resolved.mission_id = 2;
  EXPECT_EQ(evaluate(&input, resolved).reason, SiteRejection::InvalidFrame);
  resolved = localSite();
  resolved.site_id = "aruco_primary";
  EXPECT_EQ(evaluate(&input, resolved).reason, SiteRejection::InvalidFrame);
  resolved = localSite();
  resolved.source_frame = "gazebo_world_enu";
  EXPECT_EQ(evaluate(&input, resolved).reason, SiteRejection::InvalidFrame);
  resolved = localSite();
  resolved.target_frame = "px4_0/odom_ned";
  EXPECT_EQ(evaluate(&input, resolved).reason, SiteRejection::InvalidFrame);
  auto wrong_frame_catalog = catalog();
  wrong_frame_catalog.frame_id = "gazebo_world_enu";
  EXPECT_EQ(evaluate(&wrong_frame_catalog, localSite()).reason,
            SiteRejection::InvalidFrame);
}

TEST(SharedLandingSites, RejectsInvalidGeometryAndResolvedCoordinates) {
  auto input = catalog();
  input.sites[1].center.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(evaluate(&input, localSite()).reason, SiteRejection::InvalidGeometry);
  input = catalog();
  input.sites[1].id = input.sites[0].id;
  EXPECT_EQ(evaluate(&input, localSite()).reason, SiteRejection::InvalidGeometry);
  input = catalog();
  input.sites[0].clearance_radius_m = -1.0;
  EXPECT_EQ(evaluate(&input, localSite()).reason, SiteRejection::InvalidGeometry);
  input = catalog();
  auto resolved = localSite();
  resolved.local_position.y = std::numeric_limits<double>::infinity();
  EXPECT_EQ(evaluate(&input, resolved).reason, SiteRejection::InvalidFrame);
}

TEST(SharedLandingSites, VerifiedTfResultEnablesPositionStrategy) {
  const auto input = catalog();
  const auto site = evaluate(&input, localSite());
  StrategySnapshot snapshot{};
  snapshot.mission_id = 1;
  snapshot.shared_position_available = site.position_available;
  snapshot.shared_position_frame_valid = site.frame_valid;
  snapshot.position_site_verified = site.site_verified;
  const auto result = selectStrategy(snapshot, evaluatePosition(snapshot),
                                     evaluateAruco(snapshot), evaluateVisual(snapshot));
  EXPECT_EQ(result.strategy, LandingStrategy::Position);

  snapshot.vehicle_failsafe = true;
  const auto stopped = selectStrategy(snapshot, evaluatePosition(snapshot),
                                      evaluateAruco(snapshot), evaluateVisual(snapshot));
  EXPECT_EQ(stopped.strategy, LandingStrategy::SafeHold);
}
