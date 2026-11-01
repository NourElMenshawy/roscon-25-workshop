#pragma once

#include <cstdint>

namespace precision_land_executor {

enum class LandingStrategy : std::uint8_t {
  Position,
  Aruco,
  Visual,
  SafeHold,
};

enum class SelectionOutcome : std::uint8_t {
  Selected,
  Error,
};

// Immutable, frame-aware *availability* snapshot. This milestone never
// commands a landing; a true site_verified flag must come from an actual
// safety/occupancy check, never merely from detecting a marker.
struct StrategySnapshot {
  std::uint64_t mission_id{0};
  bool shared_position_available{false};
  bool shared_position_frame_valid{false};
  bool position_site_verified{false};
  bool aruco_fresh{false};
  bool aruco_site_verified{false};
  bool visual_estimator_ready{false};
  bool visual_site_verified{false};
  bool vehicle_failsafe{false};
};

enum class EligibilityReason : std::uint8_t {
  Eligible,
  Failsafe,
  MissingTarget,
  InvalidFrame,
  UnverifiedSite,
  StaleTarget,
  NotImplemented,
  Cancelled,
  NotEvaluated,
};

struct Eligibility {
  bool eligible{false};
  EligibilityReason reason{EligibilityReason::NotEvaluated};
};

struct SelectionResult {
  std::uint64_t mission_id{0};
  LandingStrategy strategy{LandingStrategy::SafeHold};
  SelectionOutcome outcome{SelectionOutcome::Selected};
  Eligibility position{};
  Eligibility aruco{};
  Eligibility visual{};
};

inline constexpr const char* toString(LandingStrategy value) noexcept
{
  switch (value) {
    case LandingStrategy::Position:
      return "POSITION_LANDING";
    case LandingStrategy::Aruco:
      return "ARUCO_LANDING";
    case LandingStrategy::Visual:
      return "VISUAL_LANDING";
    case LandingStrategy::SafeHold:
      return "SAFE_HOLD";
  }
  return "UNKNOWN";
}

inline constexpr const char* toString(EligibilityReason value) noexcept
{
  switch (value) {
    case EligibilityReason::Eligible:
      return "eligible";
    case EligibilityReason::Failsafe:
      return "vehicle_failsafe";
    case EligibilityReason::MissingTarget:
      return "missing_target";
    case EligibilityReason::InvalidFrame:
      return "invalid_shared_frame";
    case EligibilityReason::UnverifiedSite:
      return "landing_site_not_verified";
    case EligibilityReason::StaleTarget:
      return "aruco_not_fresh";
    case EligibilityReason::NotImplemented:
      return "estimator_not_available";
    case EligibilityReason::Cancelled:
      return "cancelled";
    case EligibilityReason::NotEvaluated:
      return "not_evaluated";
  }
  return "unknown";
}

inline constexpr Eligibility evaluatePosition(const StrategySnapshot& snapshot) noexcept
{
  if (snapshot.vehicle_failsafe) {
    return {.eligible = false, .reason = EligibilityReason::Failsafe};
  }
  if (!snapshot.shared_position_available) {
    return {.eligible = false, .reason = EligibilityReason::MissingTarget};
  }
  if (!snapshot.shared_position_frame_valid) {
    return {.eligible = false, .reason = EligibilityReason::InvalidFrame};
  }
  if (!snapshot.position_site_verified) {
    return {.eligible = false, .reason = EligibilityReason::UnverifiedSite};
  }
  return {.eligible = true, .reason = EligibilityReason::Eligible};
}

inline constexpr Eligibility evaluateAruco(const StrategySnapshot& snapshot) noexcept
{
  if (snapshot.vehicle_failsafe) {
    return {.eligible = false, .reason = EligibilityReason::Failsafe};
  }
  if (!snapshot.aruco_fresh) {
    return {.eligible = false, .reason = EligibilityReason::StaleTarget};
  }
  if (!snapshot.aruco_site_verified) {
    return {.eligible = false, .reason = EligibilityReason::UnverifiedSite};
  }
  return {.eligible = true, .reason = EligibilityReason::Eligible};
}

inline constexpr Eligibility evaluateVisual(const StrategySnapshot& snapshot) noexcept
{
  if (snapshot.vehicle_failsafe) {
    return {.eligible = false, .reason = EligibilityReason::Failsafe};
  }
  if (!snapshot.visual_estimator_ready) {
    return {.eligible = false, .reason = EligibilityReason::NotImplemented};
  }
  if (!snapshot.visual_site_verified) {
    return {.eligible = false, .reason = EligibilityReason::UnverifiedSite};
  }
  return {.eligible = true, .reason = EligibilityReason::Eligible};
}

// Deterministic priority policy; *always* fail closed if the vehicle is in
// failsafe, even if a check is accidentally marked eligible by its provider.
inline constexpr SelectionResult selectStrategy(const StrategySnapshot& snapshot,
                                                Eligibility position, Eligibility aruco,
                                                Eligibility visual) noexcept
{
  SelectionResult result{};
  result.mission_id = snapshot.mission_id;
  result.position = position;
  result.aruco = aruco;
  result.visual = visual;
  if (!snapshot.vehicle_failsafe) {
    if (position.eligible) {
      result.strategy = LandingStrategy::Position;
    } else if (aruco.eligible) {
      result.strategy = LandingStrategy::Aruco;
    } else if (visual.eligible) {
      result.strategy = LandingStrategy::Visual;
    }
  }
  return result;
}

}  // namespace precision_land_executor
