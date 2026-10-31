#include "landing_coordinator/landing_arbitrator.hpp"

#include <mutex>
#include <string>

namespace landing_coordinator {

LandingDecision LandingArbitrator::requestLanding(const std::string& drone_id)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (drone_id.find_first_not_of(" \t\n\r") == std::string::npos) {
    return {.granted = false,
            .owner_id = owner_id_,
            .mission_id = mission_id_,
            .reason = "invalid_drone_id"};
  }

  if (owner_id_.empty()) {
    owner_id_ = drone_id;
    mission_id_ = 1;
    return {.granted = true, .owner_id = owner_id_, .mission_id = mission_id_, .reason = "granted"};
  }

  if (owner_id_ == drone_id) {
    return {.granted = true,
            .owner_id = owner_id_,
            .mission_id = mission_id_,
            .reason = "already_owner"};
  }

  return {.granted = false,
          .owner_id = owner_id_,
          .mission_id = mission_id_,
          .reason = "reserved_by_other"};
}

}  // namespace landing_coordinator

namespace landing_coordinator {

CompletionDecision LandingArbitrator::reportLandingComplete(const std::string& drone_id,
                                                            std::uint64_t mission_id, bool success)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (drone_id.find_first_not_of(" \t\n\r") == std::string::npos) {
    return {.reason = "invalid_drone_id"};
  }
  if (owner_id_.empty()) {
    return {.reason = "no_active_mission"};
  }
  if (mission_id != mission_id_) {
    return {.reason = "wrong_mission_id"};
  }
  if (drone_id != owner_id_) {
    return {.reason = "not_owner"};
  }
  if (!success) {
    return {.completed = completed_, .reason = "unsuccessful_result"};
  }
  if (completed_) {
    return {.accepted = true, .completed = true, .reason = "already_completed"};
  }

  completed_ = true;
  return {.accepted = true, .completed = true, .newly_completed = true, .reason = "completed"};
}

}  // namespace landing_coordinator
