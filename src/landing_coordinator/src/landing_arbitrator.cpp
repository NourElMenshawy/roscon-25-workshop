#include "landing_coordinator/landing_arbitrator.hpp"

#include <mutex>
#include <string>

namespace landing_coordinator {

LandingDecision LandingArbitrator::requestLanding(const std::string& drone_id)
{
  // Protect the complete read/decide/write transaction, not just assignment.
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
    // Retries must not change ownership or create a second mission.
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
