#include "landing_mission_controller/mission_policy.hpp"

namespace landing_mission_controller {

const char* stateName(MissionState state) noexcept {
  switch (state) {
    case MissionState::SEARCHING:
      return "SEARCHING";
    case MissionState::WAITING_TO_REQUEST:
      return "WAITING_TO_REQUEST";
    case MissionState::REQUESTING:
      return "REQUESTING";
    case MissionState::PRIMARY:
      return "PRIMARY";
    case MissionState::FOLLOWER:
      return "FOLLOWER";
  }
  return "UNKNOWN";
}

void MissionPolicy::observeTag(TimePoint now) {
  if (state_ == MissionState::SEARCHING) {
    first_seen_ = now;
    state_ = MissionState::WAITING_TO_REQUEST;
  }
  if (state_ == MissionState::WAITING_TO_REQUEST) {
    last_seen_ = now;
  }
}

void MissionPolicy::expireObservation(TimePoint now, std::chrono::milliseconds max_age) {
  if (state_ == MissionState::WAITING_TO_REQUEST && now - last_seen_ > max_age) {
    state_ = MissionState::SEARCHING;
  }
}

bool MissionPolicy::readyToRequest(TimePoint now,
                                   std::chrono::milliseconds delay,
                                   std::chrono::milliseconds max_age) const {
  return state_ == MissionState::WAITING_TO_REQUEST &&
         now - first_seen_ >= delay && now - last_seen_ <= max_age;
}

void MissionPolicy::requestSent() {
  if (state_ == MissionState::WAITING_TO_REQUEST) {
    state_ = MissionState::REQUESTING;
  }
}

void MissionPolicy::requestTimedOut() {
  if (state_ == MissionState::REQUESTING) {
    // Retry only if a new observation arrives; do not keep claiming ownership blindly.
    state_ = MissionState::SEARCHING;
  }
}

void MissionPolicy::receiveDecision(bool granted, std::string_view owner_id,
                                    std::string_view local_drone_id,
                                    std::uint64_t mission_id) {
  if (state_ != MissionState::REQUESTING) {
    return;
  }

  if (mission_id == 0 || owner_id.empty()) {
    state_ = MissionState::SEARCHING;
    return;
  }

  if (granted && owner_id == local_drone_id) {
    state_ = MissionState::PRIMARY;
    mission_id_ = mission_id;
  } else if (!granted && owner_id != local_drone_id) {
    state_ = MissionState::FOLLOWER;
    mission_id_ = mission_id;
  } else {
    // Invalid/inconsistent server response: do not assign a flight role.
    state_ = MissionState::SEARCHING;
  }
}

}  // namespace landing_mission_controller
