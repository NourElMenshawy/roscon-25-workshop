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
    case MissionState::LANDED:
      return "LANDED";
    case MissionState::READY_FOR_SELECTION:
      return "READY_FOR_SELECTION";
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
    owner_id_ = std::string(owner_id);
  } else if (!granted && owner_id != local_drone_id) {
    state_ = MissionState::FOLLOWER;
    mission_id_ = mission_id;
    owner_id_ = std::string(owner_id);
  } else {
    // Invalid/inconsistent server response: do not assign a flight role.
    state_ = MissionState::SEARCHING;
  }
}

bool MissionPolicy::secondaryLandingComplete(std::uint64_t mission_id) noexcept {
  if (state_ != MissionState::READY_FOR_SELECTION || !completion_seen_ ||
      mission_id == 0 || mission_id != mission_id_) {
    return false;
  }
  state_ = MissionState::LANDED;
  return true;
}

bool MissionPolicy::receiveStatus(std::uint8_t resource_state,
                                  std::string_view owner_id,
                                  std::string_view local_drone_id,
                                  std::uint64_t mission_id) {
  constexpr std::uint8_t kAvailable = 0;
  constexpr std::uint8_t kReserved = 1;
  constexpr std::uint8_t kCompleted = 2;

  if (resource_state == kAvailable) {
    // An initial AVAILABLE cannot undo a mission already seen.
    return mission_id == 0 && owner_id.empty() && mission_id_ == 0;
  }
  if ((resource_state != kReserved && resource_state != kCompleted) ||
      mission_id == 0 || owner_id.empty() || local_drone_id.empty()) {
    return false;
  }
  if (mission_id < mission_id_ ||
      (mission_id == mission_id_ && !owner_id_.empty() && owner_id_ != owner_id)) {
    return false;
  }
  // A delayed RESERVED sample cannot reverse a COMPLETED transition.
  if (mission_id == mission_id_ && completion_seen_ &&
      resource_state == kReserved) {
    return false;
  }
  const bool new_mission = mission_id > mission_id_;
  if (new_mission) {
    completion_seen_ = false;
    state_ = MissionState::SEARCHING;
  }

  mission_id_ = mission_id;
  owner_id_ = std::string(owner_id);
  if (resource_state == kCompleted) {
    completion_seen_ = true;
    // Retained coordinator replays must not undo confirmed secondary touchdown.
    if (state_ != MissionState::LANDED || mission_id != mission_id_ ||
        owner_id == local_drone_id) {
      state_ = owner_id == local_drone_id ? MissionState::LANDED
                                          : MissionState::READY_FOR_SELECTION;
    }
  } else {
    state_ = owner_id == local_drone_id ? MissionState::PRIMARY
                                        : MissionState::FOLLOWER;
  }
  return true;
}

}  // namespace landing_mission_controller
