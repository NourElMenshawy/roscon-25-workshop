#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <string_view>

namespace landing_mission_controller {

enum class MissionState {
  SEARCHING,
  WAITING_TO_REQUEST,
  REQUESTING,
  PRIMARY,
  FOLLOWER,
  LANDED,
  READY_FOR_SELECTION
};

const char* stateName(MissionState state) noexcept;

// Pure state transitions: no ROS dependency or PX4 mode commands.
class MissionPolicy {
 public:
  using TimePoint = std::chrono::steady_clock::time_point;

  void observeTag(TimePoint now);
  void expireObservation(TimePoint now, std::chrono::milliseconds max_age);
  [[nodiscard]] bool readyToRequest(TimePoint now, std::chrono::milliseconds delay,
                                    std::chrono::milliseconds max_age) const;
  void requestSent();
  void requestTimedOut();
  void receiveDecision(bool granted, std::string_view owner_id, std::string_view local_drone_id,
                       std::uint64_t mission_id);

  [[nodiscard]] bool receiveStatus(std::uint8_t resource_state, std::string_view owner_id,
                                   std::string_view local_drone_id, std::uint64_t mission_id);

  [[nodiscard]] MissionState state() const noexcept { return state_; }
  [[nodiscard]] std::uint64_t missionId() const noexcept { return mission_id_; }

 private:
  MissionState state_{MissionState::SEARCHING};
  TimePoint first_seen_{};
  TimePoint last_seen_{};
  std::uint64_t mission_id_{0};
  std::string owner_id_;
  bool completion_seen_{false};
};

}  // namespace landing_mission_controller
