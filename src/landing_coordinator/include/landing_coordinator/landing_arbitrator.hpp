#pragma once

#include <cstdint>
#include <mutex>
#include <string>

namespace landing_coordinator {

struct LandingDecision {
  bool granted = false;
  std::string owner_id;
  std::uint64_t mission_id = 0;
  std::string reason;
};

struct CompletionDecision {
  bool accepted = false;
  bool completed = false;
  bool newly_completed = false;
  std::string reason;
};

// Thread-safe, in-memory arbitrator for ONE landing resource and ONE mission.
// Completion does NOT release the landing location for another drone.
class LandingArbitrator {
 public:
  LandingDecision requestLanding(const std::string& drone_id);
  CompletionDecision reportLandingComplete(const std::string& drone_id, std::uint64_t mission_id,
                                           bool success);

 private:
  std::mutex mutex_;
  std::string owner_id_;
  std::uint64_t mission_id_ = 0;
  bool completed_ = false;
};

}  // namespace landing_coordinator
