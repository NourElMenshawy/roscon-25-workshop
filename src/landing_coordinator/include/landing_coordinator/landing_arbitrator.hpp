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

// Thread-safe, in-memory arbitrator for ONE landing resource and ONE mission.
// A reservation is never released by this initial implementation.
class LandingArbitrator {
 public:
  LandingDecision requestLanding(const std::string& drone_id);

 private:
  std::mutex mutex_;
  std::string owner_id_;
  std::uint64_t mission_id_ = 0;
};

}  // namespace landing_coordinator
