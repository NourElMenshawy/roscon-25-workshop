#pragma once

#include <cstdint>
#include <exec/async_scope.hpp>
#include <exec/static_thread_pool.hpp>
#include <mutex>
#include <optional>
#include <stdexec/execution.hpp>
#include <stop_token>

#include "precision_land_executor/landing_strategy.hpp"

namespace precision_land_executor {

// Pure checks are injectable to permit deterministic tests of errors and
// cancellation. No ROS / PX4 calls occur on selector worker threads.
class FollowerStrategySelector {
 public:
  using Check = Eligibility (*)(const StrategySnapshot&);
  struct Checks {
    Check position{&evaluatePosition};
    Check aruco{&evaluateAruco};
    Check visual{&evaluateVisual};
  };

  FollowerStrategySelector();
  explicit FollowerStrategySelector(Checks checks);
  ~FollowerStrategySelector();

  FollowerStrategySelector(const FollowerStrategySelector&) = delete;
  FollowerStrategySelector& operator=(const FollowerStrategySelector&) = delete;

  // Starts one generation, invalidating any prior pending result.
  // Returns false only for invalid input or failure to schedule the pipeline.
  bool start(StrategySnapshot snapshot);
  void cancel() noexcept;
  std::optional<SelectionResult> takeResult() noexcept;

 private:
  void deliver(SelectionResult result, std::uint64_t generation, std::stop_token token) noexcept;

  Checks checks_;
  // Declare scheduler before scope so the pool survives all scoped work.
  exec::static_thread_pool pool_{3};
  exec::async_scope scope_;
  std::mutex mutex_;
  std::stop_source stop_source_;
  std::uint64_t generation_{0};
  std::optional<SelectionResult> completed_;
};

}  // namespace precision_land_executor
