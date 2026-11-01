#include "precision_land_executor/follower_strategy_selector.hpp"

#include <utility>

namespace precision_land_executor {

FollowerStrategySelector::FollowerStrategySelector() : FollowerStrategySelector(Checks{})
{
}

FollowerStrategySelector::FollowerStrategySelector(Checks checks) : checks_(checks)
{
}

FollowerStrategySelector::~FollowerStrategySelector()
{
  cancel();
  scope_.request_stop();
  // A worker may still reference this object; join before destroying members.
  (void)stdexec::sync_wait(scope_.on_empty());
}

void FollowerStrategySelector::cancel() noexcept
{
  std::scoped_lock lock(mutex_);
  stop_source_.request_stop();
  ++generation_;
  completed_.reset();
}

void FollowerStrategySelector::deliver(SelectionResult result, std::uint64_t generation,
                                       std::stop_token token) noexcept
{
  std::scoped_lock lock(mutex_);
  if (generation == generation_ && !token.stop_requested()) {
    completed_ = result;
  }
}

bool FollowerStrategySelector::start(StrategySnapshot snapshot)
{
  if (snapshot.mission_id == 0 || !checks_.position || !checks_.aruco || !checks_.visual) {
    return false;
  }

  std::uint64_t generation;
  std::stop_token token;
  {
    std::scoped_lock lock(mutex_);
    stop_source_.request_stop();
    stop_source_ = std::stop_source{};
    token = stop_source_.get_token();
    generation = ++generation_;
    completed_.reset();
  }

  try {
    auto scheduler = pool_.get_scheduler();

    // Each independent check is explicitly scheduled on the worker pool.
    auto position =
        stdexec::schedule(scheduler) | stdexec::then([snapshot, token, check = checks_.position]() {
          return token.stop_requested() ? Eligibility{false, EligibilityReason::Cancelled}
                                        : check(snapshot);
        });
    auto aruco =
        stdexec::schedule(scheduler) | stdexec::then([snapshot, token, check = checks_.aruco]() {
          return token.stop_requested() ? Eligibility{false, EligibilityReason::Cancelled}
                                        : check(snapshot);
        });
    auto visual =
        stdexec::schedule(scheduler) | stdexec::then([snapshot, token, check = checks_.visual]() {
          return token.stop_requested() ? Eligibility{false, EligibilityReason::Cancelled}
                                        : check(snapshot);
        });

    auto pipeline = stdexec::when_all(std::move(position), std::move(aruco), std::move(visual)) |
                    stdexec::then([this, snapshot, generation, token](Eligibility p, Eligibility a,
                                                                      Eligibility v) {
                      deliver(selectStrategy(snapshot, p, a, v), generation, token);
                    }) |
                    stdexec::upon_error([this, mission_id = snapshot.mission_id, generation,
                                         token](auto&&) noexcept {
                      // An exception from any branch fails the entire selection closed.
                      SelectionResult result{};
                      result.mission_id = mission_id;
                      result.outcome = SelectionOutcome::Error;
                      deliver(result, generation, token);
                    }) |
                    stdexec::upon_stopped([]() noexcept {
                      // A stopped pipeline must not publish an obsolete strategy.
                    });

    scope_.spawn(std::move(pipeline));
  } catch (...) {
    SelectionResult result{};
    result.mission_id = snapshot.mission_id;
    result.outcome = SelectionOutcome::Error;
    deliver(result, generation, token);
    return false;
  }
  return true;
}

std::optional<SelectionResult> FollowerStrategySelector::takeResult() noexcept
{
  std::scoped_lock lock(mutex_);
  auto result = completed_;
  completed_.reset();
  return result;
}

}  // namespace precision_land_executor
