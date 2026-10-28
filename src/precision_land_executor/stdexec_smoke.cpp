#include <exec/static_thread_pool.hpp>
#include <stdexec/execution.hpp>

#include <iostream>
#include <thread>
#include <utility>

int main() {
  exec::static_thread_pool pool{2};

  auto scheduler = pool.get_scheduler();

  std::cout << "Main thread: " << std::this_thread::get_id() << '\n';

  // Construct a lazy computation.
  auto pipeline = stdexec::schedule(scheduler) | stdexec::then([] {
                    std::cout << "Worker thread: " << std::this_thread::get_id()
                              << '\n';

                    std::cout << "Processing observation\n";
                    return 42;
                  }) |
                  stdexec::then([](int observation_id) {
                    std::cout << "Validating observation\n";
                    return observation_id + 1;
                  });

  std::cout << "Pipeline constructed\n";

  // Start execution and wait for completion.
  auto result = stdexec::sync_wait(std::move(pipeline));

  if (!result) {
    std::cerr << "Pipeline stopped\n";
    return 1;
  }

  const auto [id] = *result;

  std::cout << "Processed observation: " << id << '\n';
}
