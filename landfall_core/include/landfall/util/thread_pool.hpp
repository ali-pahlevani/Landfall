// Copyright 2026 Ali Pahlevani
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef LANDFALL_UTIL_THREAD_POOL_HPP_
#define LANDFALL_UTIL_THREAD_POOL_HPP_

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace landfall
{

/// Fixed set of worker threads for data-parallel loops. Callers write results by index, so the
/// output doesn't depend on scheduling and parallel map builds stay deterministic.
class ThreadPool
{
public:
  /// `threads` counts the calling thread too; 0 means the hardware concurrency, 1 runs every
  /// loop on the calling thread.
  explicit ThreadPool(int threads = 0);
  ThreadPool(const ThreadPool &) = delete;
  ThreadPool & operator=(const ThreadPool &) = delete;
  ~ThreadPool();

  int size() const { return static_cast<int>(workers_.size()) + 1; }

  /// Calls fn(i) for every i in [0, n) and returns when all calls have finished. If calls throw,
  /// the exception of the lowest index is rethrown (after all calls have run). Not reentrant:
  /// fn must not call parallelFor on the same pool.
  void parallelFor(std::size_t n, const std::function<void(std::size_t)> & fn);

private:
  void workerLoop();
  void runItems();

  std::vector<std::thread> workers_;
  std::mutex mutex_;
  std::condition_variable wake_;
  std::condition_variable done_;
  const std::function<void(std::size_t)> * job_ = nullptr;
  std::size_t job_size_ = 0;
  std::atomic<std::size_t> next_{0};
  std::size_t busy_workers_ = 0;
  uint64_t generation_ = 0;
  bool stop_ = false;
  std::exception_ptr error_;
  std::size_t error_index_ = 0;
};

}  // namespace landfall

#endif  // LANDFALL_UTIL_THREAD_POOL_HPP_
