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

#include "landfall/util/thread_pool.hpp"

#include <algorithm>
#include <limits>

namespace landfall
{

ThreadPool::ThreadPool(int threads)
{
  if (threads <= 0) {
    threads = static_cast<int>(std::max(1U, std::thread::hardware_concurrency()));
  }
  workers_.reserve(static_cast<std::size_t>(threads - 1));
  for (int i = 1; i < threads; ++i) {
    workers_.emplace_back([this] { workerLoop(); });
  }
}

ThreadPool::~ThreadPool()
{
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
  }
  wake_.notify_all();
  for (auto & worker : workers_) {
    worker.join();
  }
}

void ThreadPool::parallelFor(std::size_t n, const std::function<void(std::size_t)> & fn)
{
  if (n == 0) {
    return;
  }
  if (workers_.empty() || n == 1) {
    std::exception_ptr error;
    for (std::size_t i = 0; i < n; ++i) {
      try {
        fn(i);
      } catch (...) {
        if (!error) {
          error = std::current_exception();
        }
      }
    }
    if (error) {
      std::rethrow_exception(error);
    }
    return;
  }

  {
    const std::lock_guard<std::mutex> lock(mutex_);
    job_ = &fn;
    job_size_ = n;
    next_.store(0);
    error_ = nullptr;
    error_index_ = std::numeric_limits<std::size_t>::max();
    busy_workers_ = workers_.size();
    ++generation_;
  }
  wake_.notify_all();
  runItems();
  std::exception_ptr error;
  {
    std::unique_lock<std::mutex> lock(mutex_);
    // Every worker acknowledges the job, so none can still be reading job_ for this one when the
    // next parallelFor() replaces it.
    done_.wait(lock, [this] { return busy_workers_ == 0; });
    job_ = nullptr;
    error = error_;
    error_ = nullptr;
  }
  if (error) {
    std::rethrow_exception(error);
  }
}

void ThreadPool::workerLoop()
{
  uint64_t seen = 0;
  while (true) {
    {
      std::unique_lock<std::mutex> lock(mutex_);
      wake_.wait(lock, [this, seen] { return stop_ || generation_ != seen; });
      if (stop_) {
        return;
      }
      seen = generation_;
    }
    runItems();
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      if (--busy_workers_ == 0) {
        done_.notify_one();
      }
    }
  }
}

void ThreadPool::runItems()
{
  while (true) {
    const std::size_t i = next_.fetch_add(1);
    if (i >= job_size_) {
      return;
    }
    try {
      (*job_)(i);
    } catch (...) {
      const std::lock_guard<std::mutex> lock(mutex_);
      if (i < error_index_) {
        error_ = std::current_exception();
        error_index_ = i;
      }
    }
  }
}

}  // namespace landfall
