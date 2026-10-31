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

#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "landfall/util/thread_pool.hpp"

using landfall::ThreadPool;

TEST(ThreadPool, ResultsByIndexDoNotDependOnThreadCount)
{
  for (const int threads : {1, 2, 7}) {
    ThreadPool pool(threads);
    EXPECT_EQ(pool.size(), threads);
    for (int round = 0; round < 20; ++round) {
      std::vector<int64_t> out(1000, -1);
      pool.parallelFor(
        out.size(), [&](std::size_t i) { out[i] = static_cast<int64_t>(i * i) + round; });
      for (std::size_t i = 0; i < out.size(); ++i) {
        ASSERT_EQ(out[i], static_cast<int64_t>(i * i) + round);
      }
    }
  }
}

TEST(ThreadPool, EveryIndexRunsExactlyOnce)
{
  ThreadPool pool(4);
  std::vector<std::atomic<int>> calls(5000);
  pool.parallelFor(calls.size(), [&](std::size_t i) { calls[i].fetch_add(1); });
  for (const auto & c : calls) {
    ASSERT_EQ(c.load(), 1);
  }
  int none = 0;
  pool.parallelFor(0, [&](std::size_t) { ++none; });
  EXPECT_EQ(none, 0);
}

TEST(ThreadPool, RethrowsTheLowestIndexAfterAllCallsFinish)
{
  ThreadPool pool(4);
  std::atomic<int> finished{0};
  try {
    pool.parallelFor(200, [&](std::size_t i) {
      finished.fetch_add(1);
      if (i == 150 || i == 30 || i == 90) {
        throw std::runtime_error("item " + std::to_string(i));
      }
    });
    FAIL() << "expected an exception";
  } catch (const std::runtime_error & e) {
    EXPECT_STREQ(e.what(), "item 30");
  }
  EXPECT_EQ(finished.load(), 200);
  // The pool stays usable.
  std::vector<int> out(10, 0);
  pool.parallelFor(out.size(), [&](std::size_t i) { out[i] = 1; });
  EXPECT_EQ(std::vector<int>(10, 1), out);
}
