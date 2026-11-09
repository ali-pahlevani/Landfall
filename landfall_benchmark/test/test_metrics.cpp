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

#include <cmath>

#include "landfall/benchmark/metrics.hpp"

using landfall::benchmark::clopperPearsonLower;
using landfall::benchmark::clopperPearsonUpper;

TEST(ClopperPearson, NoEventsHasAClosedForm)
{
  // k = 0: the upper end is 1 - (alpha / 2)^(1 / n).
  for (const int n : {1, 10, 100, 1000}) {
    EXPECT_NEAR(clopperPearsonUpper(0, n), 1.0 - std::pow(0.025, 1.0 / n), 1e-9) << n;
    EXPECT_EQ(clopperPearsonLower(0, n), 0.0);
  }
  EXPECT_NEAR(clopperPearsonUpper(0, 100), 0.03622, 1e-5);  // the benchmark's 100 placements
}

TEST(ClopperPearson, MatchesPublishedIntervals)
{
  // Two-sided 95 % intervals, e.g. from the beta quantiles Beta(0.025; k, n - k + 1) and
  // Beta(0.975; k + 1, n - k).
  EXPECT_NEAR(clopperPearsonLower(5, 10), 0.18709, 1e-5);
  EXPECT_NEAR(clopperPearsonUpper(5, 10), 0.81291, 1e-5);
  EXPECT_NEAR(clopperPearsonLower(1, 100), 0.000253, 1e-6);
  EXPECT_NEAR(clopperPearsonUpper(1, 100), 0.05446, 1e-5);
  EXPECT_EQ(clopperPearsonUpper(10, 10), 1.0);
  EXPECT_EQ(clopperPearsonUpper(0, 0), 1.0);
  // A wider confidence widens the interval.
  EXPECT_GT(clopperPearsonUpper(0, 100, 0.99), clopperPearsonUpper(0, 100, 0.95));
}

TEST(Outcomes, LimitsAndNames)
{
  const landfall::benchmark::ErrorLimits limits;
  EXPECT_TRUE(landfall::benchmark::isRight(0.25, 5.0, limits));
  EXPECT_FALSE(landfall::benchmark::isRight(0.26, 1.0, limits));
  EXPECT_FALSE(landfall::benchmark::isWrong(0.5, 10.0, limits));
  EXPECT_TRUE(landfall::benchmark::isWrong(0.1, 10.5, limits));
  EXPECT_TRUE(landfall::benchmark::isWrong(0.51, 0.0, limits));
  using landfall::benchmark::Outcome;
  for (const Outcome o :
       {Outcome::kSuccess, Outcome::kWrong, Outcome::kInaccurate, Outcome::kMiss,
        Outcome::kInvalid}) {
    EXPECT_EQ(landfall::benchmark::parseOutcome(landfall::benchmark::toString(o)), o);
  }
  EXPECT_FALSE(landfall::benchmark::parseOutcome("maybe").has_value());
}
