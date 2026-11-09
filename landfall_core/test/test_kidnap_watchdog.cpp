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

#include "landfall/query/kidnap_watchdog.hpp"

namespace
{

landfall::KidnapWatchdogParams params()
{
  landfall::KidnapWatchdogParams p;
  p.max_translation = 1.0;
  p.max_rotation_deg = 20.0;
  p.confirmations = 2;
  p.cooldown = 30.0;
  return p;
}

}  // namespace

TEST(KidnapWatchdog, FiresAfterEnoughDisagreeingChecksInARow)
{
  landfall::KidnapWatchdog watchdog(params());
  EXPECT_FALSE(watchdog.observe(0.05, 1.0));  // agrees
  EXPECT_FALSE(watchdog.shouldFire(0.0));
  EXPECT_TRUE(watchdog.observe(8.0, 180.0));  // the localization is somewhere else
  EXPECT_EQ(watchdog.disagreements(), 1);
  EXPECT_FALSE(watchdog.shouldFire(10.0));  // one check is not enough
  EXPECT_TRUE(watchdog.observe(7.9, 179.0));
  EXPECT_TRUE(watchdog.shouldFire(20.0));
  watchdog.fired(20.0);
  EXPECT_EQ(watchdog.disagreements(), 0);
  EXPECT_FALSE(watchdog.shouldFire(20.0));
}

TEST(KidnapWatchdog, EitherLimitDisagreesAndTheLimitsThemselvesAgree)
{
  landfall::KidnapWatchdog watchdog(params());
  EXPECT_TRUE(watchdog.observe(1.01, 0.0));
  EXPECT_TRUE(watchdog.observe(0.0, 20.5));
  EXPECT_FALSE(watchdog.observe(1.0, 20.0));  // at the limits: agrees
  EXPECT_EQ(watchdog.disagreements(), 0);
}

TEST(KidnapWatchdog, AnAgreeingCheckRestartsTheCount)
{
  landfall::KidnapWatchdog watchdog(params());
  watchdog.observe(3.0, 0.0);
  watchdog.observe(0.1, 0.5);  // a single wrong check, e.g. one vision got wrong
  watchdog.observe(3.0, 0.0);
  EXPECT_FALSE(watchdog.shouldFire(0.0));
  watchdog.observe(3.0, 0.0);
  EXPECT_TRUE(watchdog.shouldFire(0.0));
  watchdog.restart();  // another request gave the localization a new pose
  EXPECT_FALSE(watchdog.shouldFire(0.0));
}

TEST(KidnapWatchdog, HonorsTheCooldown)
{
  landfall::KidnapWatchdog watchdog(params());
  watchdog.observe(5.0, 90.0);
  watchdog.observe(5.0, 90.0);
  ASSERT_TRUE(watchdog.shouldFire(100.0));
  watchdog.fired(100.0);
  // The relocalization found nothing and the localization stays wrong: the checks still disagree,
  // but the next relocalization waits for the cooldown.
  watchdog.observe(5.0, 90.0);
  watchdog.observe(5.0, 90.0);
  EXPECT_FALSE(watchdog.shouldFire(129.9));
  EXPECT_TRUE(watchdog.shouldFire(130.0));
  watchdog.reset();
  EXPECT_EQ(watchdog.disagreements(), 0);
  watchdog.observe(5.0, 90.0);
  watchdog.observe(5.0, 90.0);
  EXPECT_TRUE(watchdog.shouldFire(100.5));  // no cooldown after a reset
}

TEST(KidnapWatchdog, OneConfirmationFiresAtOnce)
{
  landfall::KidnapWatchdogParams p = params();
  p.confirmations = 1;
  p.cooldown = 0.0;
  landfall::KidnapWatchdog watchdog(p);
  watchdog.observe(2.0, 0.0);
  EXPECT_TRUE(watchdog.shouldFire(0.0));
  watchdog.fired(0.0);
  watchdog.observe(2.0, 0.0);
  EXPECT_TRUE(watchdog.shouldFire(0.0));
}
