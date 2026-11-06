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

#include "landfall/query/auto_trigger.hpp"

namespace
{

landfall::AutoTriggerParams params()
{
  landfall::AutoTriggerParams p;
  p.max_std_xy = 0.5;
  p.max_std_yaw_deg = 15.0;
  p.duration = 5.0;
  p.cooldown = 30.0;
  return p;
}

}  // namespace

TEST(AutoTrigger, FiresAfterTheUncertaintyLastsLongEnough)
{
  landfall::AutoTrigger trigger(params());
  trigger.observe(0.0, 0.1, 2.0);
  EXPECT_FALSE(trigger.shouldFire(10.0, false));
  trigger.observe(10.0, 0.8, 2.0);  // position uncertain from t = 10
  EXPECT_TRUE(trigger.uncertain());
  EXPECT_FALSE(trigger.shouldFire(14.9, false));
  // AMCL publishes only after filter updates: the last estimate holds.
  EXPECT_TRUE(trigger.shouldFire(15.0, false));
  EXPECT_FALSE(trigger.shouldFire(15.0, true));  // never while a request runs
}

TEST(AutoTrigger, YawAloneIsEnoughAndConfidenceResets)
{
  landfall::AutoTrigger trigger(params());
  trigger.observe(0.0, 0.1, 20.0);
  trigger.observe(3.0, 0.1, 2.0);  // confident again before the duration
  trigger.observe(4.0, 0.1, 20.0);
  EXPECT_FALSE(trigger.shouldFire(8.0, false));
  EXPECT_TRUE(trigger.shouldFire(9.0, false));
}

TEST(AutoTrigger, HonorsTheCooldown)
{
  landfall::AutoTrigger trigger(params());
  trigger.observe(0.0, 1.0, 30.0);
  ASSERT_TRUE(trigger.shouldFire(5.0, false));
  trigger.fired(5.0);
  for (int step = 0; step < 60; ++step) {  // every 0.5 s through the 30 s cooldown
    const double t = 5.0 + 0.5 * step;
    trigger.observe(t, 1.0, 30.0);
    EXPECT_FALSE(trigger.shouldFire(t, false)) << t;
  }
  EXPECT_TRUE(trigger.shouldFire(35.0, false));  // still uncertain: try again
  trigger.reset();
  EXPECT_FALSE(trigger.uncertain());
  EXPECT_FALSE(trigger.shouldFire(100.0, false));
}
