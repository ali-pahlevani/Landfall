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
  watchdog.fired();
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

TEST(KidnapWatchdog, HonorsTheCooldownAfterAPublishedPose)
{
  landfall::KidnapWatchdog watchdog(params());
  watchdog.observe(5.0, 90.0);
  watchdog.observe(5.0, 90.0);
  ASSERT_TRUE(watchdog.shouldFire(100.0));
  watchdog.fired();
  watchdog.published(101.0);
  // The published pose was wrong after all (the checks keep disagreeing with it): the next
  // relocalization waits for the cooldown.
  watchdog.observe(5.0, 90.0);
  watchdog.observe(5.0, 90.0);
  EXPECT_FALSE(watchdog.shouldFire(130.9));
  EXPECT_TRUE(watchdog.shouldFire(131.0));
  watchdog.reset();
  EXPECT_EQ(watchdog.disagreements(), 0);
  watchdog.observe(5.0, 90.0);
  watchdog.observe(5.0, 90.0);
  EXPECT_TRUE(watchdog.shouldFire(101.5));  // no cooldown after a reset
}

TEST(KidnapWatchdog, TriesAgainAfterARelocalizationThatPublishedNothing)
{
  landfall::KidnapWatchdog watchdog(params());
  watchdog.observe(5.0, 90.0);
  watchdog.observe(5.0, 90.0);
  ASSERT_TRUE(watchdog.shouldFire(100.0));
  watchdog.fired();
  // The relocalization found no pose and the localization stays wrong: no cooldown, the next two
  // disagreeing checks start another one.
  watchdog.observe(5.0, 90.0);
  EXPECT_FALSE(watchdog.shouldFire(110.0));
  watchdog.observe(5.0, 90.0);
  EXPECT_TRUE(watchdog.shouldFire(120.0));
}

TEST(KidnapWatchdog, OneConfirmationFiresAtOnce)
{
  landfall::KidnapWatchdogParams p = params();
  p.confirmations = 1;
  p.cooldown = 0.0;
  landfall::KidnapWatchdog watchdog(p);
  watchdog.observe(2.0, 0.0);
  EXPECT_TRUE(watchdog.shouldFire(0.0));
  watchdog.fired();
  watchdog.published(0.0);
  watchdog.observe(2.0, 0.0);
  EXPECT_TRUE(watchdog.shouldFire(0.0));
}

TEST(KidnapWatchdog, ScheduledChecksRunOncePerPeriod)
{
  landfall::KidnapWatchdogParams p = params();
  p.period = 10.0;
  landfall::KidnapWatchdog watchdog(p);
  using landfall::WatchdogCheck;
  EXPECT_EQ(watchdog.due(100.0), WatchdogCheck::kNone);  // the first one a period after the start
  EXPECT_EQ(watchdog.due(109.9), WatchdogCheck::kNone);
  ASSERT_EQ(watchdog.due(110.0), WatchdogCheck::kScheduled);
  watchdog.started(110.0);
  EXPECT_EQ(watchdog.due(119.9), WatchdogCheck::kNone);
  EXPECT_EQ(watchdog.due(120.0), WatchdogCheck::kScheduled);
  // Without checks, the budget fills up to confirmations + 2 and no further.
  EXPECT_DOUBLE_EQ(watchdog.budget(1000.0), 4.0);
}

TEST(KidnapWatchdog, ADisagreeingCheckIsConfirmedAtOnce)
{
  landfall::KidnapWatchdog watchdog(params());
  using landfall::WatchdogCheck;
  ASSERT_EQ(watchdog.due(10.0), WatchdogCheck::kNone);
  ASSERT_EQ(watchdog.due(20.0), WatchdogCheck::kScheduled);
  watchdog.started(20.0);
  EXPECT_TRUE(watchdog.observe(5.0, 90.0));                // the scheduled check disagrees
  EXPECT_EQ(watchdog.due(21.0), WatchdogCheck::kConfirm);  // not 10 s later
  watchdog.started(21.0);
  EXPECT_EQ(watchdog.due(21.5), WatchdogCheck::kNone);  // the confirmation is running
  EXPECT_TRUE(watchdog.observe(5.0, 90.0));
  EXPECT_TRUE(watchdog.shouldFire(22.0));
  watchdog.fired();
  EXPECT_EQ(watchdog.due(22.0), WatchdogCheck::kNone);
  // An agreeing check needs no confirmation.
  landfall::KidnapWatchdog other(params());
  other.due(0.0);
  other.observe(5.0, 90.0);
  other.observe(0.1, 1.0);
  EXPECT_EQ(other.due(1.0), WatchdogCheck::kNone);
}

TEST(KidnapWatchdog, ASuspicionRunsAndConfirmsAtOnceEvenRightAfterAScheduledCheck)
{
  landfall::KidnapWatchdog watchdog(params());
  using landfall::WatchdogCheck;
  watchdog.due(0.0);
  ASSERT_EQ(watchdog.due(10.0), WatchdogCheck::kScheduled);
  watchdog.started(10.0);
  watchdog.observe(0.05, 1.0);  // agrees
  // A kidnap just after: the scan stops fitting, and the check and its confirmation run at once.
  watchdog.suspect(WatchdogCheck::kScanFit);
  ASSERT_EQ(watchdog.due(11.0), WatchdogCheck::kScanFit);
  watchdog.started(11.0);
  watchdog.observe(6.0, 120.0);
  ASSERT_EQ(watchdog.due(12.0), WatchdogCheck::kConfirm);
  watchdog.started(12.0);
  watchdog.observe(6.0, 120.0);
  EXPECT_TRUE(watchdog.shouldFire(13.0));
}

TEST(KidnapWatchdog, EarlyChecksNeverAddChecks)
{
  // The scan never fits (a map that misses half the building) and the suspicion is raised all the
  // time: the watchdog still runs no more than one check per period, plus its budget.
  for (const bool suspicious : {false, true}) {
    landfall::KidnapWatchdog watchdog(params());
    int checks = 0;
    for (int tick = 0; tick <= 100000; ++tick) {  // 1000 s in 10 ms ticks
      const double t = tick * 0.01;
      if (suspicious) {
        watchdog.suspect(landfall::WatchdogCheck::kScanFit);
      }
      if (watchdog.due(t) != landfall::WatchdogCheck::kNone) {
        watchdog.started(t);
        ++checks;
      }
    }
    if (suspicious) {
      EXPECT_LE(checks, 100 + 3);  // the budget's three checks, then one per period
      EXPECT_GE(checks, 100);
    } else {
      EXPECT_EQ(checks, 100);  // at 10, 20, ... 1000 s
    }
  }
}

TEST(KidnapWatchdog, TheCooldownHoldsConfirmationsBackUntilItEnds)
{
  landfall::KidnapWatchdogParams p = params();
  p.cooldown = 10.0;
  landfall::KidnapWatchdog watchdog(p);
  using landfall::WatchdogCheck;
  watchdog.due(0.0);
  watchdog.observe(5.0, 90.0);
  watchdog.observe(5.0, 90.0);
  ASSERT_TRUE(watchdog.shouldFire(100.0));
  watchdog.fired();
  watchdog.published(101.0);
  // Kidnapped again right away: a check and its confirmation, then the cooldown holds.
  watchdog.suspect(WatchdogCheck::kScanFit);
  ASSERT_EQ(watchdog.due(102.0), WatchdogCheck::kScanFit);
  watchdog.started(102.0);
  watchdog.observe(5.0, 90.0);
  ASSERT_EQ(watchdog.due(103.0), WatchdogCheck::kConfirm);
  watchdog.started(103.0);
  watchdog.observe(5.0, 90.0);
  EXPECT_FALSE(watchdog.shouldFire(104.0));
  EXPECT_EQ(watchdog.due(104.0), WatchdogCheck::kNone);  // no use confirming it again yet
  // When the cooldown ends, one more check confirms it, and the watchdog fires.
  ASSERT_EQ(watchdog.due(111.0), WatchdogCheck::kConfirm);
  watchdog.started(111.0);
  watchdog.observe(5.0, 90.0);
  EXPECT_TRUE(watchdog.shouldFire(112.0));
}

TEST(KidnapWatchdog, APublishedPoseOrAResetForgetsTheSuspicion)
{
  landfall::KidnapWatchdog watchdog(params());
  using landfall::WatchdogCheck;
  watchdog.due(0.0);
  watchdog.suspect(WatchdogCheck::kJump);
  watchdog.restart();  // another request gave the localization a new pose
  EXPECT_EQ(watchdog.due(1.0), WatchdogCheck::kNone);
  watchdog.suspect(WatchdogCheck::kScheduled);  // not a suspicion
  EXPECT_EQ(watchdog.due(1.0), WatchdogCheck::kNone);
  watchdog.suspect(WatchdogCheck::kRetry);
  EXPECT_EQ(watchdog.due(1.0), WatchdogCheck::kRetry);
  watchdog.reset();
  EXPECT_EQ(watchdog.due(1.0), WatchdogCheck::kNone);
  EXPECT_EQ(watchdog.due(11.0), WatchdogCheck::kScheduled);
}

TEST(KidnapWatchdog, AnUnansweredSuspicionWaitsForANewView)
{
  landfall::KidnapWatchdog watchdog(params());
  using landfall::WatchdogCheck;
  watchdog.due(0.0);
  watchdog.suspect(WatchdogCheck::kScanFit);
  ASSERT_EQ(watchdog.due(1.0), WatchdogCheck::kScanFit);
  watchdog.started(1.0);
  // The camera saw nothing it knows (a wall): no use looking again from the same place.
  watchdog.unanswered(WatchdogCheck::kScanFit);
  EXPECT_EQ(watchdog.waiting(), WatchdogCheck::kScanFit);
  EXPECT_EQ(watchdog.due(2.0), WatchdogCheck::kNone);
  watchdog.newView();  // the robot drove on
  ASSERT_EQ(watchdog.due(3.0), WatchdogCheck::kScanFit);
  watchdog.started(3.0);
  watchdog.observe(6.0, 120.0);  // now it sees where it is, far from the localization
  EXPECT_EQ(watchdog.waiting(), WatchdogCheck::kNone);
  EXPECT_EQ(watchdog.due(4.0), WatchdogCheck::kConfirm);  // the budget still holds a confirmation
  // A robot that stands still gets the scheduled checks instead, from a budget that stays full.
  landfall::KidnapWatchdog still(params());
  still.due(0.0);
  still.suspect(WatchdogCheck::kJump);
  ASSERT_EQ(still.due(1.0), WatchdogCheck::kJump);
  still.started(1.0);
  still.unanswered(WatchdogCheck::kJump);
  EXPECT_EQ(still.due(19.9), WatchdogCheck::kNone);
  ASSERT_EQ(still.due(20.0), WatchdogCheck::kScheduled);  // the budget is full again
  still.started(20.0);
  EXPECT_EQ(still.waiting(), WatchdogCheck::kJump);  // until a view answers it
  still.newView();
  EXPECT_EQ(still.due(21.0), WatchdogCheck::kJump);
  // A suspicion that no longer holds is dropped; a scheduled check that found nothing leaves none.
  landfall::KidnapWatchdog fits(params());
  fits.due(0.0);
  fits.unanswered(WatchdogCheck::kScanFit);
  fits.dropWaiting(WatchdogCheck::kScanFit);
  fits.newView();
  EXPECT_EQ(fits.due(1.0), WatchdogCheck::kNone);
  fits.unanswered(WatchdogCheck::kScheduled);
  EXPECT_EQ(fits.waiting(), WatchdogCheck::kNone);
}

TEST(ScanFitTrigger, FiresOnceWhenTheScanStopsFittingAndRearmsWhenItFitsAgain)
{
  landfall::ScanFitTrigger trigger(2);
  EXPECT_FALSE(trigger.observe(0.9, 0.2));
  EXPECT_FALSE(trigger.observe(0.1, 0.2));  // one scan is not enough
  EXPECT_FALSE(trigger.observe(0.9, 0.2));
  EXPECT_FALSE(trigger.observe(0.1, 0.2));
  EXPECT_TRUE(trigger.observe(-0.3, 0.2));   // two in a row
  EXPECT_FALSE(trigger.observe(-0.3, 0.2));  // while it stays bad: once
  EXPECT_FALSE(trigger.observe(0.9, 0.2));
  EXPECT_FALSE(trigger.observe(0.1, 0.2));  // one good scan doesn't rearm it
  EXPECT_FALSE(trigger.observe(0.1, 0.2));
  EXPECT_FALSE(trigger.observe(0.9, 0.2));
  EXPECT_FALSE(trigger.observe(0.9, 0.2));  // rearmed
  EXPECT_FALSE(trigger.observe(0.0, 0.2));
  EXPECT_TRUE(trigger.observe(0.0, 0.2));
  // -1 never fires: the fit is never below it.
  landfall::ScanFitTrigger off(2);
  for (int i = 0; i < 10; ++i) {
    EXPECT_FALSE(off.observe(-1.0, -1.0));
  }
  trigger.reset();
  EXPECT_FALSE(trigger.observe(0.0, 0.2));
  EXPECT_TRUE(trigger.observe(0.0, 0.2));
}
