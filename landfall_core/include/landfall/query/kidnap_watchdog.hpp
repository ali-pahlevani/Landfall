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

#ifndef LANDFALL_QUERY_KIDNAP_WATCHDOG_HPP_
#define LANDFALL_QUERY_KIDNAP_WATCHDOG_HPP_

#include <cstdint>
#include <optional>

namespace landfall
{

struct KidnapWatchdogParams
{
  double max_translation = 1.0;    ///< m a check may differ from the localization and agree
  double max_rotation_deg = 20.0;  ///< deg, likewise
  double cooldown = 10.0;          ///< s after a pose it published before its next relocalization
  double period = 10.0;            ///< s between scheduled checks: the rate all checks draw on
  /// Below this, for a second, the scan's fit at the localization's pose (scanFit) starts an early
  /// check; -1: never.
  double min_scan_fit = 0.2;
  int confirmations = 2;  ///< disagreeing checks in a row before relocalizing
};

/// Why the kidnap watchdog's next check runs (V70).
enum class WatchdogCheck : std::uint8_t {
  kNone,       ///< not yet
  kScheduled,  ///< the budget is full: the regular check, one per period
  kConfirm,    ///< a check disagreed with the localization: confirm it at once
  kJump,       ///< the localization's pose jumped, and odometry doesn't explain the jump
  kScanFit,    ///< the scan stopped fitting the map at the localization's pose
  kRetry,      ///< the watchdog's relocalization published nothing
};

const char * toString(WatchdogCheck check);

/// Notices a wrong localization, such as a kidnapped robot whose localization stays confident
/// (plan §7.20), from the relocalizer's background checks: dry-run requests that passed every
/// check. A check disagrees when its pose is further from the localization's than either limit.
/// After `confirmations` disagreeing checks in a row, with no agreeing one between them, a
/// relocalization should start, unless one the watchdog started published a pose less than
/// `cooldown` ago. One that published nothing leaves no cooldown: the next `confirmations`
/// disagreeing checks start another (V64). Checks that found no pose say nothing about the
/// localization and are not observed.
///
/// When the checks run (V70): a scheduled check every `period`, and an early one at once when
/// something suggests the localization is wrong (a check that disagreed and needs confirming, a
/// jump of the localization's pose, a scan that stopped fitting at it). Early checks take the place
/// of scheduled ones, so they never add any: every check spends one check's worth of a budget that
/// refills at one per period, up to `confirmations` + 2. A scheduled check waits for a full
/// budget, an early one for one check's worth. Over any long time the watchdog runs at most one
/// check per period, with or without early ones, and a full budget lets a suspicion, one more look
/// and the confirmations run back to back. An early check that found no pose (the camera saw
/// nothing it knows) leaves its suspicion waiting for a new view: once the robot moved or turned
/// enough (newView()), the suspicion is checked again. A robot that stands still meanwhile gets the
/// scheduled checks, which leave the budget full for whatever comes next. Times are seconds on one
/// steady clock.
class KidnapWatchdog
{
public:
  explicit KidnapWatchdog(const KidnapWatchdogParams & params = KidnapWatchdogParams())
  : params_(params)
  {
  }

  void setParams(const KidnapWatchdogParams & params) { params_ = params; }

  /// A check that passed, `translation` m and `rotation_deg` degrees from the localization's pose.
  /// Returns true if it disagrees; a disagreeing check asks for its confirmation at once.
  bool observe(double translation, double rotation_deg);
  /// True when a relocalization should start at time t.
  bool shouldFire(double t) const;
  /// The watchdog started a relocalization: the count restarts.
  void fired();
  /// The relocalization the watchdog started published a pose at time t: the count restarts and
  /// the cooldown begins.
  void published(double t);
  /// Forgets the disagreeing checks (e.g. another request gave the localization a new pose).
  void restart();
  /// Forgets the checks, the cooldown and the budget (e.g. after reactivation).
  void reset();

  /// Something suggests the localization is wrong (kJump, kScanFit, kRetry): the next check may
  /// run early.
  void suspect(WatchdogCheck why);
  /// Whether a check should start at time t, and why (kNone: not yet). While the cooldown holds a
  /// relocalization back, no early check confirms it again.
  WatchdogCheck due(double t);
  /// A check started at time t: it spends one check's worth of the budget.
  void started(double t);
  /// Checks' worth of budget at time t.
  double budget(double t) const;
  /// The early check for `why` found no pose: the suspicion waits for a new view.
  void unanswered(WatchdogCheck why);
  /// The robot moved or turned enough since the unanswered check for the camera to see more.
  void newView() { new_view_ = true; }
  /// A suspicion waiting for a new view no longer holds (e.g. the scan fits again).
  void dropWaiting(WatchdogCheck why);
  /// The suspicion waiting for a new view (kNone: none).
  WatchdogCheck waiting() const { return waiting_; }

  int disagreements() const { return disagreements_; }

private:
  double capacity() const { return params_.confirmations + 2.0; }
  void forget();

  KidnapWatchdogParams params_;
  std::optional<double> last_published_;
  std::optional<double> budget_time_;  ///< when budget_ was last brought up to date
  double budget_ = 0.0;
  int disagreements_ = 0;
  WatchdogCheck pending_ = WatchdogCheck::kNone;  ///< the suspicion an early check answers
  WatchdogCheck waiting_ = WatchdogCheck::kNone;  ///< an unanswered one, waiting for a new view
  bool new_view_ = false;
};

/// Notices a scan that stops fitting the map at the localization's pose (V70): fires once when the
/// fit stays below the limit for `count` scans in a row, and again only after as many scans fit. A
/// kidnapped robot's localization stays where the robot was, and the scan, taken somewhere else,
/// no longer fits the map there; while the localization is right, it hardly ever fits that badly.
class ScanFitTrigger
{
public:
  explicit ScanFitTrigger(int count = 2) : count_(count) {}

  /// One scan's fit at the localization's pose (scanFit); true when the trigger fires.
  bool observe(double fit, double min_fit);
  void reset();

private:
  int count_;
  int below_ = 0;
  int above_ = 0;
  bool armed_ = true;
};

}  // namespace landfall

#endif  // LANDFALL_QUERY_KIDNAP_WATCHDOG_HPP_
