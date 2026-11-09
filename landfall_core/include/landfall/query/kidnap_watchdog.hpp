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

#include <optional>

namespace landfall
{

struct KidnapWatchdogParams
{
  double max_translation = 1.0;    ///< m a check may differ from the localization and agree
  double max_rotation_deg = 20.0;  ///< deg, likewise
  int confirmations = 2;           ///< disagreeing checks in a row before relocalizing
  double cooldown = 30.0;          ///< s after a relocalization it started before the next one
};

/// Notices a wrong localization, such as a kidnapped robot whose localization stays confident
/// (plan §7.20), from the relocalizer's background checks: dry-run requests that passed every
/// check. A check disagrees when its pose is further from the localization's than either limit.
/// After `confirmations` disagreeing checks in a row, with no agreeing one between them, a
/// relocalization should start, unless the watchdog started one less than `cooldown` ago. Checks
/// that found no pose say nothing about the localization and are not observed. Times are seconds
/// on any one clock.
class KidnapWatchdog
{
public:
  explicit KidnapWatchdog(const KidnapWatchdogParams & params = KidnapWatchdogParams())
  : params_(params)
  {
  }

  void setParams(const KidnapWatchdogParams & params) { params_ = params; }

  /// A check that passed, `translation` m and `rotation_deg` degrees from the localization's pose.
  /// Returns true if it disagrees.
  bool observe(double translation, double rotation_deg);
  /// True when a relocalization should start at time t.
  bool shouldFire(double t) const;
  /// The watchdog started a relocalization at time t: the count restarts, the cooldown begins.
  void fired(double t);
  /// Forgets the disagreeing checks (e.g. another request gave the localization a new pose).
  void restart() { disagreements_ = 0; }
  /// Forgets the checks and the cooldown (e.g. after reactivation).
  void reset();

  int disagreements() const { return disagreements_; }

private:
  KidnapWatchdogParams params_;
  int disagreements_ = 0;
  std::optional<double> last_fired_;
};

}  // namespace landfall

#endif  // LANDFALL_QUERY_KIDNAP_WATCHDOG_HPP_
