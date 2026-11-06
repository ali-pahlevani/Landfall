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

#ifndef LANDFALL_QUERY_AUTO_TRIGGER_HPP_
#define LANDFALL_QUERY_AUTO_TRIGGER_HPP_

#include <optional>

namespace landfall
{

struct AutoTriggerParams
{
  double max_std_xy = 0.5;        ///< m
  double max_std_yaw_deg = 15.0;  ///< deg
  double duration = 5.0;          ///< s the localization must stay uncertain
  double cooldown = 30.0;         ///< s after a triggered request before the next one
};

/// When to relocalize on its own (plan §7.15): the localization's reported uncertainty stays above
/// the limits for `duration`, the previous automatic request is at least `cooldown` ago, and no
/// request is running. Localization estimates arrive irregularly (AMCL publishes only after filter
/// updates), so the last estimate holds until the next one. Times are seconds on any one clock.
class AutoTrigger
{
public:
  explicit AutoTrigger(const AutoTriggerParams & params = AutoTriggerParams()) : params_(params) {}

  void setParams(const AutoTriggerParams & params) { params_ = params; }

  /// A localization estimate at time t.
  void observe(double t, double std_xy, double std_yaw_deg);
  /// True when a request should start at time t.
  bool shouldFire(double t, bool busy) const;
  /// A request started at time t: the cooldown begins.
  void fired(double t) { last_fired_ = t; }
  /// Forgets the estimates and the cooldown (e.g. after reactivation).
  void reset();

  bool uncertain() const { return uncertain_since_.has_value(); }

private:
  AutoTriggerParams params_;
  std::optional<double> uncertain_since_;
  std::optional<double> last_fired_;
};

}  // namespace landfall

#endif  // LANDFALL_QUERY_AUTO_TRIGGER_HPP_
