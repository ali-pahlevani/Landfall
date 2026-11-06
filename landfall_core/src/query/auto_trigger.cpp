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

#include "landfall/query/auto_trigger.hpp"

namespace landfall
{

void AutoTrigger::observe(double t, double std_xy, double std_yaw_deg)
{
  const bool uncertain = std_xy > params_.max_std_xy || std_yaw_deg > params_.max_std_yaw_deg;
  if (!uncertain) {
    uncertain_since_.reset();
  } else if (!uncertain_since_) {
    uncertain_since_ = t;
  }
}

bool AutoTrigger::shouldFire(double t, bool busy) const
{
  if (busy || !uncertain_since_ || t - *uncertain_since_ < params_.duration) {
    return false;
  }
  return !last_fired_ || t - *last_fired_ >= params_.cooldown;
}

void AutoTrigger::reset()
{
  uncertain_since_.reset();
  last_fired_.reset();
}

}  // namespace landfall
