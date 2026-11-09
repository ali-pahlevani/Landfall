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

#include "landfall/query/kidnap_watchdog.hpp"

namespace landfall
{

bool KidnapWatchdog::observe(double translation, double rotation_deg)
{
  const bool disagrees =
    translation > params_.max_translation || rotation_deg > params_.max_rotation_deg;
  disagreements_ = disagrees ? disagreements_ + 1 : 0;
  return disagrees;
}

bool KidnapWatchdog::shouldFire(double t) const
{
  if (disagreements_ < params_.confirmations) {
    return false;
  }
  return !last_fired_ || t - *last_fired_ >= params_.cooldown;
}

void KidnapWatchdog::fired(double t)
{
  last_fired_ = t;
  disagreements_ = 0;
}

void KidnapWatchdog::reset()
{
  disagreements_ = 0;
  last_fired_.reset();
}

}  // namespace landfall
