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

#include <algorithm>

namespace landfall
{

const char * toString(WatchdogCheck check)
{
  switch (check) {
    case WatchdogCheck::kNone:
      return "none";
    case WatchdogCheck::kScheduled:
      return "scheduled";
    case WatchdogCheck::kConfirm:
      return "confirmation";
    case WatchdogCheck::kJump:
      return "localization jump";
    case WatchdogCheck::kScanFit:
      return "scan fit";
    case WatchdogCheck::kRetry:
      return "retry";
  }
  return "unknown";
}

bool KidnapWatchdog::observe(double translation, double rotation_deg)
{
  const bool disagrees =
    translation > params_.max_translation || rotation_deg > params_.max_rotation_deg;
  disagreements_ = disagrees ? disagreements_ + 1 : 0;
  waiting_ = WatchdogCheck::kNone;  // answered
  if (disagrees) {
    pending_ = WatchdogCheck::kConfirm;
  } else if (pending_ == WatchdogCheck::kConfirm) {
    pending_ = WatchdogCheck::kNone;
  }
  return disagrees;
}

bool KidnapWatchdog::shouldFire(double t) const
{
  if (disagreements_ < params_.confirmations) {
    return false;
  }
  return !last_published_ || t - *last_published_ >= params_.cooldown;
}

void KidnapWatchdog::forget()
{
  disagreements_ = 0;
  pending_ = WatchdogCheck::kNone;
  waiting_ = WatchdogCheck::kNone;
}

void KidnapWatchdog::fired() { forget(); }

void KidnapWatchdog::published(double t)
{
  last_published_ = t;
  forget();
}

void KidnapWatchdog::restart() { forget(); }

void KidnapWatchdog::reset()
{
  forget();
  last_published_.reset();
  budget_time_.reset();
}

void KidnapWatchdog::suspect(WatchdogCheck why)
{
  if (why != WatchdogCheck::kNone && why != WatchdogCheck::kScheduled) {
    pending_ = why;
  }
}

double KidnapWatchdog::budget(double t) const
{
  if (!budget_time_) {
    return capacity() - 1.0;  // as after a check: the first scheduled one comes a period later
  }
  const double refill =
    params_.period > 0.0 ? std::max(0.0, t - *budget_time_) / params_.period : 0.0;
  return std::min(capacity(), budget_ + refill);
}

void KidnapWatchdog::unanswered(WatchdogCheck why)
{
  if (why == WatchdogCheck::kNone || why == WatchdogCheck::kScheduled) {
    return;
  }
  waiting_ = why;
  new_view_ = false;
}

void KidnapWatchdog::dropWaiting(WatchdogCheck why)
{
  if (waiting_ == why) {
    waiting_ = WatchdogCheck::kNone;
  }
}

WatchdogCheck KidnapWatchdog::due(double t)
{
  if (!budget_time_) {
    budget_ = budget(t);
    budget_time_ = t;
  }
  if (waiting_ != WatchdogCheck::kNone && pending_ == WatchdogCheck::kNone && new_view_) {
    pending_ = waiting_;
    waiting_ = WatchdogCheck::kNone;
  }
  const double available = budget(t);
  constexpr double kEpsilon = 1e-9;
  if (pending_ != WatchdogCheck::kNone && available >= 1.0 - kEpsilon) {
    // A complete count the cooldown holds back needs no further confirmation until it ends.
    const bool held = pending_ == WatchdogCheck::kConfirm &&
                      disagreements_ >= params_.confirmations && !shouldFire(t);
    if (!held) {
      return pending_;
    }
  }
  return available >= capacity() - kEpsilon ? WatchdogCheck::kScheduled : WatchdogCheck::kNone;
}

void KidnapWatchdog::started(double t)
{
  budget_ = budget(t) - 1.0;
  budget_time_ = t;
  pending_ = WatchdogCheck::kNone;
}

bool ScanFitTrigger::observe(double fit, double min_fit)
{
  if (fit < min_fit) {
    above_ = 0;
    if (++below_ >= count_ && armed_) {
      armed_ = false;
      return true;
    }
    return false;
  }
  below_ = 0;
  if (++above_ >= count_) {
    armed_ = true;
  }
  return false;
}

void ScanFitTrigger::reset()
{
  below_ = 0;
  above_ = 0;
  armed_ = true;
}

}  // namespace landfall
