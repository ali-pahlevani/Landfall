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

#include "landfall/benchmark/metrics.hpp"

#include <algorithm>
#include <cmath>

namespace landfall::benchmark
{

namespace
{

/// P(X <= k) for X ~ Binomial(n, p), summed in log space.
double binomialCdf(int k, int n, double p)
{
  if (p <= 0.0) {
    return 1.0;
  }
  if (p >= 1.0) {
    return k >= n ? 1.0 : 0.0;
  }
  const double log_p = std::log(p);
  const double log_q = std::log1p(-p);
  double sum = 0.0;
  for (int i = 0; i <= k; ++i) {
    const double log_choose =
      std::lgamma(n + 1.0) - std::lgamma(i + 1.0) - std::lgamma(n - i + 1.0);
    sum += std::exp(log_choose + i * log_p + (n - i) * log_q);
  }
  return std::min(1.0, sum);
}

/// The p in [0, 1] where f(p) crosses `target`, f decreasing in p (bisection).
template <typename F>
double solveDecreasing(F f, double target)
{
  double lo = 0.0;
  double hi = 1.0;
  for (int i = 0; i < 200; ++i) {
    const double mid = 0.5 * (lo + hi);
    if (f(mid) > target) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  return 0.5 * (lo + hi);
}

}  // namespace

const char * toString(Outcome outcome)
{
  switch (outcome) {
    case Outcome::kSuccess:
      return "success";
    case Outcome::kWrong:
      return "wrong";
    case Outcome::kInaccurate:
      return "inaccurate";
    case Outcome::kMiss:
      return "miss";
    case Outcome::kInvalid:
      return "invalid";
  }
  return "invalid";
}

std::optional<Outcome> parseOutcome(const std::string & text)
{
  for (const Outcome o :
       {Outcome::kSuccess, Outcome::kWrong, Outcome::kInaccurate, Outcome::kMiss,
        Outcome::kInvalid}) {
    if (text == toString(o)) {
      return o;
    }
  }
  return std::nullopt;
}

bool isWrong(double translation, double rotation_deg, const ErrorLimits & limits)
{
  return translation > limits.wrong_translation || rotation_deg > limits.wrong_rotation_deg;
}

bool isRight(double translation, double rotation_deg, const ErrorLimits & limits)
{
  return translation <= limits.success_translation && rotation_deg <= limits.success_rotation_deg;
}

double clopperPearsonUpper(int k, int n, double confidence)
{
  if (n <= 0 || k >= n) {
    return 1.0;
  }
  const double alpha = 1.0 - confidence;
  // The p at which seeing k or fewer events has probability alpha / 2.
  return solveDecreasing([k, n](double p) { return binomialCdf(k, n, p); }, 0.5 * alpha);
}

double clopperPearsonLower(int k, int n, double confidence)
{
  if (n <= 0 || k <= 0) {
    return 0.0;
  }
  const double alpha = 1.0 - confidence;
  // The p at which seeing k or more events has probability alpha / 2.
  return solveDecreasing([k, n](double p) { return binomialCdf(k - 1, n, p); }, 1.0 - 0.5 * alpha);
}

}  // namespace landfall::benchmark
