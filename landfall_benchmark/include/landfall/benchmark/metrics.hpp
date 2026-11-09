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

#ifndef LANDFALL_BENCHMARK_METRICS_HPP_
#define LANDFALL_BENCHMARK_METRICS_HPP_

#include <cstdint>
#include <optional>
#include <string>

namespace landfall::benchmark
{

/// The benchmark's error limits (plan §11, M7): a pose within `success_*` is right, one beyond
/// either `wrong_*` limit is wrong; between the two it is inaccurate.
struct ErrorLimits
{
  double success_translation = 0.25;  ///< m
  double success_rotation_deg = 5.0;  ///< deg
  double wrong_translation = 0.5;     ///< m
  double wrong_rotation_deg = 10.0;   ///< deg
};

/// What became of one placement.
enum class Outcome : uint8_t {
  kSuccess,     ///< the localization ended within the success limits
  kWrong,       ///< a pose beyond the wrong limits was published (or claimed by AMCL)
  kInaccurate,  ///< a pose was published or claimed, neither right nor wrong
  kMiss,        ///< no pose: the method gave up (safe)
  kInvalid,     ///< the placement couldn't be run (e.g. the teleport failed)
};

const char * toString(Outcome outcome);
std::optional<Outcome> parseOutcome(const std::string & text);

bool isWrong(double translation, double rotation_deg, const ErrorLimits & limits);
bool isRight(double translation, double rotation_deg, const ErrorLimits & limits);

/// Upper end of the two-sided Clopper-Pearson interval of a proportion: `k` events in `n` trials at
/// `confidence` (e.g. 0.95). 1 for n == 0.
double clopperPearsonUpper(int k, int n, double confidence = 0.95);
/// Lower end of the same interval. 0 for k == 0.
double clopperPearsonLower(int k, int n, double confidence = 0.95);

}  // namespace landfall::benchmark

#endif  // LANDFALL_BENCHMARK_METRICS_HPP_
