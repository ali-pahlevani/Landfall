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

// `summarize_benchmark`: the results tables of `kidnap_runner` CSV files, as Markdown.
//
//   summarize_benchmark RESULTS.csv...
//
// One row per label and method: placements run (invalid ones left out), successes, wrong poses
// with the 95 % Clopper-Pearson upper bound of their rate, inaccurate ones, misses, the time to
// localize (median and 90th percentile of the successes), for Landfall the share localized at the
// first heading, for the watchdog the time to detect and the needless relocalizations per hour.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "landfall/benchmark/metrics.hpp"

namespace
{

using landfall::benchmark::Outcome;

std::vector<std::string> split(const std::string & line)
{
  std::vector<std::string> fields;
  std::stringstream in(line);
  std::string field;
  while (std::getline(in, field, ',')) {
    fields.push_back(field);
  }
  if (!line.empty() && line.back() == ',') {
    fields.emplace_back();
  }
  return fields;
}

double quantile(std::vector<double> v, double q)
{
  if (v.empty()) {
    return NAN;
  }
  std::sort(v.begin(), v.end());
  const double position = q * static_cast<double>(v.size() - 1);
  const auto below = static_cast<std::size_t>(std::floor(position));
  const auto above = std::min(below + 1, v.size() - 1);
  return v[below] + (position - static_cast<double>(below)) * (v[above] - v[below]);
}

std::string percent(int k, int n)
{
  if (n == 0) {
    return "-";
  }
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.1f %%", 100.0 * k / n);
  return buffer;
}

std::string seconds(double v)
{
  if (std::isnan(v)) {
    return "-";
  }
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.1f s", v);
  return buffer;
}

struct Group
{
  int placements = 0;
  int invalid = 0;
  std::map<Outcome, int> outcomes;
  int first_heading = 0;
  std::vector<double> time_to_localize;
  std::vector<double> detection;
  std::vector<double> published_error_m;
  int false_relocalizations = 0;
  double observed_s = 0.0;
};

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cerr << "usage: summarize_benchmark RESULTS.csv...\n";
    return 2;
  }
  // Groups in the order they first appear.
  std::vector<std::pair<std::string, std::string>> order;
  std::map<std::pair<std::string, std::string>, Group> groups;
  try {
    for (int a = 1; a < argc; ++a) {
      std::ifstream in(argv[a]);
      if (!in) {
        throw std::runtime_error(std::string("cannot read ") + argv[a]);
      }
      std::string line;
      std::map<std::string, std::size_t> column;
      while (std::getline(in, line)) {
        const auto fields = split(line);
        if (column.empty()) {
          for (std::size_t i = 0; i < fields.size(); ++i) {
            column[fields[i]] = i;
          }
          continue;
        }
        auto get = [&](const std::string & name) -> std::string {
          const auto it = column.find(name);
          if (it == column.end() || it->second >= fields.size()) {
            return "";
          }
          return fields[it->second];
        };
        auto number = [&](const std::string & name) {
          const std::string v = get(name);
          return v.empty() ? NAN : std::stod(v);
        };
        const auto key = std::make_pair(get("label"), get("method"));
        if (groups.find(key) == groups.end()) {
          order.push_back(key);
        }
        Group & g = groups[key];
        const auto outcome = landfall::benchmark::parseOutcome(get("outcome"));
        if (!outcome) {
          throw std::runtime_error("unknown outcome '" + get("outcome") + "' in " + argv[a]);
        }
        if (*outcome == Outcome::kInvalid) {
          ++g.invalid;
          continue;
        }
        ++g.placements;
        ++g.outcomes[*outcome];
        if (*outcome == Outcome::kSuccess) {
          if (get("attempts") == "1") {
            ++g.first_heading;
          }
          const double t = number("time_to_localize_s");
          if (!std::isnan(t)) {
            g.time_to_localize.push_back(t);
          }
        }
        const double detection = number("detection_s");
        if (!std::isnan(detection)) {
          g.detection.push_back(detection);
        }
        const double error = number("published_error_m");
        if (!std::isnan(error)) {
          g.published_error_m.push_back(error);
        }
        g.false_relocalizations += static_cast<int>(number("false_relocalizations"));
        const double observed = number("observed_s");
        g.observed_s += std::isnan(observed) ? 0.0 : observed;
      }
    }
  } catch (const std::exception & e) {
    std::cerr << "summarize_benchmark: " << e.what() << "\n";
    return 1;
  }

  std::cout << "| Scenario | Method | Placements | Success | Wrong (95 % upper bound) | "
               "Inaccurate | Miss | First heading | Time to localize (median / p90) | "
               "Published error (median) |\n";
  std::cout << "|---|---|---|---|---|---|---|---|---|---|\n";
  for (const auto & key : order) {
    const Group & g = groups[key];
    const int n = g.placements;
    auto count = [&](Outcome o) {
      const auto it = g.outcomes.find(o);
      return it == g.outcomes.end() ? 0 : it->second;
    };
    const int wrong = count(Outcome::kWrong);
    char bound[32];
    std::snprintf(
      bound, sizeof(bound), "%.1f %%", 100.0 * landfall::benchmark::clopperPearsonUpper(wrong, n));
    char error[32] = "-";
    if (!g.published_error_m.empty()) {
      std::snprintf(error, sizeof(error), "%.1f cm", 100.0 * quantile(g.published_error_m, 0.5));
    }
    std::cout << "| " << key.first << " | " << key.second << " | " << n
              << (g.invalid > 0 ? " (+" + std::to_string(g.invalid) + " invalid)" : "") << " | "
              << percent(count(Outcome::kSuccess), n) << " | " << wrong << " (" << bound << ") | "
              << percent(count(Outcome::kInaccurate), n) << " | "
              << percent(count(Outcome::kMiss), n) << " | "
              << (key.second == "landfall" ? percent(g.first_heading, n) : "-") << " | "
              << seconds(quantile(g.time_to_localize, 0.5)) << " / "
              << seconds(quantile(g.time_to_localize, 0.9)) << " | " << error << " |\n";
  }
  bool watchdog = false;
  for (const auto & key : order) {
    watchdog = watchdog || key.second == "watchdog";
  }
  if (watchdog) {
    std::cout
      << "\n| Scenario | Detected | Time to detect (median / p90) | Needless relocalizations "
         "| Observed with a right localization |\n";
    std::cout << "|---|---|---|---|---|\n";
    for (const auto & key : order) {
      if (key.second != "watchdog") {
        continue;
      }
      const Group & g = groups[key];
      char per_hour[64] = "-";
      if (g.observed_s > 0.0) {
        std::snprintf(
          per_hour, sizeof(per_hour), "%d (%.1f per hour)", g.false_relocalizations,
          g.false_relocalizations / (g.observed_s / 3600.0));
      }
      char observed[32];
      std::snprintf(observed, sizeof(observed), "%.0f min", g.observed_s / 60.0);
      std::cout << "| " << key.first << " | "
                << percent(static_cast<int>(g.detection.size()), g.placements) << " | "
                << seconds(quantile(g.detection, 0.5)) << " / "
                << seconds(quantile(g.detection, 0.9)) << " | " << per_hour << " | " << observed
                << " |\n";
    }
  }
  return 0;
}
