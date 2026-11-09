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

// `summarize_shadow`: the relocalizer's shadow log (`shadow.log_path`) against the localization,
// for the field validation (docs/field_validation.md). Without ground truth, the reference is the
// localization's own pose while it reports itself confident.
//
//   summarize_shadow [--max-std-xy 0.10] [--list] SHADOW.csv...
//
// Per trigger (dry-run service requests, the extrinsics monitor's and the kidnap watchdog's
// checks): the requests, their statuses, and of the successes taken while the localization's std
// was at most --max-std-xy: how many agree with it (within 0.25 m and 5 deg), how many disagree
// (beyond 0.5 m or 10 deg: a wrong pose, unless the localization was the wrong one, which only the
// operator can tell) with the 95 % Clopper-Pearson upper bound of their rate, and the time per
// request. --list prints every disagreeing row.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
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

struct Group
{
  int requests = 0;
  std::map<std::string, int> statuses;
  int confident = 0;  // successes with a confident localization
  int agree = 0;
  int disagree = 0;
  std::vector<double> elapsed;
  std::vector<std::string> disagreeing;
};

}  // namespace

int main(int argc, char ** argv)
{
  double max_std_xy = 0.10;
  bool list = false;
  std::vector<std::string> files;
  for (int a = 1; a < argc; ++a) {
    if (std::strcmp(argv[a], "--max-std-xy") == 0 && a + 1 < argc) {
      max_std_xy = std::stod(argv[++a]);
    } else if (std::strcmp(argv[a], "--list") == 0) {
      list = true;
    } else {
      files.emplace_back(argv[a]);
    }
  }
  if (files.empty()) {
    std::cerr << "usage: summarize_shadow [--max-std-xy 0.10] [--list] SHADOW.csv...\n";
    return 2;
  }
  const landfall::benchmark::ErrorLimits limits;
  std::vector<std::string> order;
  std::map<std::string, Group> groups;
  try {
    for (const auto & file : files) {
      std::ifstream in(file);
      if (!in) {
        throw std::runtime_error("cannot read " + file);
      }
      std::string line;
      std::map<std::string, std::size_t> column;
      while (std::getline(in, line)) {
        const auto fields = split(line);
        if (column.empty()) {
          for (std::size_t i = 0; i < fields.size(); ++i) {
            column[fields[i]] = i;
          }
          for (const char * required :
               {"trigger", "status", "error_m", "error_deg", "localization_std_xy", "elapsed_s"}) {
            if (column.count(required) == 0) {
              throw std::runtime_error(file + " has no column " + required);
            }
          }
          continue;
        }
        auto get = [&](const std::string & name) -> std::string {
          const std::size_t i = column.at(name);
          return i < fields.size() ? fields[i] : "";
        };
        auto number = [&](const std::string & name) {
          const std::string v = get(name);
          return v.empty() ? NAN : std::stod(v);
        };
        const std::string trigger = get("trigger");
        if (groups.find(trigger) == groups.end()) {
          order.push_back(trigger);
        }
        Group & g = groups[trigger];
        ++g.requests;
        ++g.statuses[get("status")];
        const double elapsed = number("elapsed_s");
        if (!std::isnan(elapsed)) {
          g.elapsed.push_back(elapsed);
        }
        const double error_m = number("error_m");
        const double error_deg = number("error_deg");
        const double std_xy = number("localization_std_xy");
        if (
          get("status") != "SUCCESS" || std::isnan(error_m) || std::isnan(std_xy) ||
          std_xy > max_std_xy) {
          continue;
        }
        ++g.confident;
        if (landfall::benchmark::isRight(error_m, error_deg, limits)) {
          ++g.agree;
        } else if (landfall::benchmark::isWrong(error_m, error_deg, limits)) {
          ++g.disagree;
          std::string row = file;
          row += ": ";
          row += line;
          g.disagreeing.push_back(std::move(row));
        }
      }
    }
  } catch (const std::exception & e) {
    std::cerr << "summarize_shadow: " << e.what() << "\n";
    return 1;
  }

  std::printf(
    "| Trigger | Requests | Statuses | Successes, localization std <= %.2f m | Agree | "
    "Disagree (95 %% upper bound) | Time per request (median / p95 / max) |\n",
    max_std_xy);
  std::printf("|---|---|---|---|---|---|---|\n");
  for (const auto & trigger : order) {
    const Group & g = groups[trigger];
    std::string statuses;
    for (const auto & [status, count] : g.statuses) {
      statuses += (statuses.empty() ? "" : ", ") + status + " " + std::to_string(count);
    }
    std::printf(
      "| %s | %d | %s | %d | %d | %d (%.2f %%) | %.2f / %.2f / %.2f s |\n", trigger.c_str(),
      g.requests, statuses.c_str(), g.confident, g.agree, g.disagree,
      100.0 * landfall::benchmark::clopperPearsonUpper(g.disagree, g.confident),
      quantile(g.elapsed, 0.5), quantile(g.elapsed, 0.95), quantile(g.elapsed, 1.0));
  }
  if (list) {
    for (const auto & trigger : order) {
      for (const auto & row : groups[trigger].disagreeing) {
        std::printf("%s\n", row.c_str());
      }
    }
  }
  return 0;
}
