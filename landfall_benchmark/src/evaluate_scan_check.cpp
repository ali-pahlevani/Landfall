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

// `evaluate_scan_check`: the LiDAR scan check (plan §7.12) on a recording with ground truth. Every
// scan is checked at its true pose (it must pass) and at wrong poses 0.5 to 5 m away (they must
// fail), the M4 acceptance test.
//
//   evaluate_scan_check --bag BAG --map MAP.yaml [--world-to-map X Y YAW_DEG] [--every N]
//                       [--wrong N] [--seed S] [--scan-topic /scan] [--gt-topic T]
//                       [--base base_footprint] [--set scan_check.NAME=VALUE]...

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <map>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include "landfall/eval/trajectory.hpp"
#include "landfall/grid/likelihood_field.hpp"
#include "landfall/grid/map_loader.hpp"
#include "landfall/grid/scan_verifier.hpp"
#include "landfall/ros/bag_reader.hpp"
#include "landfall/ros/conversions.hpp"
#include "landfall/ros/scan_evaluation.hpp"

namespace
{

double percentile(std::vector<double> values, double p)
{
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const auto i = static_cast<std::size_t>(std::ceil(p * static_cast<double>(values.size())) - 1);
  return values[std::min(i, values.size() - 1)];
}

}  // namespace

int main(int argc, char ** argv)
{
  landfall::ros::ScanEvaluationOptions options;
  std::string bag;
  std::string map_yaml;
  try {
    for (int i = 1; i < argc; ++i) {
      const std::string a = argv[i];
      const auto next = [&]() -> std::string {
        if (i + 1 >= argc) {
          throw std::invalid_argument(a + " needs a value");
        }
        return argv[++i];
      };
      if (a == "--bag") {
        bag = next();
      } else if (a == "--map") {
        map_yaml = next();
      } else if (a == "--world-to-map") {
        options.world_to_map.x = std::stod(next());
        options.world_to_map.y = std::stod(next());
        options.world_to_map.yaw = std::stod(next()) * M_PI / 180.0;
      } else if (a == "--every") {
        options.every = static_cast<std::size_t>(std::stoul(next()));
      } else if (a == "--wrong") {
        options.wrong_per_scan = std::stoi(next());
      } else if (a == "--seed") {
        options.seed = std::stoul(next());
      } else if (a == "--scan-topic") {
        options.scan_topic = next();
      } else if (a == "--gt-topic") {
        options.gt_topic = next();
      } else if (a == "--base") {
        options.base_frame = next();
      } else if (a == "--set") {
        const std::string assignment = next();
        const auto eq = assignment.find('=');
        if (eq == std::string::npos) {
          throw std::invalid_argument("--set expects scan_check.NAME=VALUE");
        }
        landfall::ros::setScanCheckParameter(
          &options.params, assignment.substr(0, eq), assignment.substr(eq + 1));
      } else {
        throw std::invalid_argument("unknown argument " + a);
      }
    }
    if (bag.empty() || map_yaml.empty()) {
      throw std::invalid_argument("--bag and --map are required");
    }
  } catch (const std::exception & e) {
    std::cerr
      << "evaluate_scan_check: " << e.what()
      << "\nusage: evaluate_scan_check --bag BAG --map MAP.yaml [--world-to-map X Y YAW_DEG] "
         "[--every N] [--wrong N] [--seed S] [--scan-topic T] [--gt-topic T] [--base F] "
         "[--set scan_check.NAME=VALUE]...\n";
    return 2;
  }

  try {
    const auto map = landfall::loadMap(map_yaml, landfall::MapServerSemantics::kJazzy);
    const auto e = landfall::ros::evaluateScanCheck(bag, map.grid, options);
    std::printf(
      "scan check on %zu scans (%zu without ground truth or TF skipped)\n", e.true_checks,
      e.skipped);
    std::printf(
      "  true poses accepted: %zu of %zu (%.1f %%)\n", e.true_accepted, e.true_checks,
      e.true_checks > 0
        ? 100.0 * static_cast<double>(e.true_accepted) / static_cast<double>(e.true_checks)
        : 0.0);
    for (const auto & [reason, count] : e.true_rejections) {
      std::printf("    rejected %zu: %s\n", count, reason.c_str());
    }
    std::printf(
      "    hit ratio p5 %.3f median %.3f; violation ratio median %.3f p95 %.3f; known ratio p5 "
      "%.3f; valid beams p5 %.0f; refined by median %.1f cm\n",
      percentile(e.true_hit_ratios, 0.05), percentile(e.true_hit_ratios, 0.5),
      percentile(e.true_violation_ratios, 0.5), percentile(e.true_violation_ratios, 0.95),
      percentile(e.true_known_ratios, 0.05), percentile(e.true_valid_beams, 0.05),
      100.0 * percentile(e.true_refinements, 0.5));
    std::printf(
      "  wrong poses (0.5-5 m off) accepted: %zu of %zu\n", e.wrong_accepted, e.wrong_checks);
    for (const auto & w : e.wrong_examples) {
      std::printf(
        "    accepted at %.2f m / %.1f deg off (hit %.3f, violations %.3f)\n", w.offset,
        w.yaw_offset_deg, w.hit_ratio, w.violation_ratio);
    }
  } catch (const std::exception & ex) {
    std::cerr << "evaluate_scan_check: error: " << ex.what() << "\n";
    return 1;
  }
  return 0;
}
