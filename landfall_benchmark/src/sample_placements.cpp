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

// `sample_placements`: the kidnap benchmark's placements of a seed as CSV (index, x, y, yaw_deg,
// keyframe_distance_m, heading_difference_deg), the same ones `kidnap_runner` meets with the same
// map, database, parameters and seed (docs/benchmark.md).
//
//   sample_placements --map MAP.yaml --db DB --seed S [--count 100]
//                     [--max-keyframe-distance 1.0] [--clearance 0.4] [--min-separation 2.0]

#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "landfall/benchmark/placements.hpp"
#include "landfall/db/database.hpp"
#include "landfall/grid/map_loader.hpp"

int main(int argc, char ** argv)
{
  std::string map_yaml;
  std::string db_path;
  uint64_t seed = 1;
  int count = 100;
  landfall::benchmark::PlacementParams params;
  try {
    for (int i = 1; i < argc; ++i) {
      const std::string a = argv[i];
      if (i + 1 >= argc) {
        throw std::invalid_argument(a + " needs a value");
      }
      const std::string v = argv[++i];
      if (a == "--map") {
        map_yaml = v;
      } else if (a == "--db") {
        db_path = v;
      } else if (a == "--seed") {
        seed = std::stoull(v);
      } else if (a == "--count") {
        count = std::stoi(v);
      } else if (a == "--max-keyframe-distance") {
        params.max_keyframe_distance = std::stod(v);
      } else if (a == "--clearance") {
        params.clearance = std::stod(v);
      } else if (a == "--min-separation") {
        params.min_separation = std::stod(v);
      } else {
        throw std::invalid_argument("unknown argument " + a);
      }
    }
    if (map_yaml.empty() || db_path.empty()) {
      throw std::invalid_argument("--map and --db are required");
    }
    const auto map = landfall::loadMap(map_yaml, landfall::MapServerSemantics::kJazzy);
    const auto db =
      landfall::db::Database::open(db_path, landfall::db::Database::Access::kReadOnly);
    std::vector<landfall::Pose2> keyframes;
    for (const auto & k : db.keyframes()) {
      keyframes.push_back(k.refined);
    }
    const auto placements =
      landfall::benchmark::samplePlacements(map.grid, keyframes, params, seed, count);
    std::printf("index,x,y,yaw_deg,keyframe_distance_m,heading_difference_deg\n");
    for (std::size_t i = 0; i < placements.size(); ++i) {
      const auto & p = placements[i];
      std::printf(
        "%zu,%.3f,%.3f,%.2f,%.3f,%.2f\n", i, p.pose.x, p.pose.y, p.pose.yaw * 180.0 / M_PI,
        p.keyframe_distance, p.heading_difference * 180.0 / M_PI);
    }
  } catch (const std::exception & e) {
    std::cerr << "sample_placements: " << e.what()
              << "\nusage: sample_placements --map MAP.yaml --db DB --seed S [--count N] "
                 "[--max-keyframe-distance M] [--clearance M] [--min-separation M]\n";
    return 2;
  }
  return 0;
}
