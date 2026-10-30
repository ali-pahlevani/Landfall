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

#ifndef LANDFALL_GRID_MAP_LOADER_HPP_
#define LANDFALL_GRID_MAP_LOADER_HPP_

#include <cstdint>
#include <opencv2/core.hpp>
#include <string>

#include "landfall/grid/occupancy_grid.hpp"

namespace landfall
{

enum class MapMode : uint8_t {
  kTrinary,
  kScale,
  kRaw,
};

/// nav2_map_server's pixel conversion changed between releases, and the relocalizer compares the
/// grid it loads with the one map_server publishes, so the loader reproduces both:
///  - kHumble (Nav2 1.1): averages the color channels (in trinary mode also the alpha channel),
///    thresholds are strict, scale mode treats transparent pixels as unknown;
///  - kJazzy (Nav2 >= 1.3, also Lyrical): converts to grayscale, thresholds are inclusive, any
///    transparent pixel is unknown.
/// Both give identical grids for the usual single-channel PGM/PNG maps away from the exact
/// thresholds.
enum class MapServerSemantics : uint8_t {
  kHumble,
  kJazzy,
};

/// The parameters of a map_server YAML file.
struct MapMetadata
{
  std::string image_path;  ///< resolved against the YAML file's directory
  double resolution = 0.0;
  Pose2 origin;
  bool negate = false;
  double occupied_thresh = 0.65;
  double free_thresh = 0.25;
  MapMode mode = MapMode::kTrinary;
};

struct LoadedMap
{
  OccupancyGrid grid;
  MapMetadata metadata;
  std::string yaml_text;
  std::string yaml_sha256;
  std::string image_sha256;
};

/// Parses a map_server YAML file (same required keys and defaults as nav2_map_server; `~` is
/// expanded). Throws std::runtime_error with the reason.
MapMetadata loadMapMetadata(const std::string & yaml_path);

/// Loads a map like nav2_map_server's loadMapFromFile. Throws std::runtime_error on errors.
LoadedMap loadMap(const std::string & yaml_path, MapServerSemantics semantics);

/// Converts an image as read by cv::imread(IMREAD_UNCHANGED) (8- or 16-bit; gray, BGR or BGRA).
OccupancyGrid convertMapImage(
  const cv::Mat & image, const MapMetadata & metadata, MapServerSemantics semantics);

}  // namespace landfall

#endif  // LANDFALL_GRID_MAP_LOADER_HPP_
