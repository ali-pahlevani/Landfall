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

#include "landfall/grid/map_loader.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "landfall/util/sha256.hpp"

namespace landfall
{
namespace
{

std::string expandHome(const std::string & path)
{
  if (path.size() >= 2 && path[0] == '~' && path[1] == '/') {
    const char * home = std::getenv("HOME");
    if (home != nullptr) {
      return std::string(home) + path.substr(1);
    }
  }
  return path;
}

// nav2_map_server reads images with GraphicsMagick, which Ubuntu builds with 16-bit quanta: every
// channel below is such a quantum (8-bit values times 257).
constexpr double kMaxQuantum = 65535.0;

// Nav2 1.1 (Humble): average of the color channels (plus alpha in trinary mode). The average goes
// through ColorGray::scaleQuantumToDouble(Quantum), so it is truncated to a quantum first.
int8_t humbleCell(
  uint16_t r, uint16_t g, uint16_t b, uint16_t a, bool has_alpha, const MapMetadata & m)
{
  double sum = static_cast<double>(r) + g + b;
  int n = 3;
  if (m.mode == MapMode::kTrinary && has_alpha) {
    sum += a;
    ++n;
  }
  const auto average = static_cast<uint16_t>(sum / n);
  const double shade = average / kMaxQuantum;
  const double occ = m.negate ? shade : 1.0 - shade;
  switch (m.mode) {
    case MapMode::kTrinary:
      if (m.occupied_thresh < occ) {
        return kCellOccupied;
      }
      return occ < m.free_thresh ? kCellFree : kCellUnknown;
    case MapMode::kScale:
      if (has_alpha && a != 65535) {
        return kCellUnknown;
      }
      if (m.occupied_thresh < occ) {
        return kCellOccupied;
      }
      if (occ < m.free_thresh) {
        return kCellFree;
      }
      return static_cast<int8_t>(
        std::rint((occ - m.free_thresh) / (m.occupied_thresh - m.free_thresh) * 100.0));
    case MapMode::kRaw: {
      const double value = std::round(shade * 255.0);
      return value >= kCellFree && value <= kCellOccupied ? static_cast<int8_t>(value)
                                                          : kCellUnknown;
    }
  }
  return kCellUnknown;
}

// GraphicsMagick's GrayscaleType conversion as nav2_map_server >= 1.3 uses it: Rec. 601 luma of
// the quanta, rounded to a quantum, then written as an 8-bit "I" pixel (quantum / 257, truncated).
// Matches GraphicsMagick 1.3 (Q16) exactly for all 2^24 8-bit colors and all 16-bit grays.
uint8_t graphicsMagickGray(uint16_t r, uint16_t g, uint16_t b)
{
  const auto luma = static_cast<int64_t>(std::floor(
    0.299 * static_cast<double>(r) + 0.587 * static_cast<double>(g) +
    0.114 * static_cast<double>(b) + 0.5));
  return static_cast<uint8_t>(luma / 257);
}

// Mirrors the Eigen code of Nav2 >= 1.3: float arithmetic with the thresholds cast to float.
int8_t jazzyCell(uint8_t gray, bool transparent, const MapMetadata & m)
{
  if (m.mode == MapMode::kRaw) {
    return gray <= kCellOccupied ? static_cast<int8_t>(gray) : kCellUnknown;
  }
  const auto occupied_thresh = static_cast<float>(m.occupied_thresh);
  const auto free_thresh = static_cast<float>(m.free_thresh);
  float normalized = static_cast<float>(gray) / 255.0F;
  if (!m.negate) {
    normalized = 1.0F - normalized;
  }
  int8_t cell = kCellUnknown;
  if (normalized >= occupied_thresh) {
    cell = kCellOccupied;
  }
  if (normalized <= free_thresh) {
    cell = kCellFree;
  }
  if (m.mode == MapMode::kScale && normalized > free_thresh && normalized < occupied_thresh) {
    const float scaled = (normalized - free_thresh) / (occupied_thresh - free_thresh) * 100.0F;
    cell = static_cast<int8_t>(std::round(scaled));
  }
  return transparent ? kCellUnknown : cell;
}

}  // namespace

MapMetadata loadMapMetadata(const std::string & yaml_path)
{
  const std::string path = expandHome(yaml_path);
  MapMetadata m;
  try {
    const YAML::Node doc = YAML::LoadFile(path);
    const auto required = [&](const char * key) {
      if (!doc[key]) {
        throw std::runtime_error(path + ": missing key '" + key + "'");
      }
      return doc[key];
    };
    m.image_path = required("image").as<std::string>();
    if (m.image_path.empty()) {
      throw std::runtime_error(path + ": the 'image' key is empty");
    }
    if (m.image_path[0] != '/') {
      m.image_path = (std::filesystem::path(path).parent_path() / m.image_path).string();
    }
    m.resolution = required("resolution").as<double>();
    const auto origin = required("origin").as<std::vector<double>>();
    if (origin.size() != 3) {
      throw std::runtime_error(path + ": 'origin' must have 3 elements");
    }
    m.origin = {origin[0], origin[1], origin[2]};
    m.free_thresh = required("free_thresh").as<double>();
    m.occupied_thresh = required("occupied_thresh").as<double>();
    if (doc["mode"]) {
      auto mode = doc["mode"].as<std::string>();
      std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
      });
      if (mode == "trinary") {
        m.mode = MapMode::kTrinary;
      } else if (mode == "scale") {
        m.mode = MapMode::kScale;
      } else if (mode == "raw") {
        m.mode = MapMode::kRaw;
      } else {
        throw std::runtime_error(path + ": unknown map mode '" + mode + "'");
      }
    }
    const YAML::Node negate = required("negate");
    try {
      m.negate = negate.as<int>() != 0;
    } catch (const YAML::Exception &) {
      m.negate = negate.as<bool>();
    }
  } catch (const YAML::Exception & e) {
    throw std::runtime_error("failed to parse map YAML " + path + ": " + e.what());
  }
  if (!(m.resolution > 0.0)) {
    throw std::runtime_error(path + ": resolution must be positive");
  }
  return m;
}

OccupancyGrid convertMapImage(
  const cv::Mat & image, const MapMetadata & m, MapServerSemantics semantics)
{
  if (image.empty() || (image.depth() != CV_8U && image.depth() != CV_16U)) {
    throw std::runtime_error("map image must be an 8- or 16-bit image");
  }
  const int channels = image.channels();
  if (channels != 1 && channels != 3 && channels != 4) {
    throw std::runtime_error("map image must have 1, 3 or 4 channels");
  }
  const bool has_alpha = channels == 4;
  cv::Mat bgra;
  if (channels == 1) {
    cv::cvtColor(image, bgra, cv::COLOR_GRAY2BGRA);
  } else if (channels == 3) {
    cv::cvtColor(image, bgra, cv::COLOR_BGR2BGRA);
  } else {
    bgra = image;
  }
  cv::Mat quanta;  // CV_16UC4, BGRA
  bgra.convertTo(quanta, CV_16U, image.depth() == CV_8U ? 257.0 : 1.0);

  OccupancyGrid grid;
  grid.width = image.cols;
  grid.height = image.rows;
  grid.resolution = m.resolution;
  grid.origin = m.origin;
  grid.data.resize(static_cast<std::size_t>(grid.width) * grid.height);
  for (int y = 0; y < image.rows; ++y) {
    const auto * pixels = quanta.ptr<cv::Vec<uint16_t, 4>>(y);
    for (int x = 0; x < image.cols; ++x) {
      const uint16_t b = pixels[x][0];
      const uint16_t g = pixels[x][1];
      const uint16_t r = pixels[x][2];
      const uint16_t a = pixels[x][3];
      const int8_t cell = semantics == MapServerSemantics::kHumble
                            ? humbleCell(r, g, b, a, has_alpha, m)
                            : jazzyCell(graphicsMagickGray(r, g, b), has_alpha && a < 65535, m);
      grid.data[static_cast<std::size_t>(grid.height - 1 - y) * grid.width + x] = cell;
    }
  }
  return grid;
}

LoadedMap loadMap(const std::string & yaml_path, MapServerSemantics semantics)
{
  LoadedMap map;
  map.metadata = loadMapMetadata(yaml_path);
  const std::string path = expandHome(yaml_path);
  std::ifstream file(path, std::ios::binary);
  std::ostringstream text;
  text << file.rdbuf();
  map.yaml_text = text.str();
  map.yaml_sha256 = sha256Hex(map.yaml_text);
  const auto image_sha = sha256File(map.metadata.image_path);
  const cv::Mat image = cv::imread(map.metadata.image_path, cv::IMREAD_UNCHANGED);
  if (!image_sha || image.empty()) {
    throw std::runtime_error("failed to read map image " + map.metadata.image_path);
  }
  map.image_sha256 = *image_sha;
  map.grid = convertMapImage(image, map.metadata, semantics);
  return map;
}

}  // namespace landfall
