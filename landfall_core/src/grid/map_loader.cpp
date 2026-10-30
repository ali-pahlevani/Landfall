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

int8_t humbleCell(
  const double * channels, int count, double alpha, bool has_alpha, const MapMetadata & m)
{
  double sum = 0.0;
  for (int i = 0; i < count; ++i) {
    sum += channels[i];
  }
  int n = count;
  if (m.mode == MapMode::kTrinary && has_alpha) {
    sum += alpha;
    ++n;
  }
  const double shade = sum / n;
  const double occ = m.negate ? shade : 1.0 - shade;
  switch (m.mode) {
    case MapMode::kTrinary:
      if (m.occupied_thresh < occ) {
        return kCellOccupied;
      }
      return occ < m.free_thresh ? kCellFree : kCellUnknown;
    case MapMode::kScale:
      if (alpha < 1.0) {
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
  const double max_value = image.depth() == CV_16U ? 65535.0 : 255.0;

  cv::Mat normalized;  // CV_64FC4, BGRA in [0, 1]
  cv::Mat bgra;
  if (channels == 1) {
    cv::cvtColor(image, bgra, cv::COLOR_GRAY2BGRA);
  } else if (channels == 3) {
    cv::cvtColor(image, bgra, cv::COLOR_BGR2BGRA);
  } else {
    bgra = image;
  }
  bgra.convertTo(normalized, CV_64F, 1.0 / max_value);

  cv::Mat gray8;  // what GraphicsMagick's grayscale conversion yields, 8 bit
  if (semantics == MapServerSemantics::kJazzy) {
    cv::Mat image8;
    if (image.depth() == CV_16U) {
      image.convertTo(image8, CV_8U, 1.0 / 257.0);
    } else {
      image8 = image;
    }
    if (channels == 1) {
      gray8 = image8;
    } else {
      cv::cvtColor(image8, gray8, channels == 3 ? cv::COLOR_BGR2GRAY : cv::COLOR_BGRA2GRAY);
    }
  }

  OccupancyGrid grid;
  grid.width = image.cols;
  grid.height = image.rows;
  grid.resolution = m.resolution;
  grid.origin = m.origin;
  grid.data.resize(static_cast<std::size_t>(grid.width) * grid.height);
  for (int y = 0; y < image.rows; ++y) {
    const auto * pixels = normalized.ptr<cv::Vec4d>(y);
    for (int x = 0; x < image.cols; ++x) {
      int8_t cell = kCellUnknown;
      if (semantics == MapServerSemantics::kHumble) {
        const double rgb[3] = {pixels[x][2], pixels[x][1], pixels[x][0]};
        cell = humbleCell(rgb, 3, pixels[x][3], has_alpha, m);
      } else {
        cell = jazzyCell(gray8.at<uint8_t>(y, x), has_alpha && pixels[x][3] < 1.0, m);
      }
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
