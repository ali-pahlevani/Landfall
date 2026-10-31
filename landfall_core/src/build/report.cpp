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

#include "landfall/build/report.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

#include "landfall/grid/occupancy_grid.hpp"
#include "landfall/util/json.hpp"

namespace landfall
{

namespace
{

constexpr double kRoseCell = 1.0;  // m
constexpr int kSectors = 8;
constexpr double kArrowLength = 0.3;  // m

std::string fixed(double value, int decimals)
{
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%.*f", decimals, value);
  return buffer;
}

// Heading sectors are centred on the compass directions (0, 45, 90 deg, ...), so a robot driving
// along axis-aligned aisles stays inside one sector; sector s is centred on -180 + 45 s deg.
int sectorOf(double yaw)
{
  const double a = normalizeAngle(yaw + M_PI / kSectors) + M_PI;  // [0, 2 pi)
  return std::min(kSectors - 1, static_cast<int>(a / (2.0 * M_PI / kSectors)));
}

std::string metaOr(const db::Database & database, const std::string & key, const std::string & dflt)
{
  return database.meta(key).value_or(dflt);
}

cv::Mat renderCoverage(
  const db::MapRecord & map, const std::vector<db::KeyframeRecord> & keyframes,
  const std::vector<db::LandmarkRecord> & landmarks, const MapSummary & summary)
{
  const OccupancyGrid grid =
    decodeGridPng(map.grid, map.resolution, {map.origin_x, map.origin_y, map.origin_yaw});
  const int scale = std::clamp(1600 / std::max(grid.width, grid.height), 1, 8);
  cv::Mat image(grid.height * scale, grid.width * scale, CV_8UC3);
  for (int row = 0; row < grid.height; ++row) {
    for (int col = 0; col < grid.width; ++col) {
      const int8_t v = grid.at(col, row);
      const uint8_t shade = v < 0 ? 205 : (v >= 65 ? 40 : 255);
      image(cv::Rect(col * scale, (grid.height - 1 - row) * scale, scale, scale))
        .setTo(cv::Scalar(shade, shade, shade));
    }
  }
  const double px_per_m = scale / map.resolution;
  const auto toPixel = [&](double x, double y) {
    return cv::Point2d((x - map.origin_x) * px_per_m, image.rows - (y - map.origin_y) * px_per_m);
  };
  const auto inside = [&](const cv::Point2d & p) {
    return p.x >= 0 && p.y >= 0 && p.x < image.cols && p.y < image.rows;
  };

  // Landmark density.
  for (const auto & l : landmarks) {
    const cv::Point2d p = toPixel(l.position.x(), l.position.y());
    if (inside(p)) {
      cv::circle(image, p, std::max(1, scale / 2), cv::Scalar(180, 119, 31), cv::FILLED);
    }
  }

  // Heading rose per 1 m cell: filled sectors are driven directions.
  std::map<std::pair<int, int>, int> sectors;
  for (const auto & k : keyframes) {
    const auto cell = std::make_pair(
      static_cast<int>(std::floor((k.refined.x - map.origin_x) / kRoseCell)),
      static_cast<int>(std::floor((k.refined.y - map.origin_y) / kRoseCell)));
    sectors[cell] |= 1 << sectorOf(k.refined.yaw);
  }
  cv::Mat overlay = image.clone();
  const int radius = std::max(2, static_cast<int>(0.4 * kRoseCell * px_per_m));
  for (const auto & [cell, mask] : sectors) {
    const cv::Point2d center = toPixel(
      map.origin_x + (cell.first + 0.5) * kRoseCell,
      map.origin_y + (cell.second + 0.5) * kRoseCell);
    for (int s = 0; s < kSectors; ++s) {
      if ((mask & (1 << s)) == 0) {
        continue;
      }
      // Map angles turn counter-clockwise with y up; image angles clockwise with y down.
      const double first = -180.0 + (s - 0.5) * 360.0 / kSectors;  // map degrees
      const double start = -(first + 360.0 / kSectors);
      const double end = -first;
      cv::ellipse(
        overlay, center, cv::Size(radius, radius), 0.0, start, end, cv::Scalar(60, 170, 60),
        cv::FILLED);
    }
    cv::circle(overlay, center, radius, cv::Scalar(120, 120, 120), 1);
  }
  cv::addWeighted(overlay, 0.55, image, 0.45, 0.0, image);

  // Keyframes: arrows along the heading, flagged ones in magenta.
  for (const auto & k : keyframes) {
    const cv::Point2d from = toPixel(k.refined.x, k.refined.y);
    const cv::Point2d to = toPixel(
      k.refined.x + kArrowLength * std::cos(k.refined.yaw),
      k.refined.y + kArrowLength * std::sin(k.refined.yaw));
    const bool flagged = (k.flags & db::KeyframeRecord::kFlagBaShiftAtBound) != 0;
    cv::arrowedLine(
      image, from, to, flagged ? cv::Scalar(255, 0, 255) : cv::Scalar(0, 0, 220), 1, cv::LINE_AA, 0,
      0.3);
  }

  const std::string legend = "keyframes " + std::to_string(summary.keyframes) + "  landmarks " +
                             std::to_string(summary.landmarks) + "  cells driven both ways " +
                             fixed(100.0 * summary.both_directions_fraction, 0) + "%";
  cv::rectangle(
    image, cv::Rect(0, 0, std::min(image.cols, 560), 22), cv::Scalar(255, 255, 255), cv::FILLED);
  cv::putText(
    image, legend, cv::Point(6, 16), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 0, 0), 1,
    cv::LINE_AA);
  return image;
}

double number(const YAML::Node & node, const char * key)
{
  return node && node[key] && !node[key].IsNull() ? node[key].as<double>() : 0.0;
}

}  // namespace

MapReport makeReport(const db::Database & database)
{
  MapReport report;
  MapSummary & s = report.summary;
  const auto maps = database.maps();
  if (maps.empty()) {
    throw std::runtime_error("database has no map");
  }
  const auto keyframes = database.keyframes();
  const auto landmarks = database.landmarks();
  const auto sessions = database.sessions();
  const auto backends = database.backends();
  const auto observations = database.allObservations();

  s.keyframes = static_cast<int>(keyframes.size());
  s.sessions = static_cast<int>(sessions.size());
  s.landmarks = static_cast<int>(landmarks.size());
  s.observations = static_cast<int>(observations.size());
  double reprojection_sum = 0.0;
  for (const auto & l : landmarks) {
    reprojection_sum += l.mean_reprojection_px;
  }
  s.mean_track_length = s.landmarks > 0 ? static_cast<double>(s.observations) / s.landmarks : 0.0;
  s.mean_landmark_reprojection_px = s.landmarks > 0 ? reprojection_sum / s.landmarks : 0.0;
  std::map<int64_t, int> per_keyframe;
  for (const auto & o : observations) {
    ++per_keyframe[o.keyframe_id];
  }
  int with_100 = 0;
  double shift_sum = 0.0;
  for (const auto & k : keyframes) {
    with_100 += per_keyframe[k.id] >= 100 ? 1 : 0;
    const double shift = std::hypot(k.refined.x - k.prior.x, k.refined.y - k.prior.y);
    shift_sum += shift;
    s.ba_max_shift_m = std::max(s.ba_max_shift_m, shift);
    s.ba_max_shift_yaw_deg = std::max(
      s.ba_max_shift_yaw_deg, std::abs(normalizeAngle(k.refined.yaw - k.prior.yaw)) * 180.0 / M_PI);
    s.flagged_keyframes += (k.flags & db::KeyframeRecord::kFlagBaShiftAtBound) != 0 ? 1 : 0;
  }
  if (s.keyframes > 0) {
    s.mean_landmarks_per_keyframe = static_cast<double>(s.observations) / s.keyframes;
    s.keyframes_with_100_landmarks = static_cast<double>(with_100) / s.keyframes;
    s.ba_mean_shift_m = shift_sum / s.keyframes;
  }
  std::map<std::pair<int, int>, int> sectors;
  for (const auto & k : keyframes) {
    sectors[{
      static_cast<int>(std::floor((k.refined.x - maps[0].origin_x) / kRoseCell)),
      static_cast<int>(std::floor((k.refined.y - maps[0].origin_y) / kRoseCell))}] |=
      1 << sectorOf(k.refined.yaw);
  }
  int sector_sum = 0;
  int both_ways = 0;
  for (const auto & entry : sectors) {
    const int mask = entry.second;
    bool opposite = false;
    for (int sector = 0; sector < kSectors / 2; ++sector) {
      const int pair = (1 << sector) | (1 << (sector + kSectors / 2));
      opposite = opposite || (mask & pair) == pair;
    }
    for (int sector = 0; sector < kSectors; ++sector) {
      sector_sum += (mask >> sector) & 1;
    }
    both_ways += opposite ? 1 : 0;
  }
  if (!sectors.empty()) {
    const auto cells = static_cast<double>(sectors.size());
    s.heading_sector_fraction = static_cast<double>(sector_sum) / (kSectors * cells);
    s.both_directions_fraction = static_cast<double>(both_ways) / cells;
  }

  const std::string build_json = metaOr(database, "build_report", "{}");
  YAML::Node build;
  try {
    build = YAML::Load(build_json);
  } catch (const YAML::Exception &) {
    build = YAML::Node();
  }

  JsonWriter json;
  json.beginObject()
    .key("summary")
    .beginObject()
    .field("keyframes", s.keyframes)
    .field("sessions", s.sessions)
    .field("landmarks", s.landmarks)
    .field("observations", s.observations)
    .field("mean_track_length", s.mean_track_length)
    .field("mean_landmarks_per_keyframe", s.mean_landmarks_per_keyframe)
    .field("keyframes_with_100_landmarks", s.keyframes_with_100_landmarks)
    .field("mean_landmark_reprojection_px", s.mean_landmark_reprojection_px)
    .field("ba_mean_shift_m", s.ba_mean_shift_m)
    .field("ba_max_shift_m", s.ba_max_shift_m)
    .field("ba_max_shift_yaw_deg", s.ba_max_shift_yaw_deg)
    .field("flagged_keyframes", s.flagged_keyframes)
    .field("heading_sector_fraction", s.heading_sector_fraction)
    .field("both_directions_fraction", s.both_directions_fraction)
    .field("content_sha256", metaOr(database, "content_sha256", ""))
    .endObject();
  report.json = json.str() + ",\"build\":" + build_json + "}";

  std::string md;
  md += "# Landfall map report\n\n";
  md += "Database `" + std::filesystem::path(database.path()).filename().string() + "`, built " +
        metaOr(database, "created_utc", "?") + " by Landfall " +
        metaOr(database, "landfall_version", "?") + ".  \n";
  md += "Content SHA-256 `" + metaOr(database, "content_sha256", "?") + "`.\n\n";
  md += "![coverage](coverage.png)\n\n";
  md += "## Summary\n\n| | |\n|---|---|\n";
  md += "| Keyframes | " + std::to_string(s.keyframes) + " in " + std::to_string(s.sessions) +
        " session(s) |\n";
  md += "| Landmarks | " + std::to_string(s.landmarks) + " |\n";
  md += "| Mean track length | " + fixed(s.mean_track_length, 2) + " observations |\n";
  md += "| Landmarks per keyframe | " + fixed(s.mean_landmarks_per_keyframe, 1) + " (" +
        fixed(100.0 * s.keyframes_with_100_landmarks, 1) + " % of keyframes see >= 100) |\n";
  md += "| Reprojection RMS | " + fixed(number(build, "reprojection_rms_px"), 3) + " px |\n";
  md += "| Heading coverage | " + fixed(100.0 * s.both_directions_fraction, 1) +
        " % of visited 1 m cells driven both ways; " + fixed(100.0 * s.heading_sector_fraction, 1) +
        " % of the 8 heading sectors seen per cell |\n";
  md += "| Image pairs | " + std::to_string(static_cast<int>(number(build, "pairs_verified"))) +
        " verified of " + std::to_string(static_cast<int>(number(build, "pairs_candidate"))) +
        " candidates, " + fixed(number(build, "mean_pair_inliers"), 1) + " inliers on average |\n";
  md += "| Tracks | " + std::to_string(static_cast<int>(number(build, "tracks"))) + " (" +
        std::to_string(static_cast<int>(number(build, "track_conflicts"))) +
        " split for conflicts); rejected: " +
        std::to_string(static_cast<int>(number(build, "rejected_small_angle"))) + " small angle, " +
        std::to_string(static_cast<int>(number(build, "rejected_depth"))) + " depth, " +
        std::to_string(static_cast<int>(number(build, "rejected_below_floor"))) + " below floor, " +
        std::to_string(static_cast<int>(number(build, "rejected_reprojection"))) +
        " inconsistent |\n";
  md += "\n## Pose-prior bundle adjustment\n\n";
  if (build && build["ba_run"] && build["ba_run"].as<bool>()) {
    md += "| | |\n|---|---|\n";
    md += "| Iterations | " + std::to_string(static_cast<int>(number(build, "ba_iterations"))) +
          ", cost " + fixed(number(build, "ba_initial_cost"), 1) + " -> " +
          fixed(number(build, "ba_final_cost"), 1) + " |\n";
    md += "| Keyframe shift | mean " + fixed(100.0 * s.ba_mean_shift_m, 2) + " cm, max " +
          fixed(100.0 * s.ba_max_shift_m, 2) + " cm, max yaw " + fixed(s.ba_max_shift_yaw_deg, 2) +
          " deg |\n";
    md += "| Mean shift vector | (" + fixed(100.0 * number(build, "ba_mean_shift_x_m"), 2) + ", " +
          fixed(100.0 * number(build, "ba_mean_shift_y_m"), 2) + ") cm |\n";
    md += "| Keyframes at the shift bound | " + std::to_string(s.flagged_keyframes) + " |\n";
  } else {
    md += "Not run.\n";
  }
  if (s.flagged_keyframes > 0) {
    md += "\nFlagged keyframes (id, prior x/y/yaw):\n\n";
    for (const auto & k : keyframes) {
      if ((k.flags & db::KeyframeRecord::kFlagBaShiftAtBound) != 0) {
        md += "- " + std::to_string(k.id) + ": " + fixed(k.prior.x, 2) + ", " +
              fixed(k.prior.y, 2) + ", " + fixed(k.prior.yaw * 180.0 / M_PI, 1) + " deg\n";
      }
    }
  }
  md += "\n## Warnings\n\n";
  if (build && build["warnings"] && build["warnings"].size() > 0) {
    for (const auto & w : build["warnings"]) {
      md += "- " + w.as<std::string>() + "\n";
    }
  } else {
    md += "None.\n";
  }
  md += "\n## Inputs\n\n";
  for (const auto & session : sessions) {
    md += "- Session `" + session.name + "` from `" + session.source + "` (" + session.pose_source +
          ")\n";
  }
  for (const auto & backend : backends) {
    md += "- Backend " + backend.role + ": `" + backend.name + "` v" + backend.version + "\n";
  }
  md += "- Map: " + std::to_string(maps[0].width) + " x " + std::to_string(maps[0].height) +
        " cells at " + fixed(maps[0].resolution, 3) + " m, image SHA-256 `" + maps[0].image_sha256 +
        "`\n";
  if (build && build["stage_seconds"]) {
    md += "\n## Timings\n\n| Stage | s |\n|---|---|\n";
    for (const auto & entry : build["stage_seconds"]) {
      md +=
        "| " + entry.first.as<std::string>() + " | " + fixed(entry.second.as<double>(), 2) + " |\n";
    }
  }
  report.markdown = md;
  report.coverage = renderCoverage(maps[0], keyframes, landmarks, s);
  return report;
}

void writeReport(const MapReport & report, const std::string & directory)
{
  std::filesystem::create_directories(directory);
  const auto write = [&](const std::string & name, const std::string & content) {
    std::ofstream file(std::filesystem::path(directory) / name, std::ios::trunc);
    file << content;
    if (!file) {
      throw std::runtime_error("cannot write " + directory + "/" + name);
    }
  };
  write("report.md", report.markdown);
  write("report.json", report.json + "\n");
  const std::string png = (std::filesystem::path(directory) / "coverage.png").string();
  if (!cv::imwrite(png, report.coverage)) {
    throw std::runtime_error("cannot write " + png);
  }
}

}  // namespace landfall
