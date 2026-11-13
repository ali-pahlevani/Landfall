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
#include <string>
#include <utility>
#include <vector>

#include "landfall/build/coverage.hpp"
#include "landfall/grid/occupancy_grid.hpp"
#include "landfall/util/json.hpp"

namespace landfall
{

namespace
{

constexpr double kArrowLength = 0.3;  // m

// Reachable floor by the number of heading sectors with a view (BGR): none in red-orange, one in
// yellow, more in light to dark blue (no red-green pair, for colour-blind readers).
const cv::Scalar kNoView(70, 90, 235);
const cv::Scalar kOneDirection(80, 215, 245);
const cv::Scalar kTwoToThree(220, 170, 110);
const cv::Scalar kFourOrMore(170, 100, 40);

const cv::Scalar & coverageColor(int directions)
{
  if (directions <= 0) {
    return kNoView;
  }
  if (directions == 1) {
    return kOneDirection;
  }
  return directions <= 3 ? kTwoToThree : kFourOrMore;
}

std::string fixed(double value, int decimals)
{
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%.*f", decimals, value);
  return buffer;
}

std::string metaOr(const db::Database & database, const std::string & key, const std::string & dflt)
{
  return database.meta(key).value_or(dflt);
}

// One line of the legend, shrunk to the image's width when it doesn't fit.
void legendLine(cv::Mat & image, const std::string & text, cv::Point origin)
{
  constexpr double kScale = 0.45;
  int baseline = 0;
  const cv::Size size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, kScale, 1, &baseline);
  const int room = image.cols - origin.x - 4;
  const double scale =
    size.width > room ? std::max(0.25, kScale * room / std::max(1, size.width)) : kScale;
  cv::putText(
    image, text, origin, cv::FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
}

cv::Mat renderCoverage(
  const OccupancyGrid & grid, const std::vector<db::KeyframeRecord> & keyframes,
  const std::vector<db::LandmarkRecord> & landmarks, const CoverageMap & coverage,
  const MapSummary & summary)
{
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
  const double px_per_m = scale / grid.resolution;
  const auto toPixel = [&](double x, double y) {
    return cv::Point2d((x - grid.origin.x) * px_per_m, image.rows - (y - grid.origin.y) * px_per_m);
  };
  const auto inside = [&](const cv::Point2d & p) {
    return p.x >= 0 && p.y >= 0 && p.x < image.cols && p.y < image.rows;
  };

  // The floor the robot can reach, by the number of directions with a view within 1 m.
  cv::Mat overlay = image.clone();
  const double half = 0.5 * coverage.step * px_per_m;
  for (std::size_t p = 0; p < coverage.positions.size(); ++p) {
    const cv::Point2d c = toPixel(coverage.positions[p].x(), coverage.positions[p].y());
    cv::rectangle(
      overlay,
      cv::Point(
        static_cast<int>(std::lround(c.x - half)), static_cast<int>(std::lround(c.y - half))),
      cv::Point(
        static_cast<int>(std::lround(c.x + half)) - 1,
        static_cast<int>(std::lround(c.y + half)) - 1),
      coverageColor(coveredSectors(coverage.sectors[p])), cv::FILLED);
  }
  cv::addWeighted(overlay, 0.5, image, 0.5, 0.0, image);

  // Landmarks.
  for (const auto & l : landmarks) {
    const cv::Point2d p = toPixel(l.position.x(), l.position.y());
    if (inside(p)) {
      cv::circle(image, p, std::max(1, scale / 2), cv::Scalar(140, 50, 110), cv::FILLED);
    }
  }

  // Keyframes: arrows along the heading; at the BA bound in magenta, with a scan that fits a
  // second place in orange.
  for (const auto & k : keyframes) {
    const cv::Point2d from = toPixel(k.refined.x, k.refined.y);
    const cv::Point2d to = toPixel(
      k.refined.x + kArrowLength * std::cos(k.refined.yaw),
      k.refined.y + kArrowLength * std::sin(k.refined.yaw));
    cv::Scalar color(0, 0, 220);
    if ((k.flags & db::KeyframeRecord::kFlagBaShiftAtBound) != 0) {
      color = cv::Scalar(255, 0, 255);
    } else if ((k.flags & db::KeyframeRecord::kFlagScanAmbiguous) != 0) {
      color = cv::Scalar(0, 140, 255);
    }
    cv::arrowedLine(image, from, to, color, 1, cv::LINE_AA, 0, 0.3);
  }

  // Legend, below the map.
  constexpr int kLine = 20;
  cv::Mat out(image.rows + 4 * kLine + 8, image.cols, CV_8UC3, cv::Scalar(255, 255, 255));
  image.copyTo(out(cv::Rect(0, 0, image.cols, image.rows)));
  int y = image.rows + kLine;
  legendLine(
    out,
    "keyframes " + std::to_string(summary.keyframes) +
      " (arrows; orange: scan fits a second "
      "place, magenta: at the BA bound), landmarks " +
      std::to_string(summary.landmarks) + " (purple)",
    cv::Point(6, y));
  y += kLine;
  legendLine(
    out,
    "floor the robot can reach: " + fixed(summary.coverage_reachable_m2, 0) + " m2, at least " +
      fixed(summary.coverage_clearance_m, 2) + " m from obstacles",
    cv::Point(6, y));
  y += kLine;
  legendLine(
    out,
    fixed(100.0 * summary.coverage_any_direction, 0) + "% of it has a view within 1 m, " +
      fixed(100.0 * summary.coverage_two_directions, 0) + "% in 2 or more of 8 directions",
    cv::Point(6, y));
  y += kLine;
  int x = 6;
  const std::pair<const char *, int> keys[] = {
    {"no view", 0}, {"1 direction", 1}, {"2-3", 2}, {"4-8", 4}};
  for (const auto & [label, directions] : keys) {
    const cv::Scalar & color = coverageColor(directions);
    const cv::Scalar blended(
      0.5 * color[0] + 127.5, 0.5 * color[1] + 127.5, 0.5 * color[2] + 127.5);
    cv::rectangle(out, cv::Rect(x, y - 11, 14, 14), blended, cv::FILLED);
    cv::rectangle(out, cv::Rect(x, y - 11, 14, 14), cv::Scalar(120, 120, 120), 1);
    x += 18;
    legendLine(out, label, cv::Point(x, y));
    int baseline = 0;
    x += cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.45, 1, &baseline).width + 14;
  }
  return out;
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
  const std::string build_json = metaOr(database, "build_report", "{}");
  YAML::Node build;
  try {
    build = YAML::Load(build_json);
  } catch (const YAML::Exception &) {
    build = YAML::Node();
  }

  // Coverage, with the clearance the build used (from the recording when it didn't say).
  const OccupancyGrid grid = decodeGridPng(
    maps[0].grid, maps[0].resolution, {maps[0].origin_x, maps[0].origin_y, maps[0].origin_yaw});
  std::vector<CoverageView> views;
  views.reserve(keyframes.size());
  for (const auto & k : keyframes) {
    views.push_back({k.refined, per_keyframe[k.id]});
  }
  CoverageParams coverage_params;
  coverage_params.clearance = number(build, "coverage_clearance_m");
  const CoverageMap coverage = measureCoverage(grid, views, coverage_params);
  s.coverage_clearance_m = coverage.clearance;
  s.coverage_reachable_m2 = coverage.reachable_area;
  s.coverage_any_direction = coverage.any_direction;
  s.coverage_two_directions = coverage.two_directions;
  s.coverage_headings = coverage.headings;
  s.coverage_target = number(build, "coverage_target");

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
    .field("coverage_clearance_m", s.coverage_clearance_m)
    .field("coverage_reachable_m2", s.coverage_reachable_m2)
    .field("coverage_any_direction", s.coverage_any_direction)
    .field("coverage_two_directions", s.coverage_two_directions)
    .field("coverage_headings", s.coverage_headings)
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
  if (build && build["mean_keypoints"]) {
    const double hidden = number(build, "floor_mask_hidden");
    md += "| Features | " + fixed(number(build, "mean_keypoints"), 0) + " per keyframe; " +
          (hidden > 0.0 ? "the floor mask hid " + fixed(100.0 * hidden, 1) + " % of the image"
                        : std::string("no floor mask")) +
          " |\n";
  }
  if (coverage.positions.empty()) {
    md += "| Coverage | no keyframe on free floor of the map |\n";
  } else {
    md +=
      "| Coverage | the robot can reach " + fixed(s.coverage_reachable_m2, 0) +
      " m² (free floor at least " + fixed(s.coverage_clearance_m, 2) +
      " m from obstacles, connected to the drive): " + fixed(100.0 * s.coverage_any_direction, 1) +
      " % of it has a view within 1 m" +
      (s.coverage_target > 0.0 ? " (target " + fixed(100.0 * s.coverage_target, 0) + " %)"
                               : std::string()) +
      ", " + fixed(100.0 * s.coverage_two_directions, 1) + " % in two or more of 8 directions; " +
      fixed(100.0 * s.coverage_headings, 1) + " % of its positions x 8 headings |\n";
  }
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
  md += "\n## Keyframe poses\n\n";
  if (build && build["poses_scan_matched"]) {
    const int matched = static_cast<int>(number(build, "poses_scan_matched"));
    const int unmatched = static_cast<int>(number(build, "poses_scan_unmatched"));
    const int links = static_cast<int>(number(build, "poses_odometry_links"));
    md += "| | |\n|---|---|\n";
    md += "| Matched to the map with their scan | " + std::to_string(matched) + " of " +
          std::to_string(static_cast<int>(number(build, "keyframes"))) +
          (unmatched > 0 ? " (" + std::to_string(unmatched) + " scans didn't match)" : "") + " |\n";
    md += "| Recorded poses the scan contradicted | " +
          std::to_string(static_cast<int>(number(build, "poses_scan_contradicted"))) +
          " (their scan match was used instead) |\n";
    md += "| Odometry links | " + std::to_string(links) + " |\n";
    if (build["poses_scan_ambiguous"]) {
      const int ambiguous = static_cast<int>(number(build, "poses_scan_ambiguous"));
      md += "| Scans that fit a second place almost as well | " + std::to_string(ambiguous) +
            (ambiguous > 0 ? ", " + fixed(100.0 * number(build, "poses_ambiguity_median_m"), 0) +
                               " cm away (median), at most " +
                               fixed(100.0 * number(build, "poses_ambiguity_max_m"), 0) +
                               " cm: a wall mapped twice, or repeating structure (orange on the "
                               "coverage map)"
                           : std::string()) +
            " |\n";
    }
    if (build["odometry_straight_steps"]) {
      md += "| Odometry against the scans | " +
            std::to_string(static_cast<int>(number(build, "odometry_straight_steps"))) +
            " straight steps: " + fixed(100.0 * number(build, "odometry_straight_std_m"), 1) +
            " cm per axis (scan noise included); " +
            std::to_string(static_cast<int>(number(build, "odometry_turns"))) +
            " turns: " + fixed(number(build, "odometry_turn_std_per_rad"), 3) +
            " m per radian turned |\n";
    }
    md += "| Correction of the recorded poses | mean " +
          fixed(100.0 * number(build, "poses_mean_correction_m"), 1) + " cm, max " +
          fixed(100.0 * number(build, "poses_max_correction_m"), 1) + " cm, max yaw " +
          fixed(number(build, "poses_max_correction_yaw_deg"), 2) + " deg |\n";
  } else {
    md += "Recorded poses (built before scan matching and odometry existed).\n";
  }
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
  report.coverage = renderCoverage(grid, keyframes, landmarks, coverage, s);
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
