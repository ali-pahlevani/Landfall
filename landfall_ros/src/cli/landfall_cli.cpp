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

// `landfall`: offline tools for map databases (build, inspect, report, verify, migrate, query).

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "landfall/build/map_builder.hpp"
#include "landfall/build/report.hpp"
#include "landfall/db/database.hpp"
#include "landfall/db/migrations.hpp"
#include "landfall/grid/map_loader.hpp"
#include "landfall/grid/occupancy_grid.hpp"
#include "landfall/query/localizer.hpp"
#include "landfall/record/staging.hpp"
#include "landfall/ros/backends.hpp"
#include "landfall/ros/bag_query.hpp"
#include "landfall/ros/bag_reader.hpp"
#include "landfall/version.hpp"
#include "opencv2/imgcodecs.hpp"

namespace
{

namespace fs = std::filesystem;
using landfall::db::Database;

class UsageError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

constexpr const char * kUsage = R"(usage: landfall <command> [options]

Offline tools for Landfall map databases.

commands:
  build    Build a map database from staging folders and/or rosbag2 recordings
  inspect  Print what a database contains
  report   Write report.md, report.json and coverage.png for a database
  verify   Check a database's integrity, content hash and (optionally) its occupancy map
  migrate  Upgrade a database to this version's schema (keeps a backup)
  query    Relocalize a bag's images against a database, offline, as the relocalizer node would

landfall build --map MAP.yaml --output OUT.landfall.db (--staging DIR | --bag BAG)...
               [--config BUILD.yaml] [--report-dir DIR] [--threads N]
               [--map-semantics humble|jazzy] [--models-dir DIR] [--execution-provider P]
landfall inspect DB
landfall report DB [--output DIR]
landfall verify DB [--map MAP.yaml] [--map-semantics humble|jazzy]
landfall migrate DB [--no-backup]
landfall query DB --bag BAG [--config BUILD.yaml] [--output RESULTS.csv] [--frames N]
               [--every N] [--skip-keyframes] [--set PARAMETER=VALUE]... [--threads N]
               [--models-dir DIR] [--execution-provider P]
landfall --version

Learned backends (xfeat, eigenplaces, megaloc; docs/models.md) find their models in --models-dir,
else backend.models_dir of BUILD.yaml, else $LANDFALL_MODELS_DIR, else
~/.local/share/landfall/models. --execution-provider: cpu (default), cuda or tensorrt.
)";

// Minimal option parser: --name VALUE options (repeatable), --flag switches, positionals.
class Args
{
public:
  Args(
    const std::vector<std::string> & args, const std::set<std::string> & with_value,
    const std::set<std::string> & flags)
  {
    for (std::size_t i = 0; i < args.size(); ++i) {
      const std::string & a = args[i];
      if (a.rfind("--", 0) != 0) {
        positional.push_back(a);
      } else if (with_value.count(a) > 0) {
        if (i + 1 >= args.size()) {
          throw UsageError(a + " needs a value");
        }
        values_[a].push_back(args[++i]);
      } else if (flags.count(a) > 0) {
        flags_.insert(a);
      } else {
        throw UsageError("unknown option " + a);
      }
    }
  }

  std::vector<std::string> all(const std::string & name) const
  {
    const auto it = values_.find(name);
    return it == values_.end() ? std::vector<std::string>() : it->second;
  }

  std::optional<std::string> one(const std::string & name) const
  {
    const auto values = all(name);
    if (values.size() > 1) {
      throw UsageError(name + " given more than once");
    }
    return values.empty() ? std::nullopt : std::optional<std::string>(values.front());
  }

  std::string required(const std::string & name) const
  {
    const auto value = one(name);
    if (!value) {
      throw UsageError(name + " is required");
    }
    return *value;
  }

  bool flag(const std::string & name) const { return flags_.count(name) > 0; }

  std::vector<std::string> positional;

private:
  std::map<std::string, std::vector<std::string>> values_;
  std::set<std::string> flags_;
};

std::string onePositional(const Args & args, const char * what)
{
  if (args.positional.size() != 1) {
    throw UsageError(std::string("expected exactly one ") + what);
  }
  return args.positional.front();
}

landfall::MapServerSemantics semantics(const Args & args)
{
  const std::string name = args.one("--map-semantics").value_or(LANDFALL_DEFAULT_MAP_SEMANTICS);
  if (name == "humble") {
    return landfall::MapServerSemantics::kHumble;
  }
  if (name == "jazzy") {
    return landfall::MapServerSemantics::kJazzy;
  }
  throw UsageError("--map-semantics must be humble or jazzy");
}

std::string readFile(const std::string & path)
{
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    throw std::runtime_error("cannot read " + path);
  }
  std::ostringstream text;
  text << file.rdbuf();
  return text.str();
}

std::string fixed(double value, int decimals)
{
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%.*f", decimals, value);
  return buffer;
}

void printCounts(const std::string & what, const landfall::GateCounts & c)
{
  std::cerr << "  " << what << ": " << c.frames_seen << " frames, " << c.keyframes
            << " keyframes; rejected: no_pose " << c.no_pose << ", no_motion " << c.no_motion
            << ", pose_uncertain " << c.pose_uncertain << ", speed " << c.speed << ", tilt "
            << c.tilt << ", blur " << c.blur << ", exposure " << c.exposure << "\n";
}

bool sameCamera(const landfall::CameraModel & a, const landfall::CameraModel & b)
{
  const auto close = [](double x, double y) {
    return std::abs(x - y) <= 1e-9 * std::max(1.0, std::abs(x));
  };
  if (
    a.width() != b.width() || a.height() != b.height() || a.model() != b.model() ||
    a.coefficients().size() != b.coefficients().size() || !close(a.fx(), b.fx()) ||
    !close(a.fy(), b.fy()) || !close(a.cx(), b.cx()) || !close(a.cy(), b.cy())) {
    return false;
  }
  for (std::size_t i = 0; i < a.coefficients().size(); ++i) {
    if (!close(a.coefficients()[i], b.coefficients()[i])) {
      return false;
    }
  }
  return true;
}

// Adds one input's camera, or checks it against the camera already there (one camera per map).
void addCamera(
  landfall::BuildInput & input, const landfall::BuildCamera & camera, const std::string & from)
{
  if (input.cameras.empty()) {
    input.cameras.push_back(camera);
    return;
  }
  if (!sameCamera(input.cameras.front().model, camera.model)) {
    throw std::runtime_error(
      from +
      " was recorded with different camera intrinsics than the first input; a map is "
      "built from one camera");
  }
  if (input.cameras.front().frame_id != camera.frame_id) {
    std::cerr << "warning: " << from << " uses camera frame '" << camera.frame_id
              << "', the first input '" << input.cameras.front().frame_id << "'\n";
  }
}

int build(const std::vector<std::string> & argv)
{
  const Args args(
    argv,
    {"--map", "--output", "--staging", "--bag", "--config", "--report-dir", "--threads",
     "--map-semantics", "--models-dir", "--execution-provider"},
    {});
  if (!args.positional.empty()) {
    throw UsageError("unexpected argument " + args.positional.front());
  }
  const std::string output = args.required("--output");
  const std::string map_yaml = args.required("--map");
  const auto staging = args.all("--staging");
  const auto bags = args.all("--bag");
  if (staging.empty() && bags.empty()) {
    throw UsageError("give at least one --staging folder or --bag");
  }
  if (fs::exists(output)) {
    throw std::runtime_error(output + " already exists; remove it or choose another --output");
  }
  const auto config_file = args.one("--config");
  landfall::BuildConfigFile config =
    landfall::parseBuildConfig(config_file ? readFile(config_file.value()) : "");
  if (const auto threads = args.one("--threads")) {
    try {
      config.builder.threads = std::stoi(*threads);
    } catch (const std::exception &) {
      throw UsageError("--threads must be a number");
    }
  }
  if (const auto dir = args.one("--models-dir")) {
    config.builder.models_dir = *dir;
  }
  if (const auto provider = args.one("--execution-provider")) {
    config.builder.execution_provider = *provider;
  }
  const auto started = std::chrono::steady_clock::now();

  landfall::BuildInput input;
  input.map = landfall::loadMap(map_yaml, semantics(args));
  std::cerr << "map " << map_yaml << ": " << input.map.grid.width << " x " << input.map.grid.height
            << " cells at " << input.map.grid.resolution << " m\n";

  for (const auto & dir : staging) {
    auto folder = std::make_shared<landfall::StagingFolder>(landfall::readStaging(dir));
    const int session = static_cast<int>(input.sessions.size());
    input.sessions.push_back(
      {folder->session.name, dir, folder->session.pose_source,
       folder->keyframes.empty() ? 0 : folder->keyframes.front().stamp_ns,
       folder->session.complete ? "" : "session was not closed cleanly"});
    landfall::BuildCamera camera;
    camera.frame_id = folder->session.frames.camera;
    camera.model = folder->session.camera;
    if (!folder->keyframes.empty()) {
      camera.T_base_cam = folder->keyframes.front().T_base_cam;
    }
    addCamera(input, camera, dir);
    for (const auto & k : folder->keyframes) {
      landfall::BuildKeyframe keyframe;
      keyframe.session = session;
      keyframe.stamp_ns = k.stamp_ns;
      keyframe.prior = k.map_base;
      keyframe.covariance = k.covariance;
      keyframe.T_base_cam = k.T_base_cam;
      keyframe.odom = k.odom_base;
      keyframe.scan = k.scan;
      keyframe.quality_json = landfall::toJson(k.quality);
      keyframe.load_image = [folder, k] {
        return landfall::loadStagingImage(*folder, k, cv::IMREAD_COLOR);
      };
      input.keyframes.push_back(std::move(keyframe));
    }
    const auto scans = std::count_if(
      folder->keyframes.begin(), folder->keyframes.end(),
      [](const landfall::StagingKeyframe & k) { return k.scan.has_value(); });
    std::cerr << "staging " << dir << ": " << folder->keyframes.size() << " keyframes, " << scans
              << " with a LiDAR scan"
              << (folder->session.complete ? "" : " (session not closed cleanly)") << "\n";
  }
  for (const auto & uri : bags) {
    std::cerr << "bag " << uri << ": selecting keyframes...\n";
    std::size_t last_percent = 0;
    auto bag =
      landfall::ros::readBagKeyframes(uri, config, [&](std::size_t done, std::size_t total) {
        const std::size_t percent = total > 0 ? 100 * done / total : 0;
        if (percent >= last_percent + 10) {
          last_percent = percent;
          std::cerr << "  " << percent << " % (" << done << " images)\n";
        }
      });
    const int session = static_cast<int>(input.sessions.size());
    input.sessions.push_back(bag.session);
    addCamera(input, bag.camera, uri);
    for (auto & keyframe : bag.keyframes) {
      keyframe.session = session;
      input.keyframes.push_back(std::move(keyframe));
    }
    printCounts(uri, bag.counts);
    std::cerr << "  " << bag.with_scan << " of " << bag.keyframes.size()
              << " keyframes with a LiDAR scan ('" << config.bag.scan_topic << "')\n";
  }

  std::cerr << "building " << output << " from " << input.keyframes.size() << " keyframes\n";
  int last_stage = -1;
  std::size_t last_tenth = 0;
  const auto stats = landfall::buildMap(
    input, config, output,
    [&](landfall::BuildStage stage, std::size_t done, std::size_t total) {
      const int index = static_cast<int>(stage);
      if (index != last_stage) {
        last_stage = index;
        last_tenth = 0;
      }
      const std::size_t tenth = total > 0 ? 10 * done / total : 10;
      if (tenth > last_tenth || (done == total && last_tenth < 10)) {
        last_tenth = std::max(tenth, done == total ? std::size_t{10} : tenth);
        std::cerr << "  " << landfall::toString(stage) << " " << done << "/" << total << "\n";
      }
    },
    &landfall::ros::backendRegistry());

  const std::string report_dir = args.one("--report-dir").value_or(output + ".report");
  const Database database = Database::open(output, Database::Access::kReadOnly);
  landfall::writeReport(landfall::makeReport(database), report_dir);
  const double seconds =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();

  std::cout << "built " << output << " in " << fixed(seconds, 1) << " s\n"
            << "  keyframes " << stats.keyframes << ", landmarks " << stats.landmarks
            << ", mean track length " << fixed(stats.mean_track_length, 2) << ", reprojection RMS "
            << fixed(stats.reprojection_rms_px, 3) << " px\n"
            << "  image pairs " << stats.pairs_verified << "/" << stats.pairs_candidate
            << " verified; BA " << (stats.ba_run ? (stats.ba_usable ? "ok" : "FAILED") : "off")
            << ", shift mean " << fixed(100.0 * stats.ba_mean_shift_m, 2) << " cm, max "
            << fixed(100.0 * stats.ba_max_shift_m, 2) << " cm / "
            << fixed(stats.ba_max_shift_yaw_deg, 2) << " deg, " << stats.ba_keyframes_at_bound
            << " at the bound\n"
            << "  features: " << fixed(stats.mean_keypoints, 0) << " per keyframe, "
            << (stats.floor_mask_hidden > 0.0
                  ? "the floor mask hid " + fixed(100.0 * stats.floor_mask_hidden, 1) +
                      " % of the image"
                  : std::string("no floor mask"))
            << "\n"
            << "  keyframe poses: " << stats.poses_scan_matched
            << " matched to the map with their scan"
            << (stats.poses_scan_unmatched > 0
                  ? " (" + std::to_string(stats.poses_scan_unmatched) + " didn't match)"
                  : std::string())
            << " (" << stats.poses_scan_contradicted << " contradicted the recorded pose), "
            << stats.poses_odometry_links << " odometry links, corrected by "
            << fixed(100.0 * stats.poses_mean_correction_m, 1) << " cm on average, at most "
            << fixed(100.0 * stats.poses_max_correction_m, 1) << " cm\n"
            << "  content SHA-256 " << database.meta("content_sha256").value_or("?") << "\n"
            << "  report in " << report_dir << "\n";
  for (const auto & warning : stats.warnings) {
    std::cout << "warning: " << warning << "\n";
  }
  return 0;
}

int inspect(const std::vector<std::string> & argv)
{
  const Args args(argv, {}, {});
  const std::string path = onePositional(args, "database");
  const Database db = Database::open(path, Database::Access::kReadOnly);
  std::cout << path << "\n"
            << "  schema version " << landfall::db::kSchemaVersion << ", built by Landfall "
            << db.meta("landfall_version").value_or("?") << " at "
            << db.meta("created_utc").value_or("?") << "\n"
            << "  content SHA-256 " << db.meta("content_sha256").value_or("(not set)") << "\n";
  for (const auto & m : db.maps()) {
    std::cout << "  map: " << m.width << " x " << m.height << " cells at " << m.resolution
              << " m, origin (" << m.origin_x << ", " << m.origin_y << ", " << m.origin_yaw
              << "), grid SHA-256 " << m.grid_sha256 << "\n";
  }
  for (const auto & c : db.cameras()) {
    std::cout << "  camera " << c.id << ": frame '" << c.frame_id << "', " << c.width << " x "
              << c.height << ", " << c.model << ", fx " << c.intrinsics.at(0) << ", fy "
              << c.intrinsics.at(1) << "\n";
  }
  for (const auto & s : db.sessions()) {
    std::cout << "  session " << s.id << ": " << s.name << " from " << s.source << " ("
              << s.pose_source << ")" << (s.notes.empty() ? "" : ", " + s.notes) << "\n";
  }
  for (const auto & b : db.backends()) {
    std::cout << "  backend " << b.role << ": " << b.name << " v" << b.version << " (config "
              << b.config_sha256.substr(0, 12)
              << (b.model_sha256.empty() ? "" : ", model " + b.model_sha256.substr(0, 12)) << ")\n";
  }
  const auto keyframes = db.keyframes();
  int flagged = 0;
  for (const auto & k : keyframes) {
    flagged += (k.flags & landfall::db::KeyframeRecord::kFlagBaShiftAtBound) != 0 ? 1 : 0;
  }
  std::cout << "  keyframes " << keyframes.size() << " (" << flagged << " flagged), landmarks "
            << db.landmarks().size() << ", observations " << db.allObservations().size()
            << ", covisible pairs " << db.covisibility().size() << "\n";
  if (const auto config = db.meta("builder_config")) {
    std::cout << "  builder configuration:\n";
    std::istringstream lines(*config);
    std::string line;
    while (std::getline(lines, line)) {
      std::cout << "    " << line << "\n";
    }
  }
  return 0;
}

int report(const std::vector<std::string> & argv)
{
  const Args args(argv, {"--output"}, {});
  const std::string path = onePositional(args, "database");
  const std::string output = args.one("--output").value_or(path + ".report");
  const Database db = Database::open(path, Database::Access::kReadOnly);
  landfall::writeReport(landfall::makeReport(db), output);
  std::cout << "report written to " << output << "\n";
  return 0;
}

int verify(const std::vector<std::string> & argv)
{
  const Args args(argv, {"--map", "--map-semantics"}, {});
  const std::string path = onePositional(args, "database");
  const Database db = Database::open(path, Database::Access::kReadOnly);
  bool ok = true;
  const auto check = [&](bool passed, const std::string & what, const std::string & detail = "") {
    std::cout << (passed ? "  ok      " : "  FAILED  ") << what
              << (detail.empty() ? "" : ": " + detail) << "\n";
    ok = ok && passed;
  };
  std::cout << path << "\n";
  const std::string integrity = db.integrityCheck();
  check(integrity == "ok", "SQLite integrity", integrity);
  const auto stored = db.meta("content_sha256");
  const std::string actual = db.contentSha256();
  check(stored.has_value() && *stored == actual, "content hash", stored ? actual : "not recorded");
  if (const auto map_yaml = args.one("--map")) {
    const auto maps = db.maps();
    const auto map = landfall::loadMap(*map_yaml, semantics(args));
    if (maps.empty()) {
      check(false, "map", "the database has no map");
    } else {
      const auto & m = maps.front();
      check(
        m.width == map.grid.width && m.height == map.grid.height &&
          std::abs(m.resolution - map.grid.resolution) < 1e-6 &&
          std::abs(m.origin_x - map.grid.origin.x) < 1e-6 &&
          std::abs(m.origin_y - map.grid.origin.y) < 1e-6,
        "map size, resolution and origin");
      check(m.grid_sha256 == landfall::gridSha256(map.grid), "map cells");
      // Informational: the files may differ (e.g. re-saved) while the grid is identical.
      std::cout << "  " << (m.image_sha256 == map.image_sha256 ? "same" : "different")
                << " image file, " << (m.yaml_sha256 == map.yaml_sha256 ? "same" : "different")
                << " YAML file\n";
    }
  }
  std::cout << (ok ? "verified\n" : "verification FAILED\n");
  return ok ? 0 : 1;
}

int migrate(const std::vector<std::string> & argv)
{
  const Args args(argv, {}, {"--no-backup"});
  const std::string path = onePositional(args, "database");
  const auto result = landfall::db::migrateDatabase(
    path, landfall::db::kSchemaVersion, landfall::db::builtinMigrations(),
    !args.flag("--no-backup"));
  if (result.applied.empty()) {
    std::cout << path << " is at schema version " << result.to_version << "; nothing to do\n";
    return 0;
  }
  std::cout << path << ": schema " << result.from_version << " -> " << result.to_version << "\n";
  for (const auto & step : result.applied) {
    std::cout << "  " << step << "\n";
  }
  if (!result.backup_path.empty()) {
    std::cout << "  backup: " << result.backup_path << "\n";
  }
  return 0;
}

// --set NAME=VALUE: the relocalizer's threshold parameters, under the node's names.
struct QuerySettings
{
  landfall::LocalizerParams localizer;
  landfall::ConsistencyParams consistency;
  landfall::GravityParams gravity;
  double min_frame_interval = 0.3;
  double timeout = 10.0;
  double scan_max_time_offset = 0.5;
};

void setQueryParameter(QuerySettings & q, const std::string & assignment)
{
  const auto eq = assignment.find('=');
  if (eq == std::string::npos) {
    throw UsageError("--set expects PARAMETER=VALUE, got '" + assignment + "'");
  }
  const std::string name = assignment.substr(0, eq);
  const std::string value = assignment.substr(eq + 1);
  const auto number = [&] {
    std::size_t used = 0;
    double v = 0.0;
    try {
      v = std::stod(value, &used);
    } catch (const std::exception &) {
      used = 0;
    }
    if (used == 0 || used != value.size()) {
      throw UsageError("--set " + name + ": '" + value + "' is not a number");
    }
    return v;
  };
  auto & l = q.localizer;
  if (name == "query.top_k") {
    l.top_k = static_cast<int>(number());
  } else if (name == "query.timeout") {
    q.timeout = number();
  } else if (name == "matching.ratio") {
    l.ratio.ratio = number();
  } else if (name == "matching.min_cosine") {
    l.mnn.min_cosine = number();
  } else if (name == "pnp.solver") {
    const auto strategy = landfall::parsePnpStrategy(value);
    if (!strategy) {
      throw UsageError("--set pnp.solver: upright, p3p, upright_then_p3p or opencv");
    }
    l.pnp = *strategy;
  } else if (name == "pnp.max_reproj_error_px") {
    l.pnp_threshold_px = number();
  } else if (name == "pnp.max_iterations") {
    l.pnp_max_iterations = static_cast<int>(number());
  } else if (name == "pnp.success_prob") {
    l.pnp_success_prob = number();
  } else if (name == "sanity.min_inliers") {
    l.sanity.min_inliers = static_cast<int>(number());
  } else if (name == "sanity.min_inlier_ratio") {
    l.sanity.min_inlier_ratio = number();
  } else if (name == "sanity.max_mean_reproj_error_px") {
    l.sanity.max_mean_reprojection_px = number();
  } else if (name == "sanity.max_height_error") {
    l.sanity.max_height_error = number();
  } else if (name == "sanity.max_floor_elevation") {
    l.sanity.max_floor_elevation = number();
  } else if (name == "sanity.max_tilt_error_deg") {
    l.sanity.max_tilt_error_deg = number();
  } else if (name == "sanity.min_inlier_coverage") {
    l.sanity.min_inlier_coverage = number();
  } else if (name == "sanity.require_free_space") {
    if (value != "true" && value != "false") {
      throw UsageError("--set sanity.require_free_space: true or false");
    }
    l.sanity.require_free_space = value == "true";
  } else if (name == "ambiguity.cluster_radius") {
    l.clustering.radius = number();
  } else if (name == "ambiguity.cluster_angle_deg") {
    l.clustering.angle_deg = number();
  } else if (name == "ambiguity.max_score_ratio") {
    l.max_score_ratio = number();
  } else if (name == "consistency.min_frame_interval") {
    q.min_frame_interval = number();
  } else if (name == "consistency.max_translation_error") {
    q.consistency.max_translation_error = number();
  } else if (name == "consistency.max_rotation_error_deg") {
    q.consistency.max_rotation_error_deg = number();
  } else if (name == "scan_check.mode") {
    const auto mode = landfall::parseScanCheckMode(value);
    if (!mode) {
      throw UsageError("--set scan_check.mode: required, optional or disabled");
    }
    l.scan_mode = *mode;
  } else if (name == "scan_check.max_time_offset") {
    q.scan_max_time_offset = number();
  } else if (name == "scan_check.max_range") {
    l.scan.max_range = number();
  } else if (name == "scan_check.hit_tolerance") {
    l.scan.hit_tolerance = number();
  } else if (name == "scan_check.min_hit_ratio") {
    l.scan.min_hit_ratio = number();
  } else if (name == "scan_check.max_violation_ratio") {
    l.scan.max_violation_ratio = number();
  } else if (name == "scan_check.min_known_ratio") {
    l.scan.min_known_ratio = number();
  } else if (name == "scan_check.min_valid_beams") {
    l.scan.min_valid_beams = static_cast<int>(number());
  } else if (name == "scan_check.search_radius") {
    l.scan.search_radius = number();
  } else if (name == "scan_check.search_angle_deg") {
    l.scan.search_angle_deg = number();
  } else if (name == "scan_check.refine_pose") {
    if (value != "true" && value != "false") {
      throw UsageError("--set scan_check.refine_pose: true or false");
    }
    l.refine_with_scan = value == "true";
  } else if (name == "scan_check.check_alternatives") {
    if (value != "true" && value != "false") {
      throw UsageError("--set scan_check.check_alternatives: true or false");
    }
    l.scan.check_alternatives = value == "true";
  } else if (name == "scan_check.alternative_margin") {
    l.scan.alternative_margin = number();
  } else if (name == "imu.max_age") {
    q.gravity.max_age = number();
  } else if (name == "imu.max_angular_rate") {
    q.gravity.max_angular_rate = number();
  } else if (name == "gravity.ramp_tilt_deg") {
    q.gravity.ramp_tilt_deg = number();
  } else {
    throw UsageError("--set: unknown or structural parameter '" + name + "'");
  }
}

double percentile(std::vector<double> values, double p)
{
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const auto i = static_cast<std::size_t>(std::ceil(p * static_cast<double>(values.size())) - 1);
  return values[std::min(i, values.size() - 1)];
}

// One relocalization request replayed from the bag: frames until `frames` produced a pose, one
// was ambiguous, or `timeout` seconds of recording passed (as the node does with live images).
struct ReplayedRequest
{
  int64_t first_stamp = 0;
  int64_t last_stamp = 0;
  std::vector<landfall::FrameOutcome> outcomes;
  std::vector<int64_t> valid_stamps;
  std::vector<landfall::FrameResult> valid_frames;
  landfall::FrameResult last_frame;
};

int query(const std::vector<std::string> & argv)
{
  const Args args(
    argv,
    {"--bag", "--config", "--output", "--frames", "--every", "--set", "--threads", "--models-dir",
     "--execution-provider"},
    {"--skip-keyframes"});
  const std::string db_path = onePositional(args, "database");
  const std::string bag_uri = args.required("--bag");
  const auto config_path = args.one("--config");
  const landfall::BuildConfigFile config =
    landfall::parseBuildConfig(config_path ? readFile(*config_path) : "");
  const landfall::BagConfig & bag = config.bag;
  const int frames = std::stoi(args.one("--frames").value_or("1"));
  const int every = std::stoi(args.one("--every").value_or("1"));
  if (frames < 1 || frames > 10 || every < 1) {
    throw UsageError("--frames must be in 1..10 and --every at least 1");
  }
  QuerySettings settings;
  settings.localizer.threads = std::stoi(args.one("--threads").value_or("0"));
  for (const auto & assignment : args.all("--set")) {
    setQueryParameter(settings, assignment);
  }

  landfall::LocalizationMap::Options map_options;
  map_options.registry = &landfall::ros::backendRegistry();
  map_options.models_dir = args.one("--models-dir").value_or(config.builder.models_dir);
  map_options.execution_provider =
    args.one("--execution-provider").value_or(config.builder.execution_provider);
  map_options.threads = settings.localizer.threads;
  const auto map = landfall::LocalizationMap::load(db_path, map_options);
  landfall::Localizer localizer(map, settings.localizer);
  std::set<int64_t> keyframe_stamps;
  if (args.flag("--skip-keyframes")) {
    for (const auto & k : Database::open(db_path, Database::Access::kReadOnly).keyframes()) {
      keyframe_stamps.insert(k.stamp_ns);
    }
  }

  landfall::ros::BagQueryOptions options;
  options.bag = bag;
  options.gravity = settings.gravity;
  options.every = static_cast<std::size_t>(every);
  options.min_frame_interval = settings.min_frame_interval;
  options.skip_stamps = keyframe_stamps;
  options.scan_mode = settings.localizer.scan_mode;
  options.scan_max_time_offset = settings.scan_max_time_offset;

  std::ofstream csv_file;
  const auto output = args.one("--output");
  if (output) {
    csv_file.open(*output);
    if (!csv_file) {
      throw std::runtime_error("cannot write " + *output);
    }
  }
  std::ostream & csv = output ? static_cast<std::ostream &>(csv_file) : std::cout;
  csv << "request,first_stamp_ns,last_stamp_ns,status,frames,frames_used,pose_stamp_ns,x,y,yaw,"
         "inliers,inlier_ratio,coverage,height_error_m,tilt_error_deg,ambiguity_ratio,"
         "consistency_m,consistency_deg,scan_hit_ratio,scan_violation_ratio,features,places,"
         "best_x,best_y,best_yaw,best_score,"
         "runner_up_x,runner_up_y,runner_up_yaw,runner_up_score,message,rejection\n";

  std::map<std::string, int> statuses;
  std::vector<double> frame_seconds;
  int requests = 0;
  std::optional<ReplayedRequest> current;
  const auto timeout_ns = static_cast<int64_t>(settings.timeout * 1e9);

  const auto finish = [&](const ReplayedRequest & r) {
    landfall::DecisionInput input;
    input.frames = r.outcomes;
    input.required_frames = frames;
    const auto decision = landfall::decide(input, settings.consistency);
    ++statuses[landfall::toString(decision.status)];
    const landfall::ScoreSummary scores = landfall::summarizeScores(
      r.valid_frames.empty() ? std::vector<landfall::FrameResult>{r.last_frame} : r.valid_frames);
    std::string message = decision.message;
    std::string rejection =
      r.last_frame.hypotheses.empty() ? "" : r.last_frame.hypotheses.front().rejection;
    std::replace(message.begin(), message.end(), ',', ';');
    std::replace(rejection.begin(), rejection.end(), ',', ';');
    csv << requests++ << "," << r.first_stamp << "," << r.last_stamp << ","
        << landfall::toString(decision.status) << "," << r.outcomes.size() << ","
        << decision.frames_used << ",";
    if (decision.pose) {
      csv << r.valid_stamps.back() << "," << fixed(decision.pose->x, 4) << ","
          << fixed(decision.pose->y, 4) << "," << fixed(decision.pose->yaw, 5) << ",";
    } else {
      csv << ",,,,";
    }
    if (scores.any) {
      csv << scores.num_inliers << "," << fixed(scores.inlier_ratio, 3) << ","
          << fixed(scores.inlier_coverage, 3) << "," << fixed(scores.height_error_m, 3) << ","
          << fixed(scores.tilt_error_deg, 2) << ",";
    } else {
      csv << ",,,,,";
    }
    csv << fixed(scores.ambiguity_ratio, 3) << ","
        << fixed(decision.consistency.max_translation_error, 3) << ","
        << fixed(decision.consistency.max_rotation_error_deg, 2) << ",";
    if (scores.scan) {
      csv << fixed(scores.scan_hit_ratio, 3) << "," << fixed(scores.scan_violation_ratio, 3) << ",";
    } else {
      csv << ",,";
    }
    csv << r.last_frame.num_features << "," << r.last_frame.hypotheses.size() << ",";
    const auto & clusters = r.last_frame.clusters;
    if (clusters.empty()) {
      csv << ",,,,";
    } else {
      csv << fixed(clusters[0].pose.x, 3) << "," << fixed(clusters[0].pose.y, 3) << ","
          << fixed(clusters[0].pose.yaw, 4) << "," << fixed(clusters[0].score, 0) << ",";
    }
    if (clusters.size() > 1) {
      csv << fixed(clusters[1].pose.x, 3) << "," << fixed(clusters[1].pose.y, 3) << ","
          << fixed(clusters[1].pose.yaw, 4) << "," << fixed(clusters[1].score, 0) << ",";
    } else {
      csv << ",,,,";
    }
    csv << message << "," << rejection << "\n";
  };

  const auto counts = landfall::ros::queryBagImages(
    bag_uri, localizer, options,
    [&](int64_t stamp, const landfall::Pose2 & odom_T_base, landfall::FrameResult && result) {
      frame_seconds.push_back(result.timings.total_s);
      if (current && stamp - current->first_stamp > timeout_ns) {
        finish(*current);
        current.reset();
      }
      if (!current) {
        current = ReplayedRequest();
        current->first_stamp = stamp;
      }
      landfall::FrameOutcome outcome;
      outcome.deepest_stage = result.deepest_stage;
      outcome.ambiguous = result.ambiguity.ambiguous;
      outcome.scan_elsewhere = result.scan_alternative.has_value();
      if (result.pose && !outcome.ambiguous) {
        outcome.estimate = landfall::FrameEstimate{*result.pose, odom_T_base};
        current->valid_stamps.push_back(stamp);
        current->valid_frames.push_back(result);
      }
      current->outcomes.push_back(outcome);
      current->last_stamp = stamp;
      current->last_frame = std::move(result);
      if (outcome.ambiguous || static_cast<int>(current->valid_frames.size()) >= frames) {
        finish(*current);
        current.reset();
      }
    });
  if (current) {
    finish(*current);
  }

  std::cerr << "query: " << counts.images << " images in the bag, " << frame_seconds.size()
            << " localized frames in " << requests << " requests of " << frames << " frame"
            << (frames > 1 ? "s" : "") << "\n";
  if (
    counts.skipped_stamps + counts.skipped_tf + counts.skipped_gravity + counts.skipped_scan > 0) {
    std::cerr << "  skipped: " << counts.skipped_stamps << " database keyframes, "
              << counts.skipped_tf << " without TF, " << counts.skipped_gravity
              << " without usable IMU gravity, " << counts.skipped_scan << " without a scan\n";
  }
  std::cerr << "  results:";
  for (const auto & [status, count] : statuses) {
    std::cerr << " " << status << " " << count;
  }
  std::cerr << "\n  seconds per frame: median " << fixed(percentile(frame_seconds, 0.5), 3)
            << ", p95 " << fixed(percentile(frame_seconds, 0.95), 3) << ", max "
            << fixed(percentile(frame_seconds, 1.0), 3) << "\n";
  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  std::vector<std::string> args(argv + 1, argv + argc);
  if (args.empty()) {
    std::cerr << kUsage;
    return 2;
  }
  const std::string command = args.front();
  args.erase(args.begin());
  if (command == "-h" || command == "--help" || command == "help") {
    std::cout << kUsage;
    return 0;
  }
  if (command == "--version") {
    std::cout << "landfall " << landfall::version() << "\n";
    return 0;
  }
  try {
    if (command == "build") {
      return build(args);
    }
    if (command == "inspect") {
      return inspect(args);
    }
    if (command == "report") {
      return report(args);
    }
    if (command == "verify") {
      return verify(args);
    }
    if (command == "migrate") {
      return migrate(args);
    }
    if (command == "query") {
      return query(args);
    }
    throw UsageError("unknown command '" + command + "'");
  } catch (const UsageError & e) {
    std::cerr << "landfall " << command << ": " << e.what() << "\n\n" << kUsage;
    return 2;
  } catch (const std::exception & e) {
    std::cerr << "landfall " << command << ": error: " << e.what() << "\n";
    return 1;
  }
}
