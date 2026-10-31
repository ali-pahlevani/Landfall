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

// `landfall`: offline tools for map databases (build, inspect, report, verify, migrate).

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
#include "landfall/record/staging.hpp"
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

landfall build --map MAP.yaml --output OUT.landfall.db (--staging DIR | --bag BAG)...
               [--config BUILD.yaml] [--report-dir DIR] [--threads N]
               [--map-semantics humble|jazzy]
landfall inspect DB
landfall report DB [--output DIR]
landfall verify DB [--map MAP.yaml] [--map-semantics humble|jazzy]
landfall migrate DB [--no-backup]
landfall --version
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
     "--map-semantics"},
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
      keyframe.quality_json = landfall::toJson(k.quality);
      keyframe.load_image = [folder, k] {
        return landfall::loadStagingImage(*folder, k, cv::IMREAD_COLOR);
      };
      input.keyframes.push_back(std::move(keyframe));
    }
    std::cerr << "staging " << dir << ": " << folder->keyframes.size() << " keyframes"
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
  }

  std::cerr << "building " << output << " from " << input.keyframes.size() << " keyframes\n";
  int last_stage = -1;
  std::size_t last_tenth = 0;
  const auto stats = landfall::buildMap(
    input, config, output, [&](landfall::BuildStage stage, std::size_t done, std::size_t total) {
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
    });

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
              << b.config_sha256.substr(0, 12) << ")\n";
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
    throw UsageError("unknown command '" + command + "'");
  } catch (const UsageError & e) {
    std::cerr << "landfall " << command << ": " << e.what() << "\n\n" << kUsage;
    return 2;
  } catch (const std::exception & e) {
    std::cerr << "landfall " << command << ": error: " << e.what() << "\n";
    return 1;
  }
}
