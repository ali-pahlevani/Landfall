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

#include "landfall/record/staging.hpp"

#include <yaml-cpp/yaml.h>

#include <cstdio>
#include <filesystem>
#include <opencv2/imgcodecs.hpp>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace landfall
{

namespace
{

namespace fs = std::filesystem;

constexpr const char * kSessionFile = "session.yaml";
constexpr const char * kKeyframesFile = "keyframes.csv";
constexpr const char * kImagesDir = "images";
constexpr const char * kFormatName = "landfall_staging";
constexpr const char * kCsvHeader =
  "id,stamp_ns,image,x,y,yaw,cov_xx,cov_xy,cov_xyaw,cov_yy,cov_yyaw,cov_yawyaw,"
  "base_cam_tx,base_cam_ty,base_cam_tz,base_cam_qx,base_cam_qy,base_cam_qz,base_cam_qw,"
  "sharpness,mean_intensity,saturated_fraction,tilt_deg";
constexpr int kCsvColumns = 23;

// Shortest text that reads back to the same double.
std::string number(double value)
{
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.17g", value);
  return buffer;
}

void emitCounts(YAML::Emitter & out, const GateCounts & counts)
{
  out << YAML::Key << "counts" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "frames_seen" << YAML::Value << counts.frames_seen;
  out << YAML::Key << "keyframes" << YAML::Value << counts.keyframes;
  out << YAML::Key << "no_pose" << YAML::Value << counts.no_pose;
  out << YAML::Key << "no_motion" << YAML::Value << counts.no_motion;
  out << YAML::Key << "pose_uncertain" << YAML::Value << counts.pose_uncertain;
  out << YAML::Key << "speed" << YAML::Value << counts.speed;
  out << YAML::Key << "tilt" << YAML::Value << counts.tilt;
  out << YAML::Key << "blur" << YAML::Value << counts.blur;
  out << YAML::Key << "exposure" << YAML::Value << counts.exposure;
  out << YAML::EndMap;
}

std::vector<std::string> splitCsv(const std::string & line)
{
  std::vector<std::string> fields;
  std::string field;
  std::istringstream stream(line);
  while (std::getline(stream, field, ',')) {
    fields.push_back(field);
  }
  if (!line.empty() && line.back() == ',') {
    fields.emplace_back();
  }
  return fields;
}

// The whole text as a number, or nullopt.
template <typename T, typename Parse>
std::optional<T> parseWhole(const std::string & text, Parse parse)
{
  try {
    std::size_t used = 0;
    const T value = parse(text, &used);
    return used == text.size() ? std::optional<T>(value) : std::nullopt;
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

double parseDouble(const std::string & text, const std::string & what)
{
  const auto value = parseWhole<double>(
    text, [](const std::string & t, std::size_t * used) { return std::stod(t, used); });
  if (!value) {
    throw std::runtime_error("invalid number for " + what + ": '" + text + "'");
  }
  return *value;
}

int64_t parseInt(const std::string & text, const std::string & what)
{
  const auto value = parseWhole<int64_t>(
    text, [](const std::string & t, std::size_t * used) { return std::stoll(t, used); });
  if (!value) {
    throw std::runtime_error("invalid integer for " + what + ": '" + text + "'");
  }
  return *value;
}

StagingKeyframe parseRow(const std::vector<std::string> & f, const std::string & where)
{
  StagingKeyframe k;
  k.id = parseInt(f[0], where + " id");
  k.stamp_ns = parseInt(f[1], where + " stamp_ns");
  k.image = f[2];
  k.map_base = {
    parseDouble(f[3], where + " x"), parseDouble(f[4], where + " y"),
    parseDouble(f[5], where + " yaw")};
  bool any_cov = false;
  bool all_cov = true;
  for (int i = 6; i < 12; ++i) {
    any_cov = any_cov || !f[i].empty();
    all_cov = all_cov && !f[i].empty();
  }
  if (any_cov && !all_cov) {
    throw std::runtime_error(where + ": incomplete covariance");
  }
  if (all_cov) {
    const double xx = parseDouble(f[6], where + " cov_xx");
    const double xy = parseDouble(f[7], where + " cov_xy");
    const double xyaw = parseDouble(f[8], where + " cov_xyaw");
    const double yy = parseDouble(f[9], where + " cov_yy");
    const double yyaw = parseDouble(f[10], where + " cov_yyaw");
    const double yawyaw = parseDouble(f[11], where + " cov_yawyaw");
    Mat3 cov;
    cov << xx, xy, xyaw, xy, yy, yyaw, xyaw, yyaw, yawyaw;
    k.covariance = cov;
  }
  k.T_base_cam.translation() = Vec3(
    parseDouble(f[12], where + " base_cam_tx"), parseDouble(f[13], where + " base_cam_ty"),
    parseDouble(f[14], where + " base_cam_tz"));
  Eigen::Quaterniond q(
    parseDouble(f[18], where + " base_cam_qw"), parseDouble(f[15], where + " base_cam_qx"),
    parseDouble(f[16], where + " base_cam_qy"), parseDouble(f[17], where + " base_cam_qz"));
  if (q.norm() < 1e-6) {
    throw std::runtime_error(where + ": invalid base->camera rotation");
  }
  k.T_base_cam.linear() = q.normalized().toRotationMatrix();
  k.quality.sharpness = parseDouble(f[19], where + " sharpness");
  k.quality.mean_intensity = parseDouble(f[20], where + " mean_intensity");
  k.quality.saturated_fraction = parseDouble(f[21], where + " saturated_fraction");
  if (!f[22].empty()) {
    k.tilt_deg = parseDouble(f[22], where + " tilt_deg");
  }
  return k;
}

}  // namespace

StagingWriter::StagingWriter(std::string directory, StagingSession session)
: directory_(std::move(directory)), session_(std::move(session))
{
  if (session_.image_format != "png" && session_.image_format != "jpg") {
    throw std::invalid_argument("staging image format must be png or jpg");
  }
  if (fs::exists(directory_)) {
    throw std::runtime_error("staging folder already exists: " + directory_);
  }
  fs::create_directories(fs::path(directory_) / kImagesDir);
  session_.complete = false;
  writeSession();
  csv_.open(fs::path(directory_) / kKeyframesFile, std::ios::out | std::ios::trunc);
  if (!csv_) {
    throw std::runtime_error("cannot write " + directory_ + "/" + kKeyframesFile);
  }
  csv_ << kCsvHeader << '\n' << std::flush;
}

const StagingKeyframe & StagingWriter::add(StagingKeyframe keyframe, const cv::Mat & image)
{
  char name[32];
  std::snprintf(
    name, sizeof(name), "%06lld.%s", static_cast<long long>(next_id_),
    session_.image_format.c_str());
  keyframe.id = next_id_;
  keyframe.image = std::string(kImagesDir) + "/" + name;
  std::vector<int> params;
  if (session_.image_format == "jpg") {
    params = {cv::IMWRITE_JPEG_QUALITY, session_.jpeg_quality};
  }
  const std::string path = (fs::path(directory_) / keyframe.image).string();
  if (!cv::imwrite(path, image, params)) {
    throw std::runtime_error("cannot write image " + path);
  }

  std::string row = std::to_string(keyframe.id) + "," + std::to_string(keyframe.stamp_ns) + "," +
                    keyframe.image + "," + number(keyframe.map_base.x) + "," +
                    number(keyframe.map_base.y) + "," + number(keyframe.map_base.yaw);
  if (keyframe.covariance) {
    const Mat3 & c = *keyframe.covariance;
    for (const double v : {c(0, 0), c(0, 1), c(0, 2), c(1, 1), c(1, 2), c(2, 2)}) {
      row += "," + number(v);
    }
  } else {
    row += ",,,,,,";
  }
  const Vec3 t = keyframe.T_base_cam.translation();
  const Eigen::Quaterniond q(keyframe.T_base_cam.linear());
  for (const double v : {t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w()}) {
    row += "," + number(v);
  }
  row += "," + number(keyframe.quality.sharpness) + "," + number(keyframe.quality.mean_intensity) +
         "," + number(keyframe.quality.saturated_fraction) + ",";
  if (keyframe.tilt_deg) {
    row += number(*keyframe.tilt_deg);
  }
  csv_ << row << '\n' << std::flush;
  if (!csv_) {
    throw std::runtime_error("cannot append to " + directory_ + "/" + kKeyframesFile);
  }
  ++next_id_;
  last_ = std::move(keyframe);
  return last_;
}

void StagingWriter::close(const GateCounts & counts)
{
  csv_.close();
  session_.complete = true;
  session_.counts = counts;
  writeSession();
}

void StagingWriter::writeSession() const
{
  const CameraModel & camera = session_.camera;
  YAML::Emitter out;
  out.SetDoublePrecision(17);
  out << YAML::BeginMap;
  out << YAML::Key << "format" << YAML::Value << kFormatName;
  out << YAML::Key << "format_version" << YAML::Value << kStagingFormatVersion;
  out << YAML::Key << "session" << YAML::Value << session_.name;
  out << YAML::Key << "created_utc" << YAML::Value << session_.created_utc;
  out << YAML::Key << "landfall_version" << YAML::Value << session_.landfall_version;
  out << YAML::Key << "pose_source" << YAML::Value << session_.pose_source;
  out << YAML::Key << "frames" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "global" << YAML::Value << session_.frames.global;
  out << YAML::Key << "odom" << YAML::Value << session_.frames.odom;
  out << YAML::Key << "base" << YAML::Value << session_.frames.base;
  out << YAML::Key << "camera" << YAML::Value << session_.frames.camera;
  out << YAML::EndMap;
  out << YAML::Key << "camera" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "width" << YAML::Value << camera.width();
  out << YAML::Key << "height" << YAML::Value << camera.height();
  out << YAML::Key << "distortion_model" << YAML::Value << toString(camera.model());
  out << YAML::Key << "intrinsics" << YAML::Value << YAML::Flow
      << std::vector<double>{camera.fx(), camera.fy(), camera.cx(), camera.cy()};
  out << YAML::Key << "distortion" << YAML::Value << YAML::Flow << camera.coefficients();
  out << YAML::EndMap;
  out << YAML::Key << "image_format" << YAML::Value << session_.image_format;
  out << YAML::Key << "jpeg_quality" << YAML::Value << session_.jpeg_quality;
  out << YAML::Key << "complete" << YAML::Value << session_.complete;
  emitCounts(out, session_.counts);
  out << YAML::EndMap;

  // Write and rename, so session.yaml is never seen half-written.
  const fs::path target = fs::path(directory_) / kSessionFile;
  const fs::path temporary = fs::path(directory_) / (std::string(kSessionFile) + ".tmp");
  {
    std::ofstream file(temporary, std::ios::out | std::ios::trunc);
    file << out.c_str() << '\n';
    if (!file) {
      throw std::runtime_error("cannot write " + temporary.string());
    }
  }
  fs::rename(temporary, target);
}

StagingFolder readStaging(const std::string & directory)
{
  StagingFolder folder;
  folder.directory = directory;
  const fs::path session_path = fs::path(directory) / kSessionFile;
  if (!fs::exists(session_path)) {
    throw std::runtime_error("not a staging folder (no session.yaml): " + directory);
  }
  try {
    const YAML::Node doc = YAML::LoadFile(session_path.string());
    if (!doc["format"] || doc["format"].as<std::string>() != kFormatName) {
      throw std::runtime_error("session.yaml is not a Landfall staging session");
    }
    const int version = doc["format_version"].as<int>();
    if (version != kStagingFormatVersion) {
      throw std::runtime_error(
        "unsupported staging format version " + std::to_string(version) + " (this build reads " +
        std::to_string(kStagingFormatVersion) + ")");
    }
    StagingSession & s = folder.session;
    s.name = doc["session"].as<std::string>();
    s.created_utc = doc["created_utc"].as<std::string>("");
    s.landfall_version = doc["landfall_version"].as<std::string>("");
    s.pose_source = doc["pose_source"].as<std::string>("");
    const YAML::Node frames = doc["frames"];
    s.frames.global = frames["global"].as<std::string>();
    s.frames.odom = frames["odom"].as<std::string>();
    s.frames.base = frames["base"].as<std::string>();
    s.frames.camera = frames["camera"].as<std::string>();
    const YAML::Node camera = doc["camera"];
    const auto model = parseDistortionModel(camera["distortion_model"].as<std::string>());
    if (!model) {
      throw std::runtime_error("unknown distortion model in session.yaml");
    }
    const auto k = camera["intrinsics"].as<std::vector<double>>();
    if (k.size() != 4) {
      throw std::runtime_error("camera intrinsics must be [fx, fy, cx, cy]");
    }
    s.camera = CameraModel(
      camera["width"].as<int>(), camera["height"].as<int>(), k[0], k[1], k[2], k[3], *model,
      camera["distortion"].as<std::vector<double>>());
    s.image_format = doc["image_format"].as<std::string>();
    s.jpeg_quality = doc["jpeg_quality"].as<int>(95);
    s.complete = doc["complete"].as<bool>(false);
    if (const YAML::Node c = doc["counts"]) {
      s.counts.frames_seen = c["frames_seen"].as<uint32_t>(0);
      s.counts.keyframes = c["keyframes"].as<uint32_t>(0);
      s.counts.no_pose = c["no_pose"].as<uint32_t>(0);
      s.counts.no_motion = c["no_motion"].as<uint32_t>(0);
      s.counts.pose_uncertain = c["pose_uncertain"].as<uint32_t>(0);
      s.counts.speed = c["speed"].as<uint32_t>(0);
      s.counts.tilt = c["tilt"].as<uint32_t>(0);
      s.counts.blur = c["blur"].as<uint32_t>(0);
      s.counts.exposure = c["exposure"].as<uint32_t>(0);
    }
  } catch (const YAML::Exception & e) {
    throw std::runtime_error("invalid " + session_path.string() + ": " + e.what());
  } catch (const std::invalid_argument & e) {
    throw std::runtime_error("invalid camera in " + session_path.string() + ": " + e.what());
  }

  const fs::path csv_path = fs::path(directory) / kKeyframesFile;
  std::ifstream csv(csv_path, std::ios::binary);
  if (!csv) {
    throw std::runtime_error("cannot read " + csv_path.string());
  }
  const std::string text((std::istreambuf_iterator<char>(csv)), std::istreambuf_iterator<char>());
  std::istringstream lines(text);
  std::string line;
  if (!std::getline(lines, line) || line != kCsvHeader) {
    throw std::runtime_error(csv_path.string() + ": missing or unexpected header");
  }
  // Only lines terminated by a newline are complete; a crash can leave a partial last line.
  const std::size_t complete_end = text.rfind('\n') + 1;
  std::size_t offset = static_cast<std::size_t>(lines.tellg());
  int line_number = 1;
  while (offset < complete_end && std::getline(lines, line)) {
    ++line_number;
    offset += line.size() + 1;
    if (line.empty()) {
      continue;
    }
    const std::string where = csv_path.string() + ":" + std::to_string(line_number);
    const auto fields = splitCsv(line);
    if (static_cast<int>(fields.size()) != kCsvColumns) {
      throw std::runtime_error(where + ": expected " + std::to_string(kCsvColumns) + " columns");
    }
    folder.keyframes.push_back(parseRow(fields, where));
  }
  return folder;
}

cv::Mat loadStagingImage(const StagingFolder & folder, const StagingKeyframe & keyframe, int flags)
{
  const std::string path = (fs::path(folder.directory) / keyframe.image).string();
  cv::Mat image = cv::imread(path, flags);
  if (image.empty()) {
    throw std::runtime_error("cannot read keyframe image " + path);
  }
  return image;
}

}  // namespace landfall
