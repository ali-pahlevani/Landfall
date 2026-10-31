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

#include "landfall/build/builder_config.hpp"

#include <yaml-cpp/yaml.h>

#include <cstdio>
#include <map>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

#include "landfall/geometry/gravity.hpp"

namespace landfall
{

namespace
{

// A YAML mapping whose keys must all be consumed: leftovers are typos and fail the parse.
class Section
{
public:
  Section(const YAML::Node & node, std::string path) : node_(node), path_(std::move(path))
  {
    if (node_ && !node_.IsMap()) {
      throw std::runtime_error(where() + " must be a mapping");
    }
  }

  template <typename T>
  void read(const std::string & key, T * value)
  {
    used_.insert(key);
    const YAML::Node child = get(key);
    if (!child) {
      return;
    }
    try {
      *value = child.as<T>();
    } catch (const YAML::Exception &) {
      throw std::runtime_error("invalid value for " + name(key));
    }
  }

  Section child(const std::string & key)
  {
    used_.insert(key);
    return {get(key), path_.empty() ? key : path_ + "." + key};
  }

  /// All entries as backend options (text); lists of two integers become "AxB" (grid sizes).
  BackendOptions options(const std::string & key)
  {
    used_.insert(key);
    BackendOptions options;
    const YAML::Node child = get(key);
    if (!child) {
      return options;
    }
    if (!child.IsMap()) {
      throw std::runtime_error(name(key) + " must be a mapping");
    }
    for (const auto & entry : child) {
      const auto option = entry.first.as<std::string>();
      const YAML::Node & value = entry.second;
      if (value.IsSequence() && value.size() == 2) {
        options[option] = value[0].as<std::string>() + "x" + value[1].as<std::string>();
      } else if (value.IsScalar()) {
        options[option] = value.as<std::string>();
      } else {
        throw std::runtime_error("invalid value for " + name(key) + "." + option);
      }
    }
    return options;
  }

  void finish() const
  {
    if (!node_) {
      return;
    }
    for (const auto & entry : node_) {
      const auto key = entry.first.as<std::string>();
      if (used_.count(key) == 0) {
        throw std::runtime_error("unknown key " + name(key));
      }
    }
  }

private:
  // Const lookup (yaml-cpp's non-const operator[] may insert the key). A missing key gives an
  // undefined node; a default-constructed Node would be a defined null.
  YAML::Node get(const std::string & key) const
  {
    return node_ ? node_[key] : YAML::Node(YAML::NodeType::Undefined);
  }

  std::string name(const std::string & key) const
  {
    return "'" + (path_.empty() ? key : path_ + "." + key) + "'";
  }
  std::string where() const { return path_.empty() ? "the configuration" : "'" + path_ + "'"; }

  YAML::Node node_;
  std::string path_;
  std::set<std::string> used_;
};

void require(bool ok, const std::string & message)
{
  if (!ok) {
    throw std::runtime_error(message);
  }
}

std::string number(double value)
{
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.17g", value);
  return buffer;
}

std::string text(const std::string & value)
{
  YAML::Emitter out;
  out << YAML::DoubleQuoted << value;
  return out.c_str();
}

std::string flag(bool value) { return value ? "true" : "false"; }

}  // namespace

BuildConfigFile parseBuildConfig(const std::string & yaml_text)
{
  BuildConfigFile config;
  BuilderConfig & b = config.builder;
  KeyframeGating & g = config.gating;
  BagConfig & bag = config.bag;
  YAML::Node root;
  try {
    root = YAML::Load(yaml_text);
  } catch (const YAML::Exception & e) {
    throw std::runtime_error(std::string("invalid YAML: ") + e.what());
  }
  if (root.IsNull()) {
    root = YAML::Node(YAML::NodeType::Map);
  }
  Section top(root, "");

  Section backend = top.child("backend");
  backend.read("local", &b.local_backend);
  backend.read("global", &b.global_backend);
  backend.read("matcher", &b.matcher);
  backend.finish();
  b.local_options = top.options("features");
  b.matcher_options = top.options("matching");

  Section vocab = top.child("vocab");
  vocab.read("branching", &b.vocabulary.branching);
  vocab.read("depth", &b.vocabulary.depth);
  vocab.read("iterations", &b.vocabulary.iterations);
  vocab.read("max_training_descriptors", &b.vocabulary.max_training_descriptors);
  vocab.finish();

  Section pairs = top.child("pairs");
  pairs.read("max_distance", &b.pairs.max_distance);
  pairs.read("max_angle_deg", &b.pairs.max_angle_deg);
  pairs.read("retrieval_k", &b.pairs.retrieval_k);
  pairs.read("max_per_keyframe", &b.pairs.max_per_keyframe);
  pairs.read("min_inliers", &b.pair_min_inliers);
  pairs.read("max_epipolar_error_px", &b.pair_max_epipolar_error_px);
  pairs.read("max_rotation_deviation_deg", &b.pair_max_rotation_deviation_deg);
  pairs.read("min_baseline", &b.pair_min_baseline);
  pairs.finish();

  Section tri = top.child("triangulation");
  tri.read("max_reprojection_px", &b.triangulation.max_reprojection_px);
  tri.read("min_angle_deg", &b.triangulation.min_angle_deg);
  tri.read("min_track_length", &b.triangulation.min_track_length);
  std::vector<double> depth_range{b.triangulation.min_depth, b.triangulation.max_depth};
  tri.read("depth_range", &depth_range);
  require(depth_range.size() == 2, "'triangulation.depth_range' must be [min, max]");
  b.triangulation.min_depth = depth_range[0];
  b.triangulation.max_depth = depth_range[1];
  tri.read("min_height", &b.triangulation.min_height);
  tri.finish();

  Section ba = top.child("ba");
  ba.read("enabled", &b.ba.enabled);
  ba.read("max_shift_xy", &b.ba.max_shift_xy);
  ba.read("max_shift_yaw_deg", &b.ba.max_shift_yaw_deg);
  std::string loss = "cauchy";
  ba.read("loss", &loss);
  require(loss == "cauchy", "'ba.loss' must be cauchy");
  ba.read("loss_scale_px", &b.ba.loss_scale_px);
  ba.read("prior_floor_xy", &b.ba.prior_floor_xy);
  ba.read("prior_floor_yaw_deg", &b.ba.prior_floor_yaw_deg);
  ba.read("prior_std_xy", &b.ba.prior_std_xy);
  ba.read("prior_std_yaw_deg", &b.ba.prior_std_yaw_deg);
  ba.read("max_iterations", &b.ba.max_iterations);
  ba.read("max_reprojection_px", &b.ba_max_reprojection_px);
  ba.finish();

  Section db = top.child("db");
  db.read("prune_unobserved_features", &b.prune_unobserved_features);
  db.read("covisibility_min_shared", &b.covisibility_min_shared);
  std::string dtype = "f16";
  db.read("global_dtype", &dtype);
  require(dtype == "f16" || dtype == "f32", "'db.global_dtype' must be f16 or f32");
  b.global_dtype = dtype == "f16" ? db::BlobType::kF16 : db::BlobType::kF32;
  db.finish();

  top.read("threads", &b.threads);
  top.read("seed", &b.seed);

  Section bag_section = top.child("bag");
  bag_section.read("image_topic", &bag.image_topic);
  bag_section.read("camera_info_topic", &bag.camera_info_topic);
  bag_section.read("pose_topic", &bag.pose_topic);
  bag_section.read("imu_topic", &bag.imu_topic);
  bag_section.read("imu_frame", &bag.imu_frame);
  bag_section.read("gravity_source", &bag.gravity_source);
  bag_section.read("imu_max_age", &bag.imu_max_age);
  bag_section.read("pose_source", &bag.pose_source);
  bag_section.read("image_time_offset", &bag.image_time_offset);
  bag_section.read("speed_window", &bag.speed_window);
  Section frames = bag_section.child("frames");
  frames.read("global", &bag.frames.global);
  frames.read("odom", &bag.frames.odom);
  frames.read("base", &bag.frames.base);
  frames.read("camera", &bag.frames.camera);
  frames.finish();
  bag_section.finish();

  Section gating = top.child("gating");
  gating.read("max_std_xy", &g.max_std_xy);
  gating.read("max_std_yaw_deg", &g.max_std_yaw_deg);
  gating.read("max_covariance_age", &g.max_covariance_age);
  gating.read("min_translation", &g.min_translation);
  gating.read("min_rotation_deg", &g.min_rotation_deg);
  gating.read("max_linear_speed", &g.max_linear_speed);
  gating.read("max_angular_speed", &g.max_angular_speed);
  gating.read("max_tilt_deg", &g.max_tilt_deg);
  gating.finish();
  Section quality = top.child("quality");
  quality.read("min_sharpness", &g.min_sharpness);
  quality.read("min_mean_intensity", &g.min_mean_intensity);
  quality.read("max_mean_intensity", &g.max_mean_intensity);
  quality.read("max_saturated_fraction", &g.max_saturated_fraction);
  quality.finish();
  top.read("max_keyframes", &g.max_keyframes);
  top.finish();

  require(
    bag.pose_source == "tf_with_covariance" || bag.pose_source == "tf_only",
    "'bag.pose_source' must be tf_with_covariance or tf_only");
  require(
    parseGravitySource(bag.gravity_source).has_value(),
    "'bag.gravity_source' must be auto, imu or flat_floor");
  require(bag.imu_max_age > 0.0, "'bag.imu_max_age' must be positive");
  g.require_covariance = bag.pose_source == "tf_with_covariance";
  require(b.global_backend == "bow", "'backend.global': only bow is available in this build");
  require(b.threads >= 0, "'threads' must be >= 0");
  require(b.vocabulary.branching >= 2 && b.vocabulary.depth >= 1, "invalid 'vocab' size");
  require(b.pairs.max_per_keyframe >= 1, "'pairs.max_per_keyframe' must be >= 1");
  require(b.pair_min_inliers >= 5, "'pairs.min_inliers' must be >= 5");
  require(b.triangulation.min_track_length >= 2, "'triangulation.min_track_length' must be >= 2");
  require(
    b.triangulation.min_depth > 0.0 && b.triangulation.max_depth > b.triangulation.min_depth,
    "invalid 'triangulation.depth_range'");
  require(b.ba.max_shift_xy > 0.0 && b.ba.max_shift_yaw_deg > 0.0, "BA bounds must be positive");
  require(
    (b.ba.prior_std_xy > 0.0) == (b.ba.prior_std_yaw_deg > 0.0) && b.ba.prior_std_xy >= 0.0 &&
      b.ba.prior_std_yaw_deg >= 0.0,
    "'ba.prior_std_xy' and 'ba.prior_std_yaw_deg' are set together (0 = recorded covariance)");
  require(bag.speed_window > 0.0, "'bag.speed_window' must be positive");
  return config;
}

std::string canonicalYaml(const BuildConfigFile & config)
{
  const BuilderConfig & b = config.builder;
  const KeyframeGating & g = config.gating;
  const BagConfig & bag = config.bag;
  std::map<std::string, std::map<std::string, std::string>> sections;
  sections["backend"] = {
    {"global", text(b.global_backend)},
    {"local", text(b.local_backend)},
    {"matcher", text(b.matcher)}};
  for (const auto & [key, value] : b.local_options) {
    sections["features"][key] = text(value);
  }
  for (const auto & [key, value] : b.matcher_options) {
    sections["matching"][key] = text(value);
  }
  sections["vocab"] = {
    {"branching", std::to_string(b.vocabulary.branching)},
    {"depth", std::to_string(b.vocabulary.depth)},
    {"iterations", std::to_string(b.vocabulary.iterations)},
    {"max_training_descriptors", std::to_string(b.vocabulary.max_training_descriptors)}};
  sections["pairs"] = {
    {"max_angle_deg", number(b.pairs.max_angle_deg)},
    {"max_distance", number(b.pairs.max_distance)},
    {"max_epipolar_error_px", number(b.pair_max_epipolar_error_px)},
    {"max_per_keyframe", std::to_string(b.pairs.max_per_keyframe)},
    {"max_rotation_deviation_deg", number(b.pair_max_rotation_deviation_deg)},
    {"min_baseline", number(b.pair_min_baseline)},
    {"min_inliers", std::to_string(b.pair_min_inliers)},
    {"retrieval_k", std::to_string(b.pairs.retrieval_k)}};
  sections["triangulation"] = {
    {"depth_range",
     "[" + number(b.triangulation.min_depth) + ", " + number(b.triangulation.max_depth) + "]"},
    {"max_reprojection_px", number(b.triangulation.max_reprojection_px)},
    {"min_angle_deg", number(b.triangulation.min_angle_deg)},
    {"min_height", number(b.triangulation.min_height)},
    {"min_track_length", std::to_string(b.triangulation.min_track_length)}};
  sections["ba"] = {
    {"enabled", flag(b.ba.enabled)},
    {"loss", text("cauchy")},
    {"loss_scale_px", number(b.ba.loss_scale_px)},
    {"max_iterations", std::to_string(b.ba.max_iterations)},
    {"max_reprojection_px", number(b.ba_max_reprojection_px)},
    {"max_shift_xy", number(b.ba.max_shift_xy)},
    {"max_shift_yaw_deg", number(b.ba.max_shift_yaw_deg)},
    {"prior_floor_xy", number(b.ba.prior_floor_xy)},
    {"prior_floor_yaw_deg", number(b.ba.prior_floor_yaw_deg)},
    {"prior_std_xy", number(b.ba.prior_std_xy)},
    {"prior_std_yaw_deg", number(b.ba.prior_std_yaw_deg)}};
  sections["db"] = {
    {"covisibility_min_shared", std::to_string(b.covisibility_min_shared)},
    {"global_dtype", text(b.global_dtype == db::BlobType::kF16 ? "f16" : "f32")},
    {"prune_unobserved_features", flag(b.prune_unobserved_features)}};
  sections["bag"] = {
    {"camera_info_topic", text(bag.camera_info_topic)},
    {"frames", "{global: " + text(bag.frames.global) + ", odom: " + text(bag.frames.odom) +
                 ", base: " + text(bag.frames.base) + ", camera: " + text(bag.frames.camera) + "}"},
    {"image_time_offset", number(bag.image_time_offset)},
    {"gravity_source", text(bag.gravity_source)},
    {"image_topic", text(bag.image_topic)},
    {"imu_frame", text(bag.imu_frame)},
    {"imu_max_age", number(bag.imu_max_age)},
    {"imu_topic", text(bag.imu_topic)},
    {"pose_source", text(bag.pose_source)},
    {"pose_topic", text(bag.pose_topic)},
    {"speed_window", number(bag.speed_window)}};
  sections["gating"] = {
    {"max_angular_speed", number(g.max_angular_speed)},
    {"max_covariance_age", number(g.max_covariance_age)},
    {"max_linear_speed", number(g.max_linear_speed)},
    {"max_std_xy", number(g.max_std_xy)},
    {"max_std_yaw_deg", number(g.max_std_yaw_deg)},
    {"max_tilt_deg", number(g.max_tilt_deg)},
    {"min_rotation_deg", number(g.min_rotation_deg)},
    {"min_translation", number(g.min_translation)}};
  sections["quality"] = {
    {"max_mean_intensity", number(g.max_mean_intensity)},
    {"max_saturated_fraction", number(g.max_saturated_fraction)},
    {"min_mean_intensity", number(g.min_mean_intensity)},
    {"min_sharpness", number(g.min_sharpness)}};

  std::map<std::string, std::string> scalars = {
    {"max_keyframes", std::to_string(g.max_keyframes)},
    {"seed", std::to_string(b.seed)},
    {"threads", std::to_string(b.threads)}};
  std::string out;
  std::set<std::string> keys;
  for (const auto & entry : sections) {
    keys.insert(entry.first);
  }
  for (const auto & entry : scalars) {
    keys.insert(entry.first);
  }
  for (const auto & key : keys) {
    if (const auto it = scalars.find(key); it != scalars.end()) {
      out.append(key).append(": ").append(it->second).append("\n");
      continue;
    }
    const auto & entries = sections.at(key);
    if (entries.empty()) {
      out.append(key).append(": {}\n");
      continue;
    }
    out.append(key).append(":\n");
    for (const auto & [name, value] : entries) {
      out.append("  ").append(name).append(": ").append(value).append("\n");
    }
  }
  return out;
}

}  // namespace landfall
