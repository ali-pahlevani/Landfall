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

#ifndef LANDFALL_BUILD_MAP_BUILDER_HPP_
#define LANDFALL_BUILD_MAP_BUILDER_HPP_

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "landfall/build/builder_config.hpp"
#include "landfall/camera_model.hpp"
#include "landfall/grid/map_loader.hpp"

namespace landfall
{

struct BuildSession
{
  std::string name;
  std::string source;       ///< bag or staging folder
  std::string pose_source;  ///< tf_with_covariance, tf_only
  int64_t started_ns = 0;
  std::string notes;
};

struct BuildCamera
{
  std::string frame_id;
  CameraModel model;
  Pose3 T_base_cam = Pose3::Identity();  ///< representative extrinsics (first keyframe)
};

struct BuildKeyframe
{
  int session = 0;  ///< index into BuildInput::sessions
  int camera = 0;   ///< index into BuildInput::cameras
  int64_t stamp_ns = 0;
  Pose2 prior;
  std::optional<Mat3> covariance;
  Pose3 T_base_cam = Pose3::Identity();  ///< at this keyframe's stamp
  std::string quality_json = "{}";
  /// Loads the image (8-bit gray, BGR or BGRA). Called once, from a worker thread.
  std::function<cv::Mat()> load_image;
  /// Precomputed features instead of an image (tests); no thumbnail then.
  std::optional<Features> features;
};

struct BuildInput
{
  LoadedMap map;
  std::vector<BuildSession> sessions;
  std::vector<BuildCamera> cameras;  ///< v1.0 builds accept exactly one camera
  std::vector<BuildKeyframe> keyframes;
};

/// Statistics of one build, stored in the database (meta `build_report`) and in report.json.
struct BuildStats
{
  int keyframes = 0;
  int sessions = 0;
  int pairs_candidate = 0;
  int pairs_verified = 0;
  double mean_pair_inliers = 0.0;
  int tracks = 0;
  int track_conflicts = 0;
  int triangulated = 0;
  int rejected_small_angle = 0;
  int rejected_depth = 0;
  int rejected_below_floor = 0;
  int rejected_reprojection = 0;  ///< too few consistent observations
  int landmarks = 0;
  int observations = 0;
  double mean_track_length = 0.0;
  double reprojection_rms_px = 0.0;  ///< final, over every stored observation
  double triangulation_threshold_px = 0.0;
  bool ba_run = false;
  bool ba_usable = false;
  int ba_iterations = 0;
  double ba_initial_cost = 0.0;
  double ba_final_cost = 0.0;
  double ba_mean_shift_m = 0.0;  ///< mean |dx, dy| of the keyframe poses
  double ba_max_shift_m = 0.0;
  double ba_max_shift_yaw_deg = 0.0;
  Vec2 ba_mean_shift_vector = Vec2::Zero();  ///< a large one means a biased recording
  int ba_keyframes_at_bound = 0;
  std::vector<double> stage_seconds;  ///< per BuildStage below
  std::vector<std::string> warnings;
};

/// Stages, in order, for progress reports and timings.
enum class BuildStage : uint8_t {
  kFeatures,
  kVocabulary,
  kPairs,
  kMatching,
  kTracks,
  kTriangulation,
  kBundleAdjustment,
  kWriting,
};
constexpr int kBuildStageCount = 8;
const char * toString(BuildStage stage);

/// (stage, done, total); called from the building thread only.
using BuildProgress = std::function<void(BuildStage, std::size_t, std::size_t)>;

/// Builds a map database from keyframes with known poses: features, vocabulary and bag-of-words,
/// pair selection, matching with two-view verification, tracks, triangulation with the poses held
/// fixed, the optional bounded pose-prior BA, re-triangulation, and the database (written in one
/// transaction; its content hash depends only on the input and the configuration, not on the
/// number of threads). `db_path` must not exist. Throws std::runtime_error on invalid input.
BuildStats buildMap(
  const BuildInput & input, const BuildConfigFile & config, const std::string & db_path,
  const BuildProgress & progress = {});

/// JSON of the statistics (meta `build_report`).
std::string toJson(const BuildStats & stats);

}  // namespace landfall

#endif  // LANDFALL_BUILD_MAP_BUILDER_HPP_
