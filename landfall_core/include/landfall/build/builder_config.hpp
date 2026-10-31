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

#ifndef LANDFALL_BUILD_BUILDER_CONFIG_HPP_
#define LANDFALL_BUILD_BUILDER_CONFIG_HPP_

#include <cstdint>
#include <string>

#include "landfall/build/pair_selection.hpp"
#include "landfall/build/pose_prior_ba.hpp"
#include "landfall/build/triangulation.hpp"
#include "landfall/db/database.hpp"
#include "landfall/features/vocab_tree.hpp"
#include "landfall/record/keyframe_selector.hpp"
#include "landfall/record/staging.hpp"
#include "landfall/registry.hpp"

namespace landfall
{

namespace detail
{
/// Triangulation defaults with the reprojection threshold left to the image width ("auto").
inline TriangulationParams autoTriangulationParams()
{
  TriangulationParams p;
  p.max_reprojection_px = 0.0;
  return p;
}
}  // namespace detail

/// What `landfall build` does with its keyframes (YAML keys in parentheses).
struct BuilderConfig
{
  std::string local_backend = "orb";   ///< (backend.local)
  BackendOptions local_options;        ///< (features.*) options of the local backend
  std::string matcher = "auto";        ///< (backend.matcher) auto = ratio or mnn by descriptor type
  BackendOptions matcher_options;      ///< (matching.*)
  std::string global_backend = "bow";  ///< (backend.global)
  VocabularyParams vocabulary;         ///< (vocab.*)

  PairSelectionParams pairs;                     ///< (pairs.*)
  int pair_min_inliers = 20;                     ///< (pairs.min_inliers)
  double pair_max_epipolar_error_px = 4.0;       ///< (pairs.max_epipolar_error_px)
  double pair_max_rotation_deviation_deg = 5.0;  ///< (pairs.max_rotation_deviation_deg) vs prior
  double pair_min_baseline = 0.05;  ///< m (pairs.min_baseline); below it: rotation-only model

  /// (triangulation.*); max_reprojection_px 0 = auto: 0.6 % of the image width.
  TriangulationParams triangulation = detail::autoTriangulationParams();
  PosePriorBaParams ba;                 ///< (ba.*)
  double ba_max_reprojection_px = 0.0;  ///< (ba.max_reprojection_px) re-filter; 0 = 0.75 x above

  bool prune_unobserved_features = true;           ///< (db.prune_unobserved_features)
  int covisibility_min_shared = 5;                 ///< (db.covisibility_min_shared)
  db::BlobType global_dtype = db::BlobType::kF16;  ///< (db.global_dtype) dense descriptors only

  int threads = 0;     ///< (threads) 0 = all cores
  uint64_t seed = 42;  ///< (seed) every random choice of the build derives from it
};

/// Where `landfall build --bag` finds its data (YAML section `bag`).
struct BagConfig
{
  std::string image_topic = "/camera/image_raw";  ///< Image or CompressedImage (detected)
  std::string camera_info_topic = "/camera/camera_info";
  std::string pose_topic = "/amcl_pose";           ///< PoseWithCovarianceStamped; "" with tf_only
  std::string imu_topic;                           ///< optional: ramp gate
  std::string imu_frame;                           ///< IMU frame in TF; "" = the messages' frame_id
  std::string gravity_source = "auto";             ///< auto, imu, flat_floor (as the recorder)
  double imu_max_age = 0.1;                        ///< s
  std::string pose_source = "tf_with_covariance";  ///< or tf_only
  double image_time_offset = 0.0;                  ///< s added to image stamps
  StagingFrames frames;
  double speed_window = 0.2;  ///< s of odometry used for the speed gate
};

/// A complete `landfall build --config` file.
struct BuildConfigFile
{
  BuilderConfig builder;
  BagConfig bag;
  KeyframeGating gating;  ///< (gating.*, quality.*, max_keyframes) bags only
};

/// Parses a build configuration (missing keys keep their defaults). Throws std::runtime_error
/// naming the offending key for unknown keys or invalid values.
BuildConfigFile parseBuildConfig(const std::string & yaml_text);

/// Every effective setting as YAML with sorted keys (stored in the database as `builder_config`).
std::string canonicalYaml(const BuildConfigFile & config);

}  // namespace landfall

#endif  // LANDFALL_BUILD_BUILDER_CONFIG_HPP_
