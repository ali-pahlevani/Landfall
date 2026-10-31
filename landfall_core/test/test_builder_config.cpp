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

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

#include "landfall/build/builder_config.hpp"

using landfall::BuildConfigFile;

namespace
{

std::string errorOf(const std::string & yaml)
{
  try {
    landfall::parseBuildConfig(yaml);
  } catch (const std::runtime_error & e) {
    return e.what();
  }
  return "";
}

}  // namespace

TEST(BuilderConfig, DefaultsMatchThePlan)
{
  const BuildConfigFile c = landfall::parseBuildConfig("");
  EXPECT_EQ(c.builder.local_backend, "orb");
  EXPECT_EQ(c.builder.global_backend, "bow");
  EXPECT_EQ(c.builder.matcher, "auto");
  EXPECT_EQ(c.builder.vocabulary.branching, 10);
  EXPECT_EQ(c.builder.vocabulary.depth, 5);
  EXPECT_EQ(c.builder.pairs.max_distance, 6.0);
  EXPECT_EQ(c.builder.pairs.max_angle_deg, 60.0);
  EXPECT_EQ(c.builder.pairs.retrieval_k, 10);
  EXPECT_EQ(c.builder.pairs.max_per_keyframe, 25);
  EXPECT_EQ(c.builder.pair_min_inliers, 20);
  EXPECT_EQ(c.builder.pair_max_rotation_deviation_deg, 5.0);
  EXPECT_EQ(c.builder.triangulation.max_reprojection_px, 0.0);  // auto
  EXPECT_EQ(c.builder.triangulation.min_angle_deg, 2.0);
  EXPECT_EQ(c.builder.triangulation.min_track_length, 3);
  EXPECT_EQ(c.builder.triangulation.min_height, -0.05);
  EXPECT_TRUE(c.builder.ba.enabled);
  EXPECT_EQ(c.builder.ba.max_shift_xy, 0.10);
  EXPECT_EQ(c.builder.ba.max_shift_yaw_deg, 2.0);
  EXPECT_EQ(c.builder.ba.loss_scale_px, 2.0);
  EXPECT_EQ(c.builder.ba.prior_floor_xy, 0.02);
  EXPECT_TRUE(c.builder.prune_unobserved_features);
  EXPECT_EQ(c.gating.max_std_xy, 0.05);
  EXPECT_EQ(c.gating.min_translation, 0.5);
  EXPECT_EQ(c.gating.min_rotation_deg, 15.0);
  EXPECT_TRUE(c.gating.require_covariance);
  EXPECT_EQ(c.bag.pose_topic, "/amcl_pose");
  EXPECT_EQ(c.bag.frames.base, "base_footprint");
}

TEST(BuilderConfig, ParsesEverySection)
{
  const BuildConfigFile c = landfall::parseBuildConfig(R"(
backend: {local: sift, matcher: mnn}
features: {max_keypoints: 1500, grid: [4, 3]}
matching: {min_cosine: 0.9}
vocab: {branching: 8, depth: 4}
pairs: {max_distance: 4.5, min_inliers: 30, min_baseline: 0.1}
triangulation: {max_reprojection_px: 2.5, depth_range: [0.5, 20.0], min_height: -0.1}
ba: {enabled: false, max_shift_xy: 0.05, loss: cauchy, max_reprojection_px: 1.5,
     prior_std_xy: 0.02, prior_std_yaw_deg: 0.5}
db: {prune_unobserved_features: false, global_dtype: f32}
threads: 3
seed: 7
bag:
  image_topic: /oakd/rgb/image_raw/compressed
  pose_source: tf_only
  frames: {base: base_link, camera: oakd_rgb_camera_optical_frame}
gating: {max_std_xy: 0.08, min_translation: 0.3}
quality: {min_sharpness: 40}
max_keyframes: 500
)");
  EXPECT_EQ(c.builder.local_backend, "sift");
  EXPECT_EQ(c.builder.local_options.at("max_keypoints"), "1500");
  EXPECT_EQ(c.builder.local_options.at("grid"), "4x3");
  EXPECT_EQ(c.builder.matcher_options.at("min_cosine"), "0.9");
  EXPECT_EQ(c.builder.vocabulary.depth, 4);
  EXPECT_EQ(c.builder.pairs.max_distance, 4.5);
  EXPECT_EQ(c.builder.pair_min_inliers, 30);
  EXPECT_EQ(c.builder.pair_min_baseline, 0.1);
  EXPECT_EQ(c.builder.triangulation.max_reprojection_px, 2.5);
  EXPECT_EQ(c.builder.triangulation.min_depth, 0.5);
  EXPECT_EQ(c.builder.triangulation.max_depth, 20.0);
  EXPECT_FALSE(c.builder.ba.enabled);
  EXPECT_EQ(c.builder.ba_max_reprojection_px, 1.5);
  EXPECT_EQ(c.builder.ba.prior_std_xy, 0.02);
  EXPECT_EQ(c.builder.ba.prior_std_yaw_deg, 0.5);
  EXPECT_FALSE(c.builder.prune_unobserved_features);
  EXPECT_EQ(c.builder.global_dtype, landfall::db::BlobType::kF32);
  EXPECT_EQ(c.builder.threads, 3);
  EXPECT_EQ(c.builder.seed, 7U);
  EXPECT_EQ(c.bag.image_topic, "/oakd/rgb/image_raw/compressed");
  EXPECT_EQ(c.bag.frames.base, "base_link");
  EXPECT_EQ(c.bag.frames.global, "map");
  EXPECT_FALSE(c.gating.require_covariance);
  EXPECT_EQ(c.gating.max_std_xy, 0.08);
  EXPECT_EQ(c.gating.min_sharpness, 40.0);
  EXPECT_EQ(c.gating.max_keyframes, 500);

  // The canonical form parses back to the same configuration.
  const std::string canonical = landfall::canonicalYaml(c);
  EXPECT_EQ(landfall::canonicalYaml(landfall::parseBuildConfig(canonical)), canonical);
  EXPECT_NE(canonical.find("  max_keypoints: \"1500\"\n"), std::string::npos);
}

TEST(BuilderConfig, RejectsTyposAndInvalidValues)
{
  EXPECT_NE(errorOf("pairs: {max_distanse: 3}").find("'pairs.max_distanse'"), std::string::npos);
  EXPECT_NE(errorOf("tresholds: 1").find("'tresholds'"), std::string::npos);
  EXPECT_NE(errorOf("ba: {max_shift_xy: far}").find("'ba.max_shift_xy'"), std::string::npos);
  EXPECT_NE(errorOf("ba: {loss: huber}").find("cauchy"), std::string::npos);
  EXPECT_NE(errorOf("ba: {prior_std_xy: 0.02}").find("together"), std::string::npos);
  EXPECT_NE(errorOf("backend: {global: megaloc}").find("bow"), std::string::npos);
  EXPECT_NE(errorOf("bag: {pose_source: vio}").find("pose_source"), std::string::npos);
  EXPECT_NE(errorOf("triangulation: {depth_range: [1]}").find("depth_range"), std::string::npos);
  EXPECT_NE(errorOf("pairs: 3").find("mapping"), std::string::npos);
  EXPECT_NE(errorOf("pairs: {min_inliers: 2}").find("min_inliers"), std::string::npos);
  EXPECT_NE(errorOf("{unbalanced").find("YAML"), std::string::npos);
}
