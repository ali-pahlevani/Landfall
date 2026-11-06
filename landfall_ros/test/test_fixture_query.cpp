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

// The depot fixture's images that are not database keyframes, relocalized one frame at a time (as
// `landfall query`) against the fixture's database and scored against Gazebo's ground truth; and
// the LiDAR scan check at the fixture's true and wrong poses.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <set>
#include <string>
#include <vector>

#include "fixture_data.hpp"
#include "landfall/db/database.hpp"
#include "landfall/ros/bag_query.hpp"
#include "landfall/ros/scan_evaluation.hpp"

TEST(FixtureQuery, NonKeyframeImagesLocalizeAccuratelyAndNeverWrongly)
{
  const landfall::test::TempDir dir;
  const std::string db = dir.file("depot.landfall.db");
  landfall::test::buildFixtureDatabase(db);
  const auto messages = landfall::test::readFixtureMessages();

  landfall::ros::BagQueryOptions options;
  options.bag = landfall::parseBuildConfig(
                  landfall::test::readTextFile(landfall::test::kFixtureDir + "/build.yaml"))
                  .bag;
  for (const auto & k :
       landfall::db::Database::open(db, landfall::db::Database::Access::kReadOnly).keyframes()) {
    options.skip_stamps.insert(k.stamp_ns);
  }
  landfall::Localizer localizer(landfall::LocalizationMap::load(db), {});
  int accurate = 0;
  int wrong = 0;
  int ambiguous = 0;
  int frames = 0;
  std::vector<double> seconds;
  double worst_accurate = 0.0;
  const auto counts = landfall::ros::queryBagImages(
    landfall::test::kFixtureDir + "/bag", localizer, options,
    [&](int64_t stamp, const landfall::Pose2 &, landfall::FrameResult && result) {
      ++frames;
      seconds.push_back(result.timings.total_s);
      if (result.ambiguity.ambiguous) {
        ++ambiguous;
        return;
      }
      if (!result.pose) {
        return;
      }
      const auto truth = messages.truth.at(stamp, 250'000'000);
      ASSERT_TRUE(truth.has_value());
      const landfall::Pose2 expected = landfall::compose(landfall::test::kWorldToMap, *truth);
      const double position = landfall::translationDistance(*result.pose, expected);
      const double yaw = landfall::rotationDistance(*result.pose, expected) * 180.0 / M_PI;
      if (position <= 0.25 && yaw <= 5.0) {
        ++accurate;
        worst_accurate = std::max(worst_accurate, position);
      }
      wrong += position > 0.5 || yaw > 10.0 ? 1 : 0;
    });
  std::sort(seconds.begin(), seconds.end());
  RecordProperty("frames", frames);
  RecordProperty("accurate", accurate);
  RecordProperty("worst_accurate_cm", std::to_string(100.0 * worst_accurate));
  RecordProperty("median_frame_ms", std::to_string(1000.0 * seconds[seconds.size() / 2]));
  RecordProperty("max_frame_ms", std::to_string(1000.0 * seconds.back()));
  EXPECT_EQ(counts.skipped_stamps, options.skip_stamps.size());
  EXPECT_EQ(counts.skipped_scan, 0U);  // every image has a scan within 0.5 s
  EXPECT_GE(frames, 30);
  // The map holds only this L-shaped stretch, so nothing looks alike: no ambiguity, no wrong pose.
  EXPECT_EQ(ambiguous, 0);
  EXPECT_EQ(wrong, 0);
  // Measured 25 of 35 on Jazzy; the misses are on the L's short leg, where the database has three
  // keyframes per direction and too few landmarks (a miss, never a wrong pose).
  EXPECT_GE(accurate, 20);
  EXPECT_LT(worst_accurate, 0.10);
}

TEST(FixtureQuery, ScanCheckAcceptsTruePosesAndRejectsWrongOnes)
{
  // Plan §11, M4: every scan of the fixture at its true pose, and at 20 wrong poses 0.5 to 5 m away
  // (turned by up to 15 degrees, on known-free cells) where it must fail.
  landfall::ros::ScanEvaluationOptions options;
  options.world_to_map = landfall::test::kWorldToMap;
  options.wrong_per_scan = 20;
  const auto map = landfall::loadMap(
    landfall::test::kFixtureDir + "/depot.yaml", landfall::test::distroSemantics());
  const auto e =
    landfall::ros::evaluateScanCheck(landfall::test::kFixtureDir + "/bag", map.grid, options);
  RecordProperty("true_checks", static_cast<int>(e.true_checks));
  RecordProperty("true_accepted", static_cast<int>(e.true_accepted));
  RecordProperty("wrong_checks", static_cast<int>(e.wrong_checks));
  RecordProperty("wrong_accepted", static_cast<int>(e.wrong_accepted));
  for (const auto & [reason, count] : e.true_rejections) {
    std::cout << "true pose rejected " << count << " times: " << reason << "\n";
  }
  std::cout << "true poses accepted " << e.true_accepted << " of " << e.true_checks
            << ", wrong poses accepted " << e.wrong_accepted << " of " << e.wrong_checks << " ("
            << e.skipped << " scans without ground truth or TF)\n";
  EXPECT_LE(e.skipped, 1U);  // the first scan, at the very start of the recording
  EXPECT_GE(e.true_checks, 140U);
  // Measured 143 of 143 on Jazzy.
  EXPECT_GE(static_cast<double>(e.true_accepted), 0.95 * static_cast<double>(e.true_checks));
  EXPECT_GE(e.wrong_checks, 2000U);
  EXPECT_EQ(e.wrong_accepted, 0U);
  for (const auto & w : e.wrong_examples) {
    ADD_FAILURE() << "wrong pose accepted " << w.offset << " m / " << w.yaw_offset_deg
                  << " deg off: hit ratio " << w.hit_ratio;
  }
}
