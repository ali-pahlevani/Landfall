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

#ifndef LANDFALL_RECORD_STAGING_HPP_
#define LANDFALL_RECORD_STAGING_HPP_

#include <cstdint>
#include <fstream>
#include <optional>
#include <string>
#include <vector>

#include "landfall/camera_model.hpp"
#include "landfall/record/image_quality.hpp"
#include "landfall/record/keyframe_selector.hpp"

namespace landfall
{

/// Version of the staging folder layout (session.yaml `format_version`).
constexpr int kStagingFormatVersion = 1;

struct StagingFrames
{
  std::string global = "map";
  std::string odom = "odom";
  std::string base = "base_footprint";
  std::string camera;  ///< optical frame of the images
};

/// session.yaml of a staging folder.
struct StagingSession
{
  std::string name;
  std::string created_utc;
  std::string landfall_version;
  std::string pose_source;  ///< recorder `pose_source.type`
  StagingFrames frames;
  CameraModel camera;
  std::string image_format = "png";  ///< png or jpg
  int jpeg_quality = 95;
  bool complete = false;  ///< set when the recorder closed the session cleanly
  GateCounts counts;      ///< written when the session is closed
};

/// One row of keyframes.csv.
struct StagingKeyframe
{
  int64_t id = 0;  ///< 0, 1, 2, ... within the session
  int64_t stamp_ns = 0;
  std::string image;  ///< path relative to the folder, e.g. images/000042.png
  Pose2 map_base;
  std::optional<Mat3> covariance;
  Pose3 T_base_cam = Pose3::Identity();
  ImageQuality quality;
  std::optional<double> tilt_deg;
};

/// Writes a staging folder: session.yaml, keyframes.csv and images/. Every image is written before
/// its CSV row and every row is flushed, so a crash loses at most the keyframe being written.
class StagingWriter
{
public:
  /// Creates `directory`, which must not exist yet, and writes session.yaml.
  StagingWriter(std::string directory, StagingSession session);
  StagingWriter(const StagingWriter &) = delete;
  StagingWriter & operator=(const StagingWriter &) = delete;

  /// Saves the image (8-bit gray or BGR) and appends the keyframe; assigns its id and image path.
  const StagingKeyframe & add(StagingKeyframe keyframe, const cv::Mat & image);
  /// Marks the session complete and stores the gate counts.
  void close(const GateCounts & counts);

  const std::string & directory() const { return directory_; }
  const StagingSession & session() const { return session_; }
  int64_t size() const { return next_id_; }

private:
  void writeSession() const;

  std::string directory_;
  StagingSession session_;
  std::ofstream csv_;
  int64_t next_id_ = 0;
  StagingKeyframe last_;
};

struct StagingFolder
{
  std::string directory;
  StagingSession session;
  std::vector<StagingKeyframe> keyframes;
};

/// Reads a staging folder. A truncated last CSV line (crash while writing) is ignored; anything
/// else malformed throws std::runtime_error.
StagingFolder readStaging(const std::string & directory);

/// Loads a keyframe's image with cv::imread flags (e.g. cv::IMREAD_GRAYSCALE). Throws if missing.
cv::Mat loadStagingImage(const StagingFolder & folder, const StagingKeyframe & keyframe, int flags);

}  // namespace landfall

#endif  // LANDFALL_RECORD_STAGING_HPP_
