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

#ifndef LANDFALL_RECORD_KEYFRAME_SELECTOR_HPP_
#define LANDFALL_RECORD_KEYFRAME_SELECTOR_HPP_

#include <cstdint>
#include <functional>
#include <optional>

#include "landfall/record/image_quality.hpp"
#include "landfall/types.hpp"

namespace landfall
{

/// Keyframe gates (recorder parameters `gating.*`, `quality.*`, `max_keyframes`).
struct KeyframeGating
{
  bool require_covariance = true;  ///< false for pose sources without one (tf_only)
  double max_std_xy = 0.05;        ///< m, worst direction of the x/y covariance
  double max_std_yaw_deg = 2.0;
  double max_covariance_age = 5.0;  ///< s between the covariance estimate and the image
  double min_translation = 0.5;     ///< m since the last keyframe ...
  double min_rotation_deg = 15.0;   ///< ... or this much rotation
  double max_linear_speed = 0.6;    ///< m/s
  double max_angular_speed = 0.5;   ///< rad/s
  double max_tilt_deg = 1.5;        ///< IMU only: no keyframes on ramps
  double min_sharpness = 60.0;
  double min_mean_intensity = 30.0;
  double max_mean_intensity = 225.0;
  double max_saturated_fraction = 0.20;
  int max_keyframes = 0;  ///< 0 = unlimited
};

/// Why a frame was (not) taken. Each rejection has a counter in landfall_msgs/RecorderStatus.
enum class GateResult : uint8_t {
  kAccepted,
  kNoPose,         ///< no pose at the image stamp (TF lookup failed)
  kNoMotion,       ///< too close to the last keyframe
  kPoseUncertain,  ///< localization covariance too large, too old or missing
  kSpeed,          ///< driving or turning too fast
  kTilt,           ///< on a ramp (IMU)
  kBlur,
  kExposure,
  kLimitReached,  ///< max_keyframes reached; nothing more is recorded
};

const char * toString(GateResult result);

/// Everything about a frame except its pixels.
struct FrameCandidate
{
  Pose2 map_base;                  ///< base pose in the global frame at the image stamp
  std::optional<Mat3> covariance;  ///< x, y, yaw; nullopt if the pose source has none
  double covariance_age = 0.0;     ///< s
  double linear_speed = 0.0;       ///< m/s
  double angular_speed = 0.0;      ///< rad/s
  std::optional<double> tilt_deg;  ///< nullopt without an IMU
};

struct GateCounts
{
  uint32_t frames_seen = 0;
  uint32_t keyframes = 0;
  uint32_t no_pose = 0;
  uint32_t no_motion = 0;
  uint32_t pose_uncertain = 0;
  uint32_t speed = 0;
  uint32_t tilt = 0;
  uint32_t blur = 0;
  uint32_t exposure = 0;
};

/// Decides which frames become keyframes. Shared by the live recorder and `landfall build --bag`
/// so both select the same frames from the same data.
///
/// The motion gate runs first, so the other counters say how many due keyframes each gate cost;
/// the image is only measured when every pose gate has passed.
class KeyframeSelector
{
public:
  explicit KeyframeSelector(const KeyframeGating & gating);

  /// Evaluates a frame; `measure` is called at most once, and only if the pose gates pass. An
  /// accepted frame becomes the reference of the motion gate.
  GateResult evaluate(const FrameCandidate & frame, const std::function<ImageQuality()> & measure);
  /// Counts a frame whose pose couldn't be determined.
  void countNoPose();

  /// The pose gates alone (no counting, no state change).
  GateResult checkPose(const FrameCandidate & frame) const;
  GateResult checkImage(const ImageQuality & quality) const;

  const GateCounts & counts() const { return counts_; }
  const std::optional<Pose2> & lastKeyframe() const { return last_keyframe_; }
  const KeyframeGating & gating() const { return gating_; }

private:
  void count(GateResult result);

  KeyframeGating gating_;
  GateCounts counts_;
  std::optional<Pose2> last_keyframe_;
};

/// Standard deviation along the worst direction of the x/y block of a pose covariance.
double worstStdXy(const Mat3 & covariance);

}  // namespace landfall

#endif  // LANDFALL_RECORD_KEYFRAME_SELECTOR_HPP_
