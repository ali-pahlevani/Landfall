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

#ifndef LANDFALL_QUERY_EARLY_STOP_HPP_
#define LANDFALL_QUERY_EARLY_STOP_HPP_

#include <opencv2/core.hpp>
#include <optional>

#include "landfall/types.hpp"

namespace landfall
{

struct EarlyStopParams
{
  /// Frames from one camera pose, none of them near a pose, that end a request; 0: never.
  int frames = 2;
  /// A frame is near a pose from this closeness on (FrameResult::closeness).
  double closeness = 0.5;
  double still_translation = 0.01;  ///< m: a camera that moved less sees the same
  double still_rotation_deg = 0.5;  ///< deg, likewise
  /// Grey levels: an image that changed less (on average, in a 64 x 48 thumbnail, each image's
  /// mean brightness removed) shows the same; more is a scene that changes (a person walks by).
  double still_image_change = 2.0;
};

/// Ends a relocalization request that can no longer succeed (plan §7.10, V63). While the camera
/// stands still and the scene doesn't change, each frame shows what the last ones showed: once
/// `frames` such frames came nowhere near a pose (their closeness below `closeness`), and no frame
/// of the request came near one, the frames that follow would fail too. A camera that moves (the
/// robot turns, a mast lifts it) or an image that changes (a person walks through the view) starts
/// the count again, and one frame that came near (or found a pose) keeps the request going to its
/// end. Poses are the camera's in the odometry frame: the base's from odometry, composed with the
/// extrinsics at the image.
class EarlyStop
{
public:
  explicit EarlyStop(const EarlyStopParams & params = EarlyStopParams()) : params_(params) {}

  /// Adds a frame of the request (its image: 8-bit, grey or BGR); true when the request should
  /// end.
  bool add(const Pose3 & odom_T_cam, const cv::Mat & image, double closeness);

  /// An image's 64 x 48 grey thumbnail (float), its mean brightness removed.
  static cv::Mat thumbnail(const cv::Mat & image);
  /// How much two thumbnails differ: the mean absolute difference, in grey levels
  /// (EarlyStopParams::still_image_change).
  static double change(const cv::Mat & a, const cv::Mat & b);

  /// Frames from the latest camera pose so far.
  int stillFrames() const { return still_frames_; }
  /// The best closeness of the request's frames.
  double bestCloseness() const { return best_closeness_; }

private:
  EarlyStopParams params_;
  std::optional<Pose3> still_from_;  ///< the first frame from the latest camera pose
  cv::Mat still_thumbnail_;          ///< its thumbnail
  int still_frames_ = 0;
  double best_closeness_ = 0.0;
};

}  // namespace landfall

#endif  // LANDFALL_QUERY_EARLY_STOP_HPP_
