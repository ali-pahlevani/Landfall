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

#include "landfall/query/early_stop.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>

namespace landfall
{

cv::Mat EarlyStop::thumbnail(const cv::Mat & image)
{
  cv::Mat gray = image;
  if (image.channels() == 3) {
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  } else if (image.channels() == 4) {
    cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
  }
  cv::Mat small;
  cv::resize(gray, small, cv::Size(64, 48), 0.0, 0.0, cv::INTER_AREA);
  cv::Mat result;
  small.convertTo(result, CV_32F);
  result -= cv::mean(result)[0];
  return result;
}

double EarlyStop::change(const cv::Mat & a, const cv::Mat & b)
{
  cv::Mat difference;
  cv::absdiff(a, b, difference);
  return cv::mean(difference)[0];
}

bool EarlyStop::add(const Pose3 & odom_T_cam, const cv::Mat & image, double closeness)
{
  best_closeness_ = std::max(best_closeness_, closeness);
  cv::Mat small = image.empty() ? cv::Mat() : thumbnail(image);
  bool still = false;
  if (still_from_) {
    const Pose3 moved = still_from_->inverse() * odom_T_cam;
    const double turned_deg = Eigen::AngleAxisd(moved.linear()).angle() * 180.0 / M_PI;
    still = moved.translation().norm() <= params_.still_translation &&
            turned_deg <= params_.still_rotation_deg && !small.empty() &&
            !still_thumbnail_.empty() &&
            change(still_thumbnail_, small) <= params_.still_image_change;
  }
  if (still) {
    ++still_frames_;
  } else {
    still_from_ = odom_T_cam;
    still_thumbnail_ = small;
    still_frames_ = 1;
  }
  return params_.frames > 0 && best_closeness_ < params_.closeness &&
         still_frames_ >= params_.frames;
}

}  // namespace landfall
