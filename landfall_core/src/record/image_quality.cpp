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

#include "landfall/record/image_quality.hpp"

#include <cstdio>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

namespace landfall
{

namespace
{
constexpr int kQualityWidth = 640;
constexpr int kSaturatedLevel = 250;
}  // namespace

cv::Mat toGray(const cv::Mat & image)
{
  if (image.depth() != CV_8U) {
    throw std::invalid_argument("toGray: expected an 8-bit image");
  }
  cv::Mat gray;
  switch (image.channels()) {
    case 1:
      return image;
    case 3:
      cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
      return gray;
    case 4:
      cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
      return gray;
    default:
      throw std::invalid_argument("toGray: unsupported channel count");
  }
}

ImageQuality measureImageQuality(const cv::Mat & gray)
{
  if (gray.empty() || gray.type() != CV_8UC1) {
    throw std::invalid_argument("measureImageQuality: expected a non-empty 8-bit grayscale image");
  }
  ImageQuality quality;
  cv::Mat scaled = gray;
  if (gray.cols > kQualityWidth) {
    const double scale = static_cast<double>(kQualityWidth) / gray.cols;
    cv::resize(gray, scaled, cv::Size(), scale, scale, cv::INTER_AREA);
  }
  cv::Mat laplacian;
  cv::Laplacian(scaled, laplacian, CV_64F);
  cv::Scalar mean;
  cv::Scalar stddev;
  cv::meanStdDev(laplacian, mean, stddev);
  quality.sharpness = stddev[0] * stddev[0];
  quality.mean_intensity = cv::mean(gray)[0];
  quality.saturated_fraction = static_cast<double>(cv::countNonZero(gray >= kSaturatedLevel)) /
                               static_cast<double>(gray.total());
  return quality;
}

std::string toJson(const ImageQuality & quality)
{
  char buffer[160];
  std::snprintf(
    buffer, sizeof(buffer), R"({"sharpness":%.6g,"mean_intensity":%.6g,"saturated_fraction":%.6g})",
    quality.sharpness, quality.mean_intensity, quality.saturated_fraction);
  return buffer;
}

}  // namespace landfall
