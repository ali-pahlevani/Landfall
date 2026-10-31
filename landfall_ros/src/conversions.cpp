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

#include "landfall/ros/conversions.hpp"

#include <algorithm>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

#include "landfall/ros/compat/cv_bridge.hpp"
#include "sensor_msgs/image_encodings.hpp"

namespace landfall::ros
{

int64_t toNanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1'000'000'000LL + static_cast<int64_t>(stamp.nanosec);
}

builtin_interfaces::msg::Time toStamp(int64_t nanoseconds)
{
  builtin_interfaces::msg::Time stamp;
  int64_t sec = nanoseconds / 1'000'000'000LL;
  int64_t nsec = nanoseconds % 1'000'000'000LL;
  if (nsec < 0) {
    nsec += 1'000'000'000LL;
    --sec;
  }
  stamp.sec = static_cast<int32_t>(sec);
  stamp.nanosec = static_cast<uint32_t>(nsec);
  return stamp;
}

Pose3 toPose3(const geometry_msgs::msg::Transform & transform)
{
  Pose3 pose = Pose3::Identity();
  pose.translation() =
    Vec3(transform.translation.x, transform.translation.y, transform.translation.z);
  pose.linear() =
    Eigen::Quaterniond(
      transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z)
      .normalized()
      .toRotationMatrix();
  return pose;
}

Pose3 toPose3(const geometry_msgs::msg::Pose & msg)
{
  Pose3 pose = Pose3::Identity();
  pose.translation() = Vec3(msg.position.x, msg.position.y, msg.position.z);
  pose.linear() =
    Eigen::Quaterniond(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
      .normalized()
      .toRotationMatrix();
  return pose;
}

geometry_msgs::msg::Pose toPoseMsg(const Pose3 & pose)
{
  geometry_msgs::msg::Pose msg;
  msg.position.x = pose.translation().x();
  msg.position.y = pose.translation().y();
  msg.position.z = pose.translation().z();
  const Eigen::Quaterniond q(pose.linear());
  msg.orientation.x = q.x();
  msg.orientation.y = q.y();
  msg.orientation.z = q.z();
  msg.orientation.w = q.w();
  return msg;
}

Mat3 planarCovariance(const geometry_msgs::msg::PoseWithCovariance::_covariance_type & c)
{
  Mat3 cov;
  cov << c[0], c[1], c[5], c[6], c[7], c[11], c[30], c[31], c[35];
  return cov;
}

CameraModel toCameraModel(const sensor_msgs::msg::CameraInfo & info)
{
  const auto model = parseDistortionModel(info.distortion_model);
  if (!model) {
    throw std::invalid_argument("unsupported distortion model '" + info.distortion_model + "'");
  }
  if (info.k[0] <= 0.0 || info.k[4] <= 0.0) {
    throw std::invalid_argument("CameraInfo has no intrinsics (uncalibrated camera?)");
  }
  std::vector<double> coefficients(info.d.begin(), info.d.end());
  // Drivers pad with zeros (e.g. 14 rational_polynomial terms, of which OpenCV's thin-prism and
  // tilt terms are unused); all zeros means no distortion.
  std::size_t used = 0;
  switch (*model) {
    case DistortionModel::kPlumbBob:
      used = 5;
      break;
    case DistortionModel::kRationalPolynomial:
      used = 8;
      break;
    case DistortionModel::kEquidistant:
      used = 4;
      break;
    case DistortionModel::kNone:
      used = 0;
      break;
  }
  if (std::all_of(coefficients.begin(), coefficients.end(), [](double c) { return c == 0.0; })) {
    coefficients.clear();
  } else if (
    coefficients.size() > used && std::all_of(
                                    coefficients.begin() + static_cast<std::ptrdiff_t>(used),
                                    coefficients.end(), [](double c) { return c == 0.0; })) {
    coefficients.resize(used);
  }
  return {
    static_cast<int>(info.width),
    static_cast<int>(info.height),
    info.k[0],
    info.k[4],
    info.k[2],
    info.k[5],
    *model,
    coefficients};
}

cv::Mat decodeImage(const sensor_msgs::msg::Image & image)
{
  namespace enc = sensor_msgs::image_encodings;
  const bool mono = image.encoding == enc::MONO8 || image.encoding == enc::MONO16 ||
                    image.encoding == enc::TYPE_8UC1 || image.encoding == enc::TYPE_16UC1;
  try {
    return cv_bridge::toCvCopy(image, mono ? enc::MONO8 : enc::BGR8)->image;
  } catch (const cv_bridge::Exception & e) {
    throw std::runtime_error(
      "cannot convert image encoding '" + image.encoding + "': " + std::string(e.what()));
  } catch (const cv::Exception & e) {
    throw std::runtime_error(
      "cannot convert image encoding '" + image.encoding + "': " + std::string(e.what()));
  }
}

cv::Mat decodeImage(const sensor_msgs::msg::CompressedImage & image)
{
  if (image.data.empty()) {
    throw std::runtime_error("empty compressed image (format '" + image.format + "')");
  }
  // A header over the bytes: the field is a std::vector before Lyrical and a rosidl::Buffer since.
  const cv::Mat bytes(
    1, static_cast<int>(image.data.size()), CV_8UC1, const_cast<uint8_t *>(image.data.data()));
  cv::Mat decoded = cv::imdecode(bytes, cv::IMREAD_UNCHANGED);
  if (decoded.empty()) {
    throw std::runtime_error("cannot decode compressed image (format '" + image.format + "')");
  }
  if (decoded.depth() == CV_16U) {
    decoded.convertTo(decoded, CV_8U, 1.0 / 256.0);
  }
  if (decoded.depth() != CV_8U) {
    throw std::runtime_error("unsupported compressed image depth (format '" + image.format + "')");
  }
  if (decoded.channels() == 4) {
    cv::cvtColor(decoded, decoded, cv::COLOR_BGRA2BGR);
  }
  return decoded;
}

}  // namespace landfall::ros
