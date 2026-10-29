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

#include <opencv2/core.hpp>

#include "landfall/ros/compat/cv_bridge.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

TEST(CompatCvBridge, RoundTripsBgrImage)
{
  cv::Mat original(4, 3, CV_8UC3);
  cv::randu(original, cv::Scalar::all(0), cv::Scalar::all(255));

  std_msgs::msg::Header header;
  header.frame_id = "camera_optical_frame";
  const sensor_msgs::msg::Image::SharedPtr msg =
    cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, original).toImageMsg();
  ASSERT_EQ(msg->width, 3u);
  ASSERT_EQ(msg->height, 4u);

  const cv_bridge::CvImageConstPtr back =
    cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
  EXPECT_EQ(back->header.frame_id, "camera_optical_frame");
  EXPECT_EQ(cv::norm(back->image, original, cv::NORM_INF), 0.0);
}

TEST(CompatCvBridge, ConvertsColorToMono)
{
  const cv::Mat white(2, 2, CV_8UC3, cv::Scalar(255, 255, 255));
  const sensor_msgs::msg::Image::SharedPtr msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::BGR8, white)
      .toImageMsg();

  const cv_bridge::CvImagePtr mono = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  ASSERT_EQ(mono->image.type(), CV_8UC1);
  EXPECT_EQ(mono->image.at<uint8_t>(0, 0), 255);
}
