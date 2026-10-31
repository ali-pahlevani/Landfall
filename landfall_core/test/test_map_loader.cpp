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

#include <opencv2/imgcodecs.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include "landfall/grid/map_loader.hpp"
#include "landfall/util/sha256.hpp"
#include "test_utils.hpp"

using landfall::MapServerSemantics;
using landfall::OccupancyGrid;

namespace
{

constexpr MapServerSemantics kBoth[] = {MapServerSemantics::kHumble, MapServerSemantics::kJazzy};

// 4 x 3 gray image, top row first.
cv::Mat grayImage()
{
  const cv::Mat image =
    (cv::Mat_<uint8_t>(3, 4) << 0, 254, 205, 128, 255, 50, 200, 100, 10, 20, 30, 40);
  return image;
}

std::string yaml(const std::string & image, const std::string & extra)
{
  return "image: " + image +
         "\nresolution: 0.05\norigin: [-1.0, -2.0, 0.0]\noccupied_thresh: 0.65\n"
         "free_thresh: 0.196\n" +
         extra;
}

// Grid rows listed top row first, like the image.
std::vector<int8_t> topFirst(const OccupancyGrid & grid)
{
  std::vector<int8_t> values;
  for (int row = grid.height - 1; row >= 0; --row) {
    for (int col = 0; col < grid.width; ++col) {
      values.push_back(grid.at(col, row));
    }
  }
  return values;
}

OccupancyGrid load(
  const landfall::test::TempDir & dir, const std::string & extra, MapServerSemantics semantics)
{
  landfall::test::writeText(dir.file("map.yaml"), yaml("map.pgm", extra));
  return landfall::loadMap(dir.file("map.yaml"), semantics).grid;
}

}  // namespace

TEST(MapLoader, TrinaryModeFlipsRowsAndThresholds)
{
  const landfall::test::TempDir dir;
  ASSERT_TRUE(cv::imwrite(dir.file("map.pgm"), grayImage()));
  for (auto semantics : kBoth) {
    const OccupancyGrid grid = load(dir, "negate: 0\n", semantics);
    EXPECT_EQ(grid.width, 4);
    EXPECT_EQ(grid.height, 3);
    EXPECT_DOUBLE_EQ(grid.resolution, 0.05);
    EXPECT_DOUBLE_EQ(grid.origin.x, -1.0);
    EXPECT_DOUBLE_EQ(grid.origin.y, -2.0);
    const std::vector<int8_t> expected = {100, 0, -1, -1, 0, 100, -1, -1, 100, 100, 100, 100};
    EXPECT_EQ(topFirst(grid), expected);
    // Image row 0 is the top of the map, i.e. grid row height - 1.
    EXPECT_EQ(grid.at(0, 2), 100);
    EXPECT_EQ(grid.at(0, 1), 0);
  }
}

TEST(MapLoader, ScaleRawAndNegateModes)
{
  const landfall::test::TempDir dir;
  ASSERT_TRUE(cv::imwrite(dir.file("map.pgm"), grayImage()));
  for (auto semantics : kBoth) {
    EXPECT_EQ(
      topFirst(load(dir, "negate: 0\nmode: scale\n", semantics)),
      (std::vector<int8_t>{100, 0, 0, 67, 0, 100, 4, 91, 100, 100, 100, 100}));
    EXPECT_EQ(
      topFirst(load(dir, "negate: 0\nmode: Raw\n", semantics)),
      (std::vector<int8_t>{0, -1, -1, -1, -1, 50, -1, 100, 10, 20, 30, 40}));
    // negate may be an int or a bool. 50 / 255 = 0.19608 is just above free_thresh: unknown.
    EXPECT_EQ(
      topFirst(load(dir, "negate: true\n", semantics)),
      (std::vector<int8_t>{0, 100, 100, -1, 100, -1, 100, -1, 0, 0, 0, 0}));
  }
}

TEST(MapLoader, ColorAndAlphaFollowEachMapServerVersion)
{
  landfall::MapMetadata metadata;
  metadata.resolution = 0.05;
  metadata.free_thresh = 0.196;
  metadata.occupied_thresh = 0.65;

  // Pure green: Humble averages the channels (dark -> occupied), Jazzy uses luminance (unknown).
  const cv::Mat green(1, 1, CV_8UC3, cv::Scalar(0, 255, 0));
  EXPECT_EQ(landfall::convertMapImage(green, metadata, MapServerSemantics::kHumble).data[0], 100);
  EXPECT_EQ(landfall::convertMapImage(green, metadata, MapServerSemantics::kJazzy).data[0], -1);

  // Half-transparent white: Humble averages alpha in (free), Jazzy marks transparency unknown.
  cv::Mat bgra(1, 2, CV_8UC4);
  bgra.at<cv::Vec4b>(0, 0) = cv::Vec4b(255, 255, 255, 128);
  bgra.at<cv::Vec4b>(0, 1) = cv::Vec4b(255, 255, 255, 255);
  const auto humble = landfall::convertMapImage(bgra, metadata, MapServerSemantics::kHumble);
  const auto jazzy = landfall::convertMapImage(bgra, metadata, MapServerSemantics::kJazzy);
  EXPECT_EQ(humble.data, (std::vector<int8_t>{0, 0}));
  EXPECT_EQ(jazzy.data, (std::vector<int8_t>{-1, 0}));

  // The same pixels written to a PNG load the same way.
  const landfall::test::TempDir dir;
  ASSERT_TRUE(cv::imwrite(dir.file("map.png"), bgra));
  landfall::test::writeText(dir.file("map.yaml"), yaml("map.png", "negate: 0\n"));
  EXPECT_EQ(
    landfall::loadMap(dir.file("map.yaml"), MapServerSemantics::kJazzy).grid.data, jazzy.data);
}

TEST(MapLoader, JazzyGrayIsGraphicsMagicksExactly)
{
  // Raw mode shows the gray value itself. These colors are where GraphicsMagick's grayscale (what
  // nav2_map_server >= 1.3 uses: 16-bit quanta, rounded luma, truncated to 8 bit) and OpenCV's
  // BGR2GRAY differ by one; the expected values come from GraphicsMagick 1.3.42 (Q16).
  landfall::MapMetadata metadata;
  metadata.resolution = 0.05;
  metadata.mode = landfall::MapMode::kRaw;
  cv::Mat colors(1, 4, CV_8UC3);
  colors.at<cv::Vec3b>(0, 0) = cv::Vec3b(110, 85, 24);  // BGR
  colors.at<cv::Vec3b>(0, 1) = cv::Vec3b(115, 29, 8);
  colors.at<cv::Vec3b>(0, 2) = cv::Vec3b(120, 62, 149);
  colors.at<cv::Vec3b>(0, 3) = cv::Vec3b(180, 65, 7);
  EXPECT_EQ(
    landfall::convertMapImage(colors, metadata, MapServerSemantics::kJazzy).data,
    (std::vector<int8_t>{69, 32, 94, 60}));
  // 16-bit input is converted from its full value, not from an 8-bit copy.
  cv::Mat gray16(1, 2, CV_16UC1);
  gray16.at<uint16_t>(0, 0) = 257 * 40 + 256;  // just below 41 * 257
  gray16.at<uint16_t>(0, 1) = 257 * 41;
  EXPECT_EQ(
    landfall::convertMapImage(gray16, metadata, MapServerSemantics::kJazzy).data,
    (std::vector<int8_t>{40, 41}));
}

TEST(MapLoader, ReportsHashesAndResolvesPaths)
{
  const landfall::test::TempDir dir;
  ASSERT_TRUE(cv::imwrite(dir.file("map.pgm"), grayImage()));
  const std::string text = yaml(dir.file("map.pgm"), "negate: 0\n");  // absolute image path
  landfall::test::writeText(dir.file("abs.yaml"), text);
  const auto map = landfall::loadMap(dir.file("abs.yaml"), MapServerSemantics::kJazzy);
  EXPECT_EQ(map.yaml_text, text);
  EXPECT_EQ(map.yaml_sha256, landfall::sha256Hex(text));
  EXPECT_EQ(map.image_sha256, landfall::sha256File(dir.file("map.pgm")));
  EXPECT_EQ(map.metadata.image_path, dir.file("map.pgm"));
}

TEST(MapLoader, RejectsInvalidYaml)
{
  const landfall::test::TempDir dir;
  ASSERT_TRUE(cv::imwrite(dir.file("map.pgm"), grayImage()));
  const auto expect_error = [&](const std::string & text, const std::string & fragment) {
    landfall::test::writeText(dir.file("bad.yaml"), text);
    try {
      landfall::loadMap(dir.file("bad.yaml"), MapServerSemantics::kJazzy);
      ADD_FAILURE() << "accepted: " << text;
    } catch (const std::runtime_error & e) {
      EXPECT_NE(std::string(e.what()).find(fragment), std::string::npos) << e.what();
    }
  };
  expect_error("image: map.pgm\norigin: [0, 0, 0]\nnegate: 0\n", "resolution");
  expect_error(yaml("map.pgm", "negate: 0\nmode: fancy\n"), "fancy");
  expect_error(yaml("map.pgm", ""), "negate");
  expect_error(yaml("missing.pgm", "negate: 0\n"), "missing.pgm");
  expect_error(
    "image: map.pgm\nresolution: 0.05\norigin: [0, 0]\nnegate: 0\noccupied_thresh: 0.65\n"
    "free_thresh: 0.2\n",
    "origin");
}

TEST(OccupancyGrid, PngRoundTripAndHash)
{
  OccupancyGrid grid;
  grid.width = 5;
  grid.height = 3;
  grid.resolution = 0.05;
  grid.origin = {1.0, 2.0, 0.0};
  grid.data = {-1, 0, 100, 50, 1, 2, 3, 4, 5, 6, 99, -1, -1, 0, 0};
  const auto png = landfall::encodeGridPng(grid);
  const auto decoded = landfall::decodeGridPng(png, grid.resolution, grid.origin);
  EXPECT_EQ(decoded.width, grid.width);
  EXPECT_EQ(decoded.height, grid.height);
  EXPECT_EQ(decoded.data, grid.data);
  EXPECT_EQ(landfall::gridSha256(decoded), landfall::gridSha256(grid));
  grid.data[3] = 51;
  EXPECT_NE(landfall::gridSha256(decoded), landfall::gridSha256(grid));
}

TEST(OccupancyGrid, WorldToCellIgnoresOutsidePoints)
{
  OccupancyGrid grid;
  grid.width = 10;
  grid.height = 20;
  grid.resolution = 0.5;
  grid.origin = {-1.0, -1.0, 0.0};
  grid.data.assign(200, 0);
  int col = 0;
  int row = 0;
  ASSERT_TRUE(grid.worldToCell(-0.99, -0.99, &col, &row));
  EXPECT_EQ(col, 0);
  EXPECT_EQ(row, 0);
  ASSERT_TRUE(grid.worldToCell(3.9, 8.9, &col, &row));
  EXPECT_EQ(col, 9);
  EXPECT_EQ(row, 19);
  EXPECT_FALSE(grid.worldToCell(-1.01, 0.0, &col, &row));
  EXPECT_FALSE(grid.worldToCell(4.0, 0.0, &col, &row));
  EXPECT_TRUE(grid.cellCenter(0, 0).isApprox(landfall::Vec2(-0.75, -0.75)));
}
