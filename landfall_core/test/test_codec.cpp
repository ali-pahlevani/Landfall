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

#include <cmath>
#include <vector>

#include "landfall/db/codec.hpp"
#include "landfall/util/json.hpp"

using landfall::DescriptorType;
using landfall::Features;
using landfall::GlobalDescriptor;
using landfall::db::BlobType;

namespace
{

Features makeFeatures(DescriptorType type, int count)
{
  Features f;
  f.type = type;
  cv::RNG rng(5);
  for (int i = 0; i < count; ++i) {
    f.keypoints.push_back(
      {static_cast<float>(i) + 0.25F, 2.5F * static_cast<float>(i), 7.0F, 33.0F,
       0.001F * static_cast<float>(i)});
  }
  if (type == DescriptorType::kBinary) {
    f.descriptors.create(count, 32, CV_8UC1);
  } else {
    f.descriptors.create(count, 64, CV_32FC1);
  }
  if (count > 0) {
    rng.fill(
      f.descriptors, cv::RNG::UNIFORM, type == DescriptorType::kBinary ? 0.0 : -1.0,
      type == DescriptorType::kBinary ? 256.0 : 1.0);
  }
  return f;
}

}  // namespace

TEST(Codec, LocalFeaturesRoundTrip)
{
  const Features binary = makeFeatures(DescriptorType::kBinary, 50);
  const auto record = landfall::db::encodeLocalFeatures(3, 4, binary);
  EXPECT_EQ(record.descriptor_type, BlobType::kU8);
  EXPECT_EQ(record.descriptor_dim, 32);
  const Features back = landfall::db::decodeLocalFeatures(record);
  EXPECT_EQ(back.type, DescriptorType::kBinary);
  EXPECT_EQ(cv::norm(back.descriptors, binary.descriptors, cv::NORM_INF), 0.0);
  ASSERT_EQ(back.keypoints.size(), 50U);
  EXPECT_EQ(back.keypoints[7].x, binary.keypoints[7].x);

  const Features floats = makeFeatures(DescriptorType::kFloat32, 20);
  const Features f32 =
    landfall::db::decodeLocalFeatures(landfall::db::encodeLocalFeatures(1, 1, floats));
  EXPECT_EQ(cv::norm(f32.descriptors, floats.descriptors, cv::NORM_INF), 0.0);
  const Features f16 = landfall::db::decodeLocalFeatures(
    landfall::db::encodeLocalFeatures(1, 1, floats, BlobType::kF16));
  EXPECT_LT(cv::norm(f16.descriptors, floats.descriptors, cv::NORM_INF), 1e-3);

  auto corrupt = record;
  corrupt.descriptors.pop_back();
  EXPECT_THROW(landfall::db::decodeLocalFeatures(corrupt), landfall::db::DatabaseError);
  const Features none = makeFeatures(DescriptorType::kBinary, 0);
  EXPECT_TRUE(landfall::db::decodeLocalFeatures(landfall::db::encodeLocalFeatures(1, 1, none))
                .keypoints.empty());
}

TEST(Codec, GlobalDescriptorsSparseAndDense)
{
  GlobalDescriptor bow;
  bow.bow = {{3, 0.25F}, {17, 0.5F}, {4000000000U, 0.25F}};
  const auto sparse = landfall::db::encodeGlobalDescriptor(1, 2, bow);
  EXPECT_EQ(sparse.type, BlobType::kSparseF32);
  EXPECT_EQ(sparse.dim, 3);
  EXPECT_EQ(landfall::db::decodeGlobalDescriptor(sparse).bow, bow.bow);

  GlobalDescriptor dense;
  dense.dense = {0.6F, -0.8F, 0.0F, 0.125F};
  const auto f32 = landfall::db::encodeGlobalDescriptor(1, 2, dense, BlobType::kF32);
  EXPECT_EQ(landfall::db::decodeGlobalDescriptor(f32).dense, dense.dense);
  const auto f16 = landfall::db::encodeGlobalDescriptor(1, 2, dense);
  EXPECT_EQ(f16.type, BlobType::kF16);
  EXPECT_EQ(f16.data.size(), 8U);
  const auto back = landfall::db::decodeGlobalDescriptor(f16).dense;
  ASSERT_EQ(back.size(), 4U);
  for (std::size_t i = 0; i < back.size(); ++i) {
    EXPECT_NEAR(back[i], dense.dense[i], 1e-3);
  }
  GlobalDescriptor both = dense;
  both.bow = bow.bow;
  EXPECT_THROW(landfall::db::encodeGlobalDescriptor(1, 2, both), std::invalid_argument);
}

TEST(Json, WriterEscapesAndNests)
{
  landfall::JsonWriter json;
  json.beginObject()
    .field("name", "a \"b\"\n")
    .field("n", 3)
    .field("x", 0.5)
    .field("nan", std::nan(""))
    .field("ok", true);
  json.key("list").beginArray().value(1).value("two").beginObject().endObject().endArray();
  json.endObject();
  EXPECT_EQ(
    json.str(), R"({"name":"a \"b\"\n","n":3,"x":0.5,"nan":null,"ok":true,"list":[1,"two",{}]})");
}
