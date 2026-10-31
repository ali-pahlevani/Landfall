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

#include "landfall/db/codec.hpp"

#include <cstring>
#include <string>

namespace landfall::db
{

namespace
{

constexpr std::size_t kKeypointFloats = 5;

[[noreturn]] void corrupt(const std::string & what)
{
  throw DatabaseError(DatabaseError::Code::kSqlite, "corrupt " + what);
}

std::size_t elementSize(BlobType type)
{
  switch (type) {
    case BlobType::kU8:
      return 1;
    case BlobType::kF16:
      return 2;
    case BlobType::kF32:
      return 4;
    case BlobType::kSparseF32:
      return 8;
  }
  corrupt("blob type");
}

// Dense float rows as stored: f32 copied, f16 converted.
std::vector<uint8_t> floatBlob(const cv::Mat & rows, BlobType type)
{
  cv::Mat stored;
  if (type == BlobType::kF16) {
    rows.convertTo(stored, CV_16F);
  } else if (type == BlobType::kF32) {
    stored = rows.isContinuous() ? rows : rows.clone();
  } else {
    throw std::invalid_argument("float data must be stored as f32 or f16");
  }
  if (!stored.isContinuous()) {
    stored = stored.clone();
  }
  return {stored.data, stored.data + stored.total() * stored.elemSize()};
}

cv::Mat floatRows(const std::vector<uint8_t> & blob, int rows, int cols, BlobType type)
{
  if (type == BlobType::kF32) {
    cv::Mat m(rows, cols, CV_32F);
    std::memcpy(m.data, blob.data(), blob.size());
    return m;
  }
  cv::Mat half(rows, cols, CV_16F);
  std::memcpy(half.data, blob.data(), blob.size());
  cv::Mat m;
  half.convertTo(m, CV_32F);
  return m;
}

}  // namespace

LocalFeaturesRecord encodeLocalFeatures(
  int64_t keyframe_id, int64_t backend_id, const Features & features, BlobType float_type)
{
  if (features.descriptors.rows != static_cast<int>(features.keypoints.size())) {
    throw std::invalid_argument("features: one descriptor row per keypoint expected");
  }
  LocalFeaturesRecord record;
  record.keyframe_id = keyframe_id;
  record.backend_id = backend_id;
  record.keypoints = features.keypoints;
  if (features.type == DescriptorType::kBinary) {
    if (!features.keypoints.empty() && features.descriptors.type() != CV_8UC1) {
      throw std::invalid_argument("binary descriptors must be CV_8UC1");
    }
    record.descriptor_type = BlobType::kU8;
    record.descriptor_dim = features.descriptors.cols;
    const cv::Mat rows =
      features.descriptors.isContinuous() ? features.descriptors : features.descriptors.clone();
    record.descriptors.assign(rows.data, rows.data + rows.total());
  } else {
    if (!features.keypoints.empty() && features.descriptors.type() != CV_32FC1) {
      throw std::invalid_argument("float descriptors must be CV_32FC1");
    }
    record.descriptor_type = float_type;
    record.descriptor_dim = features.descriptors.cols;
    record.descriptors = floatBlob(features.descriptors, float_type);
  }
  return record;
}

Features decodeLocalFeatures(const LocalFeaturesRecord & record)
{
  Features features;
  features.keypoints = record.keypoints;
  const int rows = static_cast<int>(record.keypoints.size());
  const int cols = record.descriptor_dim;
  if (
    cols < 0 || record.descriptors.size() !=
                  static_cast<std::size_t>(rows) * cols * elementSize(record.descriptor_type)) {
    corrupt("local features of keyframe " + std::to_string(record.keyframe_id));
  }
  switch (record.descriptor_type) {
    case BlobType::kU8:
      features.type = DescriptorType::kBinary;
      features.descriptors = cv::Mat(rows, cols, CV_8UC1);
      if (!record.descriptors.empty()) {
        std::memcpy(
          features.descriptors.data, record.descriptors.data(), record.descriptors.size());
      }
      break;
    case BlobType::kF32:
    case BlobType::kF16:
      features.type = DescriptorType::kFloat32;
      features.descriptors = floatRows(record.descriptors, rows, cols, record.descriptor_type);
      break;
    case BlobType::kSparseF32:
      corrupt("local feature descriptor type");
  }
  return features;
}

GlobalDescriptorRecord encodeGlobalDescriptor(
  int64_t keyframe_id, int64_t backend_id, const GlobalDescriptor & descriptor, BlobType dense_type)
{
  if (!descriptor.dense.empty() && !descriptor.bow.empty()) {
    throw std::invalid_argument("a global descriptor is either dense or bag-of-words");
  }
  GlobalDescriptorRecord record;
  record.keyframe_id = keyframe_id;
  record.backend_id = backend_id;
  if (descriptor.dense.empty()) {
    record.type = BlobType::kSparseF32;
    record.dim = static_cast<int>(descriptor.bow.size());
    record.data.resize(descriptor.bow.size() * 8);
    uint8_t * out = record.data.data();
    for (const auto & [word, weight] : descriptor.bow) {
      std::memcpy(out, &word, 4);
      std::memcpy(out + 4, &weight, 4);
      out += 8;
    }
  } else {
    record.type = dense_type;
    record.dim = static_cast<int>(descriptor.dense.size());
    const cv::Mat row(1, record.dim, CV_32F, const_cast<float *>(descriptor.dense.data()));
    record.data = floatBlob(row, dense_type);
  }
  return record;
}

GlobalDescriptor decodeGlobalDescriptor(const GlobalDescriptorRecord & record)
{
  GlobalDescriptor descriptor;
  if (
    record.dim < 0 ||
    record.data.size() != static_cast<std::size_t>(record.dim) * elementSize(record.type)) {
    corrupt("global descriptor of keyframe " + std::to_string(record.keyframe_id));
  }
  switch (record.type) {
    case BlobType::kSparseF32: {
      descriptor.bow.resize(static_cast<std::size_t>(record.dim));
      const uint8_t * in = record.data.data();
      for (auto & [word, weight] : descriptor.bow) {
        std::memcpy(&word, in, 4);
        std::memcpy(&weight, in + 4, 4);
        in += 8;
      }
      break;
    }
    case BlobType::kF32:
    case BlobType::kF16: {
      const cv::Mat row = floatRows(record.data, 1, record.dim, record.type);
      descriptor.dense.assign(row.ptr<float>(), row.ptr<float>() + record.dim);
      break;
    }
    case BlobType::kU8:
      corrupt("global descriptor type");
  }
  return descriptor;
}

std::vector<uint8_t> descriptorRowBytes(const cv::Mat & descriptors, int row)
{
  const auto * begin = descriptors.ptr<uint8_t>(row);
  return {begin, begin + descriptors.cols * descriptors.elemSize()};
}

}  // namespace landfall::db
