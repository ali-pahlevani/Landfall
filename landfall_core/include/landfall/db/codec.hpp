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

#ifndef LANDFALL_DB_CODEC_HPP_
#define LANDFALL_DB_CODEC_HPP_

#include <cstdint>

#include "landfall/db/database.hpp"
#include "landfall/interfaces/global_descriptor.hpp"

namespace landfall::db
{

/// Features as a `local_features` row: binary descriptors as u8, float ones as f32 or f16.
LocalFeaturesRecord encodeLocalFeatures(
  int64_t keyframe_id, int64_t backend_id, const Features & features,
  BlobType float_type = BlobType::kF32);
/// Throws DatabaseError (kSqlite, "corrupt ...") on inconsistent sizes.
Features decodeLocalFeatures(const LocalFeaturesRecord & record);

/// Bag-of-words vectors are stored sparse (kSparseF32, dim = number of entries); dense vectors as
/// f32 or f16 (dim = length).
GlobalDescriptorRecord encodeGlobalDescriptor(
  int64_t keyframe_id, int64_t backend_id, const GlobalDescriptor & descriptor,
  BlobType dense_type = BlobType::kF16);
GlobalDescriptor decodeGlobalDescriptor(const GlobalDescriptorRecord & record);

/// One descriptor row as raw bytes (a landmark's representative descriptor).
std::vector<uint8_t> descriptorRowBytes(const cv::Mat & descriptors, int row);

}  // namespace landfall::db

#endif  // LANDFALL_DB_CODEC_HPP_
