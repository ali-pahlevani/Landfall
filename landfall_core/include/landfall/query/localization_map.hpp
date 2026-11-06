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

#ifndef LANDFALL_QUERY_LOCALIZATION_MAP_HPP_
#define LANDFALL_QUERY_LOCALIZATION_MAP_HPP_

#include <cstdint>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "landfall/camera_model.hpp"
#include "landfall/db/database.hpp"
#include "landfall/grid/likelihood_field.hpp"
#include "landfall/grid/occupancy_grid.hpp"
#include "landfall/interfaces/global_descriptor.hpp"
#include "landfall/interfaces/local_features.hpp"
#include "landfall/interfaces/retrieval_index.hpp"
#include "landfall/registry.hpp"

namespace landfall
{

/// A database keyframe as the query needs it.
struct MapKeyframe
{
  int64_t id = 0;
  Pose2 pose;  ///< base pose after the build's BA
  Pose3 T_map_cam = Pose3::Identity();
};

/// A keyframe's stored local features and the landmark each one observes.
struct KeyframeFeatures
{
  Features features;
  std::vector<int64_t> landmarks;  ///< per feature row; -1 if it observes none
};

/// A map database opened for relocalization: read-only, checked against this build's backends,
/// with the retrieval index, landmark positions and covisibility in memory. Local features are
/// loaded per keyframe on demand and cached (plan R13), so memory stays small on large maps.
/// Thread-safe after load().
class LocalizationMap
{
public:
  struct Options
  {
    std::size_t feature_cache_size = 256;
    /// The backends; nullptr: the built-in ones. Learned backends need a registry with
    /// onnx::registerBackends().
    const BackendRegistry * registry = nullptr;
    std::string models_dir;                  ///< learned backends' models; "": see modelsDir()
    std::string execution_provider = "cpu";  ///< cpu, cuda, tensorrt
    int threads = 0;                         ///< ONNX Runtime threads; 0: its default
  };

  /// Opens and validates `path`. Throws std::runtime_error with an actionable message: missing
  /// file, wrong schema (run `landfall migrate`), not exactly one map and one camera, backends
  /// this build can't reproduce (rebuild the database), or a learned backend whose model is
  /// missing or differs from the one the database was built with.
  static std::shared_ptr<LocalizationMap> load(const std::string & path, const Options & options);
  static std::shared_ptr<LocalizationMap> load(
    const std::string & path, std::size_t feature_cache_size = 256);

  LocalizationMap(const LocalizationMap &) = delete;
  LocalizationMap & operator=(const LocalizationMap &) = delete;

  const std::string & path() const { return path_; }
  /// meta `content_sha256` of the database.
  const std::string & contentSha256() const { return content_sha256_; }
  /// The map the database was built for (hashes, geometry; `grid` holds the PNG).
  const db::MapRecord & mapRecord() const { return map_record_; }
  const OccupancyGrid & grid() const { return grid_; }
  /// Likelihood field of the map (free-space check, LiDAR verification).
  const LikelihoodField & field() const { return *field_; }
  std::shared_ptr<const LikelihoodField> fieldPtr() const { return field_; }
  const db::CameraRecord & cameraRecord() const { return camera_record_; }
  /// Camera model the database's images were taken with.
  const CameraModel & camera() const { return camera_; }

  const std::vector<MapKeyframe> & keyframes() const { return keyframes_; }
  /// nullptr for an unknown id.
  const MapKeyframe * keyframe(int64_t id) const;
  std::size_t numLandmarks() const { return landmarks_.size(); }
  /// nullptr for an unknown id.
  const Vec3 * landmark(int64_t id) const;
  /// Landmarks two keyframes share (0 below the build's covisibility threshold).
  int covisibility(int64_t a, int64_t b) const;

  const ILocalFeatures & localFeatures() const { return *local_features_; }
  const IGlobalDescriptor & globalDescriptor() const { return *global_descriptor_; }
  const IRetrievalIndex & retrievalIndex() const { return *index_; }
  DescriptorType descriptorType() const { return descriptor_type_; }

  /// Stored features of a keyframe (cached). Throws std::runtime_error for an unknown keyframe or
  /// corrupt rows.
  std::shared_ptr<const KeyframeFeatures> features(int64_t keyframe_id) const;

private:
  explicit LocalizationMap(db::Database database, std::size_t feature_cache_size);

  std::string path_;
  std::string content_sha256_;
  db::MapRecord map_record_;
  OccupancyGrid grid_;
  std::shared_ptr<const LikelihoodField> field_;
  db::CameraRecord camera_record_;
  CameraModel camera_;
  std::vector<MapKeyframe> keyframes_;
  std::unordered_map<int64_t, std::size_t> keyframe_index_;
  std::unordered_map<int64_t, Vec3> landmarks_;
  std::map<std::pair<int64_t, int64_t>, int> covisibility_;
  std::unique_ptr<ILocalFeatures> local_features_;
  std::unique_ptr<IGlobalDescriptor> global_descriptor_;
  std::unique_ptr<IRetrievalIndex> index_;
  DescriptorType descriptor_type_ = DescriptorType::kBinary;
  int64_t local_backend_id_ = 0;

  // Lazy feature loading: the connection and the LRU cache share one mutex.
  mutable std::mutex mutex_;
  mutable db::Database database_;
  std::size_t cache_size_;
  mutable std::list<std::pair<int64_t, std::shared_ptr<const KeyframeFeatures>>> cache_;
  mutable std::unordered_map<
    int64_t, std::list<std::pair<int64_t, std::shared_ptr<const KeyframeFeatures>>>::iterator>
    cache_index_;
};

}  // namespace landfall

#endif  // LANDFALL_QUERY_LOCALIZATION_MAP_HPP_
