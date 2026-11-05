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

#include "landfall/query/localization_map.hpp"

#include <algorithm>
#include <stdexcept>

#include "landfall/db/codec.hpp"
#include "landfall/features/retrieval.hpp"
#include "landfall/features/vocab_tree.hpp"
#include "landfall/registry.hpp"

namespace landfall
{

namespace
{

db::BackendRecord backendWithRole(
  const std::vector<db::BackendRecord> & backends, const std::string & role)
{
  const db::BackendRecord * found = nullptr;
  for (const auto & b : backends) {
    if (b.role == role) {
      if (found != nullptr) {
        throw std::runtime_error("the database lists more than one " + role + " backend");
      }
      found = &b;
    }
  }
  if (found == nullptr) {
    throw std::runtime_error("the database has no " + role + " backend");
  }
  return *found;
}

// The relocalizer must describe images exactly as the builder did, so a backend is accepted only
// if this build reproduces its name, version and configuration.
void requireSameBackend(const db::BackendRecord & stored, const BackendId & ours)
{
  if (stored.name != ours.name || stored.version != ours.version) {
    throw std::runtime_error(
      "the database was built with " + stored.role + " backend " + stored.name + " v" +
      stored.version + ", this Landfall provides " + ours.name + " v" + ours.version +
      ": rebuild the database with `landfall build`");
  }
  if (stored.config_sha256 != configSha256(ours)) {
    throw std::runtime_error(
      "the " + stored.role + " backend " + stored.name +
      " in the database has a configuration this build reproduces differently: rebuild the "
      "database with `landfall build`");
  }
}

}  // namespace

LocalizationMap::LocalizationMap(db::Database database, std::size_t feature_cache_size)
: database_(std::move(database)), cache_size_(std::max<std::size_t>(feature_cache_size, 1))
{
}

std::shared_ptr<LocalizationMap> LocalizationMap::load(
  const std::string & path, std::size_t feature_cache_size)
{
  db::Database database = [&path] {
    try {
      return db::Database::open(path, db::Database::Access::kReadOnly);
    } catch (const db::DatabaseError & e) {
      throw std::runtime_error("cannot open map database " + path + ": " + e.what());
    }
  }();
  std::shared_ptr<LocalizationMap> map(
    new LocalizationMap(std::move(database), feature_cache_size));
  const db::Database & db = map->database_;
  map->path_ = path;
  map->content_sha256_ = db.meta("content_sha256").value_or("");

  const auto maps = db.maps();
  if (maps.size() != 1) {
    throw std::runtime_error(
      "the database holds " + std::to_string(maps.size()) + " occupancy maps, expected 1");
  }
  map->map_record_ = maps.front();
  map->grid_ = decodeGridPng(
    map->map_record_.grid, map->map_record_.resolution,
    {map->map_record_.origin_x, map->map_record_.origin_y, map->map_record_.origin_yaw});
  map->field_ = std::make_unique<LikelihoodField>(map->grid_);

  const auto cameras = db.cameras();
  if (cameras.size() != 1) {
    throw std::runtime_error(
      "the database holds " + std::to_string(cameras.size()) + " cameras, expected 1");
  }
  map->camera_record_ = cameras.front();
  const auto & c = map->camera_record_;
  const auto model = parseDistortionModel(c.model);
  if (!model || c.intrinsics.size() != 4) {
    throw std::runtime_error("the database's camera record is invalid");
  }
  map->camera_ = CameraModel(
    c.width, c.height, c.intrinsics[0], c.intrinsics[1], c.intrinsics[2], c.intrinsics[3], *model,
    c.distortion);

  // Backends: local features re-created from the stored configuration, bag of words from the
  // stored vocabulary.
  const auto backends = db.backends();
  const db::BackendRecord local = backendWithRole(backends, "local");
  BackendRegistry registry;
  registerBuiltinBackends(registry);
  try {
    map->local_features_ =
      registry.createLocalFeatures(local.name, parseBackendConfig(local.config));
  } catch (const std::invalid_argument & e) {
    throw std::runtime_error(
      std::string("the database's local feature backend is not available: ") + e.what());
  }
  requireSameBackend(local, map->local_features_->id());
  map->local_backend_id_ = local.id;

  const db::BackendRecord global = backendWithRole(backends, "global");
  if (global.name != "bow") {
    throw std::runtime_error(
      "the database uses global backend " + global.name + ", which this build doesn't provide");
  }
  const db::BackendRecord vocabulary_backend = backendWithRole(backends, "vocabulary");
  const auto vocabulary_record = db.vocabulary(vocabulary_backend.id);
  if (!vocabulary_record) {
    throw std::runtime_error("the database has no vocabulary");
  }
  auto vocabulary =
    std::make_shared<const VocabularyTree>(VocabularyTree::deserialize(vocabulary_record->data));
  map->descriptor_type_ = vocabulary->type();
  map->global_descriptor_ = std::make_unique<BowGlobalDescriptor>(vocabulary);
  requireSameBackend(global, map->global_descriptor_->id());

  auto index = std::make_unique<BowIndex>();
  std::size_t descriptors = 0;
  for (const auto & record : db.globalDescriptors(global.id)) {
    index->add(record.keyframe_id, db::decodeGlobalDescriptor(record));
    ++descriptors;
  }
  map->index_ = std::move(index);

  for (const auto & k : db.keyframes()) {
    map->keyframe_index_[k.id] = map->keyframes_.size();
    map->keyframes_.push_back({k.id, k.refined, k.T_map_cam});
  }
  if (map->keyframes_.empty()) {
    throw std::runtime_error("the database has no keyframes");
  }
  if (descriptors != map->keyframes_.size()) {
    throw std::runtime_error("the database's global descriptors don't cover every keyframe");
  }
  for (const auto & l : db.landmarks()) {
    map->landmarks_[l.id] = l.position;
  }
  for (const auto & e : db.covisibility()) {
    map->covisibility_[{e.keyframe_a, e.keyframe_b}] = e.shared;
  }
  return map;
}

const MapKeyframe * LocalizationMap::keyframe(int64_t id) const
{
  const auto it = keyframe_index_.find(id);
  return it == keyframe_index_.end() ? nullptr : &keyframes_[it->second];
}

const Vec3 * LocalizationMap::landmark(int64_t id) const
{
  const auto it = landmarks_.find(id);
  return it == landmarks_.end() ? nullptr : &it->second;
}

int LocalizationMap::covisibility(int64_t a, int64_t b) const
{
  const auto it = covisibility_.find(a < b ? std::make_pair(a, b) : std::make_pair(b, a));
  return it == covisibility_.end() ? 0 : it->second;
}

std::shared_ptr<const KeyframeFeatures> LocalizationMap::features(int64_t keyframe_id) const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  const auto cached = cache_index_.find(keyframe_id);
  if (cached != cache_index_.end()) {
    cache_.splice(cache_.begin(), cache_, cached->second);
    return cached->second->second;
  }
  const auto record = database_.localFeatures(keyframe_id, local_backend_id_);
  if (!record) {
    throw std::runtime_error(
      "the database has no local features for keyframe " + std::to_string(keyframe_id));
  }
  auto loaded = std::make_shared<KeyframeFeatures>();
  loaded->features = db::decodeLocalFeatures(*record);
  loaded->landmarks.assign(loaded->features.keypoints.size(), -1);
  for (const auto & o : database_.observations(keyframe_id)) {
    if (o.keypoint_index < 0 || o.keypoint_index >= static_cast<int>(loaded->landmarks.size())) {
      throw std::runtime_error(
        "corrupt observation of keyframe " + std::to_string(keyframe_id) + ": keypoint " +
        std::to_string(o.keypoint_index) + " doesn't exist");
    }
    loaded->landmarks[static_cast<std::size_t>(o.keypoint_index)] = o.landmark_id;
  }
  cache_.emplace_front(keyframe_id, loaded);
  cache_index_[keyframe_id] = cache_.begin();
  if (cache_.size() > cache_size_) {
    cache_index_.erase(cache_.back().first);
    cache_.pop_back();
  }
  return loaded;
}

}  // namespace landfall
