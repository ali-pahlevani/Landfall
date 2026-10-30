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

#ifndef LANDFALL_DB_DATABASE_HPP_
#define LANDFALL_DB_DATABASE_HPP_

#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "landfall/types.hpp"

struct sqlite3;

namespace landfall::db
{

/// Schema version this build reads and writes (PRAGMA user_version).
constexpr int kSchemaVersion = 1;
/// PRAGMA application_id of every Landfall database: 'LDFA'.
constexpr int32_t kApplicationId = 0x4C444641;

class DatabaseError : public std::runtime_error
{
public:
  enum class Code : uint8_t {
    kNotFound,       ///< the file doesn't exist
    kAlreadyExists,  ///< create() on an existing file
    kNotLandfall,    ///< not SQLite, or a SQLite file of another application
    kSchemaTooOld,   ///< older schema: run `landfall migrate`
    kSchemaTooNew,   ///< written by a newer Landfall
    kReadOnly,       ///< write attempted on a read-only connection
    kSqlite,         ///< any other SQLite error
  };

  DatabaseError(Code code, const std::string & message) : std::runtime_error(message), code_(code)
  {
  }
  Code code() const { return code_; }

private:
  Code code_;
};

/// Storage type of descriptor blobs.
enum class BlobType : uint8_t {
  kU8 = 0,
  kF32 = 1,
  kF16 = 2,
};

struct MapRecord
{
  int64_t id = 0;
  std::string yaml_sha256;
  std::string image_sha256;
  std::string grid_sha256;
  double resolution = 0.0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  double origin_yaw = 0.0;
  int width = 0;
  int height = 0;
  std::string yaml_text;
  std::vector<uint8_t> grid;  ///< PNG of the occupancy values (see grid/occupancy_grid.hpp)
};

struct CameraRecord
{
  int64_t id = 0;
  std::string frame_id;
  std::string model;  ///< distortion model name, as in CameraInfo
  int width = 0;
  int height = 0;
  std::vector<double> intrinsics;  ///< fx, fy, cx, cy
  std::vector<double> distortion;
  Pose3 T_base_cam = Pose3::Identity();
};

struct SessionRecord
{
  int64_t id = 0;
  std::string name;
  std::string source;       ///< bag or staging folder it was built from
  std::string pose_source;  ///< e.g. "tf_with_covariance"
  int64_t started_ns = 0;
  std::string notes;
};

struct KeyframeRecord
{
  /// Bits of `flags`.
  static constexpr uint32_t kFlagBaShiftAtBound = 1U << 0;

  int64_t id = 0;
  int64_t session_id = 0;
  int64_t camera_id = 0;
  int64_t stamp_ns = 0;
  Pose2 prior;                           ///< base pose recorded from localization
  Mat3 prior_covariance = Mat3::Zero();  ///< x, y, yaw
  Pose2 refined;                         ///< after the optional pose-prior BA
  Pose3 T_map_cam = Pose3::Identity();
  uint32_t flags = 0;
  std::string quality_json = "{}";
  std::vector<uint8_t> thumbnail_jpeg;
};

struct BackendRecord
{
  int64_t id = 0;
  std::string role;  ///< global, local, matcher or vocabulary
  std::string name;
  std::string version;
  std::string model_sha256;
  std::string config;
  std::string config_sha256;
};

struct LocalFeaturesRecord
{
  int64_t keyframe_id = 0;
  int64_t backend_id = 0;
  std::vector<Keypoint> keypoints;
  BlobType descriptor_type = BlobType::kU8;
  int descriptor_dim = 0;  ///< elements per row (bytes for kU8)
  std::vector<uint8_t> descriptors;
};

struct GlobalDescriptorRecord
{
  int64_t keyframe_id = 0;
  int64_t backend_id = 0;
  int dim = 0;
  BlobType type = BlobType::kF32;
  std::vector<uint8_t> data;
};

struct LandmarkRecord
{
  int64_t id = 0;
  Vec3 position = Vec3::Zero();
  int track_length = 0;
  double mean_reprojection_px = 0.0;
  double max_angle_deg = 0.0;
  std::vector<uint8_t> descriptor;
};

struct ObservationRecord
{
  int64_t keyframe_id = 0;
  int keypoint_index = 0;
  int64_t landmark_id = 0;
};

struct CovisibilityRecord
{
  int64_t keyframe_a = 0;  ///< always < keyframe_b
  int64_t keyframe_b = 0;
  int shared = 0;
};

struct VocabularyRecord
{
  int64_t backend_id = 0;
  int branching = 0;
  int depth = 0;
  std::vector<uint8_t> data;
};

struct PcaRecord
{
  int64_t backend_id = 0;
  std::vector<uint8_t> mean;
  std::vector<uint8_t> components;
  int out_dim = 0;
};

/// A Landfall map database (one SQLite file). Not thread-safe; use one instance per thread.
///
/// Every write goes through the caller's transactions (see Transaction); without one, each insert
/// commits on its own, which is correct but slow for bulk loads.
class Database
{
public:
  enum class Access : uint8_t {
    kReadOnly,
    kReadWrite,
  };

  /// Creates a new database with the current schema. Fails if the file exists.
  static Database create(const std::string & path);
  /// Opens an existing database and checks application id and schema version.
  static Database open(const std::string & path, Access access);

  Database(Database &&) noexcept;
  Database & operator=(Database &&) noexcept;
  Database(const Database &) = delete;
  Database & operator=(const Database &) = delete;
  ~Database();

  const std::string & path() const { return path_; }
  bool readOnly() const { return read_only_; }

  /// RAII transaction: rolls back unless commit() is called.
  class Transaction
  {
  public:
    explicit Transaction(Database & database);
    Transaction(const Transaction &) = delete;
    Transaction & operator=(const Transaction &) = delete;
    ~Transaction();
    void commit();

  private:
    Database * database_;
    bool done_ = false;
  };

  void setMeta(const std::string & key, const std::string & value);
  std::optional<std::string> meta(const std::string & key) const;

  int64_t insertMap(const MapRecord & record);
  std::vector<MapRecord> maps() const;

  int64_t insertCamera(const CameraRecord & record);
  std::vector<CameraRecord> cameras() const;

  int64_t insertSession(const SessionRecord & record);
  std::vector<SessionRecord> sessions() const;

  int64_t insertKeyframe(const KeyframeRecord & record);
  /// Updates the refined pose, camera pose and flags of an existing keyframe.
  void updateKeyframePose(
    int64_t id, const Pose2 & refined, const Pose3 & T_map_cam, uint32_t flags);
  std::vector<KeyframeRecord> keyframes() const;

  int64_t insertBackend(const BackendRecord & record);
  std::vector<BackendRecord> backends() const;

  void insertLocalFeatures(const LocalFeaturesRecord & record);
  std::optional<LocalFeaturesRecord> localFeatures(int64_t keyframe_id, int64_t backend_id) const;

  void insertGlobalDescriptor(const GlobalDescriptorRecord & record);
  std::vector<GlobalDescriptorRecord> globalDescriptors(int64_t backend_id) const;

  int64_t insertLandmark(const LandmarkRecord & record);
  std::vector<LandmarkRecord> landmarks() const;

  void insertObservation(const ObservationRecord & record);
  std::vector<ObservationRecord> observations(int64_t keyframe_id) const;
  std::vector<ObservationRecord> allObservations() const;

  void insertCovisibility(const CovisibilityRecord & record);
  std::vector<CovisibilityRecord> covisibility() const;

  void insertVocabulary(const VocabularyRecord & record);
  std::optional<VocabularyRecord> vocabulary(int64_t backend_id) const;

  void insertPca(const PcaRecord & record);
  std::optional<PcaRecord> pca(int64_t backend_id) const;

  /// Deterministic SHA-256 over every table except `meta` (rows in primary-key order). Equal
  /// inputs and seeds produce equal hashes; stored as meta `content_sha256` by the builder.
  std::string contentSha256() const;
  /// PRAGMA integrity_check; returns "ok" or SQLite's report.
  std::string integrityCheck() const;

private:
  Database(sqlite3 * handle, std::string path, bool read_only);
  void exec(const std::string & sql) const;

  sqlite3 * handle_ = nullptr;
  std::string path_;
  bool read_only_ = false;
};

/// Reads application id and schema version without validating them. Throws kNotFound/kNotLandfall.
int readSchemaVersion(const std::string & path);

}  // namespace landfall::db

#endif  // LANDFALL_DB_DATABASE_HPP_
