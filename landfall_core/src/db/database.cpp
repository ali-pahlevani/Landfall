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

#include "landfall/db/database.hpp"

#include <filesystem>
#include <memory>
#include <utility>

#include "landfall/util/sha256.hpp"
#include "landfall/version.hpp"
#include "sqlite_statement.hpp"

// Blobs hold raw little-endian values; every supported target (x86_64, aarch64) is little-endian.
static_assert(
  __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "Landfall database blobs assume a little-endian host");

namespace landfall::db
{
namespace
{

using detail::check;
using detail::Statement;

// Schema version 1. Changing it requires a migration step (migrations.cpp) and kSchemaVersion + 1.
constexpr const char * kSchemaV1 = R"sql(
CREATE TABLE meta (
  key TEXT PRIMARY KEY NOT NULL,
  value TEXT NOT NULL);
CREATE TABLE maps (
  id INTEGER PRIMARY KEY,
  yaml_sha256 TEXT NOT NULL,
  image_sha256 TEXT NOT NULL,
  grid_sha256 TEXT NOT NULL,
  resolution REAL NOT NULL,
  origin_x REAL NOT NULL,
  origin_y REAL NOT NULL,
  origin_yaw REAL NOT NULL,
  width INTEGER NOT NULL,
  height INTEGER NOT NULL,
  yaml_text TEXT NOT NULL,
  grid BLOB NOT NULL);
CREATE TABLE cameras (
  id INTEGER PRIMARY KEY,
  frame_id TEXT NOT NULL,
  model TEXT NOT NULL,
  width INTEGER NOT NULL,
  height INTEGER NOT NULL,
  intrinsics BLOB NOT NULL,
  distortion BLOB NOT NULL,
  T_base_cam BLOB NOT NULL);
CREATE TABLE sessions (
  id INTEGER PRIMARY KEY,
  name TEXT NOT NULL,
  source TEXT NOT NULL,
  pose_source TEXT NOT NULL,
  started_ns INTEGER NOT NULL,
  notes TEXT NOT NULL);
CREATE TABLE keyframes (
  id INTEGER PRIMARY KEY,
  session_id INTEGER NOT NULL REFERENCES sessions(id),
  camera_id INTEGER NOT NULL REFERENCES cameras(id),
  stamp_ns INTEGER NOT NULL,
  x REAL NOT NULL,
  y REAL NOT NULL,
  yaw REAL NOT NULL,
  cov BLOB NOT NULL,
  rx REAL NOT NULL,
  ry REAL NOT NULL,
  ryaw REAL NOT NULL,
  T_map_cam BLOB NOT NULL,
  flags INTEGER NOT NULL,
  quality TEXT NOT NULL,
  thumbnail BLOB NOT NULL);
CREATE INDEX keyframes_session ON keyframes(session_id);
CREATE TABLE backends (
  id INTEGER PRIMARY KEY,
  role TEXT NOT NULL CHECK (role IN ('global', 'local', 'matcher', 'vocabulary')),
  name TEXT NOT NULL,
  version TEXT NOT NULL,
  model_sha256 TEXT NOT NULL,
  config TEXT NOT NULL,
  config_sha256 TEXT NOT NULL);
CREATE TABLE local_features (
  keyframe_id INTEGER NOT NULL REFERENCES keyframes(id),
  backend_id INTEGER NOT NULL REFERENCES backends(id),
  num INTEGER NOT NULL,
  keypoints BLOB NOT NULL,
  desc_dtype INTEGER NOT NULL,
  desc_dim INTEGER NOT NULL,
  descriptors BLOB NOT NULL,
  PRIMARY KEY (keyframe_id, backend_id)) WITHOUT ROWID;
CREATE TABLE global_descriptors (
  keyframe_id INTEGER NOT NULL REFERENCES keyframes(id),
  backend_id INTEGER NOT NULL REFERENCES backends(id),
  dim INTEGER NOT NULL,
  dtype INTEGER NOT NULL,
  data BLOB NOT NULL,
  PRIMARY KEY (keyframe_id, backend_id)) WITHOUT ROWID;
CREATE TABLE landmarks (
  id INTEGER PRIMARY KEY,
  x REAL NOT NULL,
  y REAL NOT NULL,
  z REAL NOT NULL,
  track_length INTEGER NOT NULL,
  mean_reproj_px REAL NOT NULL,
  max_angle_deg REAL NOT NULL,
  descriptor BLOB NOT NULL);
CREATE TABLE observations (
  keyframe_id INTEGER NOT NULL REFERENCES keyframes(id),
  keypoint_idx INTEGER NOT NULL,
  landmark_id INTEGER NOT NULL REFERENCES landmarks(id),
  PRIMARY KEY (keyframe_id, keypoint_idx)) WITHOUT ROWID;
CREATE INDEX observations_landmark ON observations(landmark_id);
CREATE TABLE covisibility (
  kf_a INTEGER NOT NULL REFERENCES keyframes(id),
  kf_b INTEGER NOT NULL REFERENCES keyframes(id),
  shared INTEGER NOT NULL,
  PRIMARY KEY (kf_a, kf_b),
  CHECK (kf_a < kf_b)) WITHOUT ROWID;
CREATE TABLE vocabulary (
  backend_id INTEGER PRIMARY KEY REFERENCES backends(id),
  k INTEGER NOT NULL,
  depth INTEGER NOT NULL,
  data BLOB NOT NULL);
CREATE TABLE pca (
  backend_id INTEGER PRIMARY KEY REFERENCES backends(id),
  mean BLOB NOT NULL,
  components BLOB NOT NULL,
  out_dim INTEGER NOT NULL);
)sql";

// Tables covered by contentSha256(), with their primary-key order.
constexpr std::pair<const char *, const char *> kContentTables[] = {
  {"maps", "id"},
  {"cameras", "id"},
  {"sessions", "id"},
  {"keyframes", "id"},
  {"backends", "id"},
  {"local_features", "keyframe_id, backend_id"},
  {"global_descriptors", "keyframe_id, backend_id"},
  {"landmarks", "id"},
  {"observations", "keyframe_id, keypoint_idx"},
  {"covisibility", "kf_a, kf_b"},
  {"vocabulary", "backend_id"},
  {"pca", "backend_id"},
};

template <typename T>
std::vector<uint8_t> toBlob(const T * values, std::size_t count)
{
  std::vector<uint8_t> blob(sizeof(T) * count);
  if (count > 0) {
    std::memcpy(blob.data(), values, blob.size());
  }
  return blob;
}

template <typename T>
std::vector<T> fromBlob(const std::vector<uint8_t> & blob, const char * what)
{
  if (blob.size() % sizeof(T) != 0) {
    throw DatabaseError(DatabaseError::Code::kSqlite, std::string("corrupt blob: ") + what);
  }
  std::vector<T> values(blob.size() / sizeof(T));
  if (!values.empty()) {
    std::memcpy(values.data(), blob.data(), blob.size());
  }
  return values;
}

// Poses are stored as the row-major 3x4 matrix [R | t], so they round-trip bit-exactly.
std::vector<uint8_t> poseBlob(const Pose3 & pose)
{
  const Eigen::Matrix<double, 3, 4, Eigen::RowMajor> matrix = pose.affine();
  return toBlob(matrix.data(), 12);
}

Pose3 poseFromBlob(const std::vector<uint8_t> & blob)
{
  const auto v = fromBlob<double>(blob, "pose");
  if (v.size() != 12) {
    throw DatabaseError(DatabaseError::Code::kSqlite, "corrupt blob: pose");
  }
  Pose3 pose = Pose3::Identity();
  pose.affine() = Eigen::Map<const Eigen::Matrix<double, 3, 4, Eigen::RowMajor>>(v.data());
  return pose;
}

std::vector<uint8_t> keypointBlob(const std::vector<Keypoint> & keypoints)
{
  std::vector<float> values;
  values.reserve(5 * keypoints.size());
  for (const auto & k : keypoints) {
    values.insert(values.end(), {k.x, k.y, k.size, k.angle, k.score});
  }
  return toBlob(values.data(), values.size());
}

std::vector<Keypoint> keypointsFromBlob(const std::vector<uint8_t> & blob)
{
  const auto v = fromBlob<float>(blob, "keypoints");
  if (v.size() % 5 != 0) {
    throw DatabaseError(DatabaseError::Code::kSqlite, "corrupt blob: keypoints");
  }
  std::vector<Keypoint> keypoints(v.size() / 5);
  for (std::size_t i = 0; i < keypoints.size(); ++i) {
    keypoints[i] = {v[5 * i], v[5 * i + 1], v[5 * i + 2], v[5 * i + 3], v[5 * i + 4]};
  }
  return keypoints;
}

int pragmaInt(sqlite3 * handle, const char * pragma)
{
  Statement statement(handle, std::string("PRAGMA ") + pragma);
  return statement.step() ? static_cast<int>(statement.getInt(0)) : 0;
}

}  // namespace

namespace detail
{

void throwSqlite(sqlite3 * handle, int rc, const std::string & context)
{
  const std::string message =
    context + ": " + (handle != nullptr ? sqlite3_errmsg(handle) : sqlite3_errstr(rc));
  const int primary = rc & 0xff;
  if (primary == SQLITE_READONLY) {
    throw DatabaseError(DatabaseError::Code::kReadOnly, message);
  }
  if (primary == SQLITE_NOTADB) {
    throw DatabaseError(DatabaseError::Code::kNotLandfall, message);
  }
  throw DatabaseError(DatabaseError::Code::kSqlite, message);
}

}  // namespace detail

Database::Database(sqlite3 * handle, std::string path, bool read_only)
: handle_(handle), path_(std::move(path)), read_only_(read_only)
{
}

Database::Database(Database && other) noexcept
: handle_(std::exchange(other.handle_, nullptr)),
  path_(std::move(other.path_)),
  read_only_(other.read_only_)
{
}

Database & Database::operator=(Database && other) noexcept
{
  if (this != &other) {
    sqlite3_close(handle_);
    handle_ = std::exchange(other.handle_, nullptr);
    path_ = std::move(other.path_);
    read_only_ = other.read_only_;
  }
  return *this;
}

Database::~Database() { sqlite3_close(handle_); }

void Database::exec(const std::string & sql) const
{
  char * error = nullptr;
  const int rc = sqlite3_exec(handle_, sql.c_str(), nullptr, nullptr, &error);
  if (rc != SQLITE_OK) {
    const std::string message = error != nullptr ? error : sqlite3_errstr(rc);
    sqlite3_free(error);
    detail::throwSqlite(nullptr, rc, sql.substr(0, 60) + ": " + message);
  }
}

Database Database::create(const std::string & path)
{
  if (std::filesystem::exists(path)) {
    throw DatabaseError(DatabaseError::Code::kAlreadyExists, "database already exists: " + path);
  }
  sqlite3 * handle = nullptr;
  const int rc =
    sqlite3_open_v2(path.c_str(), &handle, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE, nullptr);
  Database database(handle, path, false);
  check(handle, rc, "create " + path);
  database.exec("PRAGMA foreign_keys = ON");
  Transaction transaction(database);
  database.exec(kSchemaV1);
  database.exec("PRAGMA application_id = " + std::to_string(kApplicationId));
  database.exec("PRAGMA user_version = " + std::to_string(kSchemaVersion));
  database.setMeta("schema_version", std::to_string(kSchemaVersion));
  database.setMeta("landfall_version", LANDFALL_VERSION_STRING);
  transaction.commit();
  return database;
}

int readSchemaVersion(const std::string & path)
{
  if (!std::filesystem::is_regular_file(path)) {
    throw DatabaseError(DatabaseError::Code::kNotFound, "database not found: " + path);
  }
  sqlite3 * handle = nullptr;
  const int rc = sqlite3_open_v2(path.c_str(), &handle, SQLITE_OPEN_READONLY, nullptr);
  const std::unique_ptr<sqlite3, int (*)(sqlite3 *)> guard(handle, sqlite3_close);
  check(handle, rc, "open " + path);
  if (pragmaInt(handle, "application_id") != kApplicationId) {
    throw DatabaseError(
      DatabaseError::Code::kNotLandfall, "not a Landfall database (wrong application id): " + path);
  }
  return pragmaInt(handle, "user_version");
}

Database Database::open(const std::string & path, Access access)
{
  const int version = readSchemaVersion(path);
  if (version < kSchemaVersion) {
    throw DatabaseError(
      DatabaseError::Code::kSchemaTooOld, path + " has schema version " + std::to_string(version) +
                                            ", this build needs " + std::to_string(kSchemaVersion) +
                                            "; run `landfall migrate " + path + "`");
  }
  if (version > kSchemaVersion) {
    throw DatabaseError(
      DatabaseError::Code::kSchemaTooNew, path + " has schema version " + std::to_string(version) +
                                            ", newer than this build (" +
                                            std::to_string(kSchemaVersion) + "); update Landfall");
  }
  const bool read_only = access == Access::kReadOnly;
  sqlite3 * handle = nullptr;
  const int rc = sqlite3_open_v2(
    path.c_str(), &handle, read_only ? SQLITE_OPEN_READONLY : SQLITE_OPEN_READWRITE, nullptr);
  Database database(handle, path, read_only);
  check(handle, rc, "open " + path);
  database.exec("PRAGMA foreign_keys = ON");
  return database;
}

Database::Transaction::Transaction(Database & database) : database_(&database)
{
  database_->exec("BEGIN");
}

Database::Transaction::~Transaction()
{
  if (!done_) {
    sqlite3_exec(database_->handle_, "ROLLBACK", nullptr, nullptr, nullptr);
  }
}

void Database::Transaction::commit()
{
  database_->exec("COMMIT");
  done_ = true;
}

void Database::setMeta(const std::string & key, const std::string & value)
{
  Statement s(handle_, "INSERT OR REPLACE INTO meta (key, value) VALUES (?, ?)");
  s.bindText(1, key);
  s.bindText(2, value);
  s.run();
}

std::optional<std::string> Database::meta(const std::string & key) const
{
  Statement s(handle_, "SELECT value FROM meta WHERE key = ?");
  s.bindText(1, key);
  if (!s.step()) {
    return std::nullopt;
  }
  return s.getText(0);
}

int64_t Database::insertMap(const MapRecord & r)
{
  Statement s(
    handle_,
    "INSERT INTO maps (yaml_sha256, image_sha256, grid_sha256, resolution, origin_x, origin_y, "
    "origin_yaw, width, height, yaml_text, grid) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)");
  s.bindText(1, r.yaml_sha256);
  s.bindText(2, r.image_sha256);
  s.bindText(3, r.grid_sha256);
  s.bindDouble(4, r.resolution);
  s.bindDouble(5, r.origin_x);
  s.bindDouble(6, r.origin_y);
  s.bindDouble(7, r.origin_yaw);
  s.bindInt(8, r.width);
  s.bindInt(9, r.height);
  s.bindText(10, r.yaml_text);
  s.bindBlob(11, r.grid);
  s.run();
  return sqlite3_last_insert_rowid(handle_);
}

std::vector<MapRecord> Database::maps() const
{
  Statement s(
    handle_,
    "SELECT id, yaml_sha256, image_sha256, grid_sha256, resolution, origin_x, origin_y, "
    "origin_yaw, width, height, yaml_text, grid FROM maps ORDER BY id");
  std::vector<MapRecord> result;
  while (s.step()) {
    MapRecord r;
    r.id = s.getInt(0);
    r.yaml_sha256 = s.getText(1);
    r.image_sha256 = s.getText(2);
    r.grid_sha256 = s.getText(3);
    r.resolution = s.getDouble(4);
    r.origin_x = s.getDouble(5);
    r.origin_y = s.getDouble(6);
    r.origin_yaw = s.getDouble(7);
    r.width = static_cast<int>(s.getInt(8));
    r.height = static_cast<int>(s.getInt(9));
    r.yaml_text = s.getText(10);
    r.grid = s.getBlob(11);
    result.push_back(std::move(r));
  }
  return result;
}

int64_t Database::insertCamera(const CameraRecord & r)
{
  Statement s(
    handle_,
    "INSERT INTO cameras (frame_id, model, width, height, intrinsics, distortion, T_base_cam) "
    "VALUES (?, ?, ?, ?, ?, ?, ?)");
  s.bindText(1, r.frame_id);
  s.bindText(2, r.model);
  s.bindInt(3, r.width);
  s.bindInt(4, r.height);
  s.bindBlob(5, toBlob(r.intrinsics.data(), r.intrinsics.size()));
  s.bindBlob(6, toBlob(r.distortion.data(), r.distortion.size()));
  s.bindBlob(7, poseBlob(r.T_base_cam));
  s.run();
  return sqlite3_last_insert_rowid(handle_);
}

std::vector<CameraRecord> Database::cameras() const
{
  Statement s(
    handle_,
    "SELECT id, frame_id, model, width, height, intrinsics, distortion, T_base_cam FROM cameras "
    "ORDER BY id");
  std::vector<CameraRecord> result;
  while (s.step()) {
    CameraRecord r;
    r.id = s.getInt(0);
    r.frame_id = s.getText(1);
    r.model = s.getText(2);
    r.width = static_cast<int>(s.getInt(3));
    r.height = static_cast<int>(s.getInt(4));
    r.intrinsics = fromBlob<double>(s.getBlob(5), "intrinsics");
    r.distortion = fromBlob<double>(s.getBlob(6), "distortion");
    r.T_base_cam = poseFromBlob(s.getBlob(7));
    result.push_back(std::move(r));
  }
  return result;
}

int64_t Database::insertSession(const SessionRecord & r)
{
  Statement s(
    handle_,
    "INSERT INTO sessions (name, source, pose_source, started_ns, notes) VALUES (?, ?, ?, ?, ?)");
  s.bindText(1, r.name);
  s.bindText(2, r.source);
  s.bindText(3, r.pose_source);
  s.bindInt(4, r.started_ns);
  s.bindText(5, r.notes);
  s.run();
  return sqlite3_last_insert_rowid(handle_);
}

std::vector<SessionRecord> Database::sessions() const
{
  Statement s(
    handle_, "SELECT id, name, source, pose_source, started_ns, notes FROM sessions ORDER BY id");
  std::vector<SessionRecord> result;
  while (s.step()) {
    result.push_back(
      {s.getInt(0), s.getText(1), s.getText(2), s.getText(3), s.getInt(4), s.getText(5)});
  }
  return result;
}

int64_t Database::insertKeyframe(const KeyframeRecord & r)
{
  Statement s(
    handle_,
    "INSERT INTO keyframes (session_id, camera_id, stamp_ns, x, y, yaw, cov, rx, ry, ryaw, "
    "T_map_cam, flags, quality, thumbnail) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)");
  const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> covariance = r.prior_covariance;
  s.bindInt(1, r.session_id);
  s.bindInt(2, r.camera_id);
  s.bindInt(3, r.stamp_ns);
  s.bindDouble(4, r.prior.x);
  s.bindDouble(5, r.prior.y);
  s.bindDouble(6, r.prior.yaw);
  s.bindBlob(7, toBlob(covariance.data(), 9));
  s.bindDouble(8, r.refined.x);
  s.bindDouble(9, r.refined.y);
  s.bindDouble(10, r.refined.yaw);
  s.bindBlob(11, poseBlob(r.T_map_cam));
  s.bindInt(12, r.flags);
  s.bindText(13, r.quality_json);
  s.bindBlob(14, r.thumbnail_jpeg);
  s.run();
  return sqlite3_last_insert_rowid(handle_);
}

void Database::updateKeyframePose(
  int64_t id, const Pose2 & refined, const Pose3 & T_map_cam, uint32_t flags)
{
  Statement s(
    handle_,
    "UPDATE keyframes SET rx = ?, ry = ?, ryaw = ?, T_map_cam = ?, flags = ? WHERE id = ?");
  s.bindDouble(1, refined.x);
  s.bindDouble(2, refined.y);
  s.bindDouble(3, refined.yaw);
  s.bindBlob(4, poseBlob(T_map_cam));
  s.bindInt(5, flags);
  s.bindInt(6, id);
  s.run();
}

std::vector<KeyframeRecord> Database::keyframes() const
{
  Statement s(
    handle_,
    "SELECT id, session_id, camera_id, stamp_ns, x, y, yaw, cov, rx, ry, ryaw, T_map_cam, flags, "
    "quality, thumbnail FROM keyframes ORDER BY id");
  std::vector<KeyframeRecord> result;
  while (s.step()) {
    KeyframeRecord r;
    r.id = s.getInt(0);
    r.session_id = s.getInt(1);
    r.camera_id = s.getInt(2);
    r.stamp_ns = s.getInt(3);
    r.prior = {s.getDouble(4), s.getDouble(5), s.getDouble(6)};
    const auto covariance = fromBlob<double>(s.getBlob(7), "covariance");
    if (covariance.size() != 9) {
      throw DatabaseError(DatabaseError::Code::kSqlite, "corrupt blob: covariance");
    }
    r.prior_covariance =
      Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(covariance.data());
    r.refined = {s.getDouble(8), s.getDouble(9), s.getDouble(10)};
    r.T_map_cam = poseFromBlob(s.getBlob(11));
    r.flags = static_cast<uint32_t>(s.getInt(12));
    r.quality_json = s.getText(13);
    r.thumbnail_jpeg = s.getBlob(14);
    result.push_back(std::move(r));
  }
  return result;
}

int64_t Database::insertBackend(const BackendRecord & r)
{
  Statement s(
    handle_,
    "INSERT INTO backends (role, name, version, model_sha256, config, config_sha256) VALUES "
    "(?, ?, ?, ?, ?, ?)");
  s.bindText(1, r.role);
  s.bindText(2, r.name);
  s.bindText(3, r.version);
  s.bindText(4, r.model_sha256);
  s.bindText(5, r.config);
  s.bindText(6, r.config_sha256);
  s.run();
  return sqlite3_last_insert_rowid(handle_);
}

std::vector<BackendRecord> Database::backends() const
{
  Statement s(
    handle_,
    "SELECT id, role, name, version, model_sha256, config, config_sha256 FROM backends ORDER BY "
    "id");
  std::vector<BackendRecord> result;
  while (s.step()) {
    result.push_back(
      {s.getInt(0), s.getText(1), s.getText(2), s.getText(3), s.getText(4), s.getText(5),
       s.getText(6)});
  }
  return result;
}

void Database::insertLocalFeatures(const LocalFeaturesRecord & r)
{
  Statement s(
    handle_,
    "INSERT INTO local_features (keyframe_id, backend_id, num, keypoints, desc_dtype, desc_dim, "
    "descriptors) VALUES (?, ?, ?, ?, ?, ?, ?)");
  s.bindInt(1, r.keyframe_id);
  s.bindInt(2, r.backend_id);
  s.bindInt(3, static_cast<int64_t>(r.keypoints.size()));
  s.bindBlob(4, keypointBlob(r.keypoints));
  s.bindInt(5, static_cast<int64_t>(r.descriptor_type));
  s.bindInt(6, r.descriptor_dim);
  s.bindBlob(7, r.descriptors);
  s.run();
}

std::optional<LocalFeaturesRecord> Database::localFeatures(
  int64_t keyframe_id, int64_t backend_id) const
{
  Statement s(
    handle_,
    "SELECT keypoints, desc_dtype, desc_dim, descriptors FROM local_features "
    "WHERE keyframe_id = ? AND backend_id = ?");
  s.bindInt(1, keyframe_id);
  s.bindInt(2, backend_id);
  if (!s.step()) {
    return std::nullopt;
  }
  LocalFeaturesRecord r;
  r.keyframe_id = keyframe_id;
  r.backend_id = backend_id;
  r.keypoints = keypointsFromBlob(s.getBlob(0));
  r.descriptor_type = static_cast<BlobType>(s.getInt(1));
  r.descriptor_dim = static_cast<int>(s.getInt(2));
  r.descriptors = s.getBlob(3);
  return r;
}

void Database::insertGlobalDescriptor(const GlobalDescriptorRecord & r)
{
  Statement s(
    handle_,
    "INSERT INTO global_descriptors (keyframe_id, backend_id, dim, dtype, data) VALUES "
    "(?, ?, ?, ?, ?)");
  s.bindInt(1, r.keyframe_id);
  s.bindInt(2, r.backend_id);
  s.bindInt(3, r.dim);
  s.bindInt(4, static_cast<int64_t>(r.type));
  s.bindBlob(5, r.data);
  s.run();
}

std::vector<GlobalDescriptorRecord> Database::globalDescriptors(int64_t backend_id) const
{
  Statement s(
    handle_,
    "SELECT keyframe_id, dim, dtype, data FROM global_descriptors WHERE backend_id = ? "
    "ORDER BY keyframe_id");
  s.bindInt(1, backend_id);
  std::vector<GlobalDescriptorRecord> result;
  while (s.step()) {
    result.push_back(
      {s.getInt(0), backend_id, static_cast<int>(s.getInt(1)), static_cast<BlobType>(s.getInt(2)),
       s.getBlob(3)});
  }
  return result;
}

int64_t Database::insertLandmark(const LandmarkRecord & r)
{
  Statement s(
    handle_,
    "INSERT INTO landmarks (x, y, z, track_length, mean_reproj_px, max_angle_deg, descriptor) "
    "VALUES (?, ?, ?, ?, ?, ?, ?)");
  s.bindDouble(1, r.position.x());
  s.bindDouble(2, r.position.y());
  s.bindDouble(3, r.position.z());
  s.bindInt(4, r.track_length);
  s.bindDouble(5, r.mean_reprojection_px);
  s.bindDouble(6, r.max_angle_deg);
  s.bindBlob(7, r.descriptor);
  s.run();
  return sqlite3_last_insert_rowid(handle_);
}

std::vector<LandmarkRecord> Database::landmarks() const
{
  Statement s(
    handle_,
    "SELECT id, x, y, z, track_length, mean_reproj_px, max_angle_deg, descriptor FROM landmarks "
    "ORDER BY id");
  std::vector<LandmarkRecord> result;
  while (s.step()) {
    LandmarkRecord r;
    r.id = s.getInt(0);
    r.position = Vec3(s.getDouble(1), s.getDouble(2), s.getDouble(3));
    r.track_length = static_cast<int>(s.getInt(4));
    r.mean_reprojection_px = s.getDouble(5);
    r.max_angle_deg = s.getDouble(6);
    r.descriptor = s.getBlob(7);
    result.push_back(std::move(r));
  }
  return result;
}

void Database::insertObservation(const ObservationRecord & r)
{
  Statement s(
    handle_, "INSERT INTO observations (keyframe_id, keypoint_idx, landmark_id) VALUES (?, ?, ?)");
  s.bindInt(1, r.keyframe_id);
  s.bindInt(2, r.keypoint_index);
  s.bindInt(3, r.landmark_id);
  s.run();
}

std::vector<ObservationRecord> Database::observations(int64_t keyframe_id) const
{
  Statement s(
    handle_,
    "SELECT keypoint_idx, landmark_id FROM observations WHERE keyframe_id = ? "
    "ORDER BY keypoint_idx");
  s.bindInt(1, keyframe_id);
  std::vector<ObservationRecord> result;
  while (s.step()) {
    result.push_back({keyframe_id, static_cast<int>(s.getInt(0)), s.getInt(1)});
  }
  return result;
}

std::vector<ObservationRecord> Database::allObservations() const
{
  Statement s(
    handle_,
    "SELECT keyframe_id, keypoint_idx, landmark_id FROM observations "
    "ORDER BY keyframe_id, keypoint_idx");
  std::vector<ObservationRecord> result;
  while (s.step()) {
    result.push_back({s.getInt(0), static_cast<int>(s.getInt(1)), s.getInt(2)});
  }
  return result;
}

void Database::insertCovisibility(const CovisibilityRecord & r)
{
  Statement s(handle_, "INSERT INTO covisibility (kf_a, kf_b, shared) VALUES (?, ?, ?)");
  s.bindInt(1, r.keyframe_a);
  s.bindInt(2, r.keyframe_b);
  s.bindInt(3, r.shared);
  s.run();
}

std::vector<CovisibilityRecord> Database::covisibility() const
{
  Statement s(handle_, "SELECT kf_a, kf_b, shared FROM covisibility ORDER BY kf_a, kf_b");
  std::vector<CovisibilityRecord> result;
  while (s.step()) {
    result.push_back({s.getInt(0), s.getInt(1), static_cast<int>(s.getInt(2))});
  }
  return result;
}

void Database::insertVocabulary(const VocabularyRecord & r)
{
  Statement s(handle_, "INSERT INTO vocabulary (backend_id, k, depth, data) VALUES (?, ?, ?, ?)");
  s.bindInt(1, r.backend_id);
  s.bindInt(2, r.branching);
  s.bindInt(3, r.depth);
  s.bindBlob(4, r.data);
  s.run();
}

std::optional<VocabularyRecord> Database::vocabulary(int64_t backend_id) const
{
  Statement s(handle_, "SELECT k, depth, data FROM vocabulary WHERE backend_id = ?");
  s.bindInt(1, backend_id);
  if (!s.step()) {
    return std::nullopt;
  }
  return VocabularyRecord{
    backend_id, static_cast<int>(s.getInt(0)), static_cast<int>(s.getInt(1)), s.getBlob(2)};
}

void Database::insertPca(const PcaRecord & r)
{
  Statement s(
    handle_, "INSERT INTO pca (backend_id, mean, components, out_dim) VALUES (?, ?, ?, ?)");
  s.bindInt(1, r.backend_id);
  s.bindBlob(2, r.mean);
  s.bindBlob(3, r.components);
  s.bindInt(4, r.out_dim);
  s.run();
}

std::optional<PcaRecord> Database::pca(int64_t backend_id) const
{
  Statement s(handle_, "SELECT mean, components, out_dim FROM pca WHERE backend_id = ?");
  s.bindInt(1, backend_id);
  if (!s.step()) {
    return std::nullopt;
  }
  return PcaRecord{backend_id, s.getBlob(0), s.getBlob(1), static_cast<int>(s.getInt(2))};
}

std::string Database::contentSha256() const
{
  Sha256 sha;
  for (const auto & [table, order] : kContentTables) {
    sha.update(table, std::strlen(table) + 1);
    Statement s(handle_, std::string("SELECT * FROM ") + table + " ORDER BY " + order);
    while (s.step()) {
      for (int column = 0; column < s.columnCount(); ++column) {
        const auto type = static_cast<uint8_t>(s.columnType(column));
        sha.update(&type, 1);
        if (type == SQLITE_INTEGER) {
          const int64_t value = s.getInt(column);
          sha.update(&value, sizeof(value));
        } else if (type == SQLITE_FLOAT) {
          const double value = s.getDouble(column);
          sha.update(&value, sizeof(value));
        } else if (type == SQLITE_TEXT || type == SQLITE_BLOB) {
          const auto bytes = s.getBlob(column);
          const uint64_t size = bytes.size();
          sha.update(&size, sizeof(size));
          sha.update(bytes.data(), bytes.size());
        }
      }
    }
  }
  return sha.finishHex();
}

std::string Database::integrityCheck() const
{
  Statement s(handle_, "PRAGMA integrity_check");
  std::string report;
  while (s.step()) {
    report += (report.empty() ? "" : "\n") + s.getText(0);
  }
  return report;
}

}  // namespace landfall::db
