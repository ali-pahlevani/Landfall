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
#include <sqlite3.h>

#include <cmath>
#include <cstring>
#include <filesystem>
#include <random>
#include <string>
#include <vector>

#include "landfall/db/database.hpp"
#include "landfall/db/migrations.hpp"
#include "landfall/util/sha256.hpp"
#include "test_utils.hpp"

namespace db = landfall::db;
using Code = db::DatabaseError::Code;

namespace
{

// Random doubles covering the full bit range, so round trips are checked bit for bit.
class RandomData
{
public:
  double value()
  {
    double d = 0.0;
    do {
      const uint64_t bits = rng_();
      std::memcpy(&d, &bits, sizeof(d));
    } while (!std::isfinite(d));
    return d;
  }
  std::vector<uint8_t> bytes(std::size_t n)
  {
    std::vector<uint8_t> b(n);
    for (auto & v : b) {
      v = static_cast<uint8_t>(rng_());
    }
    return b;
  }
  landfall::Pose3 pose()
  {
    landfall::Pose3 p = landfall::Pose3::Identity();
    p.linear() = Eigen::Quaterniond::UnitRandom().toRotationMatrix();
    p.translation() = landfall::Vec3(value(), value(), value());
    return p;
  }

private:
  std::mt19937_64 rng_{1234};
};

uint64_t bits(double value)
{
  uint64_t result = 0;
  std::memcpy(&result, &value, sizeof(result));
  return result;
}

bool sameBits(double a, double b) { return bits(a) == bits(b); }

bool sameBits(const double * a, const double * b, std::size_t count)
{
  for (std::size_t i = 0; i < count; ++i) {
    if (!sameBits(a[i], b[i])) {
      return false;
    }
  }
  return true;
}

bool samePose(const landfall::Pose3 & a, const landfall::Pose3 & b)
{
  return sameBits(a.matrix().data(), b.matrix().data(), 16);
}

void rawExec(const std::string & path, const std::string & sql)
{
  sqlite3 * handle = nullptr;
  ASSERT_EQ(sqlite3_open(path.c_str(), &handle), SQLITE_OK);
  ASSERT_EQ(sqlite3_exec(handle, sql.c_str(), nullptr, nullptr, nullptr), SQLITE_OK) << sql;
  sqlite3_close(handle);
}

Code openError(const std::string & path)
{
  try {
    db::Database::open(path, db::Database::Access::kReadOnly);
  } catch (const db::DatabaseError & e) {
    return e.code();
  }
  ADD_FAILURE() << "open succeeded";
  return Code::kSqlite;
}

}  // namespace

TEST(Database, CreateSetsIdentityAndVersion)
{
  const landfall::test::TempDir dir;
  const std::string path = dir.file("map.landfall.db");
  {
    auto database = db::Database::create(path);
    EXPECT_EQ(database.meta("schema_version"), std::to_string(db::kSchemaVersion));
    EXPECT_TRUE(database.meta("landfall_version").has_value());
    EXPECT_FALSE(database.meta("missing").has_value());
    EXPECT_EQ(database.integrityCheck(), "ok");
  }
  EXPECT_EQ(db::readSchemaVersion(path), db::kSchemaVersion);
  try {
    db::Database::create(path);
    FAIL() << "create on an existing file succeeded";
  } catch (const db::DatabaseError & e) {
    EXPECT_EQ(e.code(), Code::kAlreadyExists);
  }
}

TEST(Database, RoundTripIsBitExact)
{
  const landfall::test::TempDir dir;
  const std::string path = dir.file("map.landfall.db");
  RandomData random;

  db::MapRecord map{
    0,
    "a",
    "b",
    "c",
    random.value(),
    random.value(),
    random.value(),
    random.value(),
    120,
    80,
    "image: map.pgm\n",
    random.bytes(333)};
  db::CameraRecord camera{
    0,
    "camera_optical",
    "plumb_bob",
    640,
    480,
    {random.value(), random.value(), random.value(), random.value()},
    {random.value(), random.value(), random.value(), random.value(), random.value()},
    random.pose()};
  db::SessionRecord session{0,  "day1", "/bags/day1", "tf_with_covariance", 1700000000123456789,
                            "n"};
  db::KeyframeRecord keyframe;
  keyframe.stamp_ns = 1700000000987654321;
  keyframe.prior = {random.value(), random.value(), random.value()};
  for (int i = 0; i < 9; ++i) {
    keyframe.prior_covariance(i / 3, i % 3) = random.value();
  }
  keyframe.refined = {random.value(), random.value(), random.value()};
  keyframe.T_map_cam = random.pose();
  keyframe.flags = db::KeyframeRecord::kFlagBaShiftAtBound;
  keyframe.quality_json = R"({"sharpness": 123.4})";
  keyframe.thumbnail_jpeg = random.bytes(2000);
  db::BackendRecord backend{0, "local", "orb", "1", "", "max_keypoints: 2000", "f00"};
  db::LocalFeaturesRecord features;
  for (int i = 0; i < 50; ++i) {
    features.keypoints.push_back(
      {static_cast<float>(random.value()), static_cast<float>(i), 31.0F, 90.0F, 0.5F});
  }
  features.descriptor_type = db::BlobType::kU8;
  features.descriptor_dim = 32;
  features.descriptors = random.bytes(std::size_t{50} * 32);
  db::GlobalDescriptorRecord global{0, 0, 128, db::BlobType::kF16, random.bytes(256)};
  db::LandmarkRecord landmark{0,    landfall::Vec3(random.value(), random.value(), random.value()),
                              4,    0.7,
                              12.5, random.bytes(32)};
  db::VocabularyRecord vocabulary{0, 10, 5, random.bytes(4096)};
  db::PcaRecord pca{0, random.bytes(64), random.bytes(1024), 16};

  {
    auto database = db::Database::create(path);
    db::Database::Transaction transaction(database);
    map.id = database.insertMap(map);
    camera.id = database.insertCamera(camera);
    session.id = database.insertSession(session);
    keyframe.session_id = session.id;
    keyframe.camera_id = camera.id;
    keyframe.id = database.insertKeyframe(keyframe);
    backend.id = database.insertBackend(backend);
    features.keyframe_id = keyframe.id;
    features.backend_id = backend.id;
    database.insertLocalFeatures(features);
    global.keyframe_id = keyframe.id;
    global.backend_id = backend.id;
    database.insertGlobalDescriptor(global);
    landmark.id = database.insertLandmark(landmark);
    database.insertObservation({keyframe.id, 7, landmark.id});
    vocabulary.backend_id = backend.id;
    database.insertVocabulary(vocabulary);
    pca.backend_id = backend.id;
    database.insertPca(pca);
    transaction.commit();
  }

  const auto database = db::Database::open(path, db::Database::Access::kReadOnly);
  const auto maps = database.maps();
  ASSERT_EQ(maps.size(), 1U);
  EXPECT_EQ(maps[0].grid, map.grid);
  EXPECT_TRUE(sameBits(maps[0].resolution, map.resolution));
  EXPECT_TRUE(sameBits(maps[0].origin_yaw, map.origin_yaw));
  EXPECT_EQ(maps[0].yaml_text, map.yaml_text);

  const auto cameras = database.cameras();
  ASSERT_EQ(cameras.size(), 1U);
  EXPECT_EQ(cameras[0].intrinsics, camera.intrinsics);
  EXPECT_EQ(cameras[0].distortion, camera.distortion);
  EXPECT_TRUE(samePose(cameras[0].T_base_cam, camera.T_base_cam));

  const auto sessions = database.sessions();
  ASSERT_EQ(sessions.size(), 1U);
  EXPECT_EQ(sessions[0].started_ns, session.started_ns);

  const auto keyframes = database.keyframes();
  ASSERT_EQ(keyframes.size(), 1U);
  const auto & k = keyframes[0];
  EXPECT_EQ(k.stamp_ns, keyframe.stamp_ns);
  EXPECT_TRUE(sameBits(k.prior.x, keyframe.prior.x));
  EXPECT_TRUE(sameBits(k.prior.yaw, keyframe.prior.yaw));
  EXPECT_TRUE(sameBits(k.refined.y, keyframe.refined.y));
  EXPECT_TRUE(sameBits(k.prior_covariance.data(), keyframe.prior_covariance.data(), 9));
  EXPECT_TRUE(samePose(k.T_map_cam, keyframe.T_map_cam));
  EXPECT_EQ(k.flags, keyframe.flags);
  EXPECT_EQ(k.quality_json, keyframe.quality_json);
  EXPECT_EQ(k.thumbnail_jpeg, keyframe.thumbnail_jpeg);

  const auto stored_features = database.localFeatures(keyframe.id, backend.id);
  ASSERT_TRUE(stored_features.has_value());
  ASSERT_EQ(stored_features->keypoints.size(), features.keypoints.size());
  EXPECT_EQ(
    std::memcmp(
      stored_features->keypoints.data(), features.keypoints.data(),
      features.keypoints.size() * sizeof(landfall::Keypoint)),
    0);
  EXPECT_EQ(stored_features->descriptors, features.descriptors);
  EXPECT_FALSE(database.localFeatures(keyframe.id, backend.id + 1).has_value());

  const auto globals = database.globalDescriptors(backend.id);
  ASSERT_EQ(globals.size(), 1U);
  EXPECT_EQ(globals[0].data, global.data);
  EXPECT_EQ(globals[0].type, db::BlobType::kF16);

  const auto landmarks = database.landmarks();
  ASSERT_EQ(landmarks.size(), 1U);
  EXPECT_TRUE(sameBits(landmarks[0].position.z(), landmark.position.z()));
  EXPECT_EQ(landmarks[0].descriptor, landmark.descriptor);

  const auto observations = database.observations(keyframe.id);
  ASSERT_EQ(observations.size(), 1U);
  EXPECT_EQ(observations[0].keypoint_index, 7);
  EXPECT_EQ(observations[0].landmark_id, landmark.id);
  EXPECT_EQ(database.vocabulary(backend.id)->data, vocabulary.data);
  EXPECT_EQ(database.pca(backend.id)->components, pca.components);
}

TEST(Database, ReadOnlyConnectionRefusesWritesAndLeavesFileUntouched)
{
  const landfall::test::TempDir dir;
  const std::string path = dir.file("map.landfall.db");
  db::Database::create(path);
  const auto before = landfall::sha256File(path);
  {
    auto database = db::Database::open(path, db::Database::Access::kReadOnly);
    EXPECT_TRUE(database.readOnly());
    try {
      database.insertSession({0, "s", "src", "tf", 0, ""});
      FAIL() << "write on a read-only connection succeeded";
    } catch (const db::DatabaseError & e) {
      EXPECT_EQ(e.code(), Code::kReadOnly);
    }
  }
  EXPECT_EQ(landfall::sha256File(path), before);
}

TEST(Database, TransactionRollsBackUnlessCommitted)
{
  const landfall::test::TempDir dir;
  auto database = db::Database::create(dir.file("map.landfall.db"));
  {
    db::Database::Transaction transaction(database);
    database.insertSession({0, "rolled back", "", "", 0, ""});
  }
  EXPECT_TRUE(database.sessions().empty());
  {
    db::Database::Transaction transaction(database);
    database.insertSession({0, "kept", "", "", 0, ""});
    transaction.commit();
  }
  EXPECT_EQ(database.sessions().size(), 1U);
}

TEST(Database, RefusesForeignOrMismatchedFiles)
{
  const landfall::test::TempDir dir;
  EXPECT_EQ(openError(dir.file("missing.db")), Code::kNotFound);

  landfall::test::writeText(dir.file("text.db"), "definitely not sqlite, just some text here....");
  EXPECT_EQ(openError(dir.file("text.db")), Code::kNotLandfall);

  rawExec(dir.file("other.db"), "CREATE TABLE t (x INTEGER)");
  EXPECT_EQ(openError(dir.file("other.db")), Code::kNotLandfall);

  const std::string old_db = dir.file("old.db");
  db::Database::create(old_db);
  rawExec(old_db, "PRAGMA user_version = 0");
  EXPECT_EQ(openError(old_db), Code::kSchemaTooOld);

  const std::string new_db = dir.file("new.db");
  db::Database::create(new_db);
  rawExec(new_db, "PRAGMA user_version = " + std::to_string(db::kSchemaVersion + 1));
  EXPECT_EQ(openError(new_db), Code::kSchemaTooNew);
}

TEST(Database, ForeignKeysAreEnforced)
{
  const landfall::test::TempDir dir;
  auto database = db::Database::create(dir.file("map.landfall.db"));
  db::KeyframeRecord orphan;
  orphan.session_id = 42;
  orphan.camera_id = 42;
  EXPECT_THROW(database.insertKeyframe(orphan), db::DatabaseError);
}

TEST(Database, ContentHashIsDeterministicAndIgnoresMeta)
{
  const landfall::test::TempDir dir;
  const auto build = [&](const std::string & name, double x) {
    auto database = db::Database::create(dir.file(name));
    database.insertSession({0, "s", "src", "tf", 5, ""});
    database.insertLandmark({0, landfall::Vec3(x, 2.0, 3.0), 3, 0.5, 4.0, {1, 2, 3}});
    database.setMeta("created_utc", name);
    return database.contentSha256();
  };
  const std::string a = build("a.db", 1.0);
  EXPECT_EQ(a, build("b.db", 1.0));
  EXPECT_NE(a, build("c.db", 1.0000001));
}

TEST(Migrations, AppliesStepsWithBackupAndRefusesGaps)
{
  const landfall::test::TempDir dir;
  const std::string path = dir.file("map.landfall.db");
  db::Database::create(path);
  const auto original = landfall::sha256File(path);

  // Nothing to do at the current version.
  EXPECT_TRUE(db::migrateDatabase(path).applied.empty());

  const std::vector<db::Migration> steps = {
    {db::kSchemaVersion, "add landmark color", "ALTER TABLE landmarks ADD COLUMN color INTEGER"},
  };
  const auto report = db::migrateDatabase(path, db::kSchemaVersion + 1, steps);
  EXPECT_EQ(report.from_version, db::kSchemaVersion);
  EXPECT_EQ(report.to_version, db::kSchemaVersion + 1);
  ASSERT_EQ(report.applied.size(), 1U);
  EXPECT_EQ(landfall::sha256File(report.backup_path), original);
  EXPECT_EQ(db::readSchemaVersion(path), db::kSchemaVersion + 1);
  // This build only reads kSchemaVersion: the relocalizer never opens a migrated-ahead file.
  EXPECT_EQ(openError(path), Code::kSchemaTooNew);

  try {
    db::migrateDatabase(path, db::kSchemaVersion, steps);
    FAIL() << "migrating backwards succeeded";
  } catch (const db::DatabaseError & e) {
    EXPECT_EQ(e.code(), Code::kSchemaTooNew);
  }
  try {
    db::migrateDatabase(path, db::kSchemaVersion + 3, steps);
    FAIL() << "migrating over a missing step succeeded";
  } catch (const db::DatabaseError & e) {
    EXPECT_EQ(e.code(), Code::kSchemaTooOld);
  }
}

TEST(Migrations, FailingStepRollsBack)
{
  const landfall::test::TempDir dir;
  const std::string path = dir.file("map.landfall.db");
  db::Database::create(path);
  const std::vector<db::Migration> steps = {
    {db::kSchemaVersion, "broken", "ALTER TABLE no_such_table ADD COLUMN x INTEGER"},
  };
  EXPECT_THROW(db::migrateDatabase(path, db::kSchemaVersion + 1, steps, false), db::DatabaseError);
  EXPECT_EQ(db::readSchemaVersion(path), db::kSchemaVersion);
  EXPECT_NO_THROW(db::Database::open(path, db::Database::Access::kReadOnly));
}
