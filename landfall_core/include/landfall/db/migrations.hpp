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

#ifndef LANDFALL_DB_MIGRATIONS_HPP_
#define LANDFALL_DB_MIGRATIONS_HPP_

#include <string>
#include <vector>

#include "landfall/db/database.hpp"

namespace landfall::db
{

/// One schema step from_version -> from_version + 1, applied in a single transaction.
struct Migration
{
  int from_version = 0;
  std::string description;
  std::string sql;
};

/// The steps shipped with this build (empty while the schema is at version 1).
const std::vector<Migration> & builtinMigrations();

struct MigrationReport
{
  int from_version = 0;
  int to_version = 0;
  std::string backup_path;  ///< empty if nothing was migrated or backups were disabled
  std::vector<std::string> applied;
};

/// Brings the database at `path` to `target_version` (only `landfall migrate` calls this; the
/// relocalizer never migrates). Copies the file to `<path>.bak-v<N>` first when `backup` is set.
/// Throws DatabaseError: kSchemaTooNew if the file is newer than the target, kSchemaTooOld if a
/// step is missing. A failing step rolls back and leaves the file at the last completed version.
MigrationReport migrateDatabase(
  const std::string & path, int target_version = kSchemaVersion,
  const std::vector<Migration> & steps = builtinMigrations(), bool backup = true);

}  // namespace landfall::db

#endif  // LANDFALL_DB_MIGRATIONS_HPP_
