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

#include "landfall/db/migrations.hpp"

#include <filesystem>
#include <memory>

#include "sqlite_statement.hpp"

namespace landfall::db
{

const std::vector<Migration> & builtinMigrations()
{
  static const std::vector<Migration> steps;
  return steps;
}

MigrationReport migrateDatabase(
  const std::string & path, int target_version, const std::vector<Migration> & steps, bool backup)
{
  MigrationReport report;
  report.from_version = readSchemaVersion(path);
  report.to_version = report.from_version;
  if (report.from_version > target_version) {
    throw DatabaseError(
      DatabaseError::Code::kSchemaTooNew,
      path + " has schema version " + std::to_string(report.from_version) +
        ", newer than the target " + std::to_string(target_version));
  }
  if (report.from_version == target_version) {
    return report;
  }
  // Check that every step exists before touching the file.
  for (int version = report.from_version; version < target_version; ++version) {
    bool found = false;
    for (const auto & step : steps) {
      found = found || step.from_version == version;
    }
    if (!found) {
      throw DatabaseError(
        DatabaseError::Code::kSchemaTooOld,
        "no migration from schema version " + std::to_string(version));
    }
  }
  if (backup) {
    report.backup_path = path + ".bak-v" + std::to_string(report.from_version);
    std::filesystem::copy_file(
      path, report.backup_path, std::filesystem::copy_options::overwrite_existing);
  }

  sqlite3 * handle = nullptr;
  const int rc = sqlite3_open_v2(path.c_str(), &handle, SQLITE_OPEN_READWRITE, nullptr);
  const auto close = [](sqlite3 * h) { sqlite3_close(h); };
  const std::unique_ptr<sqlite3, decltype(close)> guard(handle, close);
  detail::check(handle, rc, "open " + path);

  for (int version = report.from_version; version < target_version; ++version) {
    for (const auto & step : steps) {
      if (step.from_version != version) {
        continue;
      }
      const std::string sql = "BEGIN;\n" + step.sql +
                              ";\nPRAGMA user_version = " + std::to_string(version + 1) +
                              ";\nCOMMIT;";
      char * error = nullptr;
      const int step_rc = sqlite3_exec(handle, sql.c_str(), nullptr, nullptr, &error);
      if (step_rc != SQLITE_OK) {
        const std::string message = error != nullptr ? error : sqlite3_errstr(step_rc);
        sqlite3_free(error);
        sqlite3_exec(handle, "ROLLBACK", nullptr, nullptr, nullptr);
        throw DatabaseError(
          DatabaseError::Code::kSqlite, "migration v" + std::to_string(version) + " -> v" +
                                          std::to_string(version + 1) + " failed: " + message);
      }
      report.applied.push_back(step.description);
      report.to_version = version + 1;
      break;
    }
  }
  return report;
}

}  // namespace landfall::db
