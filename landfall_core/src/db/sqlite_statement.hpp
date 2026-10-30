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

// Private helpers around the SQLite C API (not installed).

#ifndef LANDFALL_DB_SQLITE_STATEMENT_HPP_
#define LANDFALL_DB_SQLITE_STATEMENT_HPP_

#include <sqlite3.h>

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "landfall/db/database.hpp"

namespace landfall::db::detail
{

/// Throws DatabaseError for a failed SQLite call, mapping read-only and not-a-database errors.
void throwSqlite(sqlite3 * handle, int rc, const std::string & context);

inline void check(sqlite3 * handle, int rc, const std::string & context)
{
  if (rc != SQLITE_OK && rc != SQLITE_ROW && rc != SQLITE_DONE) {
    throwSqlite(handle, rc, context);
  }
}

/// A prepared statement; finalized on destruction.
class Statement
{
public:
  Statement(sqlite3 * handle, const std::string & sql) : handle_(handle), sql_(sql)
  {
    check(handle_, sqlite3_prepare_v2(handle_, sql.c_str(), -1, &statement_, nullptr), sql);
  }
  Statement(const Statement &) = delete;
  Statement & operator=(const Statement &) = delete;
  ~Statement() { sqlite3_finalize(statement_); }

  void bindInt(int index, int64_t value)
  {
    check(handle_, sqlite3_bind_int64(statement_, index, value), sql_);
  }
  void bindDouble(int index, double value)
  {
    check(handle_, sqlite3_bind_double(statement_, index, value), sql_);
  }
  void bindText(int index, const std::string & value)
  {
    check(
      handle_,
      sqlite3_bind_text(
        statement_, index, value.c_str(), static_cast<int>(value.size()), SQLITE_TRANSIENT),
      sql_);
  }
  /// An empty vector is stored as a zero-length blob, not NULL.
  void bindBlob(int index, const std::vector<uint8_t> & value)
  {
    if (value.empty()) {
      check(handle_, sqlite3_bind_zeroblob(statement_, index, 0), sql_);
      return;
    }
    check(
      handle_,
      sqlite3_bind_blob(
        statement_, index, value.data(), static_cast<int>(value.size()), SQLITE_TRANSIENT),
      sql_);
  }

  /// Advances; true while a row is available.
  bool step()
  {
    const int rc = sqlite3_step(statement_);
    if (rc == SQLITE_ROW) {
      return true;
    }
    if (rc == SQLITE_DONE) {
      return false;
    }
    throwSqlite(handle_, rc, sql_);
    return false;
  }
  void run()
  {
    while (step()) {
    }
  }

  int columnCount() const { return sqlite3_column_count(statement_); }
  int columnType(int column) const { return sqlite3_column_type(statement_, column); }
  int64_t getInt(int column) const { return sqlite3_column_int64(statement_, column); }
  double getDouble(int column) const { return sqlite3_column_double(statement_, column); }
  std::string getText(int column) const
  {
    const auto * text = sqlite3_column_text(statement_, column);
    const int size = sqlite3_column_bytes(statement_, column);
    return text == nullptr ? std::string()
                           : std::string(reinterpret_cast<const char *>(text), size);
  }
  std::vector<uint8_t> getBlob(int column) const
  {
    const auto * data = static_cast<const uint8_t *>(sqlite3_column_blob(statement_, column));
    const int size = sqlite3_column_bytes(statement_, column);
    return data == nullptr ? std::vector<uint8_t>() : std::vector<uint8_t>(data, data + size);
  }

private:
  sqlite3 * handle_;
  std::string sql_;
  sqlite3_stmt * statement_ = nullptr;
};

}  // namespace landfall::db::detail

#endif  // LANDFALL_DB_SQLITE_STATEMENT_HPP_
