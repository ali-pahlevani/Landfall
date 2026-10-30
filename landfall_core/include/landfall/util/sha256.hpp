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

#ifndef LANDFALL_UTIL_SHA256_HPP_
#define LANDFALL_UTIL_SHA256_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace landfall
{

/// Incremental SHA-256 (FIPS 180-4). Used for map/model/config fingerprints, not for security.
class Sha256
{
public:
  using Digest = std::array<uint8_t, 32>;

  Sha256();
  void update(const void * data, std::size_t size);
  void update(const std::string & text) { update(text.data(), text.size()); }
  /// Returns the digest and resets the object for a new message.
  Digest finish();
  std::string finishHex();

private:
  void compress(const uint8_t * block);

  std::array<uint32_t, 8> state_{};
  std::array<uint8_t, 64> buffer_{};
  std::size_t buffered_ = 0;
  uint64_t total_bytes_ = 0;
};

std::string toHex(const uint8_t * data, std::size_t size);
std::string sha256Hex(const void * data, std::size_t size);
std::string sha256Hex(const std::string & text);
/// Hash of a file's bytes, or nullopt if it can't be read.
std::optional<std::string> sha256File(const std::string & path);

}  // namespace landfall

#endif  // LANDFALL_UTIL_SHA256_HPP_
