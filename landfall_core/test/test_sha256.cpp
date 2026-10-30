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

#include <algorithm>
#include <string>

#include "landfall/util/sha256.hpp"
#include "test_utils.hpp"

// Test vectors from FIPS 180-4 examples / NIST CAVP.
TEST(Sha256, NistVectors)
{
  EXPECT_EQ(
    landfall::sha256Hex(std::string()),
    "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
  EXPECT_EQ(
    landfall::sha256Hex(std::string("abc")),
    "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  EXPECT_EQ(
    landfall::sha256Hex(std::string("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq")),
    "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");
  EXPECT_EQ(
    landfall::sha256Hex(std::string(1000000, 'a')),
    "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0");
}

TEST(Sha256, IncrementalUpdatesMatchOneShot)
{
  std::string message;
  for (int i = 0; i < 1000; ++i) {
    message += static_cast<char>(i * 37);
  }
  landfall::Sha256 sha;
  std::size_t offset = 0;
  for (std::size_t chunk = 1; offset < message.size(); chunk = chunk * 3 % 97 + 1) {
    const std::size_t take = std::min(chunk, message.size() - offset);
    sha.update(message.data() + offset, take);
    offset += take;
  }
  EXPECT_EQ(sha.finishHex(), landfall::sha256Hex(message));
  // finish() resets the object.
  sha.update(std::string("abc"));
  EXPECT_EQ(sha.finishHex(), landfall::sha256Hex(std::string("abc")));
}

TEST(Sha256, HashesFiles)
{
  const landfall::test::TempDir dir;
  landfall::test::writeText(dir.file("a.txt"), "abc");
  EXPECT_EQ(
    landfall::sha256File(dir.file("a.txt")),
    "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  EXPECT_FALSE(landfall::sha256File(dir.file("missing.txt")).has_value());
}
