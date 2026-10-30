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

#include "landfall/util/sha256.hpp"

#include <cstring>
#include <fstream>
#include <vector>

namespace landfall
{
namespace
{

constexpr std::array<uint32_t, 64> kRound = {
  0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
  0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
  0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
  0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
  0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
  0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
  0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
  0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};

constexpr std::array<uint32_t, 8> kInitialState = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
                                                   0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};

constexpr uint32_t rotr(uint32_t x, int n) { return (x >> n) | (x << (32 - n)); }

}  // namespace

Sha256::Sha256() : state_(kInitialState) {}

void Sha256::compress(const uint8_t * block)
{
  std::array<uint32_t, 64> w{};
  for (std::size_t i = 0; i < 16; ++i) {
    w[i] = (uint32_t{block[4 * i]} << 24) | (uint32_t{block[4 * i + 1]} << 16) |
           (uint32_t{block[4 * i + 2]} << 8) | uint32_t{block[4 * i + 3]};
  }
  for (std::size_t i = 16; i < 64; ++i) {
    const uint32_t s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
    const uint32_t s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
    w[i] = w[i - 16] + s0 + w[i - 7] + s1;
  }
  uint32_t a = state_[0], b = state_[1], c = state_[2], d = state_[3];
  uint32_t e = state_[4], f = state_[5], g = state_[6], h = state_[7];
  for (std::size_t i = 0; i < 64; ++i) {
    const uint32_t s1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
    const uint32_t ch = (e & f) ^ (~e & g);
    const uint32_t t1 = h + s1 + ch + kRound[i] + w[i];
    const uint32_t s0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
    const uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
    const uint32_t t2 = s0 + maj;
    h = g;
    g = f;
    f = e;
    e = d + t1;
    d = c;
    c = b;
    b = a;
    a = t1 + t2;
  }
  state_[0] += a;
  state_[1] += b;
  state_[2] += c;
  state_[3] += d;
  state_[4] += e;
  state_[5] += f;
  state_[6] += g;
  state_[7] += h;
}

void Sha256::update(const void * data, std::size_t size)
{
  const auto * bytes = static_cast<const uint8_t *>(data);
  total_bytes_ += size;
  while (size > 0) {
    const std::size_t take = std::min(size, buffer_.size() - buffered_);
    std::memcpy(buffer_.data() + buffered_, bytes, take);
    buffered_ += take;
    bytes += take;
    size -= take;
    if (buffered_ == buffer_.size()) {
      compress(buffer_.data());
      buffered_ = 0;
    }
  }
}

Sha256::Digest Sha256::finish()
{
  const uint64_t total_bits = total_bytes_ * 8;
  const uint8_t pad_start = 0x80;
  update(&pad_start, 1);
  const uint8_t zero = 0;
  while (buffered_ != 56) {
    update(&zero, 1);
  }
  std::array<uint8_t, 8> length{};
  for (std::size_t i = 0; i < 8; ++i) {
    length[i] = static_cast<uint8_t>(total_bits >> (56 - 8 * i));
  }
  update(length.data(), length.size());

  Digest digest{};
  for (std::size_t i = 0; i < 8; ++i) {
    for (std::size_t j = 0; j < 4; ++j) {
      digest[4 * i + j] = static_cast<uint8_t>(state_[i] >> (24 - 8 * j));
    }
  }
  *this = Sha256();
  return digest;
}

std::string Sha256::finishHex()
{
  const Digest digest = finish();
  return toHex(digest.data(), digest.size());
}

std::string toHex(const uint8_t * data, std::size_t size)
{
  static constexpr char kDigits[] = "0123456789abcdef";
  std::string hex(2 * size, '0');
  for (std::size_t i = 0; i < size; ++i) {
    hex[2 * i] = kDigits[data[i] >> 4];
    hex[2 * i + 1] = kDigits[data[i] & 0x0f];
  }
  return hex;
}

std::string sha256Hex(const void * data, std::size_t size)
{
  Sha256 sha;
  sha.update(data, size);
  return sha.finishHex();
}

std::string sha256Hex(const std::string & text) { return sha256Hex(text.data(), text.size()); }

std::optional<std::string> sha256File(const std::string & path)
{
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    return std::nullopt;
  }
  Sha256 sha;
  std::vector<char> chunk(1 << 16);
  while (file) {
    file.read(chunk.data(), static_cast<std::streamsize>(chunk.size()));
    sha.update(chunk.data(), static_cast<std::size_t>(file.gcount()));
  }
  if (file.bad()) {
    return std::nullopt;
  }
  return sha.finishHex();
}

}  // namespace landfall
