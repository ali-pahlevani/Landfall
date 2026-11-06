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

// Parity of the learned backends with Python (docs/PLAN.md M5): each parity file (format in
// landfall_tools/export/common.py) holds images and what PyTorch / ONNX Runtime in Python computed
// on them; the C++ backend runs on the same pixels. Keypoints within 0.5 px for at least 95 %,
// descriptors and global descriptors with a cosine of at least 0.999.
//
// The synthetic models' files (test/data) always run: their references use torch's own
// grid_sample and interpolate, the reference for the C++ post-processing. The real models' run
// when they and their parity files are in the models directory ($LANDFALL_MODELS_DIR, see
// scripts/fetch_models.sh --parity); otherwise those tests are skipped.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "landfall/backend_options.hpp"
#include "landfall/onnx/global_descriptor.hpp"
#include "landfall/onnx/xfeat.hpp"

namespace
{

const std::string kData = LANDFALL_TEST_DATA_DIR;

struct ParityImage
{
  cv::Mat pixels;  ///< gray or BGR
  std::vector<cv::Point2f> keypoints;
  cv::Mat descriptors;  ///< local: one row per keypoint; global: one row
};

class Reader
{
public:
  explicit Reader(const std::string & path)
  {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
      throw std::runtime_error("cannot read " + path);
    }
    bytes_.assign(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
  }

  template <typename T>
  T value()
  {
    T v{};
    take(&v, sizeof(T));
    return v;
  }

  void take(void * out, std::size_t size)
  {
    if (offset_ + size > bytes_.size()) {
      throw std::runtime_error("parity file truncated");
    }
    std::memcpy(out, bytes_.data() + offset_, size);
    offset_ += size;
  }

private:
  std::vector<char> bytes_;
  std::size_t offset_ = 0;
};

std::vector<ParityImage> readParity(const std::string & path, uint32_t expected_kind)
{
  Reader r(path);
  char magic[8];
  r.take(magic, sizeof(magic));
  if (std::string(magic, sizeof(magic)) != "LFPARITY" || r.value<uint32_t>() != 1) {
    throw std::runtime_error(path + ": not a version 1 parity file");
  }
  if (r.value<uint32_t>() != expected_kind) {
    throw std::runtime_error(path + ": wrong kind");
  }
  std::vector<ParityImage> images(r.value<uint32_t>());
  for (auto & image : images) {
    const auto h = static_cast<int>(r.value<uint32_t>());
    const auto w = static_cast<int>(r.value<uint32_t>());
    const auto c = static_cast<int>(r.value<uint32_t>());
    image.pixels = cv::Mat(h, w, c == 1 ? CV_8UC1 : CV_8UC3);
    r.take(image.pixels.data, image.pixels.total() * image.pixels.elemSize());
    if (expected_kind == 1) {
      const auto n = static_cast<int>(r.value<uint32_t>());
      const auto dim = static_cast<int>(r.value<uint32_t>());
      image.keypoints.resize(static_cast<std::size_t>(n));
      r.take(image.keypoints.data(), sizeof(cv::Point2f) * image.keypoints.size());
      std::vector<float> scores(static_cast<std::size_t>(n));
      r.take(scores.data(), sizeof(float) * scores.size());
      image.descriptors = cv::Mat(n, dim, CV_32F);
      r.take(image.descriptors.data, image.descriptors.total() * sizeof(float));
    } else {
      const auto dim = static_cast<int>(r.value<uint32_t>());
      image.descriptors = cv::Mat(1, dim, CV_32F);
      r.take(image.descriptors.data, image.descriptors.total() * sizeof(float));
    }
  }
  return images;
}

std::string modelsDirectory() { return landfall::modelsDir(""); }

bool present(const std::string & dir, const std::vector<std::string> & files)
{
  return std::all_of(files.begin(), files.end(), [&dir](const std::string & f) {
    return std::filesystem::exists(dir + "/" + f);
  });
}

// XFeat on every image of the parity file against its reference.
void checkLocal(const std::string & model, const std::string & parity, int max_keypoints)
{
  landfall::onnx::XFeatParams params;
  params.model = model;
  params.max_keypoints = max_keypoints;
  const landfall::onnx::XFeatFeatures xfeat(params);
  for (const ParityImage & image : readParity(parity, 1)) {
    const landfall::Features ours = xfeat.extract(image.pixels);
    std::size_t matched = 0;
    double worst_cosine = 1.0;
    for (std::size_t i = 0; i < image.keypoints.size(); ++i) {
      double best = std::numeric_limits<double>::infinity();
      int best_j = -1;
      for (std::size_t j = 0; j < ours.keypoints.size(); ++j) {
        const double d = std::hypot(
          ours.keypoints[j].x - image.keypoints[i].x, ours.keypoints[j].y - image.keypoints[i].y);
        if (d < best) {
          best = d;
          best_j = static_cast<int>(j);
        }
      }
      if (best_j < 0 || best > 0.5) {
        continue;
      }
      ++matched;
      const double cosine =
        ours.descriptors.row(best_j).dot(image.descriptors.row(static_cast<int>(i)));
      worst_cosine = std::min(worst_cosine, cosine);
    }
    const double fraction = image.keypoints.empty() ? 1.0
                                                    : static_cast<double>(matched) /
                                                        static_cast<double>(image.keypoints.size());
    std::cout << model << ": " << matched << " of " << image.keypoints.size()
              << " reference keypoints within 0.5 px (ours " << ours.keypoints.size()
              << "), worst descriptor cosine " << worst_cosine << "\n";
    EXPECT_GE(fraction, 0.95);
    EXPECT_NEAR(
      static_cast<double>(ours.keypoints.size()), static_cast<double>(image.keypoints.size()),
      0.05 * static_cast<double>(image.keypoints.size()) + 1.0);
    EXPECT_GE(worst_cosine, 0.999);
  }
}

void checkGlobal(const landfall::onnx::OnnxGlobalParams & params, const std::string & parity)
{
  const landfall::onnx::OnnxGlobalDescriptor global(params);
  for (const ParityImage & image : readParity(parity, 2)) {
    const auto ours = global.compute(image.pixels, nullptr).dense;
    ASSERT_EQ(static_cast<int>(ours.size()), image.descriptors.cols);
    double cosine = 0.0;
    for (std::size_t k = 0; k < ours.size(); ++k) {
      cosine += static_cast<double>(ours[k]) * image.descriptors.at<float>(0, static_cast<int>(k));
    }
    std::cout << params.model << ": global descriptor cosine " << cosine << "\n";
    EXPECT_GE(cosine, 0.999);
  }
}

landfall::onnx::OnnxGlobalParams preset(
  const std::string & name, const std::string & model, int width, int height)
{
  landfall::onnx::OnnxGlobalParams p;
  p.name = name;
  p.model = model;
  p.width = width;
  p.height = height;
  return p;
}

}  // namespace

TEST(OnnxParity, SyntheticXFeatMatchesTorch)
{
  checkLocal(kData + "/synthetic_xfeat.onnx", kData + "/synthetic_xfeat.parity.bin", 100000);
}

TEST(OnnxParity, SyntheticGlobalMatchesPython)
{
  checkGlobal(
    preset("eigenplaces", kData + "/synthetic_global.onnx", 0, 0),
    kData + "/synthetic_global.parity.bin");
}

TEST(OnnxParity, XFeatMatchesPyTorch)
{
  const std::string dir = modelsDirectory();
  if (!present(dir, {"xfeat.onnx", "xfeat.parity.bin"})) {
    GTEST_SKIP() << "no xfeat.onnx and xfeat.parity.bin in " << dir << " (docs/TODO.md T-007)";
  }
  checkLocal(dir + "/xfeat.onnx", dir + "/xfeat.parity.bin", 4096);
}

TEST(OnnxParity, EigenPlacesMatchesPyTorch)
{
  const std::string dir = modelsDirectory();
  if (!present(dir, {"eigenplaces_resnet18_512.onnx", "eigenplaces.parity.bin"})) {
    GTEST_SKIP() << "no EigenPlaces model and parity file in " << dir << " (docs/TODO.md T-007)";
  }
  checkGlobal(
    preset("eigenplaces", dir + "/eigenplaces_resnet18_512.onnx", 0, 0),
    dir + "/eigenplaces.parity.bin");
}

TEST(OnnxParity, MegaLocMatchesPython)
{
  const std::string dir = modelsDirectory();
  if (!present(dir, {"megaloc.onnx", "megaloc.parity.bin"})) {
    GTEST_SKIP() << "no megaloc.onnx and megaloc.parity.bin in " << dir << " (docs/TODO.md T-007)";
  }
  checkGlobal(preset("megaloc", dir + "/megaloc.onnx", 322, 322), dir + "/megaloc.parity.bin");
}
