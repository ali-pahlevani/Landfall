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

// The learned backends' plumbing on the synthetic models of test/data (written by
// landfall_tools/export/synthetic.py; no trained weights): ONNX Runtime sessions, XFeat's
// post-processing, the global descriptor's preprocessing, and the registry.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <string>
#include <vector>

#include "landfall/onnx/backends.hpp"
#include "landfall/onnx/global_descriptor.hpp"
#include "landfall/onnx/session.hpp"
#include "landfall/onnx/xfeat.hpp"
#include "landfall/util/sha256.hpp"
#include "opencv2/imgproc.hpp"

using landfall::BackendOptions;
using landfall::Features;
using landfall::onnx::ExecutionProvider;
using landfall::onnx::OnnxGlobalDescriptor;
using landfall::onnx::OnnxGlobalParams;
using landfall::onnx::Session;
using landfall::onnx::SessionOptions;
using landfall::onnx::Tensor;
using landfall::onnx::XFeatFeatures;
using landfall::onnx::XFeatMaps;
using landfall::onnx::XFeatParams;

namespace
{

const std::string kData = LANDFALL_TEST_DATA_DIR;

std::string model(const std::string & name) { return kData + "/" + name; }

// XFeat maps of h x w with an empty heatmap, a reliability of 1 and descriptor channel 0 at 1.
XFeatMaps emptyMaps(int h, int w)
{
  XFeatMaps m;
  m.height = h;
  m.width = w;
  const auto pixels = static_cast<std::size_t>(h) * static_cast<std::size_t>(w);
  const std::size_t cells = pixels / 64;
  m.heatmap.assign(pixels, 0.0F);
  m.is_max.assign(pixels, 0);
  m.reliability.assign(cells, 1.0F);
  m.descriptors.assign(64 * cells, 0.0F);
  std::fill(
    m.descriptors.begin(), m.descriptors.begin() + static_cast<std::ptrdiff_t>(cells), 1.0F);
  return m;
}

void setHeat(XFeatMaps * m, int x, int y, float value)
{
  m->heatmap
    [static_cast<std::size_t>(y) * static_cast<std::size_t>(m->width) +
     static_cast<std::size_t>(x)] = value;
}

// The 5x5 local-maximum mask, as MaxPool with -inf padding computes it.
void computeIsMax(XFeatMaps * m)
{
  for (int y = 0; y < m->height; ++y) {
    for (int x = 0; x < m->width; ++x) {
      float best = -INFINITY;
      for (int dy = -2; dy <= 2; ++dy) {
        for (int dx = -2; dx <= 2; ++dx) {
          const int xx = x + dx;
          const int yy = y + dy;
          if (xx >= 0 && xx < m->width && yy >= 0 && yy < m->height) {
            best = std::max(
              best, m->heatmap
                      [static_cast<std::size_t>(yy) * static_cast<std::size_t>(m->width) +
                       static_cast<std::size_t>(xx)]);
          }
        }
      }
      const auto i = static_cast<std::size_t>(y) * static_cast<std::size_t>(m->width) +
                     static_cast<std::size_t>(x);
      m->is_max[i] = m->heatmap[i] == best ? 1 : 0;
    }
  }
}

// PyTorch's 1-D cubic convolution (A = -0.75) of four samples at fractional offset t.
double cubic(const std::array<double, 4> & v, double t)
{
  const double a = -0.75;
  const auto inner = [a](double x) { return ((a + 2.0) * x - (a + 3.0)) * x * x + 1.0; };
  const auto outer = [a](double x) { return ((a * x - 5.0 * a) * x + 8.0 * a) * x - 4.0 * a; };
  return v[0] * outer(t + 1.0) + v[1] * inner(t) + v[2] * inner(1.0 - t) + v[3] * outer(2.0 - t);
}

double norm(const std::vector<float> & v)
{
  double s = 0.0;
  for (const float x : v) {
    s += static_cast<double>(x) * x;
  }
  return std::sqrt(s);
}

}  // namespace

TEST(OnnxSession, RunsTheSyntheticModel)
{
  const Session session(model("synthetic_xfeat.onnx"), {});
  EXPECT_EQ(session.inputNames(), std::vector<std::string>{"image"});
  EXPECT_EQ(
    session.outputNames(),
    (std::vector<std::string>{"descriptors", "heatmap", "reliability", "is_max"}));
  Tensor image;
  image.shape = {1, 1, 64, 96};
  image.values.assign(std::size_t{64} * 96, 0.0F);
  image.values[20 * 96 + 30] = 0.9F;
  const auto out = session.run({image});
  ASSERT_EQ(out.size(), 4U);
  EXPECT_EQ(out[0].shape, (std::vector<int64_t>{1, 64, 8, 12}));
  EXPECT_EQ(out[1].shape, (std::vector<int64_t>{1, 1, 64, 96}));
  EXPECT_NEAR(out[1].values[20 * 96 + 30], 0.9F + 30e-4F + 20e-5F, 1e-6);  // with the ramps
  ASSERT_EQ(out[3].flags.size(), std::size_t{64} * 96);                    // bool output
  EXPECT_EQ(out[3].flags[20 * 96 + 30], 1);
  EXPECT_EQ(out[3].flags[20 * 96 + 31], 0);
  EXPECT_FALSE(landfall::onnx::runtimeVersion().empty());
}

TEST(OnnxSession, ReportsErrors)
{
  EXPECT_THROW(Session(model("missing.onnx"), {}), std::runtime_error);
  const Session session(model("synthetic_global.onnx"), {});
  EXPECT_THROW(session.run({}), std::runtime_error);
  Tensor two_channels;
  two_channels.shape = {1, 2, 4, 4};
  two_channels.values.assign(32, 0.0F);
  EXPECT_THROW(session.run({two_channels}), std::runtime_error);
  // The rosdep ONNX Runtime is CPU-only: CUDA is refused with the provider in the message.
  SessionOptions cuda;
  cuda.provider = ExecutionProvider::kCuda;
  try {
    const Session gpu(model("synthetic_global.onnx"), cuda);
    SUCCEED() << "this ONNX Runtime has CUDA";
  } catch (const std::runtime_error & e) {
    EXPECT_NE(std::string(e.what()).find("(cuda)"), std::string::npos) << e.what();
  }
  EXPECT_EQ(landfall::onnx::parseExecutionProvider("tensorrt"), ExecutionProvider::kTensorRt);
  EXPECT_FALSE(landfall::onnx::parseExecutionProvider("gpu").has_value());
}

TEST(XFeatPostprocessing, KeepsTheBestScoredMaxima)
{
  XFeatMaps m = emptyMaps(64, 96);
  setHeat(&m, 40, 24, 0.5F);
  setHeat(&m, 70, 40, 0.8F);
  setHeat(&m, 10, 50, 0.3F);
  setHeat(&m, 50, 10, 0.04F);  // below the threshold
  setHeat(&m, 0, 0, 0.9F);     // XFeat reserves (0, 0) for padding
  setHeat(&m, 95, 30, 0.9F);   // last column: grid_sample's nearest rounds it outside
  setHeat(&m, 30, 63, 0.9F);   // last row: same
  computeIsMax(&m);
  const Features all = landfall::onnx::detectXFeat(m, 10, 0.05);
  ASSERT_EQ(all.keypoints.size(), 3U);
  EXPECT_EQ(all.type, landfall::DescriptorType::kFloat32);
  EXPECT_FLOAT_EQ(all.keypoints[0].x, 70.0F);
  EXPECT_FLOAT_EQ(all.keypoints[0].y, 40.0F);
  EXPECT_FLOAT_EQ(all.keypoints[0].score, 0.8F);  // heatmap x reliability (1 inside)
  EXPECT_FLOAT_EQ(all.keypoints[1].score, 0.5F);
  EXPECT_FLOAT_EQ(all.keypoints[2].score, 0.3F);
  const Features best = landfall::onnx::detectXFeat(m, 2, 0.05);
  ASSERT_EQ(best.keypoints.size(), 2U);
  EXPECT_EQ(best.descriptors.rows, 2);
  EXPECT_EQ(best.descriptors.cols, 64);
  EXPECT_THROW(landfall::onnx::detectXFeat(emptyMaps(60, 96), 10, 0.05), std::invalid_argument);
}

TEST(XFeatPostprocessing, SamplesDescriptorsLikeGridSample)
{
  // Channel 0 is the column of the 12 x 8 map, channel 1 is 1: a descriptor's ratio d0 / d1 is the
  // bicubic sample of the column ramp at the keypoint's grid_sample position.
  XFeatMaps m = emptyMaps(64, 96);
  const std::size_t cells = std::size_t{12} * 8;
  for (std::size_t c = 0; c < cells; ++c) {
    m.descriptors[c] = static_cast<float>(c % 12);
    m.descriptors[cells + c] = 1.0F;
  }
  setHeat(&m, 40, 24, 0.5F);
  computeIsMax(&m);
  const Features f = landfall::onnx::detectXFeat(m, 10, 0.05);
  ASSERT_EQ(f.keypoints.size(), 1U);
  // XFeat normalizes by (W - 1) of the full image; grid_sample (align_corners=False) then maps
  // onto the 12 columns.
  const double ix = 40.0 / 95.0 * 12.0 - 0.5;
  const double x0 = std::floor(ix);
  const double expected = cubic({x0 - 1.0, x0, x0 + 1.0, x0 + 2.0}, ix - x0);
  const float d0 = f.descriptors.at<float>(0, 0);
  const float d1 = f.descriptors.at<float>(0, 1);
  EXPECT_NEAR(d0 / d1, expected, 1e-4);
  EXPECT_NEAR(std::hypot(d0, d1), 1.0, 1e-5);  // L2-normalized
}

TEST(XFeatFeatures, ExtractsWithTheSyntheticModel)
{
  XFeatParams p;
  p.model = model("synthetic_xfeat.onnx");
  p.max_keypoints = 10;
  const XFeatFeatures xfeat(p);
  // 100 x 70: resized to 96 x 64 like XFeat, keypoints scaled back.
  // Smooth blobs: one maximum each (a flat plateau would be a maximum at every pixel).
  cv::Mat gray(70, 100, CV_8UC1, cv::Scalar(0));
  cv::rectangle(gray, cv::Rect(39, 29, 3, 3), cv::Scalar(255), cv::FILLED);
  cv::rectangle(gray, cv::Rect(74, 49, 3, 3), cv::Scalar(160), cv::FILLED);
  cv::GaussianBlur(gray, gray, cv::Size(5, 5), 1.0);
  const Features f = xfeat.extract(gray);
  ASSERT_EQ(f.keypoints.size(), 2U);
  EXPECT_NEAR(f.keypoints[0].x, 40.0F, 1.5F);  // brighter blob first
  EXPECT_NEAR(f.keypoints[0].y, 30.0F, 1.5F);
  EXPECT_NEAR(f.keypoints[1].x, 75.0F, 1.5F);
  EXPECT_NEAR(f.keypoints[1].y, 50.0F, 1.5F);
  EXPECT_GT(f.keypoints[0].score, f.keypoints[1].score);
  for (int r = 0; r < f.descriptors.rows; ++r) {
    EXPECT_NEAR(cv::norm(f.descriptors.row(r)), 1.0, 1e-5);
  }
  const auto id = xfeat.id();
  EXPECT_EQ(id.name, "xfeat");
  EXPECT_EQ(
    id.config, "detection_threshold: 0.05\nmax_keypoints: 10\nmodel: synthetic_xfeat.onnx\n");
  EXPECT_EQ(id.model_sha256, *landfall::sha256File(p.model));
  EXPECT_THROW(xfeat.extract(cv::Mat(20, 20, CV_8UC1, cv::Scalar(0))), std::invalid_argument);
  EXPECT_THROW(xfeat.extract(cv::Mat(64, 64, CV_8UC3, cv::Scalar(0))), std::invalid_argument);
  // Another model's outputs are refused.
  p.model = model("synthetic_global.onnx");
  EXPECT_THROW(XFeatFeatures{p}, std::runtime_error);
}

TEST(OnnxGlobalDescriptor, FeedsTheModelRgbNormalizedAndResized)
{
  OnnxGlobalParams p;
  p.name = "eigenplaces";
  p.model = model("synthetic_global.onnx");
  const OnnxGlobalDescriptor native(p);
  // A uniform image: the synthetic model returns its normalized channel means, in RGB order.
  const cv::Mat bgr(48, 64, CV_8UC3, cv::Scalar(10, 100, 200));
  const std::array<double, 3> raw{
    (200.0 / 255.0 - 0.485) / 0.229, (100.0 / 255.0 - 0.456) / 0.224,
    (10.0 / 255.0 - 0.406) / 0.225};
  const double length = std::sqrt(raw[0] * raw[0] + raw[1] * raw[1] + raw[2] * raw[2]);
  const auto check = [&](const std::vector<float> & d) {
    ASSERT_EQ(d.size(), 3U);
    for (std::size_t c = 0; c < 3; ++c) {
      EXPECT_NEAR(d[c], raw[c] / length, 1e-5) << c;
    }
    EXPECT_NEAR(norm(d), 1.0, 1e-6);
  };
  check(native.compute(bgr, nullptr).dense);
  cv::Mat bgra;
  cv::cvtColor(bgr, bgra, cv::COLOR_BGR2BGRA);
  check(native.compute(bgra, nullptr).dense);
  p.width = 32;  // a fixed input size: a uniform image keeps its means
  p.height = 24;
  check(OnnxGlobalDescriptor(p).compute(bgr, nullptr).dense);
  // Gray: three equal channels.
  const auto gray = native.compute(cv::Mat(48, 64, CV_8UC1, cv::Scalar(128)), nullptr).dense;
  const double g = 128.0 / 255.0;
  EXPECT_NEAR(gray[0] / gray[1], ((g - 0.485) / 0.229) / ((g - 0.456) / 0.224), 1e-4);
  const auto id = native.id();
  EXPECT_EQ(id.name, "eigenplaces");
  EXPECT_EQ(
    id.config,
    "height: 0\nmean: 0.485 0.456 0.406\nmodel: synthetic_global.onnx\nrgb: true\n"
    "std: 0.229 0.224 0.225\nwidth: 0\n");
  EXPECT_EQ(id.model_sha256, *landfall::sha256File(p.model));
  EXPECT_THROW(native.compute(cv::Mat(), nullptr), std::invalid_argument);
}

TEST(OnnxBackends, CreatedFromWhatTheDatabaseStores)
{
  landfall::BackendRegistry registry;
  landfall::registerBuiltinBackends(registry);
  landfall::onnx::registerBackends(registry);
  const auto withDir = [](BackendOptions o) {
    o[landfall::kModelsDirOption] = kData;
    return o;
  };
  const auto xfeat =
    registry.createLocalFeatures("xfeat", withDir({{"model", "synthetic_xfeat.onnx"}}));
  EXPECT_EQ(
    registry
      .createLocalFeatures("xfeat", withDir(landfall::parseBackendConfig(xfeat->id().config)))
      ->id()
      .config,
    xfeat->id().config);
  const auto global = registry.createGlobalDescriptor(
    "eigenplaces",
    withDir({{"model", "synthetic_global.onnx"}, {"width", "32"}, {"height", "24"}}));
  EXPECT_EQ(
    registry
      .createGlobalDescriptor(
        "eigenplaces", withDir(landfall::parseBackendConfig(global->id().config)))
      ->id()
      .config,
    global->id().config);
  EXPECT_TRUE(registry.hasGlobalDescriptor("megaloc"));
  EXPECT_FALSE(registry.hasGlobalDescriptor("bow"));  // built from the vocabulary, not here

  // A typo, a changed fixed property, an unknown provider, a missing model.
  EXPECT_THROW(
    registry.createLocalFeatures(
      "xfeat", withDir({{"model", "synthetic_xfeat.onnx"}, {"top_k", "5"}})),
    std::invalid_argument);
  EXPECT_THROW(
    registry.createGlobalDescriptor(
      "eigenplaces", withDir({{"model", "synthetic_global.onnx"}, {"mean", "0.5 0.5 0.5"}})),
    std::invalid_argument);
  EXPECT_THROW(
    registry.createGlobalDescriptor("megaloc", withDir({{landfall::kProviderOption, "gpu"}})),
    std::invalid_argument);
  EXPECT_THROW(registry.createGlobalDescriptor("megaloc", withDir({})), std::runtime_error);
  // Classical backends accept the runtime options.
  EXPECT_NO_THROW(registry.createLocalFeatures("orb", withDir({{landfall::kThreadsOption, "1"}})));
}

TEST(OnnxBackends, ModelsDirectory)
{
  EXPECT_EQ(landfall::modelsDir("/models"), "/models");
  ASSERT_EQ(setenv("LANDFALL_MODELS_DIR", "/from/env", 1), 0);
  EXPECT_EQ(landfall::modelsDir(""), "/from/env");
  ASSERT_EQ(unsetenv("LANDFALL_MODELS_DIR"), 0);
  ASSERT_EQ(setenv("HOME", "/home/someone", 1), 0);
  EXPECT_EQ(landfall::modelsDir(""), "/home/someone/.local/share/landfall/models");
}
