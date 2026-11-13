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

#ifndef LANDFALL_ONNX_XFEAT_HPP_
#define LANDFALL_ONNX_XFEAT_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "landfall/interfaces/local_features.hpp"
#include "landfall/onnx/session.hpp"

namespace landfall::onnx
{

struct XFeatParams
{
  std::string model;                  ///< xfeat.onnx (landfall_tools/export/xfeat.py)
  int max_keypoints = 2048;           ///< best by score
  double detection_threshold = 0.05;  ///< on the keypoint heatmap (XFeat's default)
  double floor_mask_distance = 0.0;   ///< m; hides the near floor (kFloorMaskOption), 0: off
  SessionOptions session;
};

/// What the XFeat model computes for one image (the contract of landfall_tools/export/xfeat.py).
struct XFeatMaps
{
  int height = 0;  ///< model input rows, a multiple of 32
  int width = 0;
  std::vector<float> descriptors;  ///< 64 x height/8 x width/8, L2-normalized over the channels
  std::vector<float> heatmap;      ///< height x width keypoint probability
  std::vector<float> reliability;  ///< height/8 x width/8
  std::vector<uint8_t> is_max;     ///< height x width: the heatmap equals its 5x5 maximum
};

/// The rest of XFeat.detectAndCompute, step by step: keypoints where the heatmap is a 5x5 maximum
/// above the threshold (none at (0, 0), which XFeat reserves for padding), scored by the heatmap
/// (nearest) times the reliability (bilinear); the best max_keypoints with a positive score;
/// descriptors sampled bicubically and L2-normalized. Sampling follows
/// torch.nn.functional.grid_sample (align_corners=False, zero padding) on positions normalized by
/// (width - 1, height - 1), like XFeat's InterpolateSparse2d. Keypoints are in model-input pixels.
/// Keypoints and descriptors from the network's maps. `mask` (8-bit, the maps' height x width,
/// or empty): keypoints only where it is non-zero, before the best `max_keypoints` are chosen.
Features detectXFeat(
  const XFeatMaps & maps, int max_keypoints, double detection_threshold,
  const cv::Mat & mask = cv::Mat());

/// XFeat local features (verlab/accelerated_features, Apache-2.0) on ONNX Runtime: 64-D float
/// descriptors, matched with `mnn`. The image is resized to a multiple of 32 like XFeat does, and
/// keypoints are scaled back.
class XFeatFeatures : public ILocalFeatures
{
public:
  /// Throws std::runtime_error if the model can't be loaded or doesn't have XFeat's outputs.
  explicit XFeatFeatures(const XFeatParams & params);
  Features extract(const cv::Mat & gray, const ImageGeometry & geometry) const override;
  BackendId id() const override;

private:
  XFeatParams params_;
  std::string model_sha256_;
  std::unique_ptr<Session> session_;
  std::vector<int> output_index_;  ///< descriptors, heatmap, reliability, is_max
};

}  // namespace landfall::onnx

#endif  // LANDFALL_ONNX_XFEAT_HPP_
