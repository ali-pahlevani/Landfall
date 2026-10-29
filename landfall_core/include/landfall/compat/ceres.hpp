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

#ifndef LANDFALL_COMPAT_CERES_HPP_
#define LANDFALL_COMPAT_CERES_HPP_

// Ceres 2.0 (Ubuntu 22.04 / Humble) only has LocalParameterization. Ceres 2.1 introduced
// Manifold, and 2.2 (Ubuntu 24.04 and 26.04 / Jazzy, Lyrical) removed LocalParameterization.
// These helpers express the parameterizations Landfall needs on every supported version.

#include <ceres/ceres.h>
#include <ceres/version.h>

#include <vector>

#if (CERES_VERSION_MAJOR > 2) || (CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR >= 1)
#define LANDFALL_CERES_HAS_MANIFOLD 1
#else
#define LANDFALL_CERES_HAS_MANIFOLD 0
#endif

namespace landfall::compat
{

/// Constrain a 4-element block holding Eigen quaternion coefficients (x, y, z, w) to unit norm.
/// The block must already be part of the problem.
inline void setEigenQuaternionManifold(ceres::Problem & problem, double * quaternion_xyzw)
{
#if LANDFALL_CERES_HAS_MANIFOLD
  problem.SetManifold(quaternion_xyzw, new ceres::EigenQuaternionManifold());
#else
  problem.SetParameterization(quaternion_xyzw, new ceres::EigenQuaternionParameterization());
#endif
}

/// Hold the listed coordinates of a parameter block constant while the others vary.
/// The block must already be part of the problem.
inline void setSubsetManifold(
  ceres::Problem & problem, double * values, int size, const std::vector<int> & constant_indices)
{
#if LANDFALL_CERES_HAS_MANIFOLD
  problem.SetManifold(values, new ceres::SubsetManifold(size, constant_indices));
#else
  problem.SetParameterization(values, new ceres::SubsetParameterization(size, constant_indices));
#endif
}

}  // namespace landfall::compat

#endif  // LANDFALL_COMPAT_CERES_HPP_
