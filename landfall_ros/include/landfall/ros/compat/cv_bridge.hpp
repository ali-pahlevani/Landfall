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

#ifndef LANDFALL_ROS_COMPAT_CV_BRIDGE_HPP_
#define LANDFALL_ROS_COMPAT_CV_BRIDGE_HPP_

// cv_bridge 3.2 (Humble) ships only cv_bridge.h; 4.x (Jazzy, Lyrical) ships only cv_bridge.hpp.
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#endif  // LANDFALL_ROS_COMPAT_CV_BRIDGE_HPP_
