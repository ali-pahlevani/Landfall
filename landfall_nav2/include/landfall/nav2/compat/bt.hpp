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

#ifndef LANDFALL_NAV2_COMPAT_BT_HPP_
#define LANDFALL_NAV2_COMPAT_BT_HPP_

// Nav2 on Humble uses BehaviorTree.CPP v3 (package behaviortree_cpp_v3); Iron and later use v4
// (package behaviortree_cpp). CMake decides from the Nav2 version and defines LANDFALL_BTCPP_V4.

#ifndef LANDFALL_BTCPP_V4
#error "LANDFALL_BTCPP_V4 must be defined by CMake (0 = BehaviorTree.CPP v3, 1 = v4)"
#endif

#if LANDFALL_BTCPP_V4
#include <behaviortree_cpp/bt_factory.h>
#else
#include <behaviortree_cpp_v3/bt_factory.h>
#endif

namespace landfall::nav2::bt
{

#if LANDFALL_BTCPP_V4
using NodeConfig = BT::NodeConfig;
#else
using NodeConfig = BT::NodeConfiguration;
#endif

}  // namespace landfall::nav2::bt

#endif  // LANDFALL_NAV2_COMPAT_BT_HPP_
