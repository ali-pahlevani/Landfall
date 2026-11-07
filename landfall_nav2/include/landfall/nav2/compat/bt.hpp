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

// What differs between the Nav2 releases Landfall supports, for its behavior tree nodes:
//
//                     Humble (Nav2 1.1)         Jazzy (1.3)               Lyrical (1.5)
//   BehaviorTree.CPP  v3 (behaviortree_cpp_v3)  v4 (behaviortree_cpp)     v4
//   blackboard "node" rclcpp::Node              rclcpp::Node              nav2::LifecycleNode
//   milliseconds port nav2 bt_conversions.hpp   nav2 bt_utils.hpp         nav2 bt_utils.hpp
//   rclcpp            16: clients take an rmw   28: rclcpp::QoS (rmw      32: rclcpp::QoS only
//                     QoS profile               profile deprecated)
//
// CMake decides from the Nav2 version and defines LANDFALL_BTCPP_V4 and
// LANDFALL_NAV2_LIFECYCLE_NODE (docs/PLAN.md §9, V1).

#ifndef LANDFALL_BTCPP_V4
#error "LANDFALL_BTCPP_V4 must be defined by CMake (0 = BehaviorTree.CPP v3, 1 = v4)"
#endif
#ifndef LANDFALL_NAV2_LIFECYCLE_NODE
#error "LANDFALL_NAV2_LIFECYCLE_NODE must be defined by CMake (1 = Nav2 1.4 or later)"
#endif

#include <memory>
#include <string>

#if LANDFALL_BTCPP_V4
#include <behaviortree_cpp/bt_factory.h>

#include "nav2_behavior_tree/bt_utils.hpp"  // convertFromString<std::chrono::milliseconds>
#else
#include <behaviortree_cpp_v3/bt_factory.h>

#include "nav2_behavior_tree/bt_conversions.hpp"  // convertFromString<std::chrono::milliseconds>
#endif

#if LANDFALL_NAV2_LIFECYCLE_NODE
#include "nav2_ros_common/lifecycle_node.hpp"
#endif
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/version.h"

namespace landfall::nav2::bt
{

#if LANDFALL_BTCPP_V4
using NodeConfig = BT::NodeConfig;
#else
using NodeConfig = BT::NodeConfiguration;
#endif

/// The ROS node Nav2's behavior tree navigator puts on the blackboard under "node".
#if LANDFALL_NAV2_LIFECYCLE_NODE
using RosNode = ::nav2::LifecycleNode;
#else
using RosNode = rclcpp::Node;
#endif

inline std::shared_ptr<RosNode> rosNode(const BT::Blackboard::Ptr & blackboard)
{
  return blackboard->get<std::shared_ptr<RosNode>>("node");
}

/// A blackboard duration (Nav2 stores "bt_loop_duration", "wait_for_service_timeout", ... as
/// std::chrono::milliseconds), or `fallback` when the blackboard doesn't have it.
inline std::chrono::milliseconds blackboardDuration(
  const BT::Blackboard::Ptr & blackboard, const std::string & key,
  std::chrono::milliseconds fallback)
{
  std::chrono::milliseconds value = fallback;
  if (!blackboard->get(key, value)) {  // the same call in v3 and v4
    return fallback;
  }
  return value;
}

/// A service client on the node's interfaces, in the given callback group, created without
/// waiting for the server.
template <typename ServiceT, typename NodePtr>
typename rclcpp::Client<ServiceT>::SharedPtr createClient(
  const NodePtr & node, const std::string & service_name, rclcpp::CallbackGroup::SharedPtr group)
{
#if RCLCPP_VERSION_MAJOR >= 28
  return rclcpp::create_client<ServiceT>(
    node->get_node_base_interface(), node->get_node_graph_interface(),
    node->get_node_services_interface(), service_name, rclcpp::ServicesQoS(), std::move(group));
#else
  return rclcpp::create_client<ServiceT>(
    node->get_node_base_interface(), node->get_node_graph_interface(),
    node->get_node_services_interface(), service_name, rmw_qos_profile_services_default,
    std::move(group));
#endif
}

}  // namespace landfall::nav2::bt

#endif  // LANDFALL_NAV2_COMPAT_BT_HPP_
