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

#ifndef LANDFALL_NAV2_RELOCALIZE_ACTION_HPP_
#define LANDFALL_NAV2_RELOCALIZE_ACTION_HPP_

#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "landfall/nav2/compat/bt.hpp"
#include "landfall_msgs/msg/relocalization_result.hpp"
#include "landfall_msgs/srv/relocalize.hpp"
#include "rclcpp/rclcpp.hpp"

namespace landfall::nav2
{

/// Behavior tree action `ReLocalize`: asks the relocalizer for a pose
/// (landfall_msgs/srv/Relocalize) and succeeds when it answers SUCCESS. See
/// docs/nav2_integration.md.
///
/// Unlike Nav2's BtServiceNode, it doesn't wait for the service when the tree is loaded. A
/// relocalizer that comes up after the navigator, or not at all, makes this node fail when it is
/// ticked rather than the whole tree fail to load (docs/PLAN.md V46).
///
/// Ports: `service_name` (default "landfall_relocalizer/relocalize"), `num_frames`, `timeout`
/// and `dry_run` (the request's fields; 0 means the relocalizer's own setting),
/// `server_timeout` (ms the node waits at most, from its start; default `timeout` + 10 s, or
/// 20 s) and `wait_for_service_timeout` (ms the service may take to appear; default the
/// navigator's blackboard value, else 1 s). A BUSY answer (the relocalizer is running another
/// request) is asked again every 0.5 s while server_timeout lasts. Outputs: `status` (SUCCESS,
/// AMBIGUOUS, NO_MATCH, ...; also UNAVAILABLE and TIMEOUT from this node) and `pose` (on SUCCESS).
class ReLocalize : public BT::StatefulActionNode
{
public:
  ReLocalize(const std::string & name, const bt::NodeConfig & config);
  ~ReLocalize() override;

  ReLocalize(const ReLocalize &) = delete;
  ReLocalize & operator=(const ReLocalize &) = delete;

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  /// The relocalizer's status by name ("SUCCESS", "AMBIGUOUS", ...).
  static std::string statusName(uint8_t status);

  /// The server_timeout used when the tree doesn't set one, for a request `timeout` in s.
  static std::chrono::milliseconds defaultServerTimeout(double timeout);

private:
  using Service = landfall_msgs::srv::Relocalize;

  BT::NodeStatus send();
  BT::NodeStatus complete(const landfall_msgs::msg::RelocalizationResult & result);
  void forget();
  std::chrono::milliseconds since(const rclcpp::Time & start) const;

  std::shared_ptr<bt::RosNode> node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Client<Service>::SharedPtr client_;
  std::string service_name_;
  std::chrono::milliseconds spin_timeout_;
  std::chrono::milliseconds default_wait_for_service_;

  // The request in flight.
  Service::Request::SharedPtr request_;
  std::optional<std::shared_future<Service::Response::SharedPtr>> future_;
  int64_t request_id_ = 0;
  rclcpp::Time started_;
  std::optional<rclcpp::Time> busy_since_;  ///< answered BUSY at; asked again kBusyRetry later
  std::chrono::milliseconds server_timeout_{0};
  std::chrono::milliseconds wait_for_service_{0};
};

}  // namespace landfall::nav2

#endif  // LANDFALL_NAV2_RELOCALIZE_ACTION_HPP_
