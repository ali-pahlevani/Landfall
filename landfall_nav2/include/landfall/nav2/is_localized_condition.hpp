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

#ifndef LANDFALL_NAV2_IS_LOCALIZED_CONDITION_HPP_
#define LANDFALL_NAV2_IS_LOCALIZED_CONDITION_HPP_

#include <array>
#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "landfall/nav2/compat/bt.hpp"
#include "rclcpp/rclcpp.hpp"

namespace landfall::nav2
{

/// Behavior tree condition `IsLocalized`: succeeds while the localization's latest pose (AMCL's
/// `amcl_pose`) is certain enough. See docs/nav2_integration.md.
///
/// Ports: `pose_topic` (default "amcl_pose"), `max_std_xy` (m, default 0.5, the worst direction
/// of the x/y covariance), `max_std_yaw_deg` (default 15), `max_age` (s, 0 = any age: AMCL
/// publishes only when its filter updates, so a still robot's pose ages) and `latched` (default
/// true: subscribe transient-local, as AMCL publishes, to get its last pose at once). The topic is
/// subscribed when the tree is loaded. Fails until a pose has arrived.
class IsLocalized : public BT::ConditionNode
{
public:
  IsLocalized(const std::string & name, const bt::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

  using Covariance = std::array<double, 36>;

  /// Standard deviation in the worst direction of x and y: the root of the larger eigenvalue of
  /// the covariance's x/y block (as the relocalizer's auto trigger computes it).
  static double worstStdXy(const Covariance & covariance);

  /// Standard deviation of the yaw, in degrees.
  static double stdYawDeg(const Covariance & covariance);

private:
  void subscribe(const std::string & topic, bool latched);

  std::shared_ptr<bt::RosNode> node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
  std::string topic_;
  bool latched_ = true;
  // Written by the subscription callback, which runs inside tick() (executor_.spin_some).
  std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> pose_;
};

}  // namespace landfall::nav2

#endif  // LANDFALL_NAV2_IS_LOCALIZED_CONDITION_HPP_
