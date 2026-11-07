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

#include "landfall/nav2/is_localized_condition.hpp"

#include <algorithm>
#include <cmath>

namespace landfall::nav2
{

namespace
{

using PoseMsg = geometry_msgs::msg::PoseWithCovarianceStamped;

// Indices of x, y and yaw in the row-major 6 x 6 covariance (x, y, z, roll, pitch, yaw).
constexpr std::size_t kX = 0;
constexpr std::size_t kY = 1;
constexpr std::size_t kYaw = 5;

double at(const IsLocalized::Covariance & c, std::size_t row, std::size_t col)
{
  return c[row * 6 + col];
}

}  // namespace

IsLocalized::IsLocalized(const std::string & name, const bt::NodeConfig & config)
: BT::ConditionNode(name, config),
  node_(bt::rosNode(config.blackboard)),
  callback_group_(node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false))
{
  executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
  std::string topic;
  bool latched = true;
  getInput("pose_topic", topic);
  getInput("latched", latched);
  subscribe(topic, latched);
}

BT::PortsList IsLocalized::providedPorts()
{
  return {
    BT::InputPort<std::string>("pose_topic", "amcl_pose", "The localization's pose topic"),
    BT::InputPort<double>(
      "max_std_xy", 0.5, "Largest position standard deviation, in m (worst direction)"),
    BT::InputPort<double>("max_std_yaw_deg", 15.0, "Largest yaw standard deviation, in degrees"),
    BT::InputPort<double>("max_age", 0.0, "Oldest pose accepted, in s (0: any)"),
    BT::InputPort<bool>("latched", true, "Subscribe transient-local, as AMCL publishes amcl_pose"),
  };
}

void IsLocalized::subscribe(const std::string & topic, bool latched)
{
  topic_ = topic;
  latched_ = latched;
  pose_.reset();
  rclcpp::QoS qos(1);
  qos.reliable();
  if (latched) {
    qos.transient_local();
  }
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  subscription_ = rclcpp::create_subscription<PoseMsg>(
    node_, topic, qos, [this](const PoseMsg & msg) { pose_ = msg; }, options);
  // An executor's first spin sets up what it waits on and, as seen with rclcpp 28, doesn't take a
  // message that arrived before it. Spinning once now, right after subscribing, makes the first
  // tick see AMCL's latched pose (test SucceedsOnItsFirstTick; Nav2's own conditions do the same).
  executor_.spin_some(std::chrono::nanoseconds(1));
}

BT::NodeStatus IsLocalized::tick()
{
  std::string topic;
  bool latched = true;
  getInput("pose_topic", topic);
  getInput("latched", latched);
  if (topic != topic_ || latched != latched_) {
    subscribe(topic, latched);
  }
  executor_.spin_some();
  if (!pose_) {
    return BT::NodeStatus::FAILURE;
  }

  double max_std_xy = 0.5;
  double max_std_yaw_deg = 15.0;
  double max_age = 0.0;
  getInput("max_std_xy", max_std_xy);
  getInput("max_std_yaw_deg", max_std_yaw_deg);
  getInput("max_age", max_age);
  const Covariance & covariance = pose_->pose.covariance;
  if (worstStdXy(covariance) > max_std_xy || stdYawDeg(covariance) > max_std_yaw_deg) {
    return BT::NodeStatus::FAILURE;
  }
  if (max_age > 0.0) {
    const rclcpp::Time stamp(pose_->header.stamp, node_->get_clock()->get_clock_type());
    if ((node_->now() - stamp).seconds() > max_age) {
      return BT::NodeStatus::FAILURE;
    }
  }
  return BT::NodeStatus::SUCCESS;
}

double IsLocalized::worstStdXy(const Covariance & covariance)
{
  // Larger eigenvalue of the symmetric 2 x 2 block [a b; b d].
  const double a = at(covariance, kX, kX);
  const double d = at(covariance, kY, kY);
  const double b = 0.5 * (at(covariance, kX, kY) + at(covariance, kY, kX));
  const double largest = 0.5 * (a + d) + std::hypot(0.5 * (a - d), b);
  return std::sqrt(std::max(0.0, largest));
}

double IsLocalized::stdYawDeg(const Covariance & covariance)
{
  return std::sqrt(std::max(0.0, at(covariance, kYaw, kYaw))) * 180.0 / M_PI;
}

}  // namespace landfall::nav2
