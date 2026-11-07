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

// The IsLocalized behavior tree condition against a latched pose publisher like AMCL's.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>

#include "bt_test_support.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "landfall/nav2/is_localized_condition.hpp"

namespace
{

using landfall::nav2::IsLocalized;
using landfall::nav2::test::makeBlackboard;
using landfall::nav2::test::makeBtNode;
using landfall::nav2::test::tickOnce;
using PoseMsg = geometry_msgs::msg::PoseWithCovarianceStamped;
using namespace std::chrono_literals;

constexpr double kPi = 3.14159265358979323846;

std::string treeXml(const std::string & attributes)
{
  return std::string(
#if LANDFALL_BTCPP_V4
           R"(<root BTCPP_format="4" main_tree_to_execute="Main">)"
#else
           R"(<root main_tree_to_execute="Main">)"
#endif
           R"(<BehaviorTree ID="Main"><IsLocalized )") +
         attributes + R"(/></BehaviorTree></root>)";
}

PoseMsg pose(double std_xy, double std_yaw_deg, const rclcpp::Time & stamp)
{
  PoseMsg msg;
  msg.header.frame_id = "map";
  msg.header.stamp = stamp;
  msg.pose.pose.orientation.w = 1.0;
  msg.pose.covariance[0] = std_xy * std_xy;
  msg.pose.covariance[7] = std_xy * std_xy;
  const double std_yaw = std_yaw_deg * kPi / 180.0;
  msg.pose.covariance[35] = std_yaw * std_yaw;
  return msg;
}

class IsLocalizedTest : public landfall::nav2::test::RosTest
{
protected:
  void SetUp() override
  {
    node_ = makeBtNode("bt_test");
    blackboard_ = makeBlackboard(node_);
    factory_.registerNodeType<IsLocalized>("IsLocalized");
    publisher_node_ = std::make_shared<rclcpp::Node>("pose_publisher");
    // As AMCL publishes amcl_pose: reliable, transient local, depth 1.
    publisher_ = publisher_node_->create_publisher<PoseMsg>(
      "amcl_pose", rclcpp::QoS(1).reliable().transient_local());
  }

  BT::Tree tree(const std::string & attributes = "")
  {
    return factory_.createTreeFromText(treeXml(attributes), blackboard_);
  }

  // Ticks until the condition gives `expected` (the pose has to arrive first) or `limit` passes.
  static BT::NodeStatus tickFor(
    BT::Tree & tree, BT::NodeStatus expected, std::chrono::milliseconds limit = 3s)
  {
    BT::NodeStatus status = BT::NodeStatus::IDLE;
    const auto end = std::chrono::steady_clock::now() + limit;
    while (std::chrono::steady_clock::now() < end) {
      status = tickOnce(tree);
      if (status == expected) {
        break;
      }
      std::this_thread::sleep_for(20ms);
    }
    return status;
  }

  std::shared_ptr<landfall::nav2::bt::RosNode> node_;
  BT::Blackboard::Ptr blackboard_;
  BT::BehaviorTreeFactory factory_;
  rclcpp::Node::SharedPtr publisher_node_;
  rclcpp::Publisher<PoseMsg>::SharedPtr publisher_;
};

TEST_F(IsLocalizedTest, FailsWithoutAPose)
{
  BT::Tree t = tree();
  for (int i = 0; i < 20; ++i) {
    EXPECT_EQ(tickOnce(t), BT::NodeStatus::FAILURE);
    std::this_thread::sleep_for(10ms);
  }
}

TEST_F(IsLocalizedTest, GetsALatchedPosePublishedBeforeTheTreeWasLoaded)
{
  publisher_->publish(pose(0.1, 3.0, node_->now()));
  BT::Tree t = tree();
  EXPECT_EQ(tickFor(t, BT::NodeStatus::SUCCESS), BT::NodeStatus::SUCCESS);
}

TEST_F(IsLocalizedTest, SucceedsOnItsFirstTick)
{
  // In a navigation tree the condition is ticked once per goal, long after the tree was loaded:
  // that one tick must see the pose AMCL latched before.
  publisher_->publish(pose(0.1, 3.0, node_->now()));
  BT::Tree t = tree();
  std::this_thread::sleep_for(1s);
  EXPECT_EQ(tickOnce(t), BT::NodeStatus::SUCCESS);
}

TEST_F(IsLocalizedTest, FailsWhileThePoseIsUncertain)
{
  BT::Tree t = tree(R"(max_std_xy="0.5" max_std_yaw_deg="15")");
  publisher_->publish(pose(0.8, 3.0, node_->now()));
  EXPECT_EQ(tickFor(t, BT::NodeStatus::SUCCESS, 1s), BT::NodeStatus::FAILURE);
  publisher_->publish(pose(0.1, 20.0, node_->now()));
  EXPECT_EQ(tickFor(t, BT::NodeStatus::SUCCESS, 1s), BT::NodeStatus::FAILURE);
  publisher_->publish(pose(0.45, 14.0, node_->now()));
  EXPECT_EQ(tickFor(t, BT::NodeStatus::SUCCESS), BT::NodeStatus::SUCCESS);
}

TEST_F(IsLocalizedTest, MaxAgeRejectsAnOldPose)
{
  BT::Tree t = tree(R"(max_age="2.0")");
  publisher_->publish(pose(0.1, 3.0, node_->now() - rclcpp::Duration(5s)));
  EXPECT_EQ(tickFor(t, BT::NodeStatus::SUCCESS, 1s), BT::NodeStatus::FAILURE);
  publisher_->publish(pose(0.1, 3.0, node_->now()));
  EXPECT_EQ(tickFor(t, BT::NodeStatus::SUCCESS), BT::NodeStatus::SUCCESS);
}

TEST_F(IsLocalizedTest, ReadsAnotherTopic)
{
  auto other = publisher_node_->create_publisher<PoseMsg>(
    "localization_pose", rclcpp::QoS(1).reliable().transient_local());
  other->publish(pose(0.1, 3.0, node_->now()));
  BT::Tree t = tree(R"(pose_topic="localization_pose")");
  EXPECT_EQ(tickFor(t, BT::NodeStatus::SUCCESS), BT::NodeStatus::SUCCESS);
}

TEST(IsLocalizedStatic, WorstStdXyIsTheLargerAxisOfTheCovariance)
{
  IsLocalized::Covariance c{};
  c[0] = 0.04;  // x: 0.2 m
  c[7] = 0.09;  // y: 0.3 m
  EXPECT_NEAR(IsLocalized::worstStdXy(c), 0.3, 1e-12);
  // Correlated: [[0.05, 0.04], [0.04, 0.05]] has eigenvalues 0.01 and 0.09.
  c[0] = 0.05;
  c[7] = 0.05;
  c[1] = 0.04;
  c[6] = 0.04;
  EXPECT_NEAR(IsLocalized::worstStdXy(c), 0.3, 1e-12);
  c[35] = 0.1 * 0.1;
  EXPECT_NEAR(IsLocalized::stdYawDeg(c), 0.1 * 180.0 / kPi, 1e-9);
}

}  // namespace
