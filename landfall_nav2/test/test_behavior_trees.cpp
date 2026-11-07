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

// The example behavior tree of this Nav2 release loads as Nav2's navigator loads it: with all of
// Nav2's behavior tree plugins and Landfall's plugin library, on a blackboard like the
// navigator's, with the servers its nodes connect to (docs/PLAN.md M6). Placeholder servers stand
// in for Nav2's: Nav2's action and service nodes wait for them when the tree is created.

#include <gtest/gtest.h>

#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "bt_test_support.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "nav2_msgs/action/wait.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "nav2_msgs/srv/is_path_valid.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#if LANDFALL_BTCPP_V4
#include <behaviortree_cpp/utils/shared_library.h>

#include "nav2_behavior_tree/plugins_list.hpp"
#else
#include <behaviortree_cpp_v3/utils/shared_library.h>
#endif

namespace
{

using landfall::nav2::test::makeBlackboard;
using landfall::nav2::test::makeBtNode;

// The plugin libraries Nav2's navigator loads by itself: from nav2_behavior_tree on Jazzy and
// later, from Humble's nav2_bringup/params/nav2_params.yaml (Nav2 1.1.20) on Humble, where they
// have to be listed.
std::vector<std::string> nav2Plugins()
{
#if LANDFALL_BTCPP_V4
  std::vector<std::string> plugins;
  std::stringstream list(nav2::details::BT_BUILTIN_PLUGINS);
  for (std::string plugin; std::getline(list, plugin, ';');) {
    plugins.push_back(plugin);
  }
  return plugins;
#else
  return {
    "nav2_compute_path_to_pose_action_bt_node",
    "nav2_compute_path_through_poses_action_bt_node",
    "nav2_smooth_path_action_bt_node",
    "nav2_follow_path_action_bt_node",
    "nav2_spin_action_bt_node",
    "nav2_wait_action_bt_node",
    "nav2_assisted_teleop_action_bt_node",
    "nav2_back_up_action_bt_node",
    "nav2_drive_on_heading_bt_node",
    "nav2_clear_costmap_service_bt_node",
    "nav2_is_stuck_condition_bt_node",
    "nav2_goal_reached_condition_bt_node",
    "nav2_goal_updated_condition_bt_node",
    "nav2_globally_updated_goal_condition_bt_node",
    "nav2_is_path_valid_condition_bt_node",
    "nav2_initial_pose_received_condition_bt_node",
    "nav2_reinitialize_global_localization_service_bt_node",
    "nav2_rate_controller_bt_node",
    "nav2_distance_controller_bt_node",
    "nav2_speed_controller_bt_node",
    "nav2_truncate_path_action_bt_node",
    "nav2_truncate_path_local_action_bt_node",
    "nav2_goal_updater_node_bt_node",
    "nav2_recovery_node_bt_node",
    "nav2_pipeline_sequence_bt_node",
    "nav2_round_robin_node_bt_node",
    "nav2_transform_available_condition_bt_node",
    "nav2_time_expired_condition_bt_node",
    "nav2_path_expiring_timer_condition",
    "nav2_distance_traveled_condition_bt_node",
    "nav2_single_trigger_bt_node",
    "nav2_goal_updated_controller_bt_node",
    "nav2_is_battery_low_condition_bt_node",
    "nav2_navigate_through_poses_action_bt_node",
    "nav2_navigate_to_pose_action_bt_node",
    "nav2_remove_passed_goals_action_bt_node",
    "nav2_planner_selector_bt_node",
    "nav2_controller_selector_bt_node",
    "nav2_goal_checker_selector_bt_node",
    "nav2_controller_cancel_bt_node",
    "nav2_path_longer_on_approach_bt_node",
    "nav2_wait_cancel_bt_node",
    "nav2_spin_cancel_bt_node",
    "nav2_back_up_cancel_bt_node",
    "nav2_assisted_teleop_cancel_bt_node",
    "nav2_drive_on_heading_cancel_bt_node",
    "nav2_is_battery_charging_condition_bt_node",
  };
#endif
}

template <typename ActionT>
typename rclcpp_action::Server<ActionT>::SharedPtr placeholderAction(
  const rclcpp::Node::SharedPtr & node, const std::string & name)
{
  using Handle = rclcpp_action::ServerGoalHandle<ActionT>;
  return rclcpp_action::create_server<ActionT>(
    node, name,
    [](const rclcpp_action::GoalUUID &, std::shared_ptr<const typename ActionT::Goal>) {
      return rclcpp_action::GoalResponse::REJECT;
    },
    [](const std::shared_ptr<Handle>) { return rclcpp_action::CancelResponse::ACCEPT; },
    [](const std::shared_ptr<Handle>) {});
}

template <typename ServiceT>
typename rclcpp::Service<ServiceT>::SharedPtr placeholderService(
  const rclcpp::Node::SharedPtr & node, const std::string & name)
{
  return node->create_service<ServiceT>(
    name, [](
            const std::shared_ptr<typename ServiceT::Request>,
            std::shared_ptr<typename ServiceT::Response>) {});
}

class BehaviorTreesTest : public landfall::nav2::test::RosTest
{
protected:
  void SetUp() override
  {
    servers_ = std::make_shared<rclcpp::Node>("nav2_placeholders");
    actions_.push_back(
      placeholderAction<nav2_msgs::action::ComputePathToPose>(servers_, "compute_path_to_pose"));
    actions_.push_back(placeholderAction<nav2_msgs::action::FollowPath>(servers_, "follow_path"));
    actions_.push_back(placeholderAction<nav2_msgs::action::Spin>(servers_, "spin"));
    actions_.push_back(placeholderAction<nav2_msgs::action::Wait>(servers_, "wait"));
    actions_.push_back(placeholderAction<nav2_msgs::action::BackUp>(servers_, "backup"));
    services_.push_back(placeholderService<nav2_msgs::srv::ClearEntireCostmap>(
      servers_, "global_costmap/clear_entirely_global_costmap"));
    services_.push_back(placeholderService<nav2_msgs::srv::ClearEntireCostmap>(
      servers_, "local_costmap/clear_entirely_local_costmap"));
    services_.push_back(placeholderService<nav2_msgs::srv::IsPathValid>(servers_, "is_path_valid"));
    executor_.add_node(servers_);
    spinner_ = std::thread([this] { executor_.spin(); });

    node_ = makeBtNode("bt_navigator_test");
    blackboard_ = makeBlackboard(node_);
  }

  void TearDown() override
  {
    executor_.cancel();
    spinner_.join();
  }

  rclcpp::Node::SharedPtr servers_;
  std::vector<std::shared_ptr<void>> actions_;
  std::vector<std::shared_ptr<void>> services_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spinner_;
  std::shared_ptr<landfall::nav2::bt::RosNode> node_;
  BT::Blackboard::Ptr blackboard_;
};

std::vector<const BT::TreeNode *> nodesNamed(const BT::TreeNode * root, const std::string & id)
{
  std::vector<const BT::TreeNode *> found;
  BT::applyRecursiveVisitor(root, [&](const BT::TreeNode * node) {
    if (node->registrationName() == id) {
      found.push_back(node);
    }
  });
  return found;
}

const BT::TreeNode * child(const BT::TreeNode * node, std::size_t index)
{
  const auto * control = dynamic_cast<const BT::ControlNode *>(node);
  if (control == nullptr || index >= control->children().size()) {
    return nullptr;
  }
  return control->children()[index];
}

TEST_F(BehaviorTreesTest, NavigateToPoseWithRelocalizationLoadsWithNav2sPlugins)
{
  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;
  for (const std::string & plugin : nav2Plugins()) {
    factory.registerFromPlugin(loader.getOSName(plugin));
  }
  factory.registerFromPlugin(LANDFALL_NAV2_PLUGIN);

  BT::Tree tree;
  ASSERT_NO_THROW(tree = factory.createTreeFromFile(LANDFALL_NAV2_TREE, blackboard_))
    << LANDFALL_NAV2_TREE;
  const BT::TreeNode * root = tree.rootNode();
  ASSERT_NE(root, nullptr);

  // Relocalizing from up to eight headings, before navigating and as a recovery.
  EXPECT_EQ(nodesNamed(root, "IsLocalized").size(), 1U);
  EXPECT_EQ(nodesNamed(root, "ReLocalize").size(), 16U);

  // Before navigating: Fallback[IsLocalized, Sequence[looking around, Wait, clear both costmaps]],
  // where looking around is Fallback[ReLocalize, Sequence[Spin, ReLocalize], ...].
  const BT::TreeNode * ensure = child(root, 0);
  ASSERT_NE(ensure, nullptr);
  EXPECT_EQ(ensure->name(), "EnsureLocalized");
  ASSERT_NE(child(ensure, 0), nullptr);
  EXPECT_EQ(child(ensure, 0)->registrationName(), "IsLocalized");
  const BT::TreeNode * before = child(ensure, 1);
  ASSERT_NE(before, nullptr);
  EXPECT_EQ(before->name(), "RelocalizeBeforeNavigating");
  const BT::TreeNode * around = child(before, 0);
  ASSERT_NE(around, nullptr);
  ASSERT_NE(child(around, 0), nullptr);
  EXPECT_EQ(child(around, 0)->registrationName(), "ReLocalize");
  ASSERT_NE(child(child(around, 1), 0), nullptr);
  EXPECT_EQ(child(child(around, 1), 0)->registrationName(), "Spin");
  ASSERT_NE(child(before, 3), nullptr);
  EXPECT_EQ(child(before, 1)->registrationName(), "Wait");
  EXPECT_EQ(child(before, 2)->registrationName(), "ClearEntireCostmap");
  EXPECT_EQ(child(before, 3)->registrationName(), "ClearEntireCostmap");

  // The first recovery: Sequence[looking around, Wait, clear both costmaps, plan again].
  std::vector<const BT::TreeNode *> recoveries;
  BT::applyRecursiveVisitor(root, [&](const BT::TreeNode * node) {
    if (node->name() == "RecoveryActions") {
      recoveries.push_back(node);
    }
  });
  ASSERT_EQ(recoveries.size(), 1U);
  const BT::TreeNode * first = child(recoveries[0], 0);
  ASSERT_NE(first, nullptr);
  EXPECT_EQ(first->name(), "RelocalizeAndReplan");
  ASSERT_NE(child(child(first, 0), 0), nullptr);
  EXPECT_EQ(child(child(first, 0), 0)->registrationName(), "ReLocalize");
  ASSERT_NE(child(first, 4), nullptr);
  EXPECT_EQ(child(first, 1)->registrationName(), "Wait");
  EXPECT_EQ(child(first, 2)->registrationName(), "ClearEntireCostmap");
  EXPECT_EQ(child(first, 3)->registrationName(), "ClearEntireCostmap");
  EXPECT_EQ(child(first, 4)->registrationName(), "ComputePathToPose");
}

}  // namespace
