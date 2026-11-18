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
// in for Nav2's: Nav2's action and service nodes wait for them when the tree is created. Spin
// turns at once, and a mock relocalizer answers, so that the look-around runs as it would.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include "bt_test_support.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
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
using landfall::nav2::test::MockRelocalizer;
using landfall::nav2::test::tickUntilDone;
using Result = landfall_msgs::msg::RelocalizationResult;
using namespace std::chrono_literals;

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

// Accepts every goal and succeeds at once, counting the goals.
template <typename ActionT>
typename rclcpp_action::Server<ActionT>::SharedPtr succeedingAction(
  const rclcpp::Node::SharedPtr & node, const std::string & name, std::atomic<int> & goals)
{
  using Handle = rclcpp_action::ServerGoalHandle<ActionT>;
  return rclcpp_action::create_server<ActionT>(
    node, name,
    [&goals](const rclcpp_action::GoalUUID &, std::shared_ptr<const typename ActionT::Goal>) {
      ++goals;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](const std::shared_ptr<Handle>) { return rclcpp_action::CancelResponse::ACCEPT; },
    [](const std::shared_ptr<Handle> handle) {
      handle->succeed(std::make_shared<typename ActionT::Result>());
    });
}

// Nav2's controller when the localization's pose jumped while it followed a path: the path no
// longer reaches the robot, and its first goal fails (Jazzy and later: INVALID_PATH; Humble's
// result has no code). It follows every later path.
rclcpp_action::Server<nav2_msgs::action::FollowPath>::SharedPtr controllerAfterPoseJump(
  const rclcpp::Node::SharedPtr & node, std::atomic<int> & goals)
{
  using Action = nav2_msgs::action::FollowPath;
  using Handle = rclcpp_action::ServerGoalHandle<Action>;
  return rclcpp_action::create_server<Action>(
    node, "follow_path",
    [](const rclcpp_action::GoalUUID &, const std::shared_ptr<const Action::Goal> &) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](const std::shared_ptr<Handle> &) { return rclcpp_action::CancelResponse::ACCEPT; },
    [&goals](const std::shared_ptr<Handle> & handle) {
      auto result = std::make_shared<Action::Result>();
      if (++goals == 1) {
#if LANDFALL_BTCPP_V4
        result->error_code = Action::Result::INVALID_PATH;
#endif
        handle->abort(result);
      } else {
        handle->succeed(result);
      }
    });
}

// Nav2's planner: a short path in the map frame for every goal.
rclcpp_action::Server<nav2_msgs::action::ComputePathToPose>::SharedPtr planner(
  const rclcpp::Node::SharedPtr & node, std::atomic<int> & goals)
{
  using Action = nav2_msgs::action::ComputePathToPose;
  using Handle = rclcpp_action::ServerGoalHandle<Action>;
  return rclcpp_action::create_server<Action>(
    node, "compute_path_to_pose",
    [&goals](const rclcpp_action::GoalUUID &, const std::shared_ptr<const Action::Goal> &) {
      ++goals;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](const std::shared_ptr<Handle> &) { return rclcpp_action::CancelResponse::ACCEPT; },
    [](const std::shared_ptr<Handle> & handle) {
      auto result = std::make_shared<Action::Result>();
      result->path.header.frame_id = "map";
      for (int i = 0; i < 5; ++i) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = 0.5 * i;
        pose.pose.orientation.w = 1.0;
        result->path.poses.push_back(pose);
      }
      handle->succeed(result);
    });
}

template <typename T, typename = void>
struct HasSuccess : std::false_type
{
};
template <typename T>
struct HasSuccess<T, std::void_t<decltype(std::declval<T &>().success)>> : std::true_type
{
};

// Answers every call; a response with `success` (Nav2 1.5, Lyrical: ClearEntireCostmap) says it
// worked, as the costmap servers do.
template <typename ServiceT>
typename rclcpp::Service<ServiceT>::SharedPtr placeholderService(
  const rclcpp::Node::SharedPtr & node, const std::string & name)
{
  return node->create_service<ServiceT>(
    name, [](
            const std::shared_ptr<typename ServiceT::Request>,
            std::shared_ptr<typename ServiceT::Response> response) {
      if constexpr (HasSuccess<typename ServiceT::Response>::value) {
        response->success = true;
      }
    });
}

class BehaviorTreesTest : public landfall::nav2::test::RosTest
{
protected:
  void SetUp() override
  {
    servers_ = std::make_shared<rclcpp::Node>("nav2_placeholders");
    addPlannerControllerAndWait();
    actions_.push_back(succeedingAction<nav2_msgs::action::Spin>(servers_, "spin", spin_goals_));
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
    for (const std::string & plugin : nav2Plugins()) {
      factory_.registerFromPlugin(loader_.getOSName(plugin));
    }
    factory_.registerFromPlugin(LANDFALL_NAV2_PLUGIN);
  }

  // Placeholders that refuse their goals; the pose-jump tests replace them.
  virtual void addPlannerControllerAndWait()
  {
    actions_.push_back(
      placeholderAction<nav2_msgs::action::ComputePathToPose>(servers_, "compute_path_to_pose"));
    actions_.push_back(placeholderAction<nav2_msgs::action::FollowPath>(servers_, "follow_path"));
    actions_.push_back(placeholderAction<nav2_msgs::action::Wait>(servers_, "wait"));
  }

  BT::Tree createTree() { return factory_.createTreeFromFile(LANDFALL_NAV2_TREE, blackboard_); }

  void TearDown() override
  {
    executor_.cancel();
    spinner_.join();
  }

  std::atomic<int> spin_goals_{0};
  rclcpp::Node::SharedPtr servers_;
  std::vector<std::shared_ptr<void>> actions_;
  std::vector<std::shared_ptr<void>> services_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spinner_;
  std::shared_ptr<landfall::nav2::bt::RosNode> node_;
  BT::Blackboard::Ptr blackboard_;
  BT::SharedLibrary loader_;
  BT::BehaviorTreeFactory factory_;
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

const BT::TreeNode * decorated(const BT::TreeNode * node)
{
  const auto * decorator = dynamic_cast<const BT::DecoratorNode *>(node);
  return decorator == nullptr ? nullptr : decorator->child();
}

std::string inputPort(const BT::TreeNode * node, const std::string & port)
{
  const auto & ports = node->config().input_ports;
  const auto it = ports.find(port);
  return it == ports.end() ? std::string() : it->second;
}

TEST_F(BehaviorTreesTest, NavigateToPoseWithRelocalizationLoadsWithNav2sPlugins)
{
  BT::Tree tree;
  ASSERT_NO_THROW(tree = createTree()) << LANDFALL_NAV2_TREE;
  const BT::TreeNode * root = tree.rootNode();
  ASSERT_NE(root, nullptr);

  // Relocalizing from up to twelve headings, before navigating and as a recovery.
  EXPECT_EQ(nodesNamed(root, "IsLocalized").size(), 1U);
  EXPECT_EQ(nodesNamed(root, "ReLocalize").size(), 4U);
  for (const BT::TreeNode * retry : nodesNamed(root, "RetryUntilSuccessful")) {
    EXPECT_EQ(inputPort(retry, "num_attempts"), "11") << retry->name();
  }
  EXPECT_EQ(nodesNamed(root, "RetryUntilSuccessful").size(), 2U);

  // Before navigating: Fallback[IsLocalized, Sequence[looking around, Wait, clear both costmaps]],
  // where looking around is Fallback[ReLocalize, RetryUntilSuccessful[Sequence[Spin, ReLocalize]]].
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
  ASSERT_NE(child(around, 1), nullptr);
  EXPECT_EQ(child(around, 1)->registrationName(), "RetryUntilSuccessful");
  const BT::TreeNode * turn = decorated(child(around, 1));
  ASSERT_NE(child(turn, 1), nullptr);
  EXPECT_EQ(child(turn, 0)->registrationName(), "Spin");
  EXPECT_EQ(inputPort(child(turn, 0), "spin_dist"), "0.524");
  EXPECT_EQ(child(turn, 1)->registrationName(), "ReLocalize");
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
  ASSERT_NE(child(child(first, 0), 1), nullptr);
  EXPECT_EQ(child(child(first, 0), 1)->registrationName(), "RetryUntilSuccessful");
}

// Without a pose from AMCL and no match anywhere, the robot asks at its heading and after each of
// 11 turns, then gives up: the tree fails before navigating.
TEST_F(BehaviorTreesTest, LooksAroundFromTwelveHeadingsBeforeGivingUp)
{
  const MockRelocalizer relocalizer({{Result::NO_MATCH, 0ms}});
  BT::Tree tree = createTree();
  EXPECT_EQ(tickUntilDone(tree), BT::NodeStatus::FAILURE);
  EXPECT_EQ(relocalizer.requests().size(), 12U);
  EXPECT_EQ(spin_goals_.load(), 11);
}

// The first pose ends the look-around: the fifth request answers, after four turns.
TEST_F(BehaviorTreesTest, StopsLookingAroundAtTheFirstPose)
{
  const MockRelocalizer relocalizer(
    {{Result::NO_MATCH, 0ms},
     {Result::NO_MATCH, 0ms},
     {Result::NO_MATCH, 0ms},
     {Result::NO_MATCH, 0ms},
     {Result::SUCCESS, 0ms}});
  BT::Tree tree = createTree();
  // The placeholder Wait refuses its goal, so the tree fails after the look-around.
  tickUntilDone(tree);
  EXPECT_EQ(relocalizer.requests().size(), 5U);
  EXPECT_EQ(spin_goals_.load(), 4);
}

// The localization's pose jumps while the robot follows its path: the relocalizer's kidnap watchdog
// or auto trigger publishes a pose, or AMCL jumps by itself. The controller fails (INVALID_PATH),
// and the tree clears the costmaps, plans again from the new pose and follows the new path: the
// goal succeeds, without a relocalization of its own. The tree relocalizes once before navigating
// (IsLocalized has no pose yet on its first tick).
class PoseJumpTest : public BehaviorTreesTest
{
protected:
  void addPlannerControllerAndWait() override
  {
    actions_.push_back(planner(servers_, plan_goals_));
    actions_.push_back(controllerAfterPoseJump(servers_, follow_goals_));
    actions_.push_back(succeedingAction<nav2_msgs::action::Wait>(servers_, "wait", wait_goals_));
  }

  std::atomic<int> plan_goals_{0};
  std::atomic<int> follow_goals_{0};
  std::atomic<int> wait_goals_{0};
};

TEST_F(PoseJumpTest, PlansAgainWhenThePathNoLongerReachesTheRobot)
{
  const MockRelocalizer relocalizer({{Result::SUCCESS, 0ms}});
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.position.x = 2.0;
  goal.pose.orientation.w = 1.0;
  blackboard_->set<geometry_msgs::msg::PoseStamped>("goal", goal);
  BT::Tree tree = createTree();
  EXPECT_EQ(tickUntilDone(tree), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(follow_goals_.load(), 2);
  // Only the relocalization before navigating: the pose jump didn't need one.
  EXPECT_EQ(relocalizer.requests().size(), 1U);
#if LANDFALL_BTCPP_V4
  // The pipeline's plan, then the recovery's from the new pose (the pipeline plans once a second).
  EXPECT_GE(plan_goals_.load(), 2);
#else
  // Humble's tree has no error codes: its controller recovery clears the local costmap and follows
  // the path again.
  EXPECT_GE(plan_goals_.load(), 1);
#endif
}

}  // namespace
