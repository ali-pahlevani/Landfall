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

#ifndef BT_TEST_SUPPORT_HPP_
#define BT_TEST_SUPPORT_HPP_

// Test support for landfall_nav2: a blackboard like the one Nav2's navigator builds, ticking a
// tree on BehaviorTree.CPP v3 or v4, and a scripted Relocalize server.

#include <gtest/gtest.h>

#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "landfall/nav2/compat/bt.hpp"
#include "landfall_msgs/srv/relocalize.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.hpp"

namespace landfall::nav2::test
{

using namespace std::chrono_literals;

class RosTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

/// The node Nav2's navigator puts on the blackboard on this distro.
inline std::shared_ptr<bt::RosNode> makeBtNode(const std::string & name)
{
  return std::make_shared<bt::RosNode>(name, rclcpp::NodeOptions());
}

/// The entries Nav2's navigator puts on the blackboard (nav2_behavior_tree's BtActionServer and
/// the navigators).
inline BT::Blackboard::Ptr makeBlackboard(const std::shared_ptr<bt::RosNode> & node)
{
  auto blackboard = BT::Blackboard::create();
  blackboard->set<std::shared_ptr<bt::RosNode>>("node", node);
  blackboard->set<std::chrono::milliseconds>("server_timeout", 20ms);
  blackboard->set<std::chrono::milliseconds>("cancel_timeout", 50ms);
  blackboard->set<std::chrono::milliseconds>("bt_loop_duration", 10ms);
  blackboard->set<std::chrono::milliseconds>("wait_for_service_timeout", 1000ms);
  blackboard->set<int>("number_recoveries", 0);
  blackboard->set<bool>("initial_pose_received", false);
  blackboard->set<std::shared_ptr<tf2_ros::Buffer>>(
    "tf_buffer", std::make_shared<tf2_ros::Buffer>(node->get_clock()));
  return blackboard;
}

inline BT::NodeStatus tickOnce(BT::Tree & tree)
{
#if LANDFALL_BTCPP_V4
  return tree.tickOnce();
#else
  return tree.tickRoot();
#endif
}

/// Ticks every 10 ms (Nav2's default loop) until the tree is no longer RUNNING or `limit` passes.
inline BT::NodeStatus tickUntilDone(BT::Tree & tree, std::chrono::milliseconds limit = 30s)
{
  const auto end = std::chrono::steady_clock::now() + limit;
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  while (std::chrono::steady_clock::now() < end) {
    status = tickOnce(tree);
    if (status != BT::NodeStatus::RUNNING) {
      return status;
    }
    std::this_thread::sleep_for(10ms);
  }
  return status;
}

/// A scripted answer of MockRelocalizer: the status, after a delay.
struct MockAnswer
{
  uint8_t status = landfall_msgs::msg::RelocalizationResult::SUCCESS;
  std::chrono::milliseconds delay{0};
};

/// A Relocalize server on its own node and thread. Each request takes the next scripted answer
/// (the last one repeats) and is recorded.
class MockRelocalizer
{
public:
  using Service = landfall_msgs::srv::Relocalize;
  using Result = landfall_msgs::msg::RelocalizationResult;
  using Answer = MockAnswer;

  explicit MockRelocalizer(
    std::vector<Answer> answers = {Answer{}},
    const std::string & service = "landfall_relocalizer/relocalize")
  : answers_(answers.begin(), answers.end()),
    node_(std::make_shared<rclcpp::Node>("mock_relocalizer"))
  {
    service_ = node_->create_service<Service>(
      service, [this](
                 const std::shared_ptr<Service::Request> request,
                 std::shared_ptr<Service::Response> response) { answer(*request, *response); });
    executor_.add_node(node_);
    thread_ = std::thread([this] { executor_.spin(); });
  }

  ~MockRelocalizer()
  {
    executor_.cancel();
    thread_.join();
  }

  MockRelocalizer(const MockRelocalizer &) = delete;
  MockRelocalizer & operator=(const MockRelocalizer &) = delete;

  std::vector<Service::Request> requests() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return requests_;
  }

private:
  void answer(const Service::Request & request, Service::Response & response)
  {
    Answer answer;
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      requests_.push_back(request);
      answer = answers_.front();
      if (answers_.size() > 1) {
        answers_.pop_front();
      }
    }
    std::this_thread::sleep_for(answer.delay);
    response.result.status = answer.status;
    response.result.trigger = "service";
    response.result.message = "mock";
    response.result.published = answer.status == Result::SUCCESS && !request.dry_run;
    if (answer.status == Result::SUCCESS) {
      response.result.pose.header.frame_id = "map";
      response.result.pose.pose.pose.position.x = 1.5;
      response.result.pose.pose.pose.position.y = -2.0;
      response.result.pose.pose.pose.orientation.w = 1.0;
    }
  }

  mutable std::mutex mutex_;
  std::deque<Answer> answers_;
  std::vector<Service::Request> requests_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<Service>::SharedPtr service_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread thread_;
};

}  // namespace landfall::nav2::test

#endif  // BT_TEST_SUPPORT_HPP_
