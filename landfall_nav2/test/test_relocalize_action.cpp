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

// The ReLocalize behavior tree action against a scripted Relocalize server.

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>

#include "bt_test_support.hpp"
#include "landfall/nav2/relocalize_action.hpp"

namespace
{

using landfall::nav2::ReLocalize;
using landfall::nav2::test::makeBlackboard;
using landfall::nav2::test::makeBtNode;
using landfall::nav2::test::MockRelocalizer;
using landfall::nav2::test::tickOnce;
using landfall::nav2::test::tickUntilDone;
using Result = landfall_msgs::msg::RelocalizationResult;
using namespace std::chrono_literals;

std::string treeXml(const std::string & attributes)
{
  return std::string(
#if LANDFALL_BTCPP_V4
           R"(<root BTCPP_format="4" main_tree_to_execute="Main">)"
#else
           R"(<root main_tree_to_execute="Main">)"
#endif
           R"(<BehaviorTree ID="Main"><ReLocalize )") +
         attributes + R"( status="{status}" pose="{pose}"/></BehaviorTree></root>)";
}

class ReLocalizeTest : public landfall::nav2::test::RosTest
{
protected:
  void SetUp() override
  {
    node_ = makeBtNode("bt_test");
    blackboard_ = makeBlackboard(node_);
    factory_.registerNodeType<ReLocalize>("ReLocalize");
  }

  BT::Tree tree(const std::string & attributes = "")
  {
    return factory_.createTreeFromText(treeXml(attributes), blackboard_);
  }

  std::string status() const { return blackboard_->get<std::string>("status"); }

  std::shared_ptr<landfall::nav2::bt::RosNode> node_;
  BT::Blackboard::Ptr blackboard_;
  BT::BehaviorTreeFactory factory_;
};

TEST_F(ReLocalizeTest, SucceedsWithThePoseAndSendsThePortValues)
{
  const MockRelocalizer server;
  BT::Tree t = tree(R"(num_frames="2" timeout="4.5" dry_run="true")");
  EXPECT_EQ(tickUntilDone(t), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(status(), "SUCCESS");
  const auto pose = blackboard_->get<geometry_msgs::msg::PoseWithCovarianceStamped>("pose");
  EXPECT_DOUBLE_EQ(pose.pose.pose.position.x, 1.5);
  EXPECT_DOUBLE_EQ(pose.pose.pose.position.y, -2.0);
  const auto requests = server.requests();
  ASSERT_EQ(requests.size(), 1U);
  EXPECT_EQ(requests[0].num_frames, 2);
  EXPECT_FLOAT_EQ(requests[0].timeout, 4.5F);
  EXPECT_TRUE(requests[0].dry_run);
}

TEST_F(ReLocalizeTest, DefaultsLeaveTheRelocalizerItsOwnSettings)
{
  const MockRelocalizer server;
  BT::Tree t = tree();
  EXPECT_EQ(tickUntilDone(t), BT::NodeStatus::SUCCESS);
  const auto requests = server.requests();
  ASSERT_EQ(requests.size(), 1U);
  EXPECT_EQ(requests[0].num_frames, 0);
  EXPECT_FLOAT_EQ(requests[0].timeout, 0.0F);
  EXPECT_FALSE(requests[0].dry_run);
}

TEST_F(ReLocalizeTest, FailsOnEveryOtherAnswer)
{
  for (const uint8_t answer :
       {Result::AMBIGUOUS, Result::NO_MATCH, Result::REJECTED_SANITY, Result::REJECTED_SCAN_CHECK,
        Result::REJECTED_CONSISTENCY, Result::NOT_READY}) {
    const MockRelocalizer server({{answer, 0ms}});
    BT::Tree t = tree();
    EXPECT_EQ(tickUntilDone(t), BT::NodeStatus::FAILURE) << ReLocalize::statusName(answer);
    EXPECT_EQ(status(), ReLocalize::statusName(answer));
  }
}

TEST_F(ReLocalizeTest, LoadsWithoutTheServiceAndFailsWhenTicked)
{
  // Nav2's BtServiceNode throws here, which fails the whole tree; ReLocalize must not.
  BT::Tree t = tree(R"(wait_for_service_timeout="300")");
  const auto start = std::chrono::steady_clock::now();
  EXPECT_EQ(tickUntilDone(t), BT::NodeStatus::FAILURE);
  EXPECT_GE(std::chrono::steady_clock::now() - start, 300ms);
  EXPECT_EQ(status(), "UNAVAILABLE");
}

TEST_F(ReLocalizeTest, UsesAServiceThatAppearsAfterTheTreeIsLoaded)
{
  BT::Tree t = tree(R"(wait_for_service_timeout="5000")");
  EXPECT_EQ(tickOnce(t), BT::NodeStatus::RUNNING);
  const MockRelocalizer server;
  EXPECT_EQ(tickUntilDone(t), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(server.requests().size(), 1U);
}

TEST_F(ReLocalizeTest, GivesUpAfterTheServerTimeout)
{
  const MockRelocalizer server({{Result::SUCCESS, 1500ms}});
  BT::Tree t = tree(R"(server_timeout="400")");
  const auto start = std::chrono::steady_clock::now();
  EXPECT_EQ(tickUntilDone(t), BT::NodeStatus::FAILURE);
  const auto elapsed = std::chrono::steady_clock::now() - start;
  EXPECT_GE(elapsed, 400ms);
  EXPECT_LT(elapsed, 1400ms);
  EXPECT_EQ(status(), "TIMEOUT");
}

TEST_F(ReLocalizeTest, AsksABusyRelocalizerAgain)
{
  const MockRelocalizer server({{Result::BUSY, 0ms}, {Result::BUSY, 0ms}, {Result::SUCCESS, 0ms}});
  BT::Tree t = tree();
  const auto start = std::chrono::steady_clock::now();
  EXPECT_EQ(tickUntilDone(t), BT::NodeStatus::SUCCESS);
  EXPECT_GE(std::chrono::steady_clock::now() - start, 1000ms);  // two retries, 0.5 s apart
  EXPECT_EQ(server.requests().size(), 3U);
  EXPECT_EQ(status(), "SUCCESS");
}

TEST_F(ReLocalizeTest, ReportsBusyWhenTheServerTimeoutRunsOut)
{
  const MockRelocalizer server({{Result::BUSY, 0ms}});
  BT::Tree t = tree(R"(server_timeout="1200")");
  EXPECT_EQ(tickUntilDone(t), BT::NodeStatus::FAILURE);
  EXPECT_EQ(status(), "BUSY");
  EXPECT_GE(server.requests().size(), 2U);
}

TEST_F(ReLocalizeTest, HaltDropsTheRequestAndTheNextRunAsksAgain)
{
  const MockRelocalizer server({{Result::SUCCESS, 600ms}, {Result::SUCCESS, 0ms}});
  BT::Tree t = tree();
  EXPECT_EQ(tickOnce(t), BT::NodeStatus::RUNNING);
  std::this_thread::sleep_for(50ms);
  EXPECT_EQ(tickOnce(t), BT::NodeStatus::RUNNING);
  t.haltTree();
  std::this_thread::sleep_for(700ms);  // the first answer arrives and is dropped
  EXPECT_EQ(tickUntilDone(t), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(server.requests().size(), 2U);
}

TEST(ReLocalizeStatic, DefaultServerTimeoutCoversTheRequest)
{
  EXPECT_EQ(ReLocalize::defaultServerTimeout(0.0), 20000ms);
  EXPECT_EQ(ReLocalize::defaultServerTimeout(4.5), 14500ms);
  EXPECT_EQ(ReLocalize::defaultServerTimeout(30.0), 40000ms);
}

TEST(ReLocalizeStatic, NamesEveryStatus)
{
  EXPECT_EQ(ReLocalize::statusName(Result::SUCCESS), "SUCCESS");
  EXPECT_EQ(ReLocalize::statusName(Result::AMBIGUOUS), "AMBIGUOUS");
  EXPECT_EQ(ReLocalize::statusName(Result::NO_MATCH), "NO_MATCH");
  EXPECT_EQ(ReLocalize::statusName(Result::REJECTED_SANITY), "REJECTED_SANITY");
  EXPECT_EQ(ReLocalize::statusName(Result::REJECTED_SCAN_CHECK), "REJECTED_SCAN_CHECK");
  EXPECT_EQ(ReLocalize::statusName(Result::REJECTED_CONSISTENCY), "REJECTED_CONSISTENCY");
  EXPECT_EQ(ReLocalize::statusName(Result::NOT_READY), "NOT_READY");
  EXPECT_EQ(ReLocalize::statusName(Result::BUSY), "BUSY");
  EXPECT_EQ(ReLocalize::statusName(200), "UNKNOWN_200");
}

}  // namespace
