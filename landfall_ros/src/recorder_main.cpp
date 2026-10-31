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

// Standalone recorder process. With `-p autostart:=true` it configures and activates itself;
// otherwise drive it with `ros2 lifecycle set` or nav2_lifecycle_manager.
//
// SIGINT/SIGTERM are handled here rather than by rclcpp, so the node's shutdown transition (which
// closes an open session) runs while ROS is still up.

#include <chrono>
#include <csignal>
#include <memory>

#include "landfall/ros/recorder_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
volatile std::sig_atomic_t g_stop = 0;
void requestStop(int) { g_stop = 1; }
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, requestStop);
  std::signal(SIGTERM, requestStop);
  auto node = std::make_shared<landfall::ros::RecorderNode>();
  if (node->get_parameter("autostart").as_bool()) {
    if (node->configure().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      RCLCPP_ERROR(node->get_logger(), "autostart: configuration failed");
    } else if (node->activate().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      RCLCPP_ERROR(node->get_logger(), "autostart: activation failed");
    }
  }
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  while (g_stop == 0 && rclcpp::ok()) {
    executor.spin_once(std::chrono::milliseconds(100));
  }
  // Closes an open session cleanly.
  if (node->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED) {
    node->shutdown();
  }
  executor.remove_node(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
