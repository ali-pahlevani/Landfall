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

// Test helper of test_map_server_parity.py: compares the grids nav2_map_server publishes with
// landfall::loadMap() of the same YAML files, using this distro's map_server pixel rules.
//
//   map_parity_check <topic> <map.yaml> [<topic> <map.yaml>]...
//
// Exits 0 when every grid matches cell for cell, 1 otherwise.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "landfall/grid/map_loader.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);
  if (args.size() < 3 || (args.size() - 1) % 2 != 0) {
    std::fprintf(stderr, "usage: map_parity_check <topic> <map.yaml> [<topic> <map.yaml>]...\n");
    return 2;
  }
  const auto semantics = std::string(LANDFALL_ROS_DISTRO) == "humble"
                           ? landfall::MapServerSemantics::kHumble
                           : landfall::MapServerSemantics::kJazzy;
  auto node = std::make_shared<rclcpp::Node>("map_parity_check");
  std::map<std::string, nav_msgs::msg::OccupancyGrid> received;
  std::vector<rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr> subscriptions;
  for (std::size_t i = 1; i < args.size(); i += 2) {
    const std::string & topic = args[i];
    subscriptions.push_back(node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      topic, rclcpp::QoS(1).reliable().transient_local(),
      [&received, topic](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & grid) {
        received[topic] = *grid;
      }));
  }
  const std::size_t expected = (args.size() - 1) / 2;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(90);
  while (rclcpp::ok() && received.size() < expected &&
         std::chrono::steady_clock::now() < deadline) {
    executor.spin_some(std::chrono::milliseconds(50));
    rclcpp::sleep_for(std::chrono::milliseconds(50));
  }

  int failures = 0;
  for (std::size_t i = 1; i < args.size(); i += 2) {
    const std::string & topic = args[i];
    const std::string & yaml = args[i + 1];
    const auto it = received.find(topic);
    if (it == received.end()) {
      std::printf("FAIL %s: nothing received\n", topic.c_str());
      ++failures;
      continue;
    }
    const nav_msgs::msg::OccupancyGrid & published = it->second;
    const landfall::OccupancyGrid ours = landfall::loadMap(yaml, semantics).grid;
    const bool geometry = static_cast<int>(published.info.width) == ours.width &&
                          static_cast<int>(published.info.height) == ours.height &&
                          std::abs(published.info.resolution - ours.resolution) < 1e-6 &&
                          std::abs(published.info.origin.position.x - ours.origin.x) < 1e-6 &&
                          std::abs(published.info.origin.position.y - ours.origin.y) < 1e-6;
    int different = 0;
    int first = -1;
    if (geometry) {
      for (std::size_t c = 0; c < published.data.size(); ++c) {
        if (published.data[c] != ours.data[c]) {
          if (first < 0) {
            first = static_cast<int>(c);
          }
          ++different;
        }
      }
    }
    const bool ok = geometry && different == 0;
    std::printf(
      "%s %s (%s): %ux%u, %d differing cells", ok ? "OK  " : "FAIL", topic.c_str(), yaml.c_str(),
      published.info.width, published.info.height, geometry ? different : -1);
    if (first >= 0) {
      std::printf(
        "; first at %d: map_server %d, Landfall %d", first, published.data[first],
        ours.data[first]);
    }
    std::printf("\n");
    failures += ok ? 0 : 1;
  }
  rclcpp::shutdown();
  return failures == 0 ? 0 : 1;
}
