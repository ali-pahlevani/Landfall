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

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <string>
#include <vector>

#include "landfall/ros/compat/rosbag2.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/time.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{

// Fresh bag path inside the system temp directory; removed on destruction.
class TempBagPath
{
public:
  TempBagPath()
  {
    const auto unique = std::chrono::steady_clock::now().time_since_epoch().count();
    path_ =
      std::filesystem::temp_directory_path() / ("landfall_compat_bag_" + std::to_string(unique));
  }
  ~TempBagPath() { std::filesystem::remove_all(path_); }
  TempBagPath(const TempBagPath &) = delete;
  TempBagPath & operator=(const TempBagPath &) = delete;

  std::string str() const { return path_.string(); }

private:
  std::filesystem::path path_;
};

rosbag2_storage::StorageOptions sqliteStorage(const std::string & uri)
{
  rosbag2_storage::StorageOptions options;
  options.uri = uri;
  options.storage_id = "sqlite3";  // readable on every supported distro by default
  return options;
}

}  // namespace

TEST(CompatRosbag2, ReceiveTimestampMatchesWrittenTime)
{
  TempBagPath bag;
  const std::vector<int64_t> stamps_ns = {1'000'000'000, 1'500'000'000, 2'250'000'000};

  {
    rosbag2_cpp::Writer writer;
    writer.open(sqliteStorage(bag.str()));
    for (const int64_t stamp : stamps_ns) {
      std_msgs::msg::String msg;
      msg.data = "probe " + std::to_string(stamp);
      writer.write(msg, "/landfall/probe", rclcpp::Time(stamp));
    }
  }

  rosbag2_cpp::Reader reader;
  reader.open(sqliteStorage(bag.str()));
  rclcpp::Serialization<std_msgs::msg::String> serialization;
  std::vector<int64_t> read_stamps;
  while (reader.has_next()) {
    const auto message = reader.read_next();
    EXPECT_EQ(message->topic_name, "/landfall/probe");
    read_stamps.push_back(landfall::ros::compat::receiveTimestamp(*message));

    const rclcpp::SerializedMessage serialized(*message->serialized_data);
    std_msgs::msg::String decoded;
    serialization.deserialize_message(&serialized, &decoded);
    EXPECT_EQ(decoded.data, "probe " + std::to_string(read_stamps.back()));
  }
  EXPECT_EQ(read_stamps, stamps_ns);
}
