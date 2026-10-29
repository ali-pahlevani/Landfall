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

#ifndef LANDFALL_ROS_COMPAT_ROSBAG2_HPP_
#define LANDFALL_ROS_COMPAT_ROSBAG2_HPP_

#include <rcutils/time.h>

#include <type_traits>
#include <utility>

#include "rosbag2_storage/serialized_bag_message.hpp"

namespace landfall::ros::compat
{

namespace detail
{

template <typename T, typename = void>
struct HasRecvTimestamp : std::false_type
{
};

template <typename T>
struct HasRecvTimestamp<T, std::void_t<decltype(std::declval<const T &>().recv_timestamp)>>
: std::true_type
{
};

}  // namespace detail

/// Time at which the recorder received a bag message, in nanoseconds.
///
/// Humble's rosbag2 calls this field `time_stamp`; Iron and later renamed it `recv_timestamp`
/// (and added `send_timestamp`). Templated so only the branch matching the installed struct
/// is instantiated.
template <typename MessageT = rosbag2_storage::SerializedBagMessage>
rcutils_time_point_value_t receiveTimestamp(const MessageT & message)
{
  if constexpr (detail::HasRecvTimestamp<MessageT>::value) {
    return message.recv_timestamp;
  } else {
    return message.time_stamp;
  }
}

}  // namespace landfall::ros::compat

#endif  // LANDFALL_ROS_COMPAT_ROSBAG2_HPP_
