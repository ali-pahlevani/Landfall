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

#include "landfall/nav2/relocalize_action.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace landfall::nav2
{

namespace
{

using Result = landfall_msgs::msg::RelocalizationResult;

constexpr std::chrono::milliseconds kDefaultServerTimeout{20000};
constexpr std::chrono::milliseconds kServerTimeoutMargin{10000};
// A BUSY relocalizer is running another request (e.g. its startup relocalization): ask again
// after this long, while the server_timeout lasts.
constexpr std::chrono::milliseconds kBusyRetry{500};

double yawDeg(const geometry_msgs::msg::Quaternion & q)
{
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  return yaw * 180.0 / M_PI;
}

}  // namespace

ReLocalize::ReLocalize(const std::string & name, const bt::NodeConfig & config)
: BT::StatefulActionNode(name, config),
  node_(bt::rosNode(config.blackboard)),
  callback_group_(
    node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false)),
  // Spin at most half a tick while waiting for the answer, as Nav2's own nodes do.
  spin_timeout_(std::max(
    std::chrono::milliseconds(1),
    bt::blackboardDuration(config.blackboard, "bt_loop_duration", std::chrono::milliseconds(10)) /
      2)),
  default_wait_for_service_(bt::blackboardDuration(
    config.blackboard, "wait_for_service_timeout", std::chrono::milliseconds(1000)))
{
  executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
  getInput("service_name", service_name_);
  client_ = bt::createClient<Service>(node_, service_name_, callback_group_);
}

ReLocalize::~ReLocalize() { forget(); }

BT::PortsList ReLocalize::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "service_name", "landfall_relocalizer/relocalize", "The relocalizer's Relocalize service"),
    BT::InputPort<int>(
      "num_frames", 0, "Frames the request uses (0: the relocalizer's consistency.num_frames)"),
    BT::InputPort<double>(
      "timeout", 0.0, "Seconds the relocalizer may take (0: its query.timeout)"),
    BT::InputPort<bool>("dry_run", false, "Run every check but publish no pose"),
    BT::InputPort<std::chrono::milliseconds>(
      "server_timeout",
      "Milliseconds the node waits at most, from its start (default: timeout + 10 s, or 20 s for "
      "timeout 0); keep it above the relocalizer's query.timeout"),
    BT::InputPort<std::chrono::milliseconds>(
      "wait_for_service_timeout",
      "Milliseconds the service may take to appear (default: the blackboard's, else 1 s)"),
    BT::OutputPort<std::string>(
      "status",
      "SUCCESS, AMBIGUOUS, NO_MATCH, REJECTED_SANITY, REJECTED_SCAN_CHECK, "
      "REJECTED_CONSISTENCY, NOT_READY, BUSY; UNAVAILABLE or TIMEOUT from this node"),
    BT::OutputPort<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "pose", "The pose found, on SUCCESS"),
  };
}

std::string ReLocalize::statusName(uint8_t status)
{
  switch (status) {
    case Result::SUCCESS:
      return "SUCCESS";
    case Result::AMBIGUOUS:
      return "AMBIGUOUS";
    case Result::NO_MATCH:
      return "NO_MATCH";
    case Result::REJECTED_SANITY:
      return "REJECTED_SANITY";
    case Result::REJECTED_SCAN_CHECK:
      return "REJECTED_SCAN_CHECK";
    case Result::REJECTED_CONSISTENCY:
      return "REJECTED_CONSISTENCY";
    case Result::NOT_READY:
      return "NOT_READY";
    case Result::BUSY:
      return "BUSY";
    default:
      return "UNKNOWN_" + std::to_string(status);
  }
}

std::chrono::milliseconds ReLocalize::defaultServerTimeout(double timeout)
{
  if (timeout <= 0.0) {
    return kDefaultServerTimeout;
  }
  return std::chrono::milliseconds(static_cast<int64_t>(std::ceil(timeout * 1000.0))) +
         kServerTimeoutMargin;
}

BT::NodeStatus ReLocalize::onStart()
{
  forget();

  std::string service_name;
  getInput("service_name", service_name);
  if (service_name != service_name_) {
    service_name_ = service_name;
    client_ = bt::createClient<Service>(node_, service_name_, callback_group_);
  }

  int num_frames = 0;
  double timeout = 0.0;
  bool dry_run = false;
  getInput("num_frames", num_frames);
  getInput("timeout", timeout);
  getInput("dry_run", dry_run);
  request_ = std::make_shared<Service::Request>();
  request_->num_frames = static_cast<uint8_t>(std::clamp(num_frames, 0, 255));
  request_->timeout = static_cast<float>(std::max(0.0, timeout));
  request_->dry_run = dry_run;

  if (!getInput("server_timeout", server_timeout_)) {
    server_timeout_ = defaultServerTimeout(timeout);
  }
  if (!getInput("wait_for_service_timeout", wait_for_service_)) {
    wait_for_service_ = default_wait_for_service_;
  }
  started_ = node_->now();
  busy_since_.reset();
  return send();
}

BT::NodeStatus ReLocalize::send()
{
  if (busy_since_ && since(*busy_since_) < kBusyRetry) {
    return BT::NodeStatus::RUNNING;
  }
  busy_since_.reset();
  if (!client_->service_is_ready()) {
    if (since(started_) < wait_for_service_) {
      return BT::NodeStatus::RUNNING;
    }
    RCLCPP_WARN(
      node_->get_logger(), "ReLocalize %s: service %s is not available", name().c_str(),
      client_->get_service_name());
    setOutput("status", std::string("UNAVAILABLE"));
    return BT::NodeStatus::FAILURE;
  }
  auto sent = client_->async_send_request(request_);
  request_id_ = sent.request_id;
  future_ = sent.future.share();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ReLocalize::onRunning()
{
  if (!future_) {
    return send();
  }
  if (
    executor_.spin_until_future_complete(*future_, spin_timeout_) ==
    rclcpp::FutureReturnCode::SUCCESS) {
    const Service::Response::SharedPtr response = future_->get();
    future_.reset();
    if (
      response->result.status == landfall_msgs::msg::RelocalizationResult::BUSY &&
      since(started_) + kBusyRetry < server_timeout_) {
      busy_since_ = node_->now();
      return BT::NodeStatus::RUNNING;
    }
    return complete(response->result);
  }
  if (since(started_) >= server_timeout_) {
    RCLCPP_WARN(
      node_->get_logger(),
      "ReLocalize %s: no answer from %s within %ld ms (server_timeout; keep it above the "
      "relocalizer's query.timeout)",
      name().c_str(), client_->get_service_name(),
      static_cast<long>(server_timeout_.count()));  // NOLINT
    forget();
    setOutput("status", std::string("TIMEOUT"));
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void ReLocalize::onHalted()
{
  // The relocalizer finishes the request anyway (the service can't be cancelled); a new request
  // before then answers BUSY.
  forget();
}

BT::NodeStatus ReLocalize::complete(const landfall_msgs::msg::RelocalizationResult & result)
{
  const std::string status = statusName(result.status);
  setOutput("status", status);
  if (result.status == Result::SUCCESS) {
    setOutput("pose", result.pose);
    const auto & p = result.pose.pose.pose;
    RCLCPP_INFO(
      node_->get_logger(), "ReLocalize %s: SUCCESS at (%.2f, %.2f, %.1f deg)%s", name().c_str(),
      p.position.x, p.position.y, yawDeg(p.orientation), result.published ? "" : ", not published");
    return BT::NodeStatus::SUCCESS;
  }
  RCLCPP_WARN(
    node_->get_logger(), "ReLocalize %s: %s: %s", name().c_str(), status.c_str(),
    result.message.c_str());
  return BT::NodeStatus::FAILURE;
}

void ReLocalize::forget()
{
  if (future_) {
    client_->remove_pending_request(request_id_);
    future_.reset();
  }
}

std::chrono::milliseconds ReLocalize::since(const rclcpp::Time & start) const
{
  return (node_->now() - start).to_chrono<std::chrono::milliseconds>();
}

}  // namespace landfall::nav2
