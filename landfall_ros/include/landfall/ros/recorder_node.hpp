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

#ifndef LANDFALL_ROS_RECORDER_NODE_HPP_
#define LANDFALL_ROS_RECORDER_NODE_HPP_

#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "bondcpp/bond.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "landfall/record/keyframe_selector.hpp"
#include "landfall/record/staging.hpp"
#include "landfall/ros/frame_gate.hpp"
#include "landfall/ros/imu_gravity_source.hpp"
#include "landfall/ros/tf_pose_source.hpp"
#include "landfall_msgs/msg/recorder_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace landfall::ros
{

/// Keyframe recorder. Active = recording: every activation opens a new staging folder (session)
/// under `output_dir`, which `landfall build --staging` turns into a map database. Keyframes are
/// selected by the same gates as `landfall build --bag` (see frame_gate.hpp).
class RecorderNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit RecorderNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~RecorderNode() override;

protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  struct Settings
  {
    std::string output_dir;
    std::string session_name;
    StagingFrames frames;
    std::string camera_info_topic;
    std::string image_topic;
    std::string image_transport;
    double image_time_offset = 0.0;
    std::string image_format;
    int jpeg_quality = 95;
    std::string pose_source;
    std::string covariance_topic;
    double tf_timeout = 0.2;
    double speed_window = 0.2;
    GravitySource gravity_source = GravitySource::kAuto;
    std::string imu_topic;
    std::string imu_frame;  ///< overrides the messages' frame_id when set
    double imu_max_age = 0.1;
    KeyframeGating gating;
    bool bond_enabled = true;
  };

  struct PendingImage
  {
    int64_t stamp_ns = 0;
    rclcpp::Time arrival;
    std::function<cv::Mat()> decode;
  };

  void declareParameters();
  Settings readSettings();
  void enqueue(int64_t stamp_ns, std::function<cv::Mat()> decode);
  void processPending();
  void handle(const PendingImage & image);
  bool openSession(const sensor_msgs::msg::CameraInfo & info);
  void closeSession();
  void publishStatus();
  void diagnose(diagnostic_updater::DiagnosticStatusWrapper & status);

  Settings settings_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<TfPoseSource> poses_;
  ImuGravitySource imu_;
  std::optional<Eigen::Quaterniond> q_base_imu_;
  std::string imu_frame_;
  std::unique_ptr<KeyframeSelector> selector_;
  std::unique_ptr<StagingWriter> writer_;
  std::optional<sensor_msgs::msg::CameraInfo> camera_info_;
  std::deque<PendingImage> pending_;
  bool paused_ = false;
  bool limit_logged_ = false;
  std::optional<rclcpp::Time> last_image_;
  std::optional<geometry_msgs::msg::PoseStamped> last_keyframe_pose_;
  std::string last_tf_error_;
  visualization_msgs::msg::MarkerArray markers_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr covariance_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp_lifecycle::LifecyclePublisher<landfall_msgs::msg::RecorderStatus>::SharedPtr status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pause_srv_;
  rclcpp::TimerBase::SharedPtr process_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  std::unique_ptr<diagnostic_updater::Updater> diagnostics_;
  std::shared_ptr<bond::Bond> bond_;
};

}  // namespace landfall::ros

#endif  // LANDFALL_ROS_RECORDER_NODE_HPP_
