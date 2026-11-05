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

#ifndef LANDFALL_ROS_RELOCALIZER_NODE_HPP_
#define LANDFALL_ROS_RELOCALIZER_NODE_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "bondcpp/bond.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "landfall/query/localizer.hpp"
#include "landfall/ros/imu_gravity_source.hpp"
#include "landfall_msgs/msg/relocalization_result.hpp"
#include "landfall_msgs/srv/relocalize.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav2_msgs/srv/set_initial_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace landfall::ros
{

/// Visual relocalizer (plan §4.2, §7.7-7.16): on request (service `~/relocalize`, or once at
/// startup) it localizes camera images against a map database, checks the result, and publishes
/// the base pose to AMCL (`/initialpose`, or AMCL's `set_initial_pose` service). Requests run on a
/// worker thread, so sensor callbacks keep flowing in a single-threaded container.
///
/// Configure opens the database read-only and checks it; a missing or unusable database doesn't
/// fail the transition (it must not block AMCL's lifecycle manager), every request answers
/// NOT_READY with the reason instead.
class RelocalizerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using Relocalize = landfall_msgs::srv::Relocalize;
  using Result = landfall_msgs::msg::RelocalizationResult;

  explicit RelocalizerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~RelocalizerNode() override;

protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  /// Fixed for the node's lifetime (read-only parameters).
  struct Settings
  {
    std::string database_path;
    std::string map_yaml;
    std::string frame_global;
    std::string frame_odom;
    std::string frame_base;
    std::string frame_camera;
    std::string camera_info_topic;
    std::string image_topic;
    std::string image_transport;
    std::string imu_topic;
    std::string imu_frame;
    std::string output_mode;
    std::string output_topic;
    std::string output_service;
    std::string amcl_node;
    double image_time_offset = 0.0;
    double startup_delay = 2.0;
    double startup_retry_interval = 5.0;
    double extrinsics_warn_translation = 0.01;
    double extrinsics_warn_rotation_deg = 0.5;
    int threads = 0;
    int startup_max_attempts = 3;
    GravitySource gravity_source = GravitySource::kAuto;
    bool require_map_match = true;
    bool startup_enabled = false;
    bool bond_enabled = true;
  };

  /// Read at the start of every request, so they can change at runtime.
  struct Thresholds
  {
    LocalizerParams localizer;
    ConsistencyParams consistency;
    GravityParams gravity;
    double tf_timeout = 0.2;
    double timeout = 10.0;
    int num_frames = 3;
    double min_frame_interval = 0.3;
    double std_xy = 0.25;
    double std_yaw_deg = 5.0;
    bool compensate_motion = true;
    bool publish_image = true;
    bool publish_markers = true;
    std::string dump_dir;
  };

  struct Request
  {
    std::string trigger;   ///< service, startup
    int num_frames = 0;    ///< 0: parameter
    double timeout = 0.0;  ///< 0: parameter
    bool dry_run = false;
    std::shared_ptr<rmw_request_id_t> header;  ///< service requests: where the answer goes
  };

  struct PendingImage
  {
    int64_t stamp_ns = 0;
    std::function<cv::Mat()> decode;
  };

  struct MapCheck
  {
    bool received = false;
    std::string mismatch;  ///< empty when the map matches the database
  };

  void declareParameters();
  Settings readSettings() const;
  Thresholds readThresholds() const;
  rcl_interfaces::msg::SetParametersResult validateParameters(
    const std::vector<rclcpp::Parameter> & parameters) const;

  void loadDatabase();
  void onMap(const nav_msgs::msg::OccupancyGrid & map);
  void onImage(int64_t stamp_ns, std::function<cv::Mat()> decode);
  void onTfStatic(const tf2_msgs::msg::TFMessage & message);

  void submit(Request request);
  void answer(const Request & request, Result result);
  void stopWorker();
  void workerLoop();
  Result run(const Request & request);
  /// Why a request can't run now; nullopt when ready. Calls AMCL's get_state when `publishing`.
  std::optional<std::string> notReady(bool publishing);
  std::optional<std::string> notReadyLocal() const;
  /// The transform target <- source at `time`; nullopt when TF can't give it.
  std::optional<Pose3> lookup(
    const std::string & target, const std::string & source, tf2::TimePoint time) const;
  bool waitForTransform(
    const std::string & target, const std::string & source, int64_t stamp_ns, double timeout) const;
  bool publish(const geometry_msgs::msg::PoseWithCovarianceStamped & pose, std::string * error);
  void startupTick();
  void checkExtrinsics();
  bool staticChain(const std::string & a, const std::string & b) const;

  void publishMarkers(const FrameResult * frame, const Result & result);
  void publishDebugImage(
    const cv::Mat & image, const std::string & camera_frame, const FrameResult & frame,
    const Result & result);
  void dump(
    const Thresholds & t, int64_t stamp_ns, const cv::Mat & image, const FrameResult & frame);
  void diagnose(diagnostic_updater::DiagnosticStatusWrapper & status);

  Settings settings_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Database and localizer (worker thread after configure).
  std::shared_ptr<const LocalizationMap> map_;
  std::unique_ptr<Localizer> localizer_;
  std::string database_error_;

  // State shared between callbacks and the worker.
  mutable std::mutex mutex_;
  std::condition_variable images_cv_;
  MapCheck map_check_;
  std::string map_files_mismatch_;
  std::optional<sensor_msgs::msg::CameraInfo> camera_info_;
  std::optional<rclcpp::Time> last_image_;
  bool collecting_ = false;
  std::deque<PendingImage> images_;
  ImuGravitySource imu_;
  std::string imu_frame_;
  std::map<std::string, std::string> static_parent_;  ///< child -> parent from /tf_static
  std::string extrinsics_status_;
  int extrinsics_level_ = 0;  ///< diagnostic level of the startup check
  std::optional<Result> last_result_;

  // Requests: one runs at a time on the worker; another one meanwhile answers BUSY.
  std::thread worker_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::optional<Request> queued_;
  bool stop_worker_ = false;
  std::atomic<bool> busy_{false};
  std::atomic<bool> cancel_{false};
  int startup_attempts_ = 0;
  bool startup_done_ = false;
  std::chrono::steady_clock::time_point startup_next_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_pub_;
  rclcpp_lifecycle::LifecyclePublisher<Result>::SharedPtr status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
  rclcpp::Service<Relocalize>::SharedPtr relocalize_srv_;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr amcl_state_client_;
  rclcpp::Client<nav2_msgs::srv::SetInitialPose>::SharedPtr set_pose_client_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_handle_;
  std::unique_ptr<diagnostic_updater::Updater> diagnostics_;
  std::shared_ptr<bond::Bond> bond_;
};

}  // namespace landfall::ros

#endif  // LANDFALL_ROS_RELOCALIZER_NODE_HPP_
