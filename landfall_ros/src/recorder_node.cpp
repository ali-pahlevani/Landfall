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

#include "landfall/ros/recorder_node.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <initializer_list>
#include <stdexcept>
#include <utility>

#include "landfall/ros/conversions.hpp"
#include "landfall/version.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/exceptions.hpp"

namespace landfall::ros
{

namespace
{

using rcl_interfaces::msg::ParameterDescriptor;

ParameterDescriptor describe(const std::string & description)
{
  ParameterDescriptor d;
  d.description = description;
  d.read_only = true;  // fixed for the node's lifetime: set them at startup
  return d;
}

ParameterDescriptor describe(const std::string & description, double from, double to)
{
  ParameterDescriptor d = describe(description);
  rcl_interfaces::msg::FloatingPointRange range;
  range.from_value = from;
  range.to_value = to;
  d.floating_point_range.push_back(range);
  return d;
}

ParameterDescriptor describeInt(const std::string & description, int64_t from, int64_t to)
{
  ParameterDescriptor d = describe(description);
  rcl_interfaces::msg::IntegerRange range;
  range.from_value = from;
  range.to_value = to;
  range.step = 1;
  d.integer_range.push_back(range);
  return d;
}

std::string expandHome(const std::string & path)
{
  if (path.rfind('~', 0) == 0) {
    const char * home = std::getenv("HOME");
    return std::string(home != nullptr ? home : "") + path.substr(1);
  }
  return path;
}

std::string utcStamp(const char * format)
{
  const std::time_t now = std::time(nullptr);
  std::tm tm{};
  gmtime_r(&now, &tm);
  char buffer[40];
  std::strftime(buffer, sizeof(buffer), format, &tm);
  return buffer;
}

void requireOneOf(
  const std::string & name, const std::string & value, std::initializer_list<const char *> allowed)
{
  for (const char * option : allowed) {
    if (value == option) {
      return;
    }
  }
  std::string list;
  for (const char * option : allowed) {
    list += list.empty() ? option : std::string(", ") + option;
  }
  throw std::invalid_argument("parameter " + name + " must be one of: " + list);
}

}  // namespace

RecorderNode::RecorderNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("landfall_recorder", options)
{
  declareParameters();
}

RecorderNode::~RecorderNode()
{
  // A node destroyed while recording still leaves a readable, closed session.
  if (writer_ && selector_) {
    try {
      writer_->close(selector_->counts());
    } catch (const std::exception & e) {
      std::fprintf(stderr, "landfall_recorder: cannot close the session: %s\n", e.what());
    }
  }
}

void RecorderNode::declareParameters()
{
  declare_parameter(
    "output_dir", "~/.local/share/landfall/sessions",
    describe("Directory that receives one staging folder per recording session"));
  declare_parameter(
    "session_name", "", describe("Name of the session folder; empty: the UTC start time"));
  declare_parameter("frames.global", "map", describe("Global (map) frame of the keyframe poses"));
  declare_parameter("frames.odom", "odom", describe("Odometry frame (speed gate)"));
  declare_parameter("frames.base", "base_footprint", describe("Robot base frame on the floor"));
  declare_parameter(
    "frames.camera", "", describe("Camera optical frame; empty: camera_info's frame_id"));
  declare_parameter("camera_info.topic", "camera_info", describe("sensor_msgs/CameraInfo topic"));
  declare_parameter("image.topic", "image", describe("Image topic (base name for compressed)"));
  declare_parameter(
    "image.transport", "raw",
    describe("raw (sensor_msgs/Image) or compressed (<topic>/compressed)"));
  declare_parameter("image.time_offset", 0.0, describe("Seconds added to image stamps", -0.5, 0.5));
  declare_parameter("image.format", "png", describe("Keyframe image files: png or jpg"));
  declare_parameter(
    "image.jpeg_quality", 95, describeInt("JPEG quality when image.format is jpg", 50, 100));
  declare_parameter(
    "pose_source.type", "tf_with_covariance",
    describe("tf_with_covariance (gate on the localization covariance) or tf_only"));
  declare_parameter(
    "pose_source.covariance_topic", "amcl_pose",
    describe("geometry_msgs/PoseWithCovarianceStamped of the localization"));
  declare_parameter(
    "pose_source.max_covariance_age", 5.0,
    describe("Oldest usable covariance estimate, seconds", 0.0, 60.0));
  declare_parameter(
    "tf_timeout", 0.2, describe("Seconds to wait for TF at an image stamp", 0.0, 5.0));
  declare_parameter(
    "gating.speed_window", 0.2, describe("Seconds of odometry for the speed gate", 0.05, 2.0));
  declare_parameter(
    "gating.max_std_xy", 0.05, describe("Largest localization std in x/y, metres", 0.001, 1.0));
  declare_parameter(
    "gating.max_std_yaw_deg", 2.0, describe("Largest localization std in yaw, degrees", 0.1, 45.0));
  declare_parameter(
    "gating.min_translation", 0.5, describe("Metres since the last keyframe ...", 0.0, 10.0));
  declare_parameter(
    "gating.min_rotation_deg", 15.0, describe("... or degrees of rotation", 0.0, 180.0));
  declare_parameter(
    "gating.max_linear_speed", 0.6, describe("Fastest speed for a keyframe, m/s", 0.0, 5.0));
  declare_parameter(
    "gating.max_angular_speed", 0.5, describe("Fastest turn for a keyframe, rad/s", 0.0, 5.0));
  declare_parameter(
    "gating.max_tilt_deg", 1.5, describe("IMU tilt above which no keyframe is taken", 0.5, 15.0));
  declare_parameter(
    "gravity.source", "auto", describe("auto (IMU when fresh), imu (required) or flat_floor"));
  declare_parameter("imu.topic", "imu", describe("sensor_msgs/Imu with orientation"));
  declare_parameter("imu.frame", "", describe("IMU frame in TF; empty: the messages' frame_id"));
  declare_parameter("imu.max_age", 0.1, describe("Oldest usable IMU sample, seconds", 0.01, 1.0));
  declare_parameter(
    "quality.min_sharpness", 60.0,
    describe("Smallest variance of the Laplacian (images at most 640 px wide)", 0.0, 1e5));
  declare_parameter(
    "quality.min_mean_intensity", 30, describeInt("Darkest mean intensity", 0, 255));
  declare_parameter(
    "quality.max_mean_intensity", 225, describeInt("Brightest mean intensity", 0, 255));
  declare_parameter(
    "quality.max_saturated_fraction", 0.20,
    describe("Largest fraction of pixels at 250 or brighter", 0.0, 1.0));
  declare_parameter(
    "max_keyframes", 0, describeInt("Stop after this many keyframes; 0: unlimited", 0, 10000000));
  declare_parameter("bond.enabled", true, describe("Heartbeat bond with nav2_lifecycle_manager"));
  declare_parameter(
    "autostart", false, describe("recorder_node executable: configure and activate on start"));
}

RecorderNode::Settings RecorderNode::readSettings()
{
  Settings s;
  s.output_dir = expandHome(get_parameter("output_dir").as_string());
  s.session_name = get_parameter("session_name").as_string();
  s.frames.global = get_parameter("frames.global").as_string();
  s.frames.odom = get_parameter("frames.odom").as_string();
  s.frames.base = get_parameter("frames.base").as_string();
  s.frames.camera = get_parameter("frames.camera").as_string();
  s.camera_info_topic = get_parameter("camera_info.topic").as_string();
  s.image_topic = get_parameter("image.topic").as_string();
  s.image_transport = get_parameter("image.transport").as_string();
  requireOneOf("image.transport", s.image_transport, {"raw", "compressed"});
  s.image_time_offset = get_parameter("image.time_offset").as_double();
  s.image_format = get_parameter("image.format").as_string();
  requireOneOf("image.format", s.image_format, {"png", "jpg"});
  s.jpeg_quality = static_cast<int>(get_parameter("image.jpeg_quality").as_int());
  s.pose_source = get_parameter("pose_source.type").as_string();
  requireOneOf("pose_source.type", s.pose_source, {"tf_with_covariance", "tf_only"});
  s.covariance_topic = get_parameter("pose_source.covariance_topic").as_string();
  s.tf_timeout = get_parameter("tf_timeout").as_double();
  s.speed_window = get_parameter("gating.speed_window").as_double();
  const std::string gravity = get_parameter("gravity.source").as_string();
  requireOneOf("gravity.source", gravity, {"auto", "imu", "flat_floor"});
  s.gravity_source = parseGravitySource(gravity).value_or(GravitySource::kAuto);  // validated
  s.imu_topic = get_parameter("imu.topic").as_string();
  s.imu_frame = get_parameter("imu.frame").as_string();
  s.imu_max_age = get_parameter("imu.max_age").as_double();
  KeyframeGating & g = s.gating;
  g.require_covariance = s.pose_source == "tf_with_covariance";
  g.max_covariance_age = get_parameter("pose_source.max_covariance_age").as_double();
  g.max_std_xy = get_parameter("gating.max_std_xy").as_double();
  g.max_std_yaw_deg = get_parameter("gating.max_std_yaw_deg").as_double();
  g.min_translation = get_parameter("gating.min_translation").as_double();
  g.min_rotation_deg = get_parameter("gating.min_rotation_deg").as_double();
  g.max_linear_speed = get_parameter("gating.max_linear_speed").as_double();
  g.max_angular_speed = get_parameter("gating.max_angular_speed").as_double();
  g.max_tilt_deg = get_parameter("gating.max_tilt_deg").as_double();
  g.min_sharpness = get_parameter("quality.min_sharpness").as_double();
  g.min_mean_intensity = static_cast<double>(get_parameter("quality.min_mean_intensity").as_int());
  g.max_mean_intensity = static_cast<double>(get_parameter("quality.max_mean_intensity").as_int());
  g.max_saturated_fraction = get_parameter("quality.max_saturated_fraction").as_double();
  g.max_keyframes = static_cast<int>(get_parameter("max_keyframes").as_int());
  if (g.min_mean_intensity >= g.max_mean_intensity) {
    throw std::invalid_argument(
      "quality.min_mean_intensity must be below quality.max_mean_intensity");
  }
  s.bond_enabled = get_parameter("bond.enabled").as_bool();
  return s;
}

RecorderNode::CallbackReturn RecorderNode::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    settings_ = readSettings();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "invalid configuration: %s", e.what());
    return CallbackReturn::FAILURE;
  }
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  poses_ = std::make_unique<TfPoseSource>(tf_buffer_, settings_.frames, settings_.speed_window);
  imu_ = ImuGravitySource();
  q_base_imu_.reset();
  imu_frame_.clear();

  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    settings_.camera_info_topic, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info) {
      if (!camera_info_) {
        camera_info_ = *info;
      } else if (
        writer_ && (info->k != camera_info_->k || info->d != camera_info_->d ||
                    info->width != camera_info_->width)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 10000,
          "camera_info changed during the session; keeping the session's calibration");
      }
    });
  if (settings_.pose_source == "tf_with_covariance") {
    covariance_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      settings_.covariance_topic, rclcpp::QoS(10).reliable(),
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & pose) {
        poses_->addCovariance(
          toNanoseconds(pose->header.stamp), planarCovariance(pose->pose.covariance));
      });
  }
  if (settings_.gravity_source != GravitySource::kFlatFloor && !settings_.imu_topic.empty()) {
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      settings_.imu_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Imu::ConstSharedPtr & imu) {
        if (imu_.add(*imu) && imu_frame_.empty()) {
          imu_frame_ = settings_.imu_frame.empty() ? imu->header.frame_id : settings_.imu_frame;
        }
      });
  }
  status_pub_ = create_publisher<landfall_msgs::msg::RecorderStatus>(
    "~/status", rclcpp::QoS(1).reliable().transient_local());
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/markers", rclcpp::QoS(1).reliable().transient_local());
  pause_srv_ = create_service<std_srvs::srv::SetBool>(
    "~/set_paused",
    // rclcpp matches service callbacks by their exact (by-value) signature.
    [this](
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,  // NOLINT
      std::shared_ptr<std_srvs::srv::SetBool::Response> response) {    // NOLINT
      paused_ = request->data;
      if (paused_) {
        pending_.clear();
      }
      response->success = true;
      response->message = paused_ ? "recording paused" : "recording resumed";
      RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
      publishStatus();
    });
  diagnostics_ = std::make_unique<diagnostic_updater::Updater>(this);
  diagnostics_->setHardwareID("landfall_recorder");
  diagnostics_->add("recording", this, &RecorderNode::diagnose);
  RCLCPP_INFO(
    get_logger(), "configured: sessions go to %s, images from '%s' (%s)",
    settings_.output_dir.c_str(), settings_.image_topic.c_str(), settings_.image_transport.c_str());
  return CallbackReturn::SUCCESS;
}

RecorderNode::CallbackReturn RecorderNode::on_activate(const rclcpp_lifecycle::State &)
{
  status_pub_->on_activate();
  marker_pub_->on_activate();
  paused_ = false;
  limit_logged_ = false;
  last_image_.reset();
  markers_.markers.clear();
  if (settings_.image_transport == "compressed") {
    compressed_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      settings_.image_topic + "/compressed", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::CompressedImage::ConstSharedPtr & image) {
        enqueue(toNanoseconds(image->header.stamp), [image] { return decodeImage(*image); });
      });
  } else {
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      settings_.image_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::ConstSharedPtr & image) {
        enqueue(toNanoseconds(image->header.stamp), [image] { return decodeImage(*image); });
      });
  }
  process_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] { processPending(); });
  status_timer_ = create_wall_timer(std::chrono::seconds(1), [this] { publishStatus(); });
  if (settings_.bond_enabled) {
    bond_ = std::make_shared<bond::Bond>("bond", get_name(), shared_from_this());
    bond_->setHeartbeatPeriod(0.10);
    bond_->setHeartbeatTimeout(4.0);
    bond_->start();
  }
  RCLCPP_INFO(get_logger(), "recording: waiting for camera_info and images");
  publishStatus();
  return CallbackReturn::SUCCESS;
}

RecorderNode::CallbackReturn RecorderNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  image_sub_.reset();
  compressed_sub_.reset();
  process_timer_.reset();
  status_timer_.reset();
  // Images still waiting for TF are decided now; whatever lacks TF counts as no_pose.
  while (!pending_.empty()) {
    const PendingImage image = std::move(pending_.front());
    pending_.pop_front();
    handle(image);
  }
  closeSession();
  publishStatus();
  status_pub_->on_deactivate();
  marker_pub_->on_deactivate();
  bond_.reset();
  return CallbackReturn::SUCCESS;
}

RecorderNode::CallbackReturn RecorderNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  diagnostics_.reset();
  pause_srv_.reset();
  status_pub_.reset();
  marker_pub_.reset();
  camera_info_sub_.reset();
  covariance_sub_.reset();
  imu_sub_.reset();
  poses_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  camera_info_.reset();
  return CallbackReturn::SUCCESS;
}

RecorderNode::CallbackReturn RecorderNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  pending_.clear();
  closeSession();
  bond_.reset();
  image_sub_.reset();
  compressed_sub_.reset();
  process_timer_.reset();
  status_timer_.reset();
  diagnostics_.reset();
  return CallbackReturn::SUCCESS;
}

void RecorderNode::enqueue(int64_t stamp_ns, std::function<cv::Mat()> decode)
{
  last_image_ = now();
  if (paused_) {
    return;
  }
  constexpr std::size_t kMaxPending = 30;
  if (pending_.size() >= kMaxPending) {
    // TF is far behind the images: decide the oldest now (it counts as no_pose).
    const PendingImage oldest = std::move(pending_.front());
    pending_.pop_front();
    handle(oldest);
  }
  pending_.push_back(
    {stamp_ns + static_cast<int64_t>(settings_.image_time_offset * 1e9), now(), std::move(decode)});
}

void RecorderNode::processPending()
{
  if (!camera_info_) {
    if (!pending_.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "images arrive but no camera_info on '%s' yet",
        settings_.camera_info_topic.c_str());
      pending_.clear();
    }
    return;
  }
  const std::string camera_frame =
    settings_.frames.camera.empty() ? camera_info_->header.frame_id : settings_.frames.camera;
  while (!pending_.empty()) {
    const PendingImage & front = pending_.front();
    const bool ready = poses_->canTransform(front.stamp_ns, camera_frame);
    const bool expired = (now() - front.arrival).seconds() > settings_.tf_timeout;
    if (!ready && !expired) {
      break;
    }
    const PendingImage image = std::move(pending_.front());
    pending_.pop_front();
    handle(image);
  }
}

void RecorderNode::handle(const PendingImage & image)
{
  if (!camera_info_) {
    return;
  }
  const sensor_msgs::msg::CameraInfo info = *camera_info_;
  if (!writer_ && !openSession(info)) {
    return;
  }
  const std::string camera_frame =
    settings_.frames.camera.empty() ? info.header.frame_id : settings_.frames.camera;
  TiltInput tilt;
  tilt.source = settings_.gravity_source;
  tilt.max_age = settings_.imu_max_age;
  if (!imu_frame_.empty() && !q_base_imu_) {
    try {
      q_base_imu_ = Eigen::Quaterniond(
        toPose3(tf_buffer_->lookupTransform(settings_.frames.base, imu_frame_, tf2::TimePointZero)
                  .transform)
          .linear());
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 10000, "no transform %s -> %s for the IMU: %s",
        settings_.frames.base.c_str(), imu_frame_.c_str(), e.what());
    }
  }
  if (q_base_imu_) {
    tilt.imu = &imu_;
    tilt.q_base_imu = *q_base_imu_;
  } else if (settings_.gravity_source == GravitySource::kImu) {
    tilt.imu = &imu_;  // no samples: every frame fails the ramp gate, as configured
  }
  const FrameDecision decision =
    gateFrame(*selector_, *poses_, tilt, image.stamp_ns, camera_frame, image.decode);
  switch (decision.result) {
    case GateResult::kNoPose:
      last_tf_error_ = decision.error;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 10000, "no pose for an image: %s", decision.error.c_str());
      return;
    case GateResult::kLimitReached:
      if (!limit_logged_) {
        RCLCPP_INFO(get_logger(), "max_keyframes reached; no more keyframes in this session");
        limit_logged_ = true;
      }
      return;
    case GateResult::kAccepted:
      break;
    default:
      return;
  }

  StagingKeyframe keyframe;
  keyframe.stamp_ns = image.stamp_ns;
  keyframe.map_base = decision.pose.map_base;
  keyframe.covariance = decision.pose.covariance;
  keyframe.T_base_cam = decision.pose.T_base_cam;
  keyframe.quality = decision.quality;
  keyframe.tilt_deg = decision.tilt_deg;
  try {
    writer_->add(keyframe, decision.image);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "cannot save keyframe: %s", e.what());
    return;
  }
  const Pose3 T_map_cam = landfall::toPose3(decision.pose.map_base) * decision.pose.T_base_cam;
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = settings_.frames.global;
  pose.header.stamp = toStamp(image.stamp_ns);
  pose.pose = toPoseMsg(T_map_cam);
  last_keyframe_pose_ = pose;

  visualization_msgs::msg::Marker marker;
  marker.header = pose.header;
  marker.ns = "keyframes";
  marker.id = static_cast<int>(markers_.markers.size());
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  const Vec3 axis = T_map_cam.linear().col(2);
  Pose3 arrow = Pose3::Identity();
  arrow.translation() = T_map_cam.translation();
  arrow.linear() =
    Eigen::AngleAxisd(std::atan2(axis.y(), axis.x()), Vec3::UnitZ()).toRotationMatrix();
  marker.pose = toPoseMsg(arrow);
  marker.scale.x = 0.3;
  marker.scale.y = 0.04;
  marker.scale.z = 0.04;
  marker.color.r = 0.9F;
  marker.color.g = 0.2F;
  marker.color.b = 0.1F;
  marker.color.a = 1.0F;
  markers_.markers.push_back(marker);
  marker_pub_->publish(markers_);
  publishStatus();
}

bool RecorderNode::openSession(const sensor_msgs::msg::CameraInfo & info)
{
  CameraModel camera;
  try {
    camera = toCameraModel(info);
  } catch (const std::invalid_argument & e) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "unusable camera_info: %s", e.what());
    return false;
  }
  StagingSession session;
  session.name =
    settings_.session_name.empty() ? utcStamp("%Y-%m-%d_%H-%M-%S") : settings_.session_name;
  session.created_utc = utcStamp("%Y-%m-%dT%H:%M:%SZ");
  session.landfall_version = version();
  session.pose_source = settings_.pose_source;
  session.frames = settings_.frames;
  if (session.frames.camera.empty()) {
    session.frames.camera = info.header.frame_id;
  }
  session.camera = camera;
  session.image_format = settings_.image_format;
  session.jpeg_quality = settings_.jpeg_quality;
  std::filesystem::path directory = std::filesystem::path(settings_.output_dir) / session.name;
  for (int suffix = 2; std::filesystem::exists(directory); ++suffix) {
    directory =
      std::filesystem::path(settings_.output_dir) / (session.name + "_" + std::to_string(suffix));
  }
  session.name = directory.filename().string();
  try {
    std::filesystem::create_directories(settings_.output_dir);
    writer_ = std::make_unique<StagingWriter>(directory.string(), session);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "cannot open a session: %s", e.what());
    return false;
  }
  selector_ = std::make_unique<KeyframeSelector>(settings_.gating);
  RCLCPP_INFO(get_logger(), "recording session %s", directory.c_str());
  return true;
}

void RecorderNode::closeSession()
{
  if (!writer_) {
    return;
  }
  const GateCounts counts = selector_->counts();
  try {
    writer_->close(counts);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "cannot close the session cleanly: %s", e.what());
  }
  RCLCPP_INFO(
    get_logger(), "session %s closed: %u keyframes from %u frames", writer_->directory().c_str(),
    counts.keyframes, counts.frames_seen);
  writer_.reset();
}

void RecorderNode::publishStatus()
{
  if (!status_pub_ || !status_pub_->is_activated()) {
    return;
  }
  landfall_msgs::msg::RecorderStatus status;
  status.stamp = now();
  status.session = writer_ ? writer_->session().name : "";
  status.recording = writer_ != nullptr && !paused_;
  status.paused = paused_;
  if (selector_) {
    const GateCounts & c = selector_->counts();
    status.frames_seen = c.frames_seen;
    status.keyframes = c.keyframes;
    status.rejected_pose_uncertain = c.pose_uncertain;
    status.rejected_no_motion = c.no_motion;
    status.rejected_speed = c.speed;
    status.rejected_tilt = c.tilt;
    status.rejected_blur = c.blur;
    status.rejected_exposure = c.exposure;
    status.rejected_tf = c.no_pose;
  }
  if (last_keyframe_pose_) {
    status.last_keyframe_pose = *last_keyframe_pose_;
  }
  status_pub_->publish(status);
}

void RecorderNode::diagnose(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  const auto state = get_current_state().id();
  if (state != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    status.summary(DiagnosticStatus::OK, "not recording (inactive)");
    return;
  }
  if (paused_) {
    status.summary(DiagnosticStatus::OK, "paused");
  } else if (!last_image_ || (now() - *last_image_).seconds() > 2.0) {
    status.summary(DiagnosticStatus::WARN, "no images on '" + settings_.image_topic + "'");
  } else if (!camera_info_) {
    status.summary(
      DiagnosticStatus::WARN, "no camera_info on '" + settings_.camera_info_topic + "'");
  } else {
    status.summary(DiagnosticStatus::OK, "recording");
  }
  if (selector_) {
    const GateCounts & c = selector_->counts();
    status.add("session", writer_ ? writer_->directory() : "");
    status.add("frames_seen", c.frames_seen);
    status.add("keyframes", c.keyframes);
    status.add("rejected_no_pose", c.no_pose);
    status.add("rejected_pose_uncertain", c.pose_uncertain);
    status.add("rejected_speed", c.speed);
    status.add("rejected_tilt", c.tilt);
    status.add("rejected_blur", c.blur);
    status.add("rejected_exposure", c.exposure);
    if (c.frames_seen > 20 && c.no_pose * 2 > c.frames_seen) {
      status.mergeSummary(DiagnosticStatus::WARN, "most frames have no pose: " + last_tf_error_);
    }
  }
}

}  // namespace landfall::ros

RCLCPP_COMPONENTS_REGISTER_NODE(landfall::ros::RecorderNode)
