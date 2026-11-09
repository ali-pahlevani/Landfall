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

#include "landfall/ros/relocalizer_node.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include "landfall/grid/map_loader.hpp"
#include "landfall/query/extrinsics_monitor.hpp"
#include "landfall/record/keyframe_selector.hpp"
#include "landfall/ros/backends.hpp"
#include "landfall/ros/compat/cv_bridge.hpp"
#include "landfall/ros/conversions.hpp"
#include "landfall/ros/parameters.hpp"
#include "landfall/ros/scan_input.hpp"
#include "landfall/util/json.hpp"
#include "landfall/util/sha256.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/exceptions.hpp"

namespace landfall::ros
{

namespace
{

using Clock = std::chrono::steady_clock;

double secondsSince(Clock::time_point start)
{
  return std::chrono::duration<double>(Clock::now() - start).count();
}

double deg(double rad) { return rad * 180.0 / M_PI; }

tf2::TimePoint timePoint(int64_t stamp_ns)
{
  return tf2::TimePoint(std::chrono::nanoseconds(stamp_ns));
}

std::string fixed(double value, int decimals)
{
  std::ostringstream out;
  out.setf(std::ios::fixed);
  out.precision(decimals);
  out << value;
  return out.str();
}

std::string poseText(const Pose2 & p)
{
  return "(" + fixed(p.x, 2) + ", " + fixed(p.y, 2) + ", " + fixed(deg(p.yaw), 1) + " deg)";
}

}  // namespace

RelocalizerNode::RelocalizerNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("landfall_relocalizer", options)
{
  declareParameters();
  parameters_handle_ =
    add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & parameters) {
      return validateParameters(parameters);
    });
}

RelocalizerNode::~RelocalizerNode() { stopWorker(); }

void RelocalizerNode::declareParameters()
{
  constexpr bool kDynamic = false;  // read_only = false: applies to the next request
  // Structural: fixed for the node's lifetime.
  declare_parameter(
    "database_path", "", describe("Map database (*.landfall.db) built by `landfall build`"));
  declare_parameter(
    "map_yaml", "",
    describe("Optional map_server YAML; its file hashes must equal the database's"));
  declare_parameter(
    "require_map_match", true,
    describe("Refuse requests unless the map on 'map' equals the database's occupancy map"));
  declare_parameter("frames.global", "map", describe("Global frame of the published pose"));
  declare_parameter("frames.odom", "odom", describe("Odometry frame (motion between frames)"));
  declare_parameter("frames.base", "base_footprint", describe("Robot base frame on the floor"));
  declare_parameter(
    "frames.camera", "", describe("Camera optical frame; empty: camera_info's frame_id"));
  declare_parameter("camera_info.topic", "camera_info", describe("sensor_msgs/CameraInfo topic"));
  declare_parameter("image.topic", "image", describe("Image topic (base name for compressed)"));
  declare_parameter(
    "image.transport", "raw",
    describe("raw (sensor_msgs/Image) or compressed (<topic>/compressed)"));
  declare_parameter("image.time_offset", 0.0, describe("Seconds added to image stamps", -0.5, 0.5));
  declare_parameter(
    "gravity.source", "auto", describe("auto (IMU when fresh), imu (required) or flat_floor"));
  declare_parameter("imu.topic", "imu", describe("sensor_msgs/Imu with orientation"));
  declare_parameter("imu.frame", "", describe("IMU frame in TF; empty: the messages' frame_id"));
  declare_parameter(
    "backend.num_threads", 0,
    describeInt(
      "Threads solving places in parallel, and ONNX Runtime's for learned backends; 0: hardware "
      "concurrency",
      0, 64));
  declare_parameter(
    "backend.models_dir", "",
    describe("Models of learned backends (docs/models.md); empty: $LANDFALL_MODELS_DIR, else "
             "~/.local/share/landfall/models"));
  declare_parameter(
    "backend.execution_provider", "cpu",
    describe(
      "Where learned backends run: cpu, cuda or tensorrt (the last two need an ONNX Runtime built "
      "with them)"));
  declare_parameter(
    "output.mode", "topic",
    describe("topic (publish output.topic) or service (call AMCL's output.service)"));
  declare_parameter(
    "output.topic", "/initialpose", describe("geometry_msgs/PoseWithCovarianceStamped topic"));
  declare_parameter(
    "output.service", "/set_initial_pose", describe("nav2_msgs/srv/SetInitialPose service"));
  declare_parameter(
    "output.amcl_node", "amcl",
    describe("AMCL node that must be active before publishing; empty: don't check"));
  declare_parameter(
    "startup.enabled", false, describe("Relocalize once after activation, when ready"));
  declare_parameter(
    "startup.delay", 2.0, describe("Seconds after activation before the first attempt", 0.0, 60.0));
  declare_parameter(
    "startup.max_attempts", 3,
    describeInt("Attempts that ran (not counting NOT_READY) before giving up", 1, 100));
  declare_parameter(
    "startup.retry_interval", 5.0, describe("Seconds between startup attempts", 0.0, 300.0));
  declare_parameter("bond.enabled", true, describe("Heartbeat bond with nav2_lifecycle_manager"));
  declare_parameter(
    "extrinsics_check.warn_translation", 0.01,
    describe(
      "Startup check: warn if base->camera differs from the database's by more metres", 0.001,
      1.0));
  declare_parameter(
    "extrinsics_check.warn_rotation_deg", 0.5,
    describe(
      "Startup check: warn if base->camera differs from the database's by more degrees", 0.1,
      10.0));
  declare_parameter(
    "autostart", false, describe("relocalizer_node executable: configure and activate on start"));
  declare_parameter(
    "scan_check.mode", "required",
    describe("LiDAR check of every pose: required, optional (when a scan is there) or disabled"));
  declare_parameter("scan_check.topic", "scan", describe("sensor_msgs/LaserScan topic"));
  declare_parameter(
    "auto_trigger.enabled", false,
    describe("Relocalize on its own while the localization stays uncertain"));
  declare_parameter(
    "auto_trigger.pose_topic", "amcl_pose",
    describe("PoseWithCovarianceStamped of the localization (auto trigger, extrinsics monitor, "
             "kidnap watchdog)"));
  declare_parameter(
    "extrinsics_monitor.enabled", true,
    describe("Compare vision's camera pose with TF's while the localization is confident"));
  declare_parameter(
    "extrinsics_monitor.period", 60.0,
    describe("Seconds between the monitor's dry-run requests", 1.0, 3600.0));
  declare_parameter(
    "kidnap_watchdog.enabled", false,
    describe("Check the localization's pose with dry-run requests, and relocalize when they keep "
             "disagreeing with it"));
  declare_parameter(
    "kidnap_watchdog.period", 10.0,
    describe("Seconds between the watchdog's dry-run requests", 1.0, 3600.0));
  declare_parameter(
    "shadow.log_path", "",
    describe("If set, every request that publishes nothing (dry runs, the monitor's and the "
             "watchdog's checks) is appended there as CSV with the localization's pose"));

  // Thresholds: read at the start of every request.
  declare_parameter(
    "tf_timeout", 0.2, describe("Seconds to wait for TF at an image stamp", 0.0, 5.0, kDynamic));
  declare_parameter(
    "imu.max_age", 0.1, describe("Oldest usable IMU sample, seconds", 0.01, 1.0, kDynamic));
  declare_parameter(
    "imu.max_angular_rate", 0.1,
    describe("IMU gravity is used only below this turn rate, rad/s", 0.0, 2.0, kDynamic));
  declare_parameter(
    "gravity.ramp_tilt_deg", 1.5,
    describe("IMU tilt above which the robot is on a ramp, degrees", 0.5, 15.0, kDynamic));
  declare_parameter(
    "query.top_k", 10, describeInt("Keyframes retrieved per frame", 1, 100, kDynamic));
  declare_parameter(
    "query.timeout", 10.0,
    describe("Seconds for a whole request, frame collection included", 1.0, 120.0, kDynamic));
  declare_parameter(
    "matching.ratio", 0.8,
    describe("Lowe ratio for binary descriptors (with a mutual check)", 0.5, 1.0, kDynamic));
  declare_parameter(
    "matching.min_cosine", 0.82,
    describe("Smallest cosine for float descriptors (mutual nearest)", 0.0, 1.0, kDynamic));
  declare_parameter(
    "pnp.solver", "upright_then_p3p",
    describe("upright, p3p, upright_then_p3p or opencv", kDynamic));
  declare_parameter(
    "pnp.max_reproj_error_px", 0.0,
    describe("RANSAC inlier threshold in pixels; 0: 1 % of the image width", 0.0, 50.0, kDynamic));
  declare_parameter(
    "pnp.max_iterations", 10000, describeInt("RANSAC iterations", 100, 1000000, kDynamic));
  declare_parameter(
    "pnp.success_prob", 0.9999, describe("RANSAC confidence", 0.9, 0.999999, kDynamic));
  declare_parameter(
    "sanity.min_inliers", 40, describeInt("Fewest PnP inliers", 6, 10000, kDynamic));
  declare_parameter(
    "sanity.min_inlier_ratio", 0.25, describe("Smallest inlier ratio", 0.0, 1.0, kDynamic));
  declare_parameter(
    "sanity.max_mean_reproj_error_px", 0.0,
    describe(
      "Largest mean inlier reprojection error; 0: half the PnP threshold", 0.0, 20.0, kDynamic));
  declare_parameter(
    "sanity.max_height_error", 0.10,
    describe("Largest base height above the floor, metres", 0.01, 1.0, kDynamic));
  declare_parameter(
    "sanity.max_floor_elevation", 0.0,
    describe("Extra height allowed on a ramp, metres; 0: refuse ramp queries", 0.0, 5.0, kDynamic));
  declare_parameter(
    "sanity.max_tilt_error_deg", 3.0,
    describe("Largest roll/pitch difference to gravity, degrees", 0.5, 30.0, kDynamic));
  declare_parameter(
    "sanity.min_inlier_coverage", 0.15,
    describe("Smallest fraction of the 8x6 image grid holding inliers", 0.0, 1.0, kDynamic));
  declare_parameter(
    "sanity.require_free_space", true,
    describe("The base must stand on a known-free map cell", kDynamic));
  declare_parameter(
    "consistency.num_frames", 3, describeInt("Frames that must agree", 1, 10, kDynamic));
  declare_parameter(
    "consistency.min_frame_interval", 0.3,
    describe("Seconds between the frames used", 0.0, 5.0, kDynamic));
  declare_parameter(
    "consistency.max_translation_error", 0.15,
    describe("Largest disagreement with odometry between frames, metres", 0.01, 2.0, kDynamic));
  declare_parameter(
    "consistency.max_rotation_error_deg", 5.0,
    describe("Largest disagreement with odometry between frames, degrees", 0.5, 45.0, kDynamic));
  declare_parameter(
    "ambiguity.cluster_radius", 0.5,
    describe(
      "Poses this close (and within cluster_angle_deg) are one place, metres", 0.05, 5.0,
      kDynamic));
  declare_parameter(
    "ambiguity.cluster_angle_deg", 15.0,
    describe(
      "Poses this close (and within cluster_radius) are one place, degrees", 1.0, 180.0, kDynamic));
  declare_parameter(
    "ambiguity.max_score_ratio", 0.0,
    describe(
      "A second valid place with at least sanity.min_inliers inliers and this fraction of the "
      "best one's makes the result AMBIGUOUS; 0: any such place",
      0.0, 1.0, kDynamic));
  declare_parameter(
    "output.std_xy", 0.25,
    describe("Standard deviation in x and y of the published pose, metres", 0.01, 5.0, kDynamic));
  declare_parameter(
    "output.std_yaw_deg", 5.0,
    describe("Standard deviation in yaw of the published pose, degrees", 0.1, 90.0, kDynamic));
  declare_parameter(
    "output.compensate_motion", true,
    describe("Move the pose to the latest odometry before publishing", kDynamic));
  declare_parameter(
    "debug.publish_image", true,
    describe("Publish ~/debug_image (only rendered with subscribers)", kDynamic));
  declare_parameter("debug.publish_markers", true, describe("Publish ~/markers", kDynamic));
  declare_parameter(
    "debug.dump_dir", "",
    describe("If set, every query frame and its result are saved there for tuning", kDynamic));
  declare_parameter(
    "scan_check.max_time_offset", 0.5,
    describe(
      "Largest time between an image and the scan that checks it, seconds", 0.0, 5.0, kDynamic));
  declare_parameter(
    "scan_check.max_range", 10.0,
    describe("Longer readings are ignored, metres", 0.5, 100.0, kDynamic));
  declare_parameter(
    "scan_check.hit_tolerance", 0.10,
    describe("A beam ending this close to an obstacle is a hit, metres", 0.01, 1.0, kDynamic));
  declare_parameter(
    "scan_check.min_hit_ratio", 0.85,
    describe(
      "Smallest fraction of hits among the beams ending in known space", 0.0, 1.0, kDynamic));
  declare_parameter(
    "scan_check.max_violation_ratio", 0.05,
    describe("Largest fraction of beams that see through an obstacle", 0.0, 1.0, kDynamic));
  declare_parameter(
    "scan_check.min_known_ratio", 0.5,
    describe("Smallest fraction of beams ending in known space", 0.0, 1.0, kDynamic));
  declare_parameter(
    "scan_check.min_valid_beams", 60, describeInt("Fewest usable beams", 10, 100000, kDynamic));
  declare_parameter(
    "scan_check.search_radius", 0.10,
    describe("Local search window around a pose, metres", 0.0, 0.5, kDynamic));
  declare_parameter(
    "scan_check.search_angle_deg", 3.0,
    describe("Local search window around a pose, degrees", 0.0, 10.0, kDynamic));
  declare_parameter(
    "scan_check.check_alternatives", true,
    describe(
      "Search where else in the map the scan fits; a place it fits clearly better makes the "
      "pose ambiguous",
      kDynamic));
  declare_parameter(
    "scan_check.alternative_margin", 0.01,
    describe(
      "How much better (hit ratio less violation ratio) the scan must fit elsewhere to make a "
      "pose ambiguous",
      0.0, 0.5, kDynamic));
  declare_parameter(
    "scan_check.refine_pose", false,
    describe(
      "Publish the scan check's best pose of its search window instead of vision's", kDynamic));
  declare_parameter(
    "auto_trigger.max_std_xy", 0.5,
    describe(
      "The localization is uncertain above this std in x or y, metres", 0.05, 10.0, kDynamic));
  declare_parameter(
    "auto_trigger.max_std_yaw_deg", 15.0,
    describe("The localization is uncertain above this std in yaw, degrees", 1.0, 180.0, kDynamic));
  declare_parameter(
    "auto_trigger.duration", 5.0,
    describe(
      "Seconds the localization must stay uncertain before a request", 0.0, 600.0, kDynamic));
  declare_parameter(
    "auto_trigger.cooldown", 30.0,
    describe("Seconds after an automatic request before the next one", 0.0, 3600.0, kDynamic));
  declare_parameter(
    "extrinsics_monitor.max_amcl_std_xy", 0.05,
    describe(
      "The monitor samples only while the localization's std is below this, metres", 0.01, 1.0,
      kDynamic));
  declare_parameter(
    "extrinsics_monitor.max_linear_speed", 0.2,
    describe(
      "The monitor samples only while the robot is slower than this, m/s", 0.0, 5.0, kDynamic));
  declare_parameter(
    "extrinsics_monitor.max_angular_speed", 0.1,
    describe(
      "The monitor samples only while the robot turns slower than this, rad/s", 0.0, 5.0,
      kDynamic));
  declare_parameter(
    "extrinsics_monitor.min_samples", 10,
    describeInt("Samples needed before the monitor reports an offset", 3, 1000, kDynamic));
  declare_parameter(
    "extrinsics_monitor.warn_translation", 0.02,
    describe(
      "WARN when the camera moved this much on its mount, metres (ERROR at 2.5 times)", 0.001, 1.0,
      kDynamic));
  declare_parameter(
    "extrinsics_monitor.warn_rotation_deg", 1.0,
    describe(
      "WARN when the camera turned this much on its mount, degrees (ERROR at 2.5 times)", 0.1, 10.0,
      kDynamic));
  declare_parameter(
    "kidnap_watchdog.max_translation", 1.0,
    describe(
      "A check disagrees with the localization when their poses differ by more metres", 0.1, 10.0,
      kDynamic));
  declare_parameter(
    "kidnap_watchdog.max_rotation_deg", 20.0,
    describe(
      "A check disagrees with the localization when their headings differ by more degrees", 1.0,
      180.0, kDynamic));
  declare_parameter(
    "kidnap_watchdog.confirmations", 2,
    describeInt(
      "Disagreeing checks in a row (any check: the monitor's too) before relocalizing", 1, 10,
      kDynamic));
  declare_parameter(
    "kidnap_watchdog.cooldown", 30.0,
    describe("Seconds after a watchdog relocalization before the next one", 0.0, 3600.0, kDynamic));
}

RelocalizerNode::Settings RelocalizerNode::readSettings() const
{
  Settings s;
  s.database_path = expandHome(get_parameter("database_path").as_string());
  s.map_yaml = expandHome(get_parameter("map_yaml").as_string());
  s.require_map_match = get_parameter("require_map_match").as_bool();
  s.frame_global = get_parameter("frames.global").as_string();
  s.frame_odom = get_parameter("frames.odom").as_string();
  s.frame_base = get_parameter("frames.base").as_string();
  s.frame_camera = get_parameter("frames.camera").as_string();
  s.camera_info_topic = get_parameter("camera_info.topic").as_string();
  s.image_topic = get_parameter("image.topic").as_string();
  s.image_transport = get_parameter("image.transport").as_string();
  requireOneOf("image.transport", s.image_transport, {"raw", "compressed"});
  s.image_time_offset = get_parameter("image.time_offset").as_double();
  const std::string gravity = get_parameter("gravity.source").as_string();
  requireOneOf("gravity.source", gravity, {"auto", "imu", "flat_floor"});
  s.gravity_source = parseGravitySource(gravity).value_or(GravitySource::kAuto);  // validated
  s.imu_topic = get_parameter("imu.topic").as_string();
  s.imu_frame = get_parameter("imu.frame").as_string();
  s.threads = static_cast<int>(get_parameter("backend.num_threads").as_int());
  s.models_dir = expandHome(get_parameter("backend.models_dir").as_string());
  s.execution_provider = get_parameter("backend.execution_provider").as_string();
  requireOneOf("backend.execution_provider", s.execution_provider, {"cpu", "cuda", "tensorrt"});
  s.output_mode = get_parameter("output.mode").as_string();
  requireOneOf("output.mode", s.output_mode, {"topic", "service"});
  s.output_topic = get_parameter("output.topic").as_string();
  s.output_service = get_parameter("output.service").as_string();
  s.amcl_node = get_parameter("output.amcl_node").as_string();
  s.startup_enabled = get_parameter("startup.enabled").as_bool();
  s.startup_delay = get_parameter("startup.delay").as_double();
  s.startup_max_attempts = static_cast<int>(get_parameter("startup.max_attempts").as_int());
  s.startup_retry_interval = get_parameter("startup.retry_interval").as_double();
  s.bond_enabled = get_parameter("bond.enabled").as_bool();
  s.extrinsics_warn_translation = get_parameter("extrinsics_check.warn_translation").as_double();
  s.extrinsics_warn_rotation_deg = get_parameter("extrinsics_check.warn_rotation_deg").as_double();
  const std::string scan_mode = get_parameter("scan_check.mode").as_string();
  requireOneOf("scan_check.mode", scan_mode, {"required", "optional", "disabled"});
  s.scan_mode = parseScanCheckMode(scan_mode).value_or(ScanCheckMode::kRequired);  // validated
  s.scan_topic = get_parameter("scan_check.topic").as_string();
  s.auto_trigger_enabled = get_parameter("auto_trigger.enabled").as_bool();
  s.localization_pose_topic = get_parameter("auto_trigger.pose_topic").as_string();
  s.monitor_enabled = get_parameter("extrinsics_monitor.enabled").as_bool();
  s.monitor_period = get_parameter("extrinsics_monitor.period").as_double();
  s.watchdog_enabled = get_parameter("kidnap_watchdog.enabled").as_bool();
  s.watchdog_period = get_parameter("kidnap_watchdog.period").as_double();
  s.shadow_log_path = expandHome(get_parameter("shadow.log_path").as_string());
  return s;
}

RelocalizerNode::Thresholds RelocalizerNode::readThresholds() const
{
  Thresholds t;
  LocalizerParams & l = t.localizer;
  l.top_k = static_cast<int>(get_parameter("query.top_k").as_int());
  l.ratio.ratio = get_parameter("matching.ratio").as_double();
  l.mnn.min_cosine = get_parameter("matching.min_cosine").as_double();
  l.pnp = parsePnpStrategy(get_parameter("pnp.solver").as_string())
            .value_or(PnpStrategy::kUprightThenP3P);  // validated on set
  l.pnp_threshold_px = get_parameter("pnp.max_reproj_error_px").as_double();
  l.pnp_max_iterations = static_cast<int>(get_parameter("pnp.max_iterations").as_int());
  l.pnp_success_prob = get_parameter("pnp.success_prob").as_double();
  l.sanity.min_inliers = static_cast<int>(get_parameter("sanity.min_inliers").as_int());
  l.sanity.min_inlier_ratio = get_parameter("sanity.min_inlier_ratio").as_double();
  l.sanity.max_mean_reprojection_px = get_parameter("sanity.max_mean_reproj_error_px").as_double();
  l.sanity.max_height_error = get_parameter("sanity.max_height_error").as_double();
  l.sanity.max_floor_elevation = get_parameter("sanity.max_floor_elevation").as_double();
  l.sanity.max_tilt_error_deg = get_parameter("sanity.max_tilt_error_deg").as_double();
  l.sanity.min_inlier_coverage = get_parameter("sanity.min_inlier_coverage").as_double();
  l.sanity.require_free_space = get_parameter("sanity.require_free_space").as_bool();
  l.clustering.radius = get_parameter("ambiguity.cluster_radius").as_double();
  l.clustering.angle_deg = get_parameter("ambiguity.cluster_angle_deg").as_double();
  l.max_score_ratio = get_parameter("ambiguity.max_score_ratio").as_double();
  l.threads = settings_.threads;
  l.scan_mode = settings_.scan_mode;
  l.scan.max_range = get_parameter("scan_check.max_range").as_double();
  l.scan.hit_tolerance = get_parameter("scan_check.hit_tolerance").as_double();
  l.scan.min_hit_ratio = get_parameter("scan_check.min_hit_ratio").as_double();
  l.scan.max_violation_ratio = get_parameter("scan_check.max_violation_ratio").as_double();
  l.scan.min_known_ratio = get_parameter("scan_check.min_known_ratio").as_double();
  l.scan.min_valid_beams = static_cast<int>(get_parameter("scan_check.min_valid_beams").as_int());
  l.scan.search_radius = get_parameter("scan_check.search_radius").as_double();
  l.scan.search_angle_deg = get_parameter("scan_check.search_angle_deg").as_double();
  l.refine_with_scan = get_parameter("scan_check.refine_pose").as_bool();
  l.scan.check_alternatives = get_parameter("scan_check.check_alternatives").as_bool();
  l.scan.alternative_margin = get_parameter("scan_check.alternative_margin").as_double();
  t.scan_max_time_offset = get_parameter("scan_check.max_time_offset").as_double();
  t.auto_trigger.max_std_xy = get_parameter("auto_trigger.max_std_xy").as_double();
  t.auto_trigger.max_std_yaw_deg = get_parameter("auto_trigger.max_std_yaw_deg").as_double();
  t.auto_trigger.duration = get_parameter("auto_trigger.duration").as_double();
  t.auto_trigger.cooldown = get_parameter("auto_trigger.cooldown").as_double();
  t.monitor.min_samples =
    static_cast<std::size_t>(get_parameter("extrinsics_monitor.min_samples").as_int());
  t.monitor.warn_translation = get_parameter("extrinsics_monitor.warn_translation").as_double();
  t.monitor.warn_rotation_deg = get_parameter("extrinsics_monitor.warn_rotation_deg").as_double();
  t.monitor_max_amcl_std_xy = get_parameter("extrinsics_monitor.max_amcl_std_xy").as_double();
  t.monitor_max_linear_speed = get_parameter("extrinsics_monitor.max_linear_speed").as_double();
  t.monitor_max_angular_speed = get_parameter("extrinsics_monitor.max_angular_speed").as_double();
  t.watchdog.max_translation = get_parameter("kidnap_watchdog.max_translation").as_double();
  t.watchdog.max_rotation_deg = get_parameter("kidnap_watchdog.max_rotation_deg").as_double();
  t.watchdog.confirmations =
    static_cast<int>(get_parameter("kidnap_watchdog.confirmations").as_int());
  t.watchdog.cooldown = get_parameter("kidnap_watchdog.cooldown").as_double();
  t.consistency.max_translation_error =
    get_parameter("consistency.max_translation_error").as_double();
  t.consistency.max_rotation_error_deg =
    get_parameter("consistency.max_rotation_error_deg").as_double();
  t.gravity.max_age = get_parameter("imu.max_age").as_double();
  t.gravity.max_angular_rate = get_parameter("imu.max_angular_rate").as_double();
  t.gravity.ramp_tilt_deg = get_parameter("gravity.ramp_tilt_deg").as_double();
  t.tf_timeout = get_parameter("tf_timeout").as_double();
  t.timeout = get_parameter("query.timeout").as_double();
  t.num_frames = static_cast<int>(get_parameter("consistency.num_frames").as_int());
  t.min_frame_interval = get_parameter("consistency.min_frame_interval").as_double();
  t.std_xy = get_parameter("output.std_xy").as_double();
  t.std_yaw_deg = get_parameter("output.std_yaw_deg").as_double();
  t.compensate_motion = get_parameter("output.compensate_motion").as_bool();
  t.publish_image = get_parameter("debug.publish_image").as_bool();
  t.publish_markers = get_parameter("debug.publish_markers").as_bool();
  t.dump_dir = expandHome(get_parameter("debug.dump_dir").as_string());
  return t;
}

rcl_interfaces::msg::SetParametersResult RelocalizerNode::validateParameters(
  const std::vector<rclcpp::Parameter> & parameters) const
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & p : parameters) {
    if (
      p.get_name() == "pnp.solver" && p.get_type() == rclcpp::ParameterType::PARAMETER_STRING &&
      !parsePnpStrategy(p.as_string())) {
      result.successful = false;
      result.reason = "pnp.solver must be one of: upright, p3p, upright_then_p3p, opencv";
    }
  }
  return result;
}

RelocalizerNode::CallbackReturn RelocalizerNode::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    settings_ = readSettings();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "invalid configuration: %s", e.what());
    return CallbackReturn::FAILURE;
  }
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    map_check_ = MapCheck();
    map_files_mismatch_.clear();
    camera_info_.reset();
    last_image_.reset();
    imu_ = ImuGravitySource();
    imu_frame_.clear();
    static_parent_.clear();
    extrinsics_status_.clear();
    extrinsics_level_ = 0;
    last_result_.reset();
    scans_.clear();
    last_scan_.reset();
    estimate_.reset();
    monitor_.reset();
    watchdog_.reset();
  }
  auto_trigger_.reset();
  loadDatabase();

  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    settings_.camera_info_topic, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info) {
      const std::lock_guard<std::mutex> lock(mutex_);
      camera_info_ = *info;
    });
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map", rclcpp::QoS(1).reliable().transient_local(),
    [this](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & map) { onMap(*map); });
  if (settings_.gravity_source != GravitySource::kFlatFloor && !settings_.imu_topic.empty()) {
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      settings_.imu_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Imu::ConstSharedPtr & imu) {
        const std::lock_guard<std::mutex> lock(mutex_);
        if (imu_.add(*imu) && imu_frame_.empty()) {
          imu_frame_ = settings_.imu_frame.empty() ? imu->header.frame_id : settings_.imu_frame;
        }
      });
  }
  tf_static_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf_static", rclcpp::QoS(100).reliable().transient_local(),
    [this](const tf2_msgs::msg::TFMessage::ConstSharedPtr & message) { onTfStatic(*message); });
  if (settings_.scan_mode != ScanCheckMode::kDisabled) {
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      settings_.scan_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan) { onScan(*scan); });
  }
  if (settings_.auto_trigger_enabled || settings_.monitor_enabled || settings_.watchdog_enabled) {
    // AMCL publishes amcl_pose latched (transient local) on every filter update.
    localization_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      settings_.localization_pose_topic, rclcpp::QoS(1).reliable(),
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & pose) {
        onLocalizationPose(*pose);
      });
  }

  initial_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    settings_.output_topic, rclcpp::QoS(1).reliable());
  status_pub_ = create_publisher<Result>("~/status", rclcpp::QoS(1).reliable().transient_local());
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/markers", rclcpp::QoS(1).reliable().transient_local());
  debug_image_pub_ = create_publisher<sensor_msgs::msg::Image>("~/debug_image", rclcpp::QoS(1));
  relocalize_srv_ = create_service<Relocalize>(
    "~/relocalize",
    // Deferred response: the worker thread answers when the request has run.
    [this](
      const std::shared_ptr<rmw_request_id_t> header,        // NOLINT
      const std::shared_ptr<Relocalize::Request> request) {  // NOLINT
      Request r;
      r.trigger = "service";
      r.num_frames = request->num_frames;
      r.timeout = request->timeout;
      r.dry_run = request->dry_run;
      r.header = header;
      submit(std::move(r));
    });
  if (!settings_.amcl_node.empty()) {
    const std::string name =
      settings_.amcl_node.front() == '/' ? settings_.amcl_node : "/" + settings_.amcl_node;
    amcl_state_client_ = create_client<lifecycle_msgs::srv::GetState>(name + "/get_state");
  }
  if (settings_.output_mode == "service") {
    set_pose_client_ = create_client<nav2_msgs::srv::SetInitialPose>(settings_.output_service);
  }
  diagnostics_ = std::make_unique<diagnostic_updater::Updater>(this);
  diagnostics_->setHardwareID("landfall_relocalizer");
  diagnostics_->add("relocalizer", this, &RelocalizerNode::diagnose);
  RCLCPP_INFO(
    get_logger(), "configured: database %s, images from '%s' (%s)",
    map_ ? settings_.database_path.c_str() : "(not loaded)", settings_.image_topic.c_str(),
    settings_.image_transport.c_str());
  return CallbackReturn::SUCCESS;
}

void RelocalizerNode::loadDatabase()
{
  map_.reset();
  localizer_.reset();
  database_error_.clear();
  if (settings_.database_path.empty()) {
    database_error_ = "parameter database_path is not set";
    RCLCPP_ERROR(get_logger(), "%s: every request will answer NOT_READY", database_error_.c_str());
    return;
  }
  try {
    const auto started = Clock::now();
    LocalizationMap::Options options;
    options.registry = &backendRegistry();
    options.models_dir = settings_.models_dir;
    options.execution_provider = settings_.execution_provider;
    options.threads = settings_.threads;
    map_ = LocalizationMap::load(settings_.database_path, options);
    localizer_ = std::make_unique<Localizer>(map_, readThresholds().localizer);
    RCLCPP_INFO(
      get_logger(),
      "map database %s: %zu keyframes, %zu landmarks, backends %s + %s (loaded in %.2f s)",
      settings_.database_path.c_str(), map_->keyframes().size(), map_->numLandmarks(),
      map_->localFeatures().id().name.c_str(), map_->globalDescriptor().id().name.c_str(),
      secondsSince(started));
  } catch (const std::exception & e) {
    map_.reset();
    database_error_ = e.what();
    RCLCPP_ERROR(get_logger(), "%s: every request will answer NOT_READY", e.what());
    return;
  }
  if (settings_.map_yaml.empty()) {
    return;
  }
  // The map files AMCL uses must be the ones the database was built for.
  std::string mismatch;
  try {
    const auto yaml_sha = sha256File(settings_.map_yaml);
    const auto metadata = loadMapMetadata(settings_.map_yaml);
    const auto image_sha = sha256File(metadata.image_path);
    if (!yaml_sha || !image_sha) {
      mismatch = "cannot read " + settings_.map_yaml + " or its image";
    } else if (*yaml_sha != map_->mapRecord().yaml_sha256) {
      mismatch = settings_.map_yaml + " is not the map YAML the database was built with";
    } else if (*image_sha != map_->mapRecord().image_sha256) {
      mismatch = metadata.image_path + " is not the map image the database was built with";
    }
  } catch (const std::exception & e) {
    mismatch = std::string("cannot check map_yaml: ") + e.what();
  }
  if (!mismatch.empty()) {
    RCLCPP_ERROR(get_logger(), "%s", mismatch.c_str());
  }
  const std::lock_guard<std::mutex> lock(mutex_);
  map_files_mismatch_ = mismatch;
}

RelocalizerNode::CallbackReturn RelocalizerNode::on_activate(const rclcpp_lifecycle::State &)
{
  initial_pose_pub_->on_activate();
  status_pub_->on_activate();
  marker_pub_->on_activate();
  debug_image_pub_->on_activate();
  if (settings_.image_transport == "compressed") {
    compressed_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      settings_.image_topic + "/compressed", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::CompressedImage::ConstSharedPtr & image) {
        onImage(toNanoseconds(image->header.stamp), [image] { return decodeImage(*image); });
      });
  } else {
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      settings_.image_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::ConstSharedPtr & image) {
        onImage(toNanoseconds(image->header.stamp), [image] { return decodeImage(*image); });
      });
  }
  {
    const std::lock_guard<std::mutex> lock(queue_mutex_);
    stop_worker_ = false;
    queued_.reset();
    startup_attempts_ = 0;
    startup_done_ = !settings_.startup_enabled;
    startup_next_ = Clock::now() + std::chrono::duration_cast<Clock::duration>(
                                     std::chrono::duration<double>(settings_.startup_delay));
  }
  cancel_ = false;
  busy_ = false;
  worker_ = std::thread([this] { workerLoop(); });
  if (settings_.startup_enabled) {
    startup_timer_ = create_wall_timer(std::chrono::milliseconds(200), [this] { startupTick(); });
  }
  auto_trigger_.reset();
  if (settings_.auto_trigger_enabled) {
    auto_trigger_timer_ =
      create_wall_timer(std::chrono::milliseconds(250), [this] { autoTriggerTick(); });
  }
  if (settings_.monitor_enabled) {
    monitor_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(settings_.monitor_period)),
      [this] { monitorTick(); });
  }
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    watchdog_.reset();
  }
  if (settings_.watchdog_enabled) {
    watchdog_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(settings_.watchdog_period)),
      [this] { watchdogTick(); });
  }
  if (settings_.bond_enabled) {
    bond_ = std::make_shared<bond::Bond>("bond", get_name(), shared_from_this());
    bond_->setHeartbeatPeriod(0.10);
    bond_->setHeartbeatTimeout(4.0);
    bond_->start();
  }
  RCLCPP_INFO(
    get_logger(), "active: call %s/relocalize",
    get_node_base_interface()->get_fully_qualified_name());
  return CallbackReturn::SUCCESS;
}

RelocalizerNode::CallbackReturn RelocalizerNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  startup_timer_.reset();
  auto_trigger_timer_.reset();
  monitor_timer_.reset();
  watchdog_timer_.reset();
  stopWorker();
  image_sub_.reset();
  compressed_sub_.reset();
  initial_pose_pub_->on_deactivate();
  status_pub_->on_deactivate();
  marker_pub_->on_deactivate();
  debug_image_pub_->on_deactivate();
  bond_.reset();
  return CallbackReturn::SUCCESS;
}

RelocalizerNode::CallbackReturn RelocalizerNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  diagnostics_.reset();
  relocalize_srv_.reset();
  amcl_state_client_.reset();
  set_pose_client_.reset();
  initial_pose_pub_.reset();
  status_pub_.reset();
  marker_pub_.reset();
  debug_image_pub_.reset();
  camera_info_sub_.reset();
  map_sub_.reset();
  imu_sub_.reset();
  tf_static_sub_.reset();
  scan_sub_.reset();
  localization_pose_sub_.reset();
  localizer_.reset();
  map_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  return CallbackReturn::SUCCESS;
}

RelocalizerNode::CallbackReturn RelocalizerNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  startup_timer_.reset();
  auto_trigger_timer_.reset();
  monitor_timer_.reset();
  watchdog_timer_.reset();
  stopWorker();
  bond_.reset();
  image_sub_.reset();
  compressed_sub_.reset();
  diagnostics_.reset();
  return CallbackReturn::SUCCESS;
}

void RelocalizerNode::stopWorker()
{
  {
    const std::lock_guard<std::mutex> lock(queue_mutex_);
    stop_worker_ = true;
  }
  cancel_ = true;
  queue_cv_.notify_all();
  images_cv_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
}

void RelocalizerNode::onMap(const nav_msgs::msg::OccupancyGrid & msg)
{
  if (!map_) {
    return;
  }
  OccupancyGrid grid;
  grid.width = static_cast<int>(msg.info.width);
  grid.height = static_cast<int>(msg.info.height);
  grid.resolution = msg.info.resolution;
  grid.origin = toPose2(toPose3(msg.info.origin));
  grid.data.assign(msg.data.begin(), msg.data.end());
  const OccupancyGrid & ours = map_->grid();
  std::string mismatch;
  if (grid.width != ours.width || grid.height != ours.height) {
    mismatch = "size " + std::to_string(grid.width) + "x" + std::to_string(grid.height) + " vs " +
               std::to_string(ours.width) + "x" + std::to_string(ours.height);
  } else if (std::abs(grid.resolution - ours.resolution) > 1e-6) {
    mismatch =
      "resolution " + std::to_string(grid.resolution) + " vs " + std::to_string(ours.resolution);
  } else if (
    std::abs(grid.origin.x - ours.origin.x) > 1e-6 ||
    std::abs(grid.origin.y - ours.origin.y) > 1e-6 ||
    std::abs(normalizeAngle(grid.origin.yaw - ours.origin.yaw)) > 1e-6) {
    mismatch = "origin " + poseText(grid.origin) + " vs " + poseText(ours.origin);
  } else if (gridSha256(grid) != map_->mapRecord().grid_sha256) {
    std::size_t differ = 0;
    for (std::size_t i = 0; i < grid.data.size(); ++i) {
      differ += grid.data[i] != ours.data[i] ? 1 : 0;
    }
    mismatch = std::to_string(differ) + " cells differ";
  }
  if (mismatch.empty()) {
    RCLCPP_INFO(get_logger(), "the map on 'map' matches the database");
  } else {
    RCLCPP_ERROR(
      get_logger(), "the map on 'map' differs from the database's (%s)%s", mismatch.c_str(),
      settings_.require_map_match ? ": requests answer NOT_READY" : "");
  }
  const std::lock_guard<std::mutex> lock(mutex_);
  map_check_.received = true;
  map_check_.mismatch = mismatch;
}

void RelocalizerNode::onImage(int64_t stamp_ns, std::function<cv::Mat()> decode)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  last_image_ = now();
  if (!collecting_) {
    return;
  }
  constexpr std::size_t kMaxQueued = 3;
  if (images_.size() >= kMaxQueued) {
    images_.pop_front();
  }
  images_.push_back(
    {stamp_ns + static_cast<int64_t>(settings_.image_time_offset * 1e9), std::move(decode)});
  images_cv_.notify_one();
}

void RelocalizerNode::onTfStatic(const tf2_msgs::msg::TFMessage & message)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & t : message.transforms) {
    static_parent_[t.child_frame_id] = t.header.frame_id;
  }
}

void RelocalizerNode::onScan(const sensor_msgs::msg::LaserScan & scan)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  last_scan_ = now();
  scans_.push_back(scan);
  // Keep a few seconds: enough for the image-scan offset and for TF to catch up.
  const int64_t newest = toNanoseconds(scans_.back().header.stamp);
  while (scans_.size() > 1 && toNanoseconds(scans_.front().header.stamp) < newest - 3'000'000'000) {
    scans_.pop_front();
  }
}

void RelocalizerNode::onLocalizationPose(const geometry_msgs::msg::PoseWithCovarianceStamped & pose)
{
  const Mat3 covariance = planarCovariance(pose.pose.covariance);
  LocalizationEstimate e;
  e.pose = toPose2(toPose3(pose.pose.pose));
  e.std_xy = worstStdXy(covariance);
  e.std_yaw_deg = deg(std::sqrt(std::max(0.0, covariance(2, 2))));
  auto_trigger_.observe(now().seconds(), e.std_xy, e.std_yaw_deg);
  const std::lock_guard<std::mutex> lock(mutex_);
  estimate_ = e;
}

void RelocalizerNode::autoTriggerTick()
{
  auto_trigger_.setParams(readThresholds().auto_trigger);
  const double t = now().seconds();
  if (!auto_trigger_.shouldFire(t, busy_)) {
    return;
  }
  auto_trigger_.fired(t);
  RCLCPP_INFO(get_logger(), "the localization stays uncertain: relocalizing (auto trigger)");
  Request r;
  r.trigger = "auto";
  submit(std::move(r));
}

void RelocalizerNode::monitorTick()
{
  if (busy_) {
    return;
  }
  const Thresholds t = readThresholds();
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (!estimate_ || estimate_->std_xy > t.monitor_max_amcl_std_xy) {
      return;  // only while the localization is confident
    }
  }
  // A full request, as the service would run it, that publishes nothing: its result feeds the
  // shadow log (field validation, M8) and, on success, the monitor.
  Request r;
  r.trigger = "monitor";
  r.timeout = std::min(t.timeout, 5.0);
  r.dry_run = true;
  r.background = true;
  submit(std::move(r));
}

void RelocalizerNode::watchdogTick()
{
  if (busy_) {
    return;
  }
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (!estimate_) {
      return;  // the localization has no pose to check yet
    }
  }
  // Whatever the localization's own uncertainty: a confident localization can be wrong (a kidnap),
  // and an uncertain one is welcome to a pose.
  Request r;
  r.trigger = "watchdog";
  r.timeout = std::min(readThresholds().timeout, 5.0);
  r.dry_run = true;
  r.background = true;
  submit(std::move(r));
}

std::optional<RelocalizerNode::Request> RelocalizerNode::watch(
  const Request & request, const Result & result)
{
  if (result.published) {
    // The localization got a new pose: earlier disagreements are about the old one.
    const std::lock_guard<std::mutex> lock(mutex_);
    watchdog_.restart();
    return std::nullopt;
  }
  if (!settings_.watchdog_enabled || !request.background || result.status != Result::SUCCESS) {
    return std::nullopt;
  }
  const int64_t stamp = toNanoseconds(result.pose.header.stamp);
  const auto localization = localizationAt(stamp);
  if (!localization) {
    return std::nullopt;  // no map -> odom: the localization has no pose to compare
  }
  const Pose2 pose = toPose2(toPose3(result.pose.pose.pose));
  const double translation = translationDistance(pose, *localization);
  const double rotation_deg = deg(rotationDistance(pose, *localization));
  const double t = now().seconds();
  const KidnapWatchdogParams params = readThresholds().watchdog;
  int count = 0;
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    watchdog_.setParams(params);
    if (!watchdog_.observe(translation, rotation_deg) || !watchdog_.shouldFire(t)) {
      if (watchdog_.disagreements() > 0) {
        RCLCPP_INFO(
          get_logger(),
          "kidnap watchdog: a %s check puts the robot %.2f m / %.1f deg from the localization (%d "
          "in a row)",
          request.trigger.c_str(), translation, rotation_deg, watchdog_.disagreements());
      }
      return std::nullopt;
    }
    count = watchdog_.disagreements();
    watchdog_.fired(t);
  }
  RCLCPP_WARN(
    get_logger(),
    "kidnap watchdog: vision puts the robot %.2f m / %.1f deg from the localization in %d checks "
    "in a row: relocalizing",
    translation, rotation_deg, count);
  Request relocalization;
  relocalization.trigger = "watchdog";
  return relocalization;
}

bool RelocalizerNode::staticChain(const std::string & a, const std::string & b) const
{
  // True if a and b are joined by /tf_static transforms only (a common static ancestor).
  std::set<std::string> ancestors;
  for (std::string f = a; ancestors.insert(f).second;) {
    const auto it = static_parent_.find(f);
    if (it == static_parent_.end()) {
      break;
    }
    f = it->second;
  }
  std::set<std::string> seen;
  for (std::string f = b; seen.insert(f).second;) {
    if (ancestors.count(f) > 0) {
      return true;
    }
    const auto it = static_parent_.find(f);
    if (it == static_parent_.end()) {
      break;
    }
    f = it->second;
  }
  return false;
}

void RelocalizerNode::submit(Request request)
{
  std::optional<Request> refused;
  bool preempting = false;
  {
    const std::lock_guard<std::mutex> lock(queue_mutex_);
    if (!busy_) {
      busy_ = true;
      queued_ = std::move(request);
    } else if (running_background_ && !request.background && !queued_) {
      // A background check gives way: it stops at its next frame and this request runs next.
      queued_ = std::move(request);
      preempt_ = true;
      preempting = true;
    } else {
      refused = std::move(request);
    }
  }
  if (refused) {
    Result busy;
    busy.status = Result::BUSY;
    busy.trigger = refused->trigger;
    busy.message = "a relocalization is already running";
    answer(*refused, busy);
    return;
  }
  if (preempting) {
    {
      const std::lock_guard<std::mutex> lock(mutex_);  // the frame loop waits on images_cv_
    }
    images_cv_.notify_all();
  }
  queue_cv_.notify_one();
}

void RelocalizerNode::answer(const Request & request, Result result)
{
  result.stamp = now();
  // Background checks are not relocalizations: they go to /diagnostics, the extrinsics monitor,
  // the kidnap watchdog and the shadow log.
  if (result.status != Result::BUSY && !request.background) {
    if (status_pub_ && status_pub_->is_activated()) {
      status_pub_->publish(result);
    }
    const std::lock_guard<std::mutex> lock(mutex_);
    last_result_ = result;
  }
  if (request.header && relocalize_srv_) {
    Relocalize::Response response;
    response.result = result;
    relocalize_srv_->send_response(*request.header, response);
  }
}

void RelocalizerNode::workerLoop()
{
  while (true) {
    Request request;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(lock, [this] { return stop_worker_ || queued_.has_value(); });
      if (!queued_) {
        return;
      }
      request = std::move(*queued_);
      queued_.reset();
      running_trigger_ = request.trigger;
      running_background_ = request.background;
      preempt_ = false;
    }
    Result result;
    bool preempted = false;
    try {
      result = run(request, &preempted);
    } catch (const std::exception & e) {
      result = Result();
      result.status = Result::NOT_READY;
      result.message = std::string("internal error: ") + e.what();
      RCLCPP_ERROR(get_logger(), "relocalization failed: %s", e.what());
    }
    result.trigger = request.trigger;
    answer(request, result);
    if (request.dry_run && !preempted) {
      logShadow(request, result);
    }
    std::optional<Request> next = preempted ? std::nullopt : watch(request, result);
    if (request.trigger == "startup") {
      const std::lock_guard<std::mutex> lock(queue_mutex_);
      if (result.status == Result::SUCCESS) {
        startup_done_ = true;
      } else if (result.status != Result::NOT_READY && result.status != Result::BUSY) {
        if (++startup_attempts_ >= settings_.startup_max_attempts) {
          startup_done_ = true;
          RCLCPP_WARN(
            get_logger(), "startup relocalization gave up after %d attempts: %s", startup_attempts_,
            result.message.c_str());
        }
      } else {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 10000, "startup relocalization waits: %s",
          result.message.c_str());
      }
    }
    const std::lock_guard<std::mutex> lock(queue_mutex_);
    running_trigger_.clear();
    running_background_ = false;
    if (next && !queued_ && !stop_worker_) {
      queued_ = std::move(next);  // the watchdog's relocalization runs next; still busy
    }
    if (!queued_) {
      busy_ = false;  // else the request that preempted this one runs next
    }
  }
}

void RelocalizerNode::startupTick()
{
  {
    const std::lock_guard<std::mutex> lock(queue_mutex_);
    if (startup_done_ || Clock::now() < startup_next_) {
      if (startup_done_ && startup_timer_) {
        startup_timer_->cancel();
      }
      return;
    }
    startup_next_ =
      Clock::now() + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(
                       std::max(settings_.startup_retry_interval, 0.2)));
  }
  if (busy_) {
    return;
  }
  Request r;
  r.trigger = "startup";
  submit(std::move(r));
}

std::optional<std::string> RelocalizerNode::notReadyLocal() const
{
  if (!map_) {
    return "map database not loaded: " + database_error_;
  }
  const std::lock_guard<std::mutex> lock(mutex_);
  if (!map_files_mismatch_.empty()) {
    return map_files_mismatch_;
  }
  if (settings_.require_map_match) {
    if (!map_check_.received) {
      return std::string("no occupancy grid on 'map' yet");
    }
    if (!map_check_.mismatch.empty()) {
      return "the map on 'map' differs from the database's (" + map_check_.mismatch + ")";
    }
  }
  if (!camera_info_) {
    return "no camera_info on '" + settings_.camera_info_topic + "'";
  }
  if (
    settings_.scan_mode == ScanCheckMode::kRequired &&
    (!last_scan_ || (now() - *last_scan_).seconds() > 2.0)) {
    return "no LaserScan on '" + settings_.scan_topic + "' (scan_check.mode is required)";
  }
  CameraModel camera;
  try {
    camera = toCameraModel(*camera_info_);
  } catch (const std::invalid_argument & e) {
    return std::string("unusable camera_info: ") + e.what();
  }
  if (camera.width() != map_->camera().width() || camera.height() != map_->camera().height()) {
    return "the camera is " + std::to_string(camera.width()) + "x" +
           std::to_string(camera.height()) + ", the database was built for " +
           std::to_string(map_->camera().width()) + "x" + std::to_string(map_->camera().height());
  }
  return std::nullopt;
}

std::optional<std::string> RelocalizerNode::notReady(bool publishing)
{
  if (auto reason = notReadyLocal()) {
    return reason;
  }
  if (!publishing) {
    return std::nullopt;
  }
  if (settings_.output_mode == "topic") {
    if (initial_pose_pub_->get_subscription_count() == 0) {
      return "nothing subscribes to " + settings_.output_topic;
    }
  } else if (!set_pose_client_->service_is_ready()) {
    return "service " + settings_.output_service + " is not available";
  }
  if (amcl_state_client_) {
    const std::string node = settings_.amcl_node;
    if (!amcl_state_client_->service_is_ready()) {
      return "AMCL (" + node + ") is not running: no get_state service";
    }
    auto future = amcl_state_client_->async_send_request(
      std::make_shared<lifecycle_msgs::srv::GetState::Request>());
    if (future.future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      amcl_state_client_->remove_pending_request(future.request_id);
      return "AMCL (" + node + ") does not answer get_state";
    }
    const auto state = future.future.get()->current_state;
    if (state.id != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      return "AMCL (" + node + ") is " + state.label + ", not active";
    }
  }
  return std::nullopt;
}

std::optional<Pose3> RelocalizerNode::lookup(
  const std::string & target, const std::string & source, tf2::TimePoint time) const
{
  try {
    return toPose3(tf_buffer_->lookupTransform(target, source, time).transform);
  } catch (const tf2::TransformException &) {
    return std::nullopt;
  }
}

bool RelocalizerNode::waitForTransform(
  const std::string & target, const std::string & source, int64_t stamp_ns, double timeout) const
{
  const auto deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(
                                         std::chrono::duration<double>(timeout));
  while (true) {
    if (tf_buffer_->canTransform(target, source, timePoint(stamp_ns))) {
      return true;
    }
    if (Clock::now() >= deadline || cancel_) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void RelocalizerNode::checkExtrinsics()
{
  std::string camera_frame = settings_.frame_camera;
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (!extrinsics_status_.empty() || !map_) {
      return;
    }
    if (camera_frame.empty() && camera_info_) {
      camera_frame = camera_info_->header.frame_id;
    }
  }
  if (camera_frame.empty()) {
    return;
  }
  Pose3 current;
  try {
    current =
      toPose3(tf_buffer_->lookupTransform(settings_.frame_base, camera_frame, tf2::TimePointZero)
                .transform);
  } catch (const tf2::TransformException &) {
    return;  // not yet: retried on the next request
  }
  const std::lock_guard<std::mutex> lock(mutex_);
  if (!staticChain(settings_.frame_base, camera_frame)) {
    extrinsics_status_ =
      "skipped: " + settings_.frame_base + " -> " + camera_frame + " moves (camera on a joint)";
    extrinsics_level_ = diagnostic_msgs::msg::DiagnosticStatus::OK;
    return;
  }
  const auto difference = compareExtrinsics(map_->cameraRecord().T_base_cam, current);
  const std::string text =
    fixed(difference.translation, 3) + " m / " + fixed(difference.rotation_deg, 2) + " deg";
  if (
    difference.translation > settings_.extrinsics_warn_translation ||
    difference.rotation_deg > settings_.extrinsics_warn_rotation_deg) {
    extrinsics_status_ = "base -> camera differs from the database's by " + text +
                         " (database built for another camera mount or URDF?)";
    extrinsics_level_ = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    RCLCPP_WARN(get_logger(), "%s", extrinsics_status_.c_str());
  } else {
    extrinsics_status_ = "matches the database (" + text + ")";
    extrinsics_level_ = diagnostic_msgs::msg::DiagnosticStatus::OK;
  }
}

RelocalizerNode::Result RelocalizerNode::run(const Request & request, bool * preempted)
{
  const auto started = Clock::now();
  const Thresholds t = readThresholds();
  Result result;
  result.trigger = request.trigger;
  const bool publishing = !request.dry_run;
  if (auto reason = notReady(publishing)) {
    result.status = Result::NOT_READY;
    result.message = *reason;
    result.scores.elapsed_s = static_cast<float>(secondsSince(started));
    RCLCPP_WARN(
      get_logger(), "relocalization (%s): NOT_READY: %s", request.trigger.c_str(), reason->c_str());
    return result;
  }
  checkExtrinsics();
  localizer_->setParams(t.localizer);
  sensor_msgs::msg::CameraInfo info;
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    info = *camera_info_;  // checked by notReady()
  }
  const CameraModel camera = toCameraModel(info);
  const std::string camera_frame =
    settings_.frame_camera.empty() ? info.header.frame_id : settings_.frame_camera;
  const int required = request.num_frames > 0 ? request.num_frames : t.num_frames;
  const double timeout = request.timeout > 0.0 ? request.timeout : t.timeout;
  const auto deadline =
    started + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(timeout));
  const auto min_interval_ns = static_cast<int64_t>(t.min_frame_interval * 1e9);

  {
    const std::lock_guard<std::mutex> lock(mutex_);
    images_.clear();
    collecting_ = true;
  }
  std::vector<FrameOutcome> outcomes;
  std::vector<FrameResult> valid_frames;
  std::vector<int64_t> valid_stamps;
  FrameResult last_frame;
  cv::Mat last_image;
  int64_t last_stamp = std::numeric_limits<int64_t>::min();
  std::string skipped;  // why the latest image couldn't be used
  int valid = 0;
  double localize_s = 0.0;
  const bool yields = request.background;  // to any other request (submit())
  const bool samples_extrinsics = request.trigger == "monitor";
  std::optional<std::pair<Pose3, Pose3>> monitor_sample;
  while (!cancel_ && !(yields && preempt_) && valid < required && Clock::now() < deadline) {
    PendingImage image;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      images_cv_.wait_until(lock, deadline, [this, yields] {
        return cancel_ || (yields && preempt_) || !images_.empty();
      });
      if (images_.empty()) {
        continue;
      }
      image = std::move(images_.front());
      images_.pop_front();
    }
    if (
      last_stamp != std::numeric_limits<int64_t>::min() &&
      image.stamp_ns < last_stamp + min_interval_ns) {
      continue;
    }
    if (!waitForTransform(settings_.frame_base, camera_frame, image.stamp_ns, t.tf_timeout)) {
      skipped = "no transform " + settings_.frame_base + " -> " + camera_frame + " at the image";
      continue;
    }
    if (!waitForTransform(
          settings_.frame_odom, settings_.frame_base, image.stamp_ns, t.tf_timeout)) {
      skipped =
        "no transform " + settings_.frame_odom + " -> " + settings_.frame_base + " at the image";
      continue;
    }
    QueryFrame frame;
    frame.stamp_ns = image.stamp_ns;
    frame.camera = camera;
    Pose2 odom_T_base;
    try {
      frame.T_base_cam = toPose3(
        tf_buffer_->lookupTransform(settings_.frame_base, camera_frame, timePoint(image.stamp_ns))
          .transform);
      odom_T_base = toPose2(toPose3(
        tf_buffer_
          ->lookupTransform(settings_.frame_odom, settings_.frame_base, timePoint(image.stamp_ns))
          .transform));
    } catch (const tf2::TransformException & e) {
      skipped = e.what();
      continue;
    }
    if (settings_.gravity_source != GravitySource::kFlatFloor) {
      std::optional<Eigen::Quaterniond> q_base_imu;
      std::string imu_frame;
      {
        const std::lock_guard<std::mutex> lock(mutex_);
        imu_frame = imu_frame_;
      }
      if (!imu_frame.empty()) {
        if (const auto T_base_imu = lookup(settings_.frame_base, imu_frame, tf2::TimePointZero)) {
          q_base_imu = Eigen::Quaterniond(T_base_imu->linear());
        }
      }
      std::optional<GravityEstimate> gravity;
      if (q_base_imu) {
        const std::lock_guard<std::mutex> lock(mutex_);
        gravity = imu_.at(image.stamp_ns, *q_base_imu, settings_.gravity_source, t.gravity);
      } else if (settings_.gravity_source == GravitySource::kAuto) {
        gravity = GravityEstimate();  // no IMU: flat floor
      }
      if (!gravity) {
        skipped = "no usable IMU gravity (stale, or the robot is turning)";
        continue;
      }
      frame.gravity = *gravity;
    }
    if (settings_.scan_mode != ScanCheckMode::kDisabled) {
      std::optional<sensor_msgs::msg::LaserScan> scan;
      {
        const std::lock_guard<std::mutex> lock(mutex_);
        if (
          const auto * nearest = nearestScan(
            scans_, image.stamp_ns, static_cast<int64_t>(t.scan_max_time_offset * 1e9))) {
          scan = *nearest;
        }
      }
      std::string scan_error;
      if (scan) {
        // The laser at the scan's stamp and the odometry at both stamps.
        if (
          waitForTransform(
            settings_.frame_base, scan->header.frame_id, toNanoseconds(scan->header.stamp),
            t.tf_timeout) &&
          waitForTransform(
            settings_.frame_odom, settings_.frame_base, toNanoseconds(scan->header.stamp),
            t.tf_timeout)) {
          frame.scan = scanAtImage(
            *scan, *tf_buffer_, settings_.frame_odom, settings_.frame_base, image.stamp_ns,
            &scan_error);
        } else {
          scan_error = "no transform to the laser at the scan";
        }
      } else {
        scan_error = "no scan on '" + settings_.scan_topic + "' within " +
                     fixed(t.scan_max_time_offset, 2) + " s of the image";
      }
      if (!frame.scan && settings_.scan_mode == ScanCheckMode::kRequired) {
        skipped = scan_error;
        continue;
      }
    }
    try {
      frame.image = image.decode();
    } catch (const std::exception & e) {
      skipped = e.what();
      continue;
    }
    FrameResult result_frame = localizer_->localize(frame);
    localize_s += result_frame.timings.total_s;
    last_stamp = image.stamp_ns;
    FrameOutcome outcome;
    outcome.deepest_stage = result_frame.deepest_stage;
    outcome.ambiguous = result_frame.ambiguity.ambiguous;
    outcome.scan_elsewhere = result_frame.scan_alternative.has_value();
    if (result_frame.pose && !outcome.ambiguous) {
      outcome.estimate = FrameEstimate{*result_frame.pose, odom_T_base};
      ++valid;
      valid_frames.push_back(result_frame);
      valid_stamps.push_back(image.stamp_ns);
      if (samples_extrinsics) {
        if (auto sample = extrinsicsSample(t, image.stamp_ns, frame, result_frame)) {
          monitor_sample = std::move(sample);
        }
      }
    }
    outcomes.push_back(outcome);
    if (!t.dump_dir.empty()) {
      dump(t, image.stamp_ns, frame.image, result_frame);
    }
    RCLCPP_DEBUG(
      get_logger(), "frame %zu: %d features, %zu places, stage %d%s in %.3f s", outcomes.size(),
      result_frame.num_features, result_frame.hypotheses.size(),
      static_cast<int>(result_frame.deepest_stage), outcome.ambiguous ? " (ambiguous)" : "",
      result_frame.timings.total_s);
    last_frame = std::move(result_frame);
    last_image = frame.image;
    if (outcome.ambiguous) {
      break;
    }
  }
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    collecting_ = false;
    images_.clear();
  }
  if (cancel_) {
    result.status = Result::NOT_READY;
    result.message = "the relocalizer stopped during the request";
    return result;
  }
  if (yields && preempt_) {
    *preempted = true;
    result.status = Result::NOT_READY;
    result.message = "stopped for another request";
    return result;
  }
  if (outcomes.empty()) {
    result.status = Result::NOT_READY;
    if (!skipped.empty()) {
      result.message = skipped;
    } else {
      result.message =
        "no image on '" + settings_.image_topic + "' within " + fixed(timeout, 1) + " s";
    }
    result.scores.elapsed_s = static_cast<float>(secondsSince(started));
    RCLCPP_WARN(
      get_logger(), "relocalization (%s): NOT_READY: %s", request.trigger.c_str(),
      result.message.c_str());
    return result;
  }

  DecisionInput input;
  input.frames = outcomes;
  input.required_frames = required;
  const Decision decision = decide(input, t.consistency);
  result.status = static_cast<uint8_t>(decision.status);
  result.message = decision.message;
  if (monitor_sample && decision.status == Status::kSuccess) {
    // Only requests that passed every check: one sample, from the latest slow frame.
    const std::lock_guard<std::mutex> lock(mutex_);
    monitor_.setParams(t.monitor);
    monitor_.addSample(monitor_sample->first, monitor_sample->second);
    const auto estimate = monitor_.estimate();
    RCLCPP_DEBUG(
      get_logger(), "extrinsics monitor: %zu samples, offset %.3f m / %.2f deg", estimate.samples,
      estimate.translation_norm, estimate.rotation_deg);
  }

  // Scores: worst across the frames that produced a pose (else the last frame's).
  auto & s = result.scores;
  s.frames_used = static_cast<uint32_t>(decision.frames_used);
  const ScoreSummary summary =
    summarizeScores(valid_frames.empty() ? std::vector<FrameResult>{last_frame} : valid_frames);
  s.num_inliers = static_cast<uint32_t>(summary.num_inliers);
  s.inlier_ratio = static_cast<float>(summary.inlier_ratio);
  s.mean_reprojection_error_px = static_cast<float>(summary.mean_reprojection_px);
  s.height_error_m = static_cast<float>(summary.height_error_m);
  s.tilt_error_deg = static_cast<float>(summary.tilt_error_deg);
  s.inlier_coverage = static_cast<float>(summary.inlier_coverage);
  s.ambiguity_ratio = static_cast<float>(summary.ambiguity_ratio);
  s.scan_hit_ratio = static_cast<float>(summary.scan_hit_ratio);
  s.scan_violation_ratio = static_cast<float>(summary.scan_violation_ratio);
  s.consistency_translation_error_m =
    static_cast<float>(decision.consistency.max_translation_error);
  s.consistency_rotation_error_deg =
    static_cast<float>(decision.consistency.max_rotation_error_deg);

  for (std::size_t i = 0; i < last_frame.hypotheses.size() && i < 10; ++i) {
    const Hypothesis & h = last_frame.hypotheses[i];
    landfall_msgs::msg::Hypothesis msg;
    if (h.solved) {
      msg.pose = toPoseMsg(h.sanity.T_map_base);
    }
    msg.cluster_id = h.cluster >= 0 ? static_cast<uint32_t>(h.cluster) : 0U;
    msg.num_inliers = static_cast<uint32_t>(h.sanity.num_inliers);
    msg.inlier_ratio = static_cast<float>(h.sanity.inlier_ratio);
    for (const int64_t id : h.keyframe_ids) {
      msg.keyframe_ids.push_back(static_cast<uint32_t>(id));
    }
    msg.stage_reached = static_cast<uint8_t>(h.stage);
    msg.rejection_reason = h.rejection;
    result.hypotheses.push_back(msg);
  }

  if (decision.status == Status::kSuccess && decision.pose) {
    // decide() returns the pose at the last valid frame; move it to the latest odometry.
    Pose2 pose = *decision.pose;
    int64_t stamp_ns = valid_stamps.back();
    if (t.compensate_motion) {
      try {
        const auto latest = tf_buffer_->lookupTransform(
          settings_.frame_odom, settings_.frame_base, tf2::TimePointZero);
        Pose2 odom_at_last;
        for (auto it = outcomes.rbegin(); it != outcomes.rend(); ++it) {
          if (it->estimate) {
            odom_at_last = it->estimate->odom_T_base;
            break;
          }
        }
        pose = propagate(pose, odom_at_last, toPose2(toPose3(latest.transform)));
        stamp_ns = toNanoseconds(latest.header.stamp);
      } catch (const tf2::TransformException & e) {
        RCLCPP_WARN(get_logger(), "no current odometry to compensate motion: %s", e.what());
      }
    }
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.frame_id = settings_.frame_global;
    msg.header.stamp = toStamp(stamp_ns);
    msg.pose.pose = toPoseMsg(landfall::toPose3(pose));
    const double var_xy = t.std_xy * t.std_xy;
    const double std_yaw = t.std_yaw_deg * M_PI / 180.0;
    msg.pose.covariance[0] = var_xy;
    msg.pose.covariance[7] = var_xy;
    msg.pose.covariance[35] = std_yaw * std_yaw;
    result.pose = msg;
    result.message = "localized at " + poseText(pose) + " from " +
                     std::to_string(decision.frames_used) + " frames";
    if (publishing) {
      std::string error;
      result.published = publish(msg, &error);
      if (!result.published) {
        result.message += "; not delivered: " + error;
      }
    } else {
      result.message += " (dry run, not published)";
    }
  }
  s.elapsed_s = static_cast<float>(secondsSince(started));

  if (t.publish_markers) {
    publishMarkers(&last_frame, result);
  }
  if (t.publish_image && !last_image.empty()) {
    publishDebugImage(last_image, camera_frame, last_frame, result);
  }
  RCLCPP_INFO(
    get_logger(), "relocalization (%s): %s in %.2f s (%zu frames, %.0f ms per frame%s): %s",
    request.trigger.c_str(), toString(decision.status), s.elapsed_s, outcomes.size(),
    1000.0 * localize_s / static_cast<double>(outcomes.size()),
    result.published ? ", published" : "", result.message.c_str());
  return result;
}

bool RelocalizerNode::publish(
  const geometry_msgs::msg::PoseWithCovarianceStamped & pose, std::string * error)
{
  if (settings_.output_mode == "topic") {
    initial_pose_pub_->publish(pose);
    return true;
  }
  auto request = std::make_shared<nav2_msgs::srv::SetInitialPose::Request>();
  request->pose = pose;
  auto future = set_pose_client_->async_send_request(request);
  if (future.future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
    set_pose_client_->remove_pending_request(future.request_id);
    *error = settings_.output_service + " did not answer within 5 s";
    return false;
  }
  return true;
}

void RelocalizerNode::publishMarkers(const FrameResult * frame, const Result & result)
{
  if (!marker_pub_ || !marker_pub_->is_activated()) {
    return;
  }
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);
  const auto arrow =
    [&](const std::string & ns, int id, const Pose2 & p, float r, float g, float b, double length) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = settings_.frame_global;
      m.header.stamp = now();
      m.ns = ns;
      m.id = id;
      m.type = visualization_msgs::msg::Marker::ARROW;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose = toPoseMsg(landfall::toPose3(p));
      m.scale.x = length;
      m.scale.y = 0.06;
      m.scale.z = 0.06;
      m.color.r = r;
      m.color.g = g;
      m.color.b = b;
      m.color.a = 1.0F;
      markers.markers.push_back(m);
    };
  if (frame != nullptr) {
    int id = 0;
    for (const auto & h : frame->hypotheses) {
      if (!h.solved) {
        continue;
      }
      const bool ok = h.stage == (frame->scan_checked ? Stage::kScanPassed : Stage::kSanityPassed);
      arrow("hypotheses", id++, h.sanity.pose, ok ? 0.1F : 0.9F, ok ? 0.8F : 0.5F, 0.1F, 0.5);
    }
    if (frame->best >= 0) {
      int k = 0;
      for (const int64_t kid :
           frame->hypotheses[static_cast<std::size_t>(frame->best)].keyframe_ids) {
        if (const MapKeyframe * keyframe = map_->keyframe(kid)) {
          arrow("matched_keyframes", k++, keyframe->pose, 0.6F, 0.6F, 0.6F, 0.3);
        }
      }
    }
  }
  if (result.status == Result::SUCCESS) {
    arrow("result", 0, toPose2(toPose3(result.pose.pose.pose)), 0.1F, 0.3F, 1.0F, 1.0);
  }
  marker_pub_->publish(markers);
}

void RelocalizerNode::publishDebugImage(
  const cv::Mat & image, const std::string & camera_frame, const FrameResult & frame,
  const Result & result)
{
  if (
    !debug_image_pub_ || !debug_image_pub_->is_activated() ||
    debug_image_pub_->get_subscription_count() == 0) {
    return;
  }
  cv::Mat canvas;
  if (image.channels() == 1) {
    cv::cvtColor(image, canvas, cv::COLOR_GRAY2BGR);
  } else {
    canvas = image.clone();
  }
  const Hypothesis * h = frame.best >= 0 ? &frame.hypotheses[static_cast<std::size_t>(frame.best)]
                         : frame.hypotheses.empty() ? nullptr
                                                    : &frame.hypotheses.front();
  if (h != nullptr) {
    for (const auto & p : h->outlier_pixels) {
      cv::circle(canvas, cv::Point2d(p.x(), p.y()), 3, cv::Scalar(0, 0, 255), 1);
    }
    for (const auto & p : h->inlier_pixels) {
      cv::circle(canvas, cv::Point2d(p.x(), p.y()), 3, cv::Scalar(0, 220, 0), 2);
    }
  }
  const std::string text = std::string(toString(static_cast<Status>(result.status))) + "  " +
                           std::to_string(h != nullptr ? h->sanity.num_inliers : 0) + " inliers";
  cv::putText(
    canvas, text, cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
  cv_bridge::CvImage out;
  out.header.frame_id = camera_frame;
  out.header.stamp = now();
  out.encoding = "bgr8";
  out.image = canvas;
  debug_image_pub_->publish(*out.toImageMsg());
}

void RelocalizerNode::dump(
  const Thresholds & t, int64_t stamp_ns, const cv::Mat & image, const FrameResult & frame)
{
  try {
    std::filesystem::create_directories(t.dump_dir);
    const std::string name = std::to_string(stamp_ns);
    cv::imwrite((std::filesystem::path(t.dump_dir) / (name + ".jpg")).string(), image);
    JsonWriter json;
    json.beginObject()
      .field("stamp_ns", stamp_ns)
      .field("image", name + ".jpg")
      .field("features", frame.num_features)
      .field("deepest_stage", static_cast<int>(frame.deepest_stage))
      .field("ambiguous", frame.ambiguity.ambiguous)
      .field("ambiguity_ratio", frame.ambiguity.score_ratio)
      .field("seconds", frame.timings.total_s);
    if (frame.pose) {
      json.key("pose")
        .beginArray()
        .value(frame.pose->x)
        .value(frame.pose->y)
        .value(frame.pose->yaw);
      json.endArray();
    }
    json.key("hypotheses").beginArray();
    for (const auto & h : frame.hypotheses) {
      json.beginObject()
        .field("stage", static_cast<int>(h.stage))
        .field("correspondences", h.correspondences)
        .field("inliers", h.sanity.num_inliers)
        .field("rejection", h.rejection);
      if (h.scan_checked) {
        json.field("scan_hit_ratio", h.scan.score.hit_ratio)
          .field("scan_violation_ratio", h.scan.score.violation_ratio);
      }
      json.key("keyframes").beginArray();
      for (const int64_t id : h.keyframe_ids) {
        json.value(static_cast<int64_t>(id));
      }
      json.endArray().endObject();
    }
    json.endArray().endObject();
    std::ofstream out(
      (std::filesystem::path(t.dump_dir) / "queries.jsonl").string(), std::ios::app);
    out << json.str() << "\n";
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 10000, "cannot write to debug.dump_dir: %s", e.what());
  }
}

std::optional<std::pair<Pose3, Pose3>> RelocalizerNode::extrinsicsSample(
  const Thresholds & t, int64_t stamp_ns, const QueryFrame & frame,
  const FrameResult & result) const
{
  // Only while the robot is slow: time sync and the localization's lag would look like an offset.
  const auto window = std::chrono::milliseconds(200);
  const tf2::TimePoint at = timePoint(stamp_ns);
  const auto before = lookup(settings_.frame_odom, settings_.frame_base, at - window);
  const auto after = lookup(settings_.frame_odom, settings_.frame_base, at);
  const auto T_map_base = lookup(settings_.frame_global, settings_.frame_base, at);
  if (!before || !after || !T_map_base || result.best < 0) {
    return std::nullopt;
  }
  const double seconds = std::chrono::duration<double>(window).count();
  const Pose2 motion = between(toPose2(*before), toPose2(*after));
  if (
    std::hypot(motion.x, motion.y) / seconds > t.monitor_max_linear_speed ||
    std::abs(motion.yaw) / seconds > t.monitor_max_angular_speed) {
    return std::nullopt;
  }
  const Pose3 & visual = result.hypotheses[static_cast<std::size_t>(result.best)].T_map_cam;
  return std::make_pair(visual, *T_map_base * frame.T_base_cam);
}

void RelocalizerNode::logShadow(const Request & request, const Result & result)
{
  if (settings_.shadow_log_path.empty()) {
    return;
  }
  std::ostringstream line;
  line << toNanoseconds(result.stamp) << "," << request.trigger << ","
       << toString(static_cast<Status>(result.status)) << "," << result.scores.frames_used << ",";
  std::optional<LocalizationEstimate> estimate;
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    estimate = estimate_;
  }
  if (result.status == Result::SUCCESS) {
    const int64_t stamp = toNanoseconds(result.pose.header.stamp);
    const Pose2 pose = toPose2(toPose3(result.pose.pose.pose));
    line << stamp << "," << fixed(pose.x, 4) << "," << fixed(pose.y, 4) << "," << fixed(pose.yaw, 5)
         << ",";
    // The localization's own pose at the same stamp, through TF.
    if (const auto localization = localizationAt(stamp)) {
      const Pose2 & other = *localization;
      line << fixed(other.x, 4) << "," << fixed(other.y, 4) << "," << fixed(other.yaw, 5) << ","
           << fixed(translationDistance(pose, other), 4) << ","
           << fixed(deg(rotationDistance(pose, other)), 3) << ",";
    } else {
      line << ",,,,,";
    }
  } else {
    line << ",,,,,,,,,";
  }
  if (estimate) {
    line << fixed(estimate->std_xy, 4) << "," << fixed(estimate->std_yaw_deg, 3) << ",";
  } else {
    line << ",,";
  }
  std::string message = result.message;
  std::replace(message.begin(), message.end(), ',', ';');
  line << result.scores.num_inliers << "," << fixed(result.scores.scan_hit_ratio, 3) << ","
       << fixed(result.scores.elapsed_s, 3) << "," << message;
  try {
    const bool fresh = !std::filesystem::exists(settings_.shadow_log_path);
    std::ofstream out(settings_.shadow_log_path, std::ios::app);
    if (fresh) {
      out << "stamp_ns,trigger,status,frames_used,pose_stamp_ns,x,y,yaw,localization_x,"
             "localization_y,localization_yaw,error_m,error_deg,localization_std_xy,"
             "localization_std_yaw_deg,inliers,scan_hit_ratio,elapsed_s,message\n";
    }
    out << line.str() << "\n";
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 10000, "cannot write shadow.log_path: %s", e.what());
  }
}

std::optional<Pose2> RelocalizerNode::localizationAt(int64_t stamp_ns) const
{
  if (
    const auto T_map_base =
      lookup(settings_.frame_global, settings_.frame_base, timePoint(stamp_ns))) {
    return toPose2(*T_map_base);
  }
  return std::nullopt;
}

void RelocalizerNode::diagnose(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  checkExtrinsics();
  if (const auto reason = notReadyLocal()) {
    // Without a database nothing works; the rest can still come (map, camera_info).
    status.summary(
      map_ ? DiagnosticStatus::WARN : DiagnosticStatus::ERROR, "not ready: " + *reason);
  } else if (busy_) {
    status.summary(DiagnosticStatus::OK, "relocalizing");
  } else {
    status.summary(DiagnosticStatus::OK, "ready");
  }
  status.add("database", settings_.database_path);
  if (map_) {
    status.add("keyframes", map_->keyframes().size());
    status.add("landmarks", map_->numLandmarks());
    status.add("content_sha256", map_->contentSha256());
  }
  const auto confirmations = get_parameter("kidnap_watchdog.confirmations").as_int();
  const std::lock_guard<std::mutex> lock(mutex_);
  if (!extrinsics_status_.empty()) {
    status.add("extrinsics", extrinsics_status_);
    if (extrinsics_level_ != DiagnosticStatus::OK) {
      status.mergeSummary(static_cast<unsigned char>(extrinsics_level_), extrinsics_status_);
    }
  }
  if (last_result_) {
    status.add("last_status", toString(static_cast<Status>(last_result_->status)));
    status.add("last_message", last_result_->message);
    status.add("last_trigger", last_result_->trigger);
    status.add("last_elapsed_s", last_result_->scores.elapsed_s);
  }
  if (settings_.watchdog_enabled) {
    status.add(
      "kidnap_watchdog", std::to_string(watchdog_.disagreements()) +
                           " disagreeing check(s) in a row (relocalizes at " +
                           std::to_string(confirmations) + ")");
  }
  if (settings_.monitor_enabled) {
    // Detection only: a camera that moved on its mount is reported, never corrected.
    const auto e = monitor_.estimate();
    status.add("extrinsics_monitor_samples", e.samples);
    if (e.level == HealthLevel::kInsufficientData) {
      status.add("extrinsics_monitor", "collecting samples");
    } else {
      const std::string text = "camera " + fixed(100.0 * e.translation_norm, 1) + " cm / " +
                               fixed(e.rotation_deg, 2) + " deg from where TF puts it (median of " +
                               std::to_string(e.samples) + " samples)";
      status.add("extrinsics_monitor", text);
      if (e.level == HealthLevel::kWarn) {
        status.mergeSummary(DiagnosticStatus::WARN, "extrinsics monitor: " + text);
      } else if (e.level == HealthLevel::kError) {
        status.mergeSummary(DiagnosticStatus::ERROR, "extrinsics monitor: " + text);
      }
    }
  }
}

}  // namespace landfall::ros

RCLCPP_COMPONENTS_REGISTER_NODE(landfall::ros::RelocalizerNode)
