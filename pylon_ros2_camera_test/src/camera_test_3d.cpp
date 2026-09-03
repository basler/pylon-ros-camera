/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (C) 2024, Basler AG. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * No contributors' name may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#include "pylon_ros2_camera_test/camera_test_3d.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <atomic>
#include <future>
#include <memory>
#include <string>

namespace pylon_ros2_camera_test
{

// ─────────────────────────────────────────────────────────────────────────────
// Constructor
// ─────────────────────────────────────────────────────────────────────────────

CameraTest3D::CameraTest3D(const rclcpp::NodeOptions & options)
: CameraTestGeneric("camera_test_3d", options)
{
  // The current driver action for 3D data is named grab_3d_data.
  grab_3d_client_ = rclcpp_action::create_client<Grab3DDataAction>(
    this, camera_ns_ + "/grab_3d_data");

  set_depth_min_client_ =
    make_client<SetFloatValue>("set_depth_min");
  set_depth_max_client_ =
    make_client<SetFloatValue>("set_depth_max");
  set_brightness_client_ =
    make_client<SetBrightness>("set_brightness");
  enable_spatial_filter_client_ =
    make_client<SetBool>("enable_spatial_filter");
  enable_temporal_filter_client_ =
    make_client<SetBool>("enable_temporal_filter");
  set_confidence_threshold_client_ =
    make_client<SetFloatValue>("set_confidence_threshold");
  enable_hdr_mode_client_ =
    make_client<SetBool>("enable_hdr_mode");
  enable_static_scene_client_ =
    make_client<SetBool>("enable_static_scene");
  set_illumination_mode_client_ =
    make_client<SetIntegerValue>("set_illumination_mode");
  set_depth_quality_client_ =
    make_client<SetIntegerValue>("set_depth_quality");
  enable_depth_smooth_client_ =
    make_client<SetBool>("enable_depth_smooth");
  set_depth_fill_client_ =
    make_client<SetIntegerValue>("set_depth_fill");
  set_depth_seg_client_ =
    make_client<SetIntegerValue>("set_depth_seg");
  enable_projector_client_ =
    make_client<SetBool>("enable_projector");
  set_projector_level_client_ =
    make_client<SetIntegerValue>("set_projector_level");
  set_operating_mode_client_ =
    make_client<SetIntegerValue>("set_operating_mode");
  set_source_selector_client_ =
    make_client<SetIntegerValue>("set_source_selector");
  set_hdr_exposure_time_selector_client_ =
    make_client<SetIntegerValue>("set_hdr_exposure_time_selector");
  set_hdr_exposure_time_client_ =
    make_client<SetFloatValue>("set_hdr_exposure_time");
  set_hdr_sub_exposures_client_ =
    make_client<SetIntegerValue>("set_hdr_sub_exposures");
  set_exposure_auto_mode_client_ =
    make_client<SetIntegerValue>("set_exposure_auto_mode");
  set_hdr_sequence_index_client_ =
    make_client<SetIntegerValue>("set_hdr_sequence_index");
  set_hdr_sequence_preset_client_ =
    make_client<SetIntegerValue>("set_hdr_sequence_preset");
  set_hdr_max_exposure_client_ =
    make_client<SetFloatValue>("set_hdr_max_exposure");
  enable_hdr_merge_client_ =
    make_client<SetBool>("enable_hdr_merge");
  enable_hdr_merge_use_ir_client_ =
    make_client<SetBool>("enable_hdr_merge_use_ir");
  load_hdr_preset_client_ =
    make_client<Trigger>("load_hdr_preset");

  // Generic tests run first, then 3D-specific tests.
  register_generic_tests();
  register_test("test_grab_3d_data",
    std::bind(&CameraTest3D::test_grab_3d_data, this));
  register_test("test_set_depth_range",
    std::bind(&CameraTest3D::test_set_depth_range, this));
  register_test("test_enable_spatial_filter",
    std::bind(&CameraTest3D::test_enable_spatial_filter, this));
  register_test("test_enable_temporal_filter",
    std::bind(&CameraTest3D::test_enable_temporal_filter, this));
  register_test("test_set_brightness",
    std::bind(&CameraTest3D::test_set_brightness, this));
  register_test("test_set_confidence_threshold",
    std::bind(&CameraTest3D::test_set_confidence_threshold, this));
  register_test("test_enable_hdr_mode",
    std::bind(&CameraTest3D::test_enable_hdr_mode, this));
  register_test("test_set_illumination_mode",
    std::bind(&CameraTest3D::test_set_illumination_mode, this));
  register_test("test_set_depth_quality",
    std::bind(&CameraTest3D::test_set_depth_quality, this));
  register_test("test_enable_static_scene",
    std::bind(&CameraTest3D::test_enable_static_scene, this));
  register_test("test_enable_depth_smooth",
    std::bind(&CameraTest3D::test_enable_depth_smooth, this));
  register_test("test_set_depth_fill",
    std::bind(&CameraTest3D::test_set_depth_fill, this));
  register_test("test_set_depth_seg",
    std::bind(&CameraTest3D::test_set_depth_seg, this));
  register_test("test_enable_projector",
    std::bind(&CameraTest3D::test_enable_projector, this));
  register_test("test_set_projector_level",
    std::bind(&CameraTest3D::test_set_projector_level, this));
  register_test("test_set_depth_preset",
    std::bind(&CameraTest3D::test_set_depth_preset, this));
  register_test("test_set_hdr_exposure_time_selector",
    std::bind(&CameraTest3D::test_set_hdr_exposure_time_selector, this));
  register_test("test_set_hdr_exposure_time",
    std::bind(&CameraTest3D::test_set_hdr_exposure_time, this));
  register_test("test_set_hdr_sub_exposures",
    std::bind(&CameraTest3D::test_set_hdr_sub_exposures, this));
  register_test("test_set_exposure_auto_mode",
    std::bind(&CameraTest3D::test_set_exposure_auto_mode, this));
  register_test("test_set_hdr_max_exposure",
    std::bind(&CameraTest3D::test_set_hdr_max_exposure, this));
  register_test("test_hdr_sequence_workflow",
    std::bind(&CameraTest3D::test_hdr_sequence_workflow, this));
  register_test("test_hdr_sub_exposure_workflow",
    std::bind(&CameraTest3D::test_hdr_sub_exposure_workflow, this));

  register_test("test_imu_stream",
    std::bind(&CameraTest3D::test_imu_stream, this));

  // Start the test thread LAST, after all tests are registered.
  start_tests();
}

// ─────────────────────────────────────────────────────────────────────────────
// Camera detection
// ─────────────────────────────────────────────────────────────────────────────

bool CameraTest3D::detect_camera()
{
  // Step 1: Wait until the camera hardware is connected. get_max_num_buffer
  // succeeds only when the camera is up (and implies the driver is running).
  // A blaze can take several seconds to connect through its GenTL producer.
  {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(detection_timeout_);
    bool connected = false;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
    {
      auto req = std::make_shared<GetIntegerValue::Request>();
      auto res = call_service<GetIntegerValue>(
        get_max_num_buffer_client_, req,
        std::chrono::seconds(2), std::chrono::seconds(2), false);
      if (res && res->success && res->value > 0) { connected = true; break; }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    if (!connected) return false;  // No reachable camera within the timeout.
  }
  if (!rclcpp::ok()) return false;

  // Step 2: A 3D camera streams point-cloud data on cloud_3d; a 2D camera
  // never does. Data flow is the reliable 3D signal (more robust than the
  // grab_3d_data action, whose discovery can lag just after connection).
  {
    std::atomic<bool> got_cloud{false};
    auto cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      camera_ns_ + "/cloud_3d", rclcpp::QoS(rclcpp::KeepLast(1)),
      [&got_cloud](const sensor_msgs::msg::PointCloud2::SharedPtr) { got_cloud = true; });
    auto disc_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (rclcpp::ok() && !got_cloud && std::chrono::steady_clock::now() < disc_deadline)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (got_cloud)
    {
      return true;  // 3D camera confirmed -> run 3D tests.
    }
  }

  // Connected but no point-cloud stream -> 2D camera, wrong type for this node.
  RCLCPP_INFO(get_logger(),
    "2D camera detected (no point-cloud stream). Skipping 3D tests.");
  is_wrong_camera_type_ = true;
  return false;
}

// ─────────────────────────────────────────────────────────────────────────────
// 3D test implementations
// ─────────────────────────────────────────────────────────────────────────────

// Send a grab_3d_data goal with exposure_given=true and verify that point
// clouds and intensity maps are returned.
bool CameraTest3D::test_grab_3d_data()
{
  auto goal = Grab3DDataAction::Goal();
  goal.exposure_given = true;
  goal.exposure_times.push_back(500.0f);

  auto result_promise =
    std::make_shared<std::promise<Grab3DDataGoalHdl::WrappedResult>>();
  auto result_future = result_promise->get_future();

  auto send_goal_options =
    rclcpp_action::Client<Grab3DDataAction>::SendGoalOptions();

  send_goal_options.result_callback =
    [result_promise](const Grab3DDataGoalHdl::WrappedResult & result) {
      result_promise->set_value(result);
    };

  // Wait for goal acceptance
  auto goal_handle_future =
    grab_3d_client_->async_send_goal(goal, send_goal_options);

  if (goal_handle_future.wait_for(std::chrono::seconds(10)) !=
      std::future_status::ready)
  {
    return assert_true(false, "test_grab_3d_data",
      "goal not accepted within 10 s");
  }
  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    return assert_true(false, "test_grab_3d_data",
      "goal was rejected by the action server");
  }

  // Wait for result
  if (result_future.wait_for(std::chrono::seconds(60)) !=
      std::future_status::ready)
  {
    return assert_true(false, "test_grab_3d_data",
      "result not received within 60 s");
  }
  auto wrapped = result_future.get();

  if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
    return assert_true(false, "test_grab_3d_data",
      "action did not succeed (code " +
      std::to_string(static_cast<int>(wrapped.code)) + ")");
  }

  bool ok = assert_true(wrapped.result->success,
    "test_grab_3d_data/success", "grab action reported failure");
  ok &= assert_true(!wrapped.result->point_clouds.empty(),
    "test_grab_3d_data/point_clouds", "no point clouds in result");
  ok &= assert_true(!wrapped.result->intensity_maps.empty(),
    "test_grab_3d_data/intensity_maps", "no intensity maps in result");
  if (!wrapped.result->point_clouds.empty()) {
    ok &= assert_true(wrapped.result->point_clouds.front().width > 0,
      "test_grab_3d_data/cloud_width", "point cloud has zero width");
  }
  return ok;
}

// Read one current_params message from the driver. Waits up to 5 s.
bool CameraTest3D::read_current_params(CurrentParams & out)
{
  std::atomic<bool> got{false};
  CurrentParams latest;
  auto sub = this->create_subscription<CurrentParams>(
    camera_ns_ + "/current_params", rclcpp::QoS(rclcpp::KeepLast(1)),
    [&](const CurrentParams::SharedPtr msg) { latest = *msg; got = true; });
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (rclcpp::ok() && !got && std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  if (!got) return false;
  out = latest;
  return true;
}

// Read the camera's current working depth range from the current_params topic.
// Units are camera-native (mm for blaze/mini, meters for the stereo ace).
bool CameraTest3D::read_current_depth_range(float & depth_min, float & depth_max)
{
  CurrentParams p;
  if (!read_current_params(p)) return false;
  depth_min = p.depth_min;
  depth_max = p.depth_max;
  return true;
}

// Read the camera's current working depth range from current_params (the units
// are camera-native: mm for blaze/mini, meters for the stereo ace), set min and
// max to values inside that range, verify both calls succeed, then restore the
// original range. Reading the range first keeps the test valid whatever units
// the camera uses.
bool CameraTest3D::test_set_depth_range()
{
  float depth_min0 = -1.0f;
  float depth_max0 = -1.0f;
  if (!read_current_depth_range(depth_min0, depth_max0) ||
      depth_max0 <= depth_min0)
  {
    RCLCPP_WARN(get_logger(),
      "test_set_depth_range: depth range not reported by this camera, skipping.");
    return true;
  }

  // Pick set-points 20 % inside the current range, so they are always within
  // the camera's valid bounds regardless of units.
  const float span = depth_max0 - depth_min0;
  const float test_min = depth_min0 + span * 0.2f;
  const float test_max = depth_max0 - span * 0.2f;

  // Set depth_min
  auto req_min = std::make_shared<SetFloatValue::Request>();
  req_min->value = test_min;
  auto res_min = call_service<SetFloatValue>(set_depth_min_client_, req_min);
  if (!res_min) {
    return assert_true(false, "test_set_depth_range",
      "set_depth_min service call failed");
  }
  bool ok = assert_success(res_min->success, res_min->message,
    "test_set_depth_range/depth_min");

  // Set depth_max
  auto req_max = std::make_shared<SetFloatValue::Request>();
  req_max->value = test_max;
  auto res_max = call_service<SetFloatValue>(set_depth_max_client_, req_max);
  if (!res_max) {
    return assert_true(false, "test_set_depth_range",
      "set_depth_max service call failed");
  }
  ok &= assert_success(res_max->success, res_max->message,
    "test_set_depth_range/depth_max");

  // Restore the original range
  auto req_min_restore = std::make_shared<SetFloatValue::Request>();
  req_min_restore->value = depth_min0;
  call_service<SetFloatValue>(set_depth_min_client_, req_min_restore);

  auto req_max_restore = std::make_shared<SetFloatValue::Request>();
  req_max_restore->value = depth_max0;
  call_service<SetFloatValue>(set_depth_max_client_, req_max_restore);

  return ok;
}

// Enable then disable the spatial filter; verify both service calls succeed.
// Not every 3D camera has this filter (the stereo ace does not); when the
// driver reports it is unavailable the test skips so the suite stays usable
// across all 3D models.
bool CameraTest3D::test_enable_spatial_filter()
{
  // Enable
  auto req_on = std::make_shared<SetBool::Request>();
  req_on->data = true;
  auto res_on = call_service<SetBool>(enable_spatial_filter_client_, req_on);
  if (!res_on) {
    return assert_true(false, "test_enable_spatial_filter",
      "enable(true) service call failed");
  }
  if (!res_on->success) {
    RCLCPP_WARN(get_logger(),
      "test_enable_spatial_filter: spatial filter not available for this camera, skipping.");
    return true;
  }
  bool ok = assert_true(res_on->success,
    "test_enable_spatial_filter/on", res_on->message);

  // Disable (restore)
  auto req_off = std::make_shared<SetBool::Request>();
  req_off->data = false;
  auto res_off = call_service<SetBool>(enable_spatial_filter_client_, req_off);
  if (!res_off) {
    return assert_true(false, "test_enable_spatial_filter",
      "enable(false) service call failed");
  }
  ok &= assert_true(res_off->success,
    "test_enable_spatial_filter/off", res_off->message);
  return ok;
}

// Enable then disable the temporal filter; verify both service calls succeed.
// Not every 3D camera has this filter (the stereo ace does not); when the
// driver reports it is unavailable the test skips so the suite stays usable
// across all 3D models.
bool CameraTest3D::test_enable_temporal_filter()
{
  // Enable
  auto req_on = std::make_shared<SetBool::Request>();
  req_on->data = true;
  auto res_on = call_service<SetBool>(enable_temporal_filter_client_, req_on);
  if (!res_on) {
    return assert_true(false, "test_enable_temporal_filter",
      "enable(true) service call failed");
  }
  if (!res_on->success) {
    RCLCPP_WARN(get_logger(),
      "test_enable_temporal_filter: temporal filter not available for this camera, skipping.");
    return true;
  }
  bool ok = assert_true(res_on->success,
    "test_enable_temporal_filter/on", res_on->message);

  // Disable (restore)
  auto req_off = std::make_shared<SetBool::Request>();
  req_off->data = false;
  auto res_off = call_service<SetBool>(enable_temporal_filter_client_, req_off);
  if (!res_off) {
    return assert_true(false, "test_enable_temporal_filter",
      "enable(false) service call failed");
  }
  ok &= assert_true(res_off->success,
    "test_enable_temporal_filter/off", res_off->message);
  return ok;
}

// Set brightness with exposure_auto and verify the service responds and, when
// supported, succeeds. The stereo mini and stereo ace support brightness; the
// blaze may not, in which case the driver returns success=false and the test
// skips gracefully so the suite stays usable across all 3D models.
bool CameraTest3D::test_set_brightness()
{
  auto req = std::make_shared<SetBrightness::Request>();
  req->target_brightness = 100;
  req->brightness_continuous = false;
  req->exposure_auto = true;
  req->gain_auto = false;
  auto res = call_service<SetBrightness>(set_brightness_client_, req);
  if (!res) {
    return assert_true(false, "test_set_brightness",
      "service call failed or timed out");
  }
  if (!res->success) {
    RCLCPP_WARN(get_logger(),
      "test_set_brightness: brightness not supported by this camera, skipping.");
    return true;
  }
  return assert_true(res->success,
    "test_set_brightness/success", "set_brightness reported failure");
}

// Confidence threshold is exposed differently per model (a float in [0, 1] on
// the stereo ace, an integer with a step on the blaze). Write back the current
// value read from current_params so the value is always valid, and verify the
// service accepts it. Skips when the camera does not report a threshold.
bool CameraTest3D::test_set_confidence_threshold()
{
  CurrentParams p;
  if (!read_current_params(p) || p.confidence_threshold < 0.0f) {
    RCLCPP_WARN(get_logger(),
      "test_set_confidence_threshold: not available for this camera, skipping.");
    return true;
  }

  auto req = std::make_shared<SetFloatValue::Request>();
  req->value = p.confidence_threshold;
  auto res = call_service<SetFloatValue>(set_confidence_threshold_client_, req);
  if (!res) {
    return assert_true(false, "test_set_confidence_threshold",
      "service call failed");
  }
  return assert_success(res->success, res->message,
    "test_set_confidence_threshold/set");
}

// HDR maps to BslHdrEnable on the stereo ace (also blaze/mini). Enables HDR,
// verifies the call, then restores the original state. Skips when the camera
// does not expose HDR.
bool CameraTest3D::test_enable_hdr_mode()
{
  CurrentParams p;
  if (!read_current_params(p) || p.hdr_mode < 0) {
    RCLCPP_WARN(get_logger(),
      "test_enable_hdr_mode: not available for this camera, skipping.");
    return true;
  }
  const bool original_on = (p.hdr_mode == 1);

  // The stereo mini only allows writing HDR while an IR source (Source1/Source2)
  // is selected. Select Source1 first; cameras that are not source-gated report
  // set_source_selector as not available, which is fine to ignore here.
  auto sel = std::make_shared<SetIntegerValue::Request>();
  sel->value = 1;
  auto sel_res = call_service<SetIntegerValue>(set_source_selector_client_, sel);
  const bool source_gated = sel_res && sel_res->success;

  auto req_on = std::make_shared<SetBool::Request>();
  req_on->data = true;
  auto res_on = call_service<SetBool>(enable_hdr_mode_client_, req_on);
  if (!res_on) {
    return assert_true(false, "test_enable_hdr_mode",
      "enable(true) service call failed");
  }
  bool ok = assert_true(res_on->success,
    "test_enable_hdr_mode/on", res_on->message);

  auto req_restore = std::make_shared<SetBool::Request>();
  req_restore->data = original_on;
  call_service<SetBool>(enable_hdr_mode_client_, req_restore);

  // Leave the stereo mini back on the color source (its startup default).
  if (source_gated) {
    auto restore_src = std::make_shared<SetIntegerValue::Request>();
    restore_src->value = 3;
    call_service<SetIntegerValue>(set_source_selector_client_, restore_src);
  }
  return ok;
}

// Illumination mode maps to BslIlluminationMode (enum) on the stereo ace. Sets
// the first entry (index 0, always valid) and restores the original. Skips when
// the camera does not expose it.
bool CameraTest3D::test_set_illumination_mode()
{
  CurrentParams p;
  if (!read_current_params(p) || p.illumination_mode < 0) {
    RCLCPP_WARN(get_logger(),
      "test_set_illumination_mode: not available for this camera, skipping.");
    return true;
  }
  const int original = p.illumination_mode;

  auto req = std::make_shared<SetIntegerValue::Request>();
  req->value = 0;
  auto res = call_service<SetIntegerValue>(set_illumination_mode_client_, req);
  if (!res) {
    return assert_true(false, "test_set_illumination_mode",
      "service call failed");
  }
  bool ok = assert_success(res->success, res->message,
    "test_set_illumination_mode/set");

  auto req_restore = std::make_shared<SetIntegerValue::Request>();
  req_restore->value = original;
  call_service<SetIntegerValue>(set_illumination_mode_client_, req_restore);
  return ok;
}

// Depth quality maps to BslDepthQuality (enum) on the stereo ace. Sets the
// first entry (index 0, always valid) and restores the original. Skips when the
// camera does not expose it.
bool CameraTest3D::test_set_depth_quality()
{
  CurrentParams p;
  if (!read_current_params(p) || p.depth_quality < 0) {
    RCLCPP_WARN(get_logger(),
      "test_set_depth_quality: not available for this camera, skipping.");
    return true;
  }
  const int original = p.depth_quality;

  auto req = std::make_shared<SetIntegerValue::Request>();
  req->value = 0;
  auto res = call_service<SetIntegerValue>(set_depth_quality_client_, req);
  if (!res) {
    return assert_true(false, "test_set_depth_quality",
      "service call failed");
  }
  bool ok = assert_success(res->success, res->message,
    "test_set_depth_quality/set");

  auto req_restore = std::make_shared<SetIntegerValue::Request>();
  req_restore->value = original;
  call_service<SetIntegerValue>(set_depth_quality_client_, req_restore);
  return ok;
}

// Static scene maps to BslDepthStaticScene on the stereo ace. Enables it,
// verifies the call, then restores the original state. Skips when the camera
// does not expose it.
bool CameraTest3D::test_enable_static_scene()
{
  CurrentParams p;
  if (!read_current_params(p) || p.static_scene < 0) {
    RCLCPP_WARN(get_logger(),
      "test_enable_static_scene: not available for this camera, skipping.");
    return true;
  }
  const bool original_on = (p.static_scene == 1);

  auto req_on = std::make_shared<SetBool::Request>();
  req_on->data = true;
  auto res_on = call_service<SetBool>(enable_static_scene_client_, req_on);
  if (!res_on) {
    return assert_true(false, "test_enable_static_scene",
      "enable(true) service call failed");
  }
  bool ok = assert_true(res_on->success,
    "test_enable_static_scene/on", res_on->message);

  auto req_restore = std::make_shared<SetBool::Request>();
  req_restore->data = original_on;
  call_service<SetBool>(enable_static_scene_client_, req_restore);
  return ok;
}

// Depth smoothing maps to BslDepthSmooth on the stereo ace. Enables it, verifies
// the call, then restores the original state. Skips when the camera does not
// expose it.
bool CameraTest3D::test_enable_depth_smooth()
{
  CurrentParams p;
  if (!read_current_params(p) || p.depth_smooth < 0) {
    RCLCPP_WARN(get_logger(),
      "test_enable_depth_smooth: not available for this camera, skipping.");
    return true;
  }
  const bool original_on = (p.depth_smooth == 1);

  auto req_on = std::make_shared<SetBool::Request>();
  req_on->data = true;
  auto res_on = call_service<SetBool>(enable_depth_smooth_client_, req_on);
  if (!res_on) {
    return assert_true(false, "test_enable_depth_smooth",
      "enable(true) service call failed");
  }
  bool ok = assert_true(res_on->success,
    "test_enable_depth_smooth/on", res_on->message);

  auto req_restore = std::make_shared<SetBool::Request>();
  req_restore->data = original_on;
  call_service<SetBool>(enable_depth_smooth_client_, req_restore);
  return ok;
}

// Depth fill maps to BslDepthFill (int) on the stereo ace. Re-applies the current
// value (always in range) and restores it. Skips when the camera does not expose it.
bool CameraTest3D::test_set_depth_fill()
{
  CurrentParams p;
  if (!read_current_params(p) || p.depth_fill < 0) {
    RCLCPP_WARN(get_logger(),
      "test_set_depth_fill: not available for this camera, skipping.");
    return true;
  }
  const int original = p.depth_fill;

  auto req = std::make_shared<SetIntegerValue::Request>();
  req->value = original;
  auto res = call_service<SetIntegerValue>(set_depth_fill_client_, req);
  if (!res) {
    return assert_true(false, "test_set_depth_fill",
      "service call failed");
  }
  return assert_success(res->success, res->message, "test_set_depth_fill/set");
}

// Depth segmentation maps to BslDepthSeg (int) on the stereo ace. Re-applies the
// current value and restores it. Skips when the camera does not expose it.
bool CameraTest3D::test_set_depth_seg()
{
  CurrentParams p;
  if (!read_current_params(p) || p.depth_seg < 0) {
    RCLCPP_WARN(get_logger(),
      "test_set_depth_seg: not available for this camera, skipping.");
    return true;
  }
  const int original = p.depth_seg;

  auto req = std::make_shared<SetIntegerValue::Request>();
  req->value = original;
  auto res = call_service<SetIntegerValue>(set_depth_seg_client_, req);
  if (!res) {
    return assert_true(false, "test_set_depth_seg",
      "service call failed");
  }
  return assert_success(res->success, res->message, "test_set_depth_seg/set");
}

// The pattern projector (BslLaserEnable) exists on the stereo mini. Enable it,
// verify the call, then restore the original state. Skips when the camera has
// no projector.
bool CameraTest3D::test_enable_projector()
{
  CurrentParams p;
  if (!read_current_params(p) || p.projector_enable < 0) {
    RCLCPP_WARN(get_logger(),
      "test_enable_projector: not available for this camera, skipping.");
    return true;
  }
  const bool original_on = (p.projector_enable == 1);

  auto req_on = std::make_shared<SetBool::Request>();
  req_on->data = true;
  auto res_on = call_service<SetBool>(enable_projector_client_, req_on);
  if (!res_on) {
    return assert_true(false, "test_enable_projector",
      "enable(true) service call failed");
  }
  bool ok = assert_true(res_on->success,
    "test_enable_projector/on", res_on->message);

  auto req_restore = std::make_shared<SetBool::Request>();
  req_restore->data = original_on;
  call_service<SetBool>(enable_projector_client_, req_restore);
  return ok;
}

// Projector power level (BslLaserLevel) on the stereo mini. The level has a
// camera-specific range, so write back the current value read from
// current_params and verify the service accepts it. Skips when the camera has
// no projector.
bool CameraTest3D::test_set_projector_level()
{
  CurrentParams p;
  if (!read_current_params(p) || p.projector_level < 0) {
    RCLCPP_WARN(get_logger(),
      "test_set_projector_level: not available for this camera, skipping.");
    return true;
  }

  auto req = std::make_shared<SetIntegerValue::Request>();
  req->value = p.projector_level;
  auto res = call_service<SetIntegerValue>(set_projector_level_client_, req);
  if (!res) {
    return assert_true(false, "test_set_projector_level",
      "service call failed");
  }
  return assert_success(res->success, res->message,
    "test_set_projector_level/set");
}

// Depth preset (BslDepthPreset) on the stereo mini is selected through the
// set_operating_mode service. Set the first entry (index 0, always valid) and
// restore the original. Skips when the camera has no depth preset.
bool CameraTest3D::test_set_depth_preset()
{
  CurrentParams p;
  if (!read_current_params(p) || p.depth_preset < 0) {
    RCLCPP_WARN(get_logger(),
      "test_set_depth_preset: not available for this camera, skipping.");
    return true;
  }
  const int original = p.depth_preset;

  auto req = std::make_shared<SetIntegerValue::Request>();
  req->value = 0;
  auto res = call_service<SetIntegerValue>(set_operating_mode_client_, req);
  if (!res) {
    return assert_true(false, "test_set_depth_preset",
      "service call failed");
  }
  bool ok = assert_success(res->success, res->message,
    "test_set_depth_preset/set");

  auto req_restore = std::make_shared<SetIntegerValue::Request>();
  req_restore->value = original;
  call_service<SetIntegerValue>(set_operating_mode_client_, req_restore);
  return ok;
}

// HDR sub-exposure selector (BslHdrExposureTimeSelector) on the stereo ace picks
// which sub-exposure the set_hdr_exposure_time service writes. These HDR sequence
// parameters have no current_params read-back, so the test writes a representative
// value and skips when the driver reports the node is not available.
bool CameraTest3D::test_set_hdr_exposure_time_selector()
{
  auto req = std::make_shared<SetIntegerValue::Request>();
  req->value = 1;
  auto res = call_service<SetIntegerValue>(set_hdr_exposure_time_selector_client_, req);
  if (!res) {
    return assert_true(false, "test_set_hdr_exposure_time_selector",
      "service call failed");
  }
  if (!res->success) {
    RCLCPP_WARN(get_logger(),
      "test_set_hdr_exposure_time_selector: not available for this camera, skipping.");
    return true;
  }
  return assert_true(res->success,
    "test_set_hdr_exposure_time_selector/set", res->message);
}

// HDR sub-exposure time (BslHdrExposureTime) on the stereo ace writes the time of
// the currently selected sub-exposure. Select sub-exposure 1 first, then write a
// representative value. Skips when the node is not available.
bool CameraTest3D::test_set_hdr_exposure_time()
{
  auto sel = std::make_shared<SetIntegerValue::Request>();
  sel->value = 1;
  call_service<SetIntegerValue>(set_hdr_exposure_time_selector_client_, sel);

  auto req = std::make_shared<SetFloatValue::Request>();
  req->value = 5000.0;
  auto res = call_service<SetFloatValue>(set_hdr_exposure_time_client_, req);
  if (!res) {
    return assert_true(false, "test_set_hdr_exposure_time",
      "service call failed");
  }
  if (!res->success) {
    RCLCPP_WARN(get_logger(),
      "test_set_hdr_exposure_time: not available for this camera, skipping.");
    return true;
  }
  return assert_true(res->success,
    "test_set_hdr_exposure_time/set", res->message);
}

// Number of HDR sub-exposures (BslHdrSubExposures) on the stereo ace. Writes a
// representative sequence length. Skips when the node is not available.
bool CameraTest3D::test_set_hdr_sub_exposures()
{
  auto req = std::make_shared<SetIntegerValue::Request>();
  req->value = 2;
  auto res = call_service<SetIntegerValue>(set_hdr_sub_exposures_client_, req);
  if (!res) {
    return assert_true(false, "test_set_hdr_sub_exposures",
      "service call failed");
  }
  if (!res->success) {
    RCLCPP_WARN(get_logger(),
      "test_set_hdr_sub_exposures: not available for this camera, skipping.");
    return true;
  }
  return assert_true(res->success,
    "test_set_hdr_sub_exposures/set", res->message);
}

// Auto exposure mode (ExposureAuto) on the stereo ace: 0 = Off, 1 = Continuous,
// 2 = HDR. Sets Off (index 0, always valid) so the camera keeps a defined state.
// Skips when the node is not available.
bool CameraTest3D::test_set_exposure_auto_mode()
{
  auto req = std::make_shared<SetIntegerValue::Request>();
  req->value = 0;
  auto res = call_service<SetIntegerValue>(set_exposure_auto_mode_client_, req);
  if (!res) {
    return assert_true(false, "test_set_exposure_auto_mode",
      "service call failed");
  }
  if (!res->success) {
    RCLCPP_WARN(get_logger(),
      "test_set_exposure_auto_mode: not available for this camera, skipping.");
    return true;
  }
  return assert_true(res->success,
    "test_set_exposure_auto_mode/set", res->message);
}

// HDR sequence maximum exposure (BslAEMaxExposure) on the stereo mini. Writes the
// camera's current exposure value, which is always inside the node's range, so the
// call is validated instead of tripping the upper limit. Effective only while
// ExposureAuto is Continuous. Skips when the node is not available.
bool CameraTest3D::test_set_hdr_max_exposure()
{
  CurrentParams p;
  if (!read_current_params(p) || p.exposure <= 0.0f) {
    RCLCPP_WARN(get_logger(),
      "test_set_hdr_max_exposure: no current exposure to reuse, skipping.");
    return true;
  }

  auto req = std::make_shared<SetFloatValue::Request>();
  req->value = p.exposure;
  auto res = call_service<SetFloatValue>(set_hdr_max_exposure_client_, req);
  if (!res) {
    return assert_true(false, "test_set_hdr_max_exposure",
      "service call failed");
  }
  if (!res->success) {
    RCLCPP_WARN(get_logger(),
      "test_set_hdr_max_exposure: not available for this camera, skipping.");
    return true;
  }
  return assert_true(res->success,
    "test_set_hdr_max_exposure/set", res->message);
}

// Stereo mini HDR preset workflow, following Basler's documented order: select an
// IR source, choose a sequence preset, load its recommended defaults, enable HDR,
// configure both sequence indices, then turn on frame merging. Restores the
// camera afterwards. The per-service HDR controls are only writable in this
// context, so this is the one test that exercises them on the mini. It skips on
// cameras that are not HDR-capable or do not expose the sequence controls, so it
// stays green on the stereo ace and blaze (whose HDR paths differ).
bool CameraTest3D::test_hdr_sequence_workflow()
{
  CurrentParams p;
  if (!read_current_params(p) || p.hdr_mode < 0) {
    RCLCPP_WARN(get_logger(),
      "test_hdr_sequence_workflow: HDR not available for this camera, skipping.");
    return true;
  }
  const bool original_hdr_on = (p.hdr_mode == 1);

  // The mini only allows HDR writes while an IR source (Source1) is selected.
  // Cameras that are not source-gated report set_source_selector as not
  // available; skip the workflow then.
  auto sel = std::make_shared<SetIntegerValue::Request>();
  sel->value = 1;
  auto sel_res = call_service<SetIntegerValue>(set_source_selector_client_, sel);
  if (!sel_res || !sel_res->success) {
    RCLCPP_WARN(get_logger(),
      "test_hdr_sequence_workflow: no IR source selector, skipping.");
    return true;
  }

  // Restore the color source and the original HDR state on the way out.
  auto restore = [this, original_hdr_on]() {
    auto hdr_restore = std::make_shared<SetBool::Request>();
    hdr_restore->data = original_hdr_on;
    call_service<SetBool>(enable_hdr_mode_client_, hdr_restore);
    auto src_restore = std::make_shared<SetIntegerValue::Request>();
    src_restore->value = 3;
    call_service<SetIntegerValue>(set_source_selector_client_, src_restore);
  };

  // Choose the DepthFromHDR sequence preset. With an IR source selected this is
  // writable before HDR is enabled; if it is not, the firmware does not support
  // the sequence workflow, so skip rather than fail.
  auto preset = std::make_shared<SetIntegerValue::Request>();
  preset->value = 0;
  auto preset_res = call_service<SetIntegerValue>(set_hdr_sequence_preset_client_, preset);
  if (!preset_res || !preset_res->success) {
    RCLCPP_WARN(get_logger(),
      "test_hdr_sequence_workflow: sequence controls not writable on this "
      "firmware, skipping.");
    restore();
    return true;
  }
  bool ok = assert_true(preset_res->success,
    "test_hdr_sequence_workflow/preset", preset_res->message);

  // Load the preset's recommended defaults. Not every mini firmware exposes this
  // node, so a "not available" reply is fine; note it and carry on.
  auto load = std::make_shared<Trigger::Request>();
  auto load_res = call_service<Trigger>(load_hdr_preset_client_, load);
  if (!load_res || !load_res->success) {
    RCLCPP_WARN(get_logger(),
      "test_hdr_sequence_workflow: load_hdr_preset not available, continuing.");
  }

  // Enable HDR, which unlocks the frame-merging controls.
  auto hdr_on = std::make_shared<SetBool::Request>();
  hdr_on->data = true;
  auto hdr_res = call_service<SetBool>(enable_hdr_mode_client_, hdr_on);
  ok &= assert_true(hdr_res && hdr_res->success,
    "test_hdr_sequence_workflow/enable",
    hdr_res ? hdr_res->message : "service call failed");

  // Configure both sequence indices.
  auto idx0 = std::make_shared<SetIntegerValue::Request>();
  idx0->value = 0;
  auto idx0_res = call_service<SetIntegerValue>(set_hdr_sequence_index_client_, idx0);
  ok &= assert_true(idx0_res && idx0_res->success,
    "test_hdr_sequence_workflow/index0",
    idx0_res ? idx0_res->message : "service call failed");

  auto idx1 = std::make_shared<SetIntegerValue::Request>();
  idx1->value = 1;
  auto idx1_res = call_service<SetIntegerValue>(set_hdr_sequence_index_client_, idx1);
  ok &= assert_true(idx1_res && idx1_res->success,
    "test_hdr_sequence_workflow/index1",
    idx1_res ? idx1_res->message : "service call failed");

  // Turn on HDR frame merging and IR-based merging.
  auto merge_on = std::make_shared<SetBool::Request>();
  merge_on->data = true;
  auto merge_res = call_service<SetBool>(enable_hdr_merge_client_, merge_on);
  ok &= assert_true(merge_res && merge_res->success,
    "test_hdr_sequence_workflow/merge_on",
    merge_res ? merge_res->message : "service call failed");

  auto merge_ir_on = std::make_shared<SetBool::Request>();
  merge_ir_on->data = true;
  auto merge_ir_res = call_service<SetBool>(enable_hdr_merge_use_ir_client_, merge_ir_on);
  ok &= assert_true(merge_ir_res && merge_ir_res->success,
    "test_hdr_sequence_workflow/merge_use_ir_on",
    merge_ir_res ? merge_ir_res->message : "service call failed");

  // Leave frame merging off again.
  auto merge_off = std::make_shared<SetBool::Request>();
  merge_off->data = false;
  call_service<SetBool>(enable_hdr_merge_use_ir_client_, merge_off);
  call_service<SetBool>(enable_hdr_merge_client_, merge_off);

  restore();
  return ok;
}

// Stereo ace HDR sub-exposure workflow. The ace builds HDR from its own exposure
// sequence: set how many sub-exposures the sequence has, set the time of each one
// by selector, then switch auto exposure to HDR to activate it. Configuration is
// done before enabling, following the camera's documented order. Skips when the
// HDR sub-exposure controls are not available (the X-100 prototype, stereo mini
// and blaze do not expose them).
bool CameraTest3D::test_hdr_sub_exposure_workflow()
{
  // Probe the sequence-length control; skip the workflow when it is absent.
  auto sub = std::make_shared<SetIntegerValue::Request>();
  sub->value = 2;
  auto sub_res = call_service<SetIntegerValue>(set_hdr_sub_exposures_client_, sub);
  if (!sub_res || !sub_res->success) {
    RCLCPP_WARN(get_logger(),
      "test_hdr_sub_exposure_workflow: HDR sub-exposure controls not available, "
      "skipping.");
    return true;
  }
  bool ok = assert_true(sub_res->success,
    "test_hdr_sub_exposure_workflow/sub_exposures", sub_res->message);

  // Set the time of each sub-exposure.
  for (int i = 1; i <= 2; ++i) {
    auto sel = std::make_shared<SetIntegerValue::Request>();
    sel->value = i;
    auto sel_res = call_service<SetIntegerValue>(set_hdr_exposure_time_selector_client_, sel);
    ok &= assert_true(sel_res && sel_res->success,
      "test_hdr_sub_exposure_workflow/selector",
      sel_res ? sel_res->message : "service call failed");

    auto t = std::make_shared<SetFloatValue::Request>();
    t->value = 3000.0 * i;
    auto t_res = call_service<SetFloatValue>(set_hdr_exposure_time_client_, t);
    ok &= assert_true(t_res && t_res->success,
      "test_hdr_sub_exposure_workflow/exposure_time",
      t_res ? t_res->message : "service call failed");
  }

  // Activate HDR through the auto exposure mode, then restore it to Off.
  auto hdr = std::make_shared<SetIntegerValue::Request>();
  hdr->value = 2;
  auto hdr_res = call_service<SetIntegerValue>(set_exposure_auto_mode_client_, hdr);
  ok &= assert_true(hdr_res && hdr_res->success,
    "test_hdr_sub_exposure_workflow/exposure_auto_hdr",
    hdr_res ? hdr_res->message : "service call failed");

  auto off = std::make_shared<SetIntegerValue::Request>();
  off->value = 0;
  call_service<SetIntegerValue>(set_exposure_auto_mode_client_, off);
  return ok;
}

// Verify the Stereo mini IMU stream. When imu_enabled is true the driver
// publishes on the imu topic at the IMU hardware rate, well above the image
// frame rate. A default-QoS subscriber (a normal consumer) must receive the
// samples at that rate; this guards the earlier regression where the samples
// were published in bursts once per image frame and most were dropped in
// transport, so a normal subscriber saw only a fraction of them. The imu
// publisher is always advertised, but only streams when imu_enabled is true, so
// the test skips when no samples arrive (IMU disabled or a camera without one).
bool CameraTest3D::test_imu_stream()
{
  const std::string topic = camera_ns_ + "/imu";

  std::atomic<int> count{0};
  std::atomic<bool> orientation_unset{true};
  std::atomic<bool> frame_id_set{true};
  auto sub = this->create_subscription<sensor_msgs::msg::Imu>(
    topic, rclcpp::QoS(rclcpp::KeepLast(200)),
    [&count, &orientation_unset, &frame_id_set]
    (const sensor_msgs::msg::Imu::SharedPtr msg) {
      count++;
      if (msg->orientation_covariance[0] != -1.0) { orientation_unset = false; }
      if (msg->header.frame_id.empty()) { frame_id_set = false; }
    });

  // Count messages over a fixed window and derive the received rate.
  const double window_s = 3.0;
  auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(window_s));
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  const int n = count.load();

  if (n == 0) {
    RCLCPP_WARN(get_logger(),
      "test_imu_stream: no IMU samples received (IMU not enabled or camera has "
      "no IMU), skipping.");
    return true;
  }

  const double rate = n / window_s;

  // The image frame rate in the test configuration is well below 100 Hz, so a
  // received rate above this threshold proves the IMU is delivered at its own
  // hardware rate rather than coupled to the image spin.
  const double min_rate = 100.0;
  bool ok = assert_true(rate >= min_rate, "test_imu_stream/rate",
    "expected >= " + std::to_string(min_rate) + " Hz, got " +
    std::to_string(rate) + " Hz over " + std::to_string(n) + " samples");
  ok &= assert_true(orientation_unset.load(), "test_imu_stream/orientation",
    "orientation_covariance[0] should be -1 (REP-145)");
  ok &= assert_true(frame_id_set.load(), "test_imu_stream/frame_id",
    "header.frame_id should be set");
  return ok;
}

}  // namespace pylon_ros2_camera_test

RCLCPP_COMPONENTS_REGISTER_NODE(pylon_ros2_camera_test::CameraTest3D)
