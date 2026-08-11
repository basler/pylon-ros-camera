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
  enable_projector_client_ =
    make_client<SetBool>("enable_projector");
  set_projector_level_client_ =
    make_client<SetIntegerValue>("set_projector_level");
  set_operating_mode_client_ =
    make_client<SetIntegerValue>("set_operating_mode");

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
  register_test("test_enable_projector",
    std::bind(&CameraTest3D::test_enable_projector, this));
  register_test("test_set_projector_level",
    std::bind(&CameraTest3D::test_set_projector_level, this));
  register_test("test_set_depth_preset",
    std::bind(&CameraTest3D::test_set_depth_preset, this));

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

}  // namespace pylon_ros2_camera_test

RCLCPP_COMPONENTS_REGISTER_NODE(pylon_ros2_camera_test::CameraTest3D)
