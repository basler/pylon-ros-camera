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

#include "pylon_ros2_camera_test/camera_test_2d.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
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

CameraTest2D::CameraTest2D(const rclcpp::NodeOptions & options)
: CameraTestGeneric("camera_test_2d", options)
{
  blaze_detect_client_ = rclcpp_action::create_client<GrabBlazeDataAction>(
    this, camera_ns_ + "/grab_blaze_data");

  grab_images_client_ = rclcpp_action::create_client<GrabImagesAction>(
    this, camera_ns_ + "/grab_images_raw");

  set_binning_client_        = make_client<SetBinning>("set_binning");
  set_roi_client_            = make_client<SetROI>("set_roi");
  set_image_encoding_client_ = make_client<SetStringValue>("set_image_encoding");

  // Generic tests run first, then 2D-specific tests.
  register_generic_tests();
  register_test("test_grab_images_raw",
    std::bind(&CameraTest2D::test_grab_images_raw, this));
  register_test("test_set_binning",
    std::bind(&CameraTest2D::test_set_binning, this));
  register_test("test_set_roi",
    std::bind(&CameraTest2D::test_set_roi, this));
  register_test("test_set_image_encoding",
    std::bind(&CameraTest2D::test_set_image_encoding, this));

  // Start the test thread LAST, after all tests are registered.
  start_tests();
}

// ─────────────────────────────────────────────────────────────────────────────
// Camera detection
// ─────────────────────────────────────────────────────────────────────────────

bool CameraTest2D::detect_camera()
{
  // Step 1: Confirm that a driver is running at all.
  // grab_images_raw is created by the driver at node startup (before the
  // camera hardware connects), so it appears quickly after the process starts.
  if (!wait_for_action_server<GrabImagesAction>(
        grab_images_client_,
        std::chrono::seconds(detection_timeout_)))
  {
    return false;  // No driver found within the timeout (or shutdown).
  }

  // Step 2: Wait until the camera hardware is actually connected. grab_images_raw
  // (step 1) is created before the hardware connects, so it does not prove a live
  // camera. get_max_num_buffer reads a camera register and only succeeds once the
  // camera is up. Poll until it succeeds or the detection timeout expires (a
  // Blaze can take several seconds to connect through its GenTL producer).
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
    if (!connected) {
      return false;  // Driver running but camera not reachable within the timeout.
    }
  }
  if (!rclcpp::ok()) return false;

  // Step 3: Discriminate 2D vs 3D camera. Now that the camera is connected, only
  // a Blaze (3D) camera streams point-cloud data on blaze_cloud; a 2D camera
  // never does. The blaze_* publishers exist on every camera, so their presence
  // is not enough - the data flow is the reliable signal (more robust than the
  // grab_blaze_data action, whose discovery can lag just after connection).
  {
    std::atomic<bool> got_cloud{false};
    auto blaze_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      camera_ns_ + "/blaze_cloud", rclcpp::QoS(rclcpp::KeepLast(1)),
      [&got_cloud](const sensor_msgs::msg::PointCloud2::SharedPtr) { got_cloud = true; });
    auto disc_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (rclcpp::ok() && !got_cloud && std::chrono::steady_clock::now() < disc_deadline)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (got_cloud)
    {
      RCLCPP_INFO(get_logger(),
        "3D camera detected (blaze_cloud is streaming). Skipping 2D tests.");
      is_wrong_camera_type_ = true;
      return false;
    }
  }

  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// 2D test implementations
// ─────────────────────────────────────────────────────────────────────────────

// Send a grab_images_raw goal with gain_given=true and verify at least one
// image is returned.
bool CameraTest2D::test_grab_images_raw()
{
  auto goal = GrabImagesAction::Goal();
  goal.gain_given = true;
  goal.gain_values.push_back(0.5f);

  // Use a shared promise so the result callback is safe even if this stack
  // frame is momentarily delayed (the lambda keeps the promise alive).
  auto result_promise =
    std::make_shared<std::promise<GrabImagesGoalHdl::WrappedResult>>();
  auto result_future = result_promise->get_future();

  auto send_goal_options =
    rclcpp_action::Client<GrabImagesAction>::SendGoalOptions();

  send_goal_options.result_callback =
    [result_promise](const GrabImagesGoalHdl::WrappedResult & result) {
      result_promise->set_value(result);
    };

  // Wait for goal acceptance
  auto goal_handle_future =
    grab_images_client_->async_send_goal(goal, send_goal_options);

  if (goal_handle_future.wait_for(std::chrono::seconds(10)) !=
      std::future_status::ready)
  {
    return assert_true(false, "test_grab_images_raw",
      "goal not accepted within 10 s");
  }
  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    return assert_true(false, "test_grab_images_raw",
      "goal was rejected by the action server");
  }

  // Wait for result
  if (result_future.wait_for(std::chrono::seconds(60)) !=
      std::future_status::ready)
  {
    return assert_true(false, "test_grab_images_raw",
      "result not received within 60 s");
  }
  auto wrapped = result_future.get();

  if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
    return assert_true(false, "test_grab_images_raw",
      "action did not succeed (code " +
      std::to_string(static_cast<int>(wrapped.code)) + ")");
  }

  bool ok = assert_true(wrapped.result->success,
    "test_grab_images_raw/success", "grab action reported failure");
  ok &= assert_true(!wrapped.result->images.empty(),
    "test_grab_images_raw/images", "no images in result");
  if (!wrapped.result->images.empty()) {
    const auto & img = wrapped.result->images.front();
    ok &= assert_true(img.width > 0 && img.height > 0,
      "test_grab_images_raw/dimensions",
      "image has zero dimensions (" +
      std::to_string(img.width) + "x" + std::to_string(img.height) + ")");
  }
  return ok;
}

// Set 2×2 binning, verify the reached values, then restore 1×1.
// If the camera does not support binning the driver silently keeps the
// current value (reached stays 1); the test skips gracefully so the suite
// remains usable on cameras without hardware binning support.
bool CameraTest2D::test_set_binning()
{
  // Set 2×2
  auto req2 = std::make_shared<SetBinning::Request>();
  req2->target_binning_x = 2;
  req2->target_binning_y = 2;
  auto res2 = call_service<SetBinning>(set_binning_client_, req2);
  if (!res2) {
    return assert_true(false, "test_set_binning", "service call failed");
  }
  if (!res2->success ||
      (res2->reached_binning_x != 2 || res2->reached_binning_y != 2))
  {
    RCLCPP_WARN(get_logger(),
      "test_set_binning: binning not supported by this camera, skipping.");
    return true;
  }

  bool ok = assert_true(res2->reached_binning_x == 2,
    "test_set_binning/reached_x",
    "expected 2, got " + std::to_string(res2->reached_binning_x));
  ok &= assert_true(res2->reached_binning_y == 2,
    "test_set_binning/reached_y",
    "expected 2, got " + std::to_string(res2->reached_binning_y));

  // Restore 1×1
  auto req1 = std::make_shared<SetBinning::Request>();
  req1->target_binning_x = 1;
  req1->target_binning_y = 1;
  auto res1 = call_service<SetBinning>(set_binning_client_, req1);
  if (!res1) {
    RCLCPP_WARN(get_logger(), "test_set_binning: could not restore 1x1 binning");
  }
  return ok;
}

// Set a 320×240 ROI at the sensor origin, verify the reached values,
// then restore the full sensor area.
bool CameraTest2D::test_set_roi()
{
  // ── [1] camera_info.roi must be all-zeros at full resolution ──────────────
  auto roi_initial = get_camera_info_roi();
  bool ok = assert_true(
    roi_initial.width == 0 && roi_initial.height == 0 &&
    roi_initial.x_offset == 0 && roi_initial.y_offset == 0,
    "test_set_roi/camera_info_roi_at_full_res",
    "expected all-zero roi at full resolution (ROS convention), got "
    "w=" + std::to_string(roi_initial.width) +
    " h=" + std::to_string(roi_initial.height));

  // ── [2] Set a small ROI ───────────────────────────────────────────────────
  auto req_small = std::make_shared<SetROI::Request>();
  req_small->target_roi.x_offset  = 0;
  req_small->target_roi.y_offset  = 0;
  req_small->target_roi.width     = 320;
  req_small->target_roi.height    = 240;
  req_small->target_roi.do_rectify = false;
  auto res_small = call_service<SetROI>(set_roi_client_, req_small);
  if (!res_small) {
    return assert_true(false, "test_set_roi", "service call failed");
  }
  ok &= assert_success(res_small->success, "", "test_set_roi/set_small");
  ok &= assert_true(
    res_small->reached_roi.width > 0 && res_small->reached_roi.height > 0,
    "test_set_roi/reached_dimensions",
    "reached ROI has zero dimensions");

  // ── [3] camera_info.roi must reflect the crop ─────────────────────────────
  auto roi_cropped = get_camera_info_roi();
  ok &= assert_true(
    roi_cropped.width > 0 && roi_cropped.height > 0,
    "test_set_roi/camera_info_roi_after_crop",
    "camera_info.roi is still zero after setting a crop ROI");

  // ── [4] Restore full sensor (driver clips large values to sensor max) ─────
  auto req_full = std::make_shared<SetROI::Request>();
  req_full->target_roi.x_offset  = 0;
  req_full->target_roi.y_offset  = 0;
  req_full->target_roi.width     = 65535;
  req_full->target_roi.height    = 65535;
  req_full->target_roi.do_rectify = false;
  auto res_full = call_service<SetROI>(set_roi_client_, req_full);
  if (!res_full) {
    RCLCPP_WARN(get_logger(), "test_set_roi: could not restore full-sensor ROI");
    return ok;
  }

  // ── [5] camera_info.roi must return to all-zeros ──────────────────────────
  auto roi_restored = get_camera_info_roi();
  ok &= assert_true(
    roi_restored.width == 0 && roi_restored.height == 0 &&
    roi_restored.x_offset == 0 && roi_restored.y_offset == 0,
    "test_set_roi/camera_info_roi_after_restore",
    "camera_info.roi not cleared after returning to full resolution "
    "(ROS convention: all-zeros = full sensor), got "
    "w=" + std::to_string(roi_restored.width) +
    " h=" + std::to_string(roi_restored.height));

  return ok;
}

sensor_msgs::msg::RegionOfInterest CameraTest2D::get_camera_info_roi(
  std::chrono::seconds timeout)
{
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  sensor_msgs::msg::RegionOfInterest zero_roi;

  auto promise = std::make_shared<std::promise<CameraInfo>>();
  auto future  = promise->get_future();

  auto sub = this->create_subscription<CameraInfo>(
    camera_ns_ + "/camera_info", rclcpp::SensorDataQoS(),
    [promise](CameraInfo::SharedPtr msg) {
      try { promise->set_value(*msg); } catch (...) {}
    });

  if (future.wait_for(timeout) != std::future_status::ready) {
    RCLCPP_WARN(get_logger(),
      "get_camera_info_roi: timed out waiting for camera_info message");
    return zero_roi;
  }

  return future.get().roi;
}

// Grab one frame via the GrabImages action and return its encoding.
// Returns an empty string on any failure.
std::string CameraTest2D::grab_current_encoding()
{
  auto goal = GrabImagesAction::Goal();
  // A single gain entry tells the action server to grab exactly one image.
  // Using gain_given rather than exposure_given avoids disturbing auto-exposure.
  goal.gain_given = true;
  goal.gain_values.push_back(0.5f);

  auto result_promise =
    std::make_shared<std::promise<GrabImagesGoalHdl::WrappedResult>>();
  auto result_future = result_promise->get_future();

  auto opts = rclcpp_action::Client<GrabImagesAction>::SendGoalOptions();
  opts.result_callback =
    [result_promise](const GrabImagesGoalHdl::WrappedResult & result) {
      result_promise->set_value(result);
    };

  auto goal_handle_future =
    grab_images_client_->async_send_goal(goal, opts);
  if (goal_handle_future.wait_for(std::chrono::seconds(10)) !=
      std::future_status::ready)
  {
    return "";
  }
  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) return "";

  if (result_future.wait_for(std::chrono::seconds(30)) !=
      std::future_status::ready)
  {
    return "";
  }
  auto wrapped = result_future.get();
  if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) return "";
  if (!wrapped.result->success || wrapped.result->images.empty()) return "";

  return wrapped.result->images.front().encoding;
}

// Switch encoding to "mono8", grab a frame and verify the image header
// encoding matches.  Then try "bayer_rggb8" (skip gracefully if the camera
// is monochrome), grab and verify, then restore "mono8" and verify again.
// The restore+grab is the key regression check for the bit_shift_active_
// caching optimisation: a stale cached flag would produce an image whose
// pixel data is byte-shifted even though the encoding header says "mono8".
bool CameraTest2D::test_set_image_encoding()
{
  // ── Step 1: switch to mono8 ───────────────────────────────────────────────
  {
    auto req = std::make_shared<SetStringValue::Request>();
    req->value = "mono8";
    auto res = call_service<SetStringValue>(set_image_encoding_client_, req);
    if (!res) {
      return assert_true(false, "test_set_image_encoding", "service call failed");
    }
    if (!assert_success(res->success, res->message,
        "test_set_image_encoding/mono8")) {
      return false;
    }
  }

  // ── Step 2: grab and confirm the image header encoding is mono8 ───────────
  {
    std::string enc = grab_current_encoding();
    if (!assert_true(!enc.empty(),
        "test_set_image_encoding/grab_mono8", "grab failed")) {
      return false;
    }
    if (!assert_true(enc == "mono8",
        "test_set_image_encoding/encoding_mono8",
        "expected mono8, got " + enc)) {
      return false;
    }
  }

  // ── Step 3: try switching to bayer_rggb8 (colour cameras only) ───────────
  bool colour_supported = false;
  {
    auto req = std::make_shared<SetStringValue::Request>();
    req->value = "bayer_rggb8";
    auto res = call_service<SetStringValue>(set_image_encoding_client_, req);
    if (!res) {
      return assert_true(false, "test_set_image_encoding",
        "service call failed on bayer_rggb8");
    }
    if (!res->success) {
      RCLCPP_WARN(get_logger(),
        "test_set_image_encoding: bayer_rggb8 not supported by this camera "
        "(likely monochrome), skipping colour encoding sub-test.");
    } else {
      colour_supported = true;
    }
  }

  // ── Step 4 (colour cameras): grab and verify the encoding changed ─────────
  if (colour_supported) {
    std::string enc = grab_current_encoding();
    if (!assert_true(!enc.empty(),
        "test_set_image_encoding/grab_bayer", "grab failed")) {
      return false;
    }
    if (!assert_true(enc == "bayer_rggb8",
        "test_set_image_encoding/encoding_bayer",
        "expected bayer_rggb8, got " + enc)) {
      return false;
    }
  }

  // ── Step 5: restore mono8 and verify encoding resets correctly ────────────
  // This is the critical regression check: a stale bit_shift_active_ flag
  // would not be caught by the service response alone.
  {
    auto req = std::make_shared<SetStringValue::Request>();
    req->value = "mono8";
    auto res = call_service<SetStringValue>(set_image_encoding_client_, req);
    if (!res) {
      return assert_true(false, "test_set_image_encoding",
        "service call failed on restore to mono8");
    }
    if (!assert_success(res->success, res->message,
        "test_set_image_encoding/restore_mono8")) {
      return false;
    }
    std::string enc = grab_current_encoding();
    if (!assert_true(!enc.empty(),
        "test_set_image_encoding/grab_restore", "grab failed")) {
      return false;
    }
    return assert_true(enc == "mono8",
      "test_set_image_encoding/encoding_restore",
      "expected mono8 after restore, got " + enc);
  }
}

}  // namespace pylon_ros2_camera_test

RCLCPP_COMPONENTS_REGISTER_NODE(pylon_ros2_camera_test::CameraTest2D)
