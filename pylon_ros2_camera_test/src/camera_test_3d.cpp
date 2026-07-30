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
  // The current driver action for 3D data is named grab_blaze_data.
  grab_3d_client_ = rclcpp_action::create_client<GrabBlazeDataAction>(
    this, camera_ns_ + "/grab_blaze_data");

  set_depth_min_client_ =
    make_client<SetIntegerValue>("set_depth_min");
  set_depth_max_client_ =
    make_client<SetIntegerValue>("set_depth_max");
  enable_spatial_filter_client_ =
    make_client<SetBool>("enable_spatial_filter");
  enable_temporal_filter_client_ =
    make_client<SetBool>("enable_temporal_filter");

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

  // Step 2: A 3D camera streams point-cloud data on blaze_cloud; a 2D camera
  // never does. Data flow is the reliable 3D signal (more robust than the
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

// Send a grab_blaze_data goal with exposure_given=true and verify that point
// clouds and intensity maps are returned.
bool CameraTest3D::test_grab_3d_data()
{
  auto goal = GrabBlazeDataAction::Goal();
  goal.exposure_given = true;
  goal.exposure_times.push_back(500.0f);

  auto result_promise =
    std::make_shared<std::promise<GrabBlazeDataGoalHdl::WrappedResult>>();
  auto result_future = result_promise->get_future();

  auto send_goal_options =
    rclcpp_action::Client<GrabBlazeDataAction>::SendGoalOptions();

  send_goal_options.result_callback =
    [result_promise](const GrabBlazeDataGoalHdl::WrappedResult & result) {
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

// Set depth_min=200 mm and depth_max=2000 mm, verify both calls succeed,
// then restore to default range (0 – 7500 mm).
bool CameraTest3D::test_set_depth_range()
{
  // Set depth_min
  auto req_min = std::make_shared<SetIntegerValue::Request>();
  req_min->value = 200;
  auto res_min = call_service<SetIntegerValue>(set_depth_min_client_, req_min);
  if (!res_min) {
    return assert_true(false, "test_set_depth_range",
      "set_depth_min service call failed");
  }
  bool ok = assert_success(res_min->success, res_min->message,
    "test_set_depth_range/depth_min");

  // Set depth_max
  auto req_max = std::make_shared<SetIntegerValue::Request>();
  req_max->value = 2000;
  auto res_max = call_service<SetIntegerValue>(set_depth_max_client_, req_max);
  if (!res_max) {
    return assert_true(false, "test_set_depth_range",
      "set_depth_max service call failed");
  }
  ok &= assert_success(res_max->success, res_max->message,
    "test_set_depth_range/depth_max");

  // Restore defaults
  auto req_min_restore = std::make_shared<SetIntegerValue::Request>();
  req_min_restore->value = 0;
  call_service<SetIntegerValue>(set_depth_min_client_, req_min_restore);

  auto req_max_restore = std::make_shared<SetIntegerValue::Request>();
  req_max_restore->value = 7500;
  call_service<SetIntegerValue>(set_depth_max_client_, req_max_restore);

  return ok;
}

// Enable then disable the spatial filter; verify both service calls succeed.
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

}  // namespace pylon_ros2_camera_test

RCLCPP_COMPONENTS_REGISTER_NODE(pylon_ros2_camera_test::CameraTest3D)
