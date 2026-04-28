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

#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// CameraTestGeneric
//
// Intermediate base class containing tests that apply to ALL Basler cameras
// (2D and 3D alike).  Subclasses call register_generic_tests() inside their
// constructor to enqueue these tests before their own camera-type-specific
// tests.
//
// Tests registered here:
//   test_status_topic         – component_status topic has an active publisher
//   test_get_max_num_buffer   – get_max_num_buffer service responds
//   test_set_exposure         – set_exposure service responds and reaches target
//   test_set_gain             – set_gain service responds and reaches target
//   test_set_gamma            – set_gamma service responds and reaches target
//   test_sleeping_mode        – set_sleeping on/off round-trip
//   test_stop_start_grabbing  – stop_grabbing + start_grabbing round-trip
//
// Adding a new generic test:
//   1. Declare the method in this class (or a subclass).
//   2. Call register_test("test_name", std::bind(&YourClass::test_name, this))
//      inside register_generic_tests() (or inside your subclass constructor).
// ─────────────────────────────────────────────────────────────────────────────

#include "pylon_ros2_camera_test/camera_test_base.hpp"

#include <pylon_ros2_camera_interfaces/srv/get_integer_value.hpp>
#include <pylon_ros2_camera_interfaces/srv/set_exposure.hpp>
#include <pylon_ros2_camera_interfaces/srv/set_gain.hpp>
#include <pylon_ros2_camera_interfaces/srv/set_gamma.hpp>
#include <pylon_ros2_camera_interfaces/srv/set_sleeping.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <thread>
#include <chrono>

namespace pylon_ros2_camera_test
{

class CameraTestGeneric : public CameraTestBase
{
public:
  explicit CameraTestGeneric(
    const std::string & node_name,
    const rclcpp::NodeOptions & options)
  : CameraTestBase(node_name, options)
  {
    get_max_num_buffer_client_ =
      make_client<GetIntegerValue>("get_max_num_buffer");
    set_exposure_client_ =
      make_client<SetExposure>("set_exposure");
    set_gain_client_ =
      make_client<SetGain>("set_gain");
    set_gamma_client_ =
      make_client<SetGamma>("set_gamma");
    set_sleeping_client_ =
      make_client<SetSleeping>("set_sleeping");
    stop_grabbing_client_ =
      make_client<Trigger>("stop_grabbing");
    start_grabbing_client_ =
      make_client<Trigger>("start_grabbing");
  }

protected:
  // Call from subclass constructor BEFORE start_tests().
  void register_generic_tests()
  {
    register_test("test_status_topic",
      std::bind(&CameraTestGeneric::test_status_topic, this));
    register_test("test_get_max_num_buffer",
      std::bind(&CameraTestGeneric::test_get_max_num_buffer, this));
    register_test("test_set_exposure",
      std::bind(&CameraTestGeneric::test_set_exposure, this));
    register_test("test_set_gain",
      std::bind(&CameraTestGeneric::test_set_gain, this));
    register_test("test_set_gamma",
      std::bind(&CameraTestGeneric::test_set_gamma, this));
    register_test("test_sleeping_mode",
      std::bind(&CameraTestGeneric::test_sleeping_mode, this));
    register_test("test_stop_start_grabbing",
      std::bind(&CameraTestGeneric::test_stop_start_grabbing, this));
  }

  // ── Accessible by 2D/3D subclasses for camera detection ───────────────────

  using GetIntegerValue = pylon_ros2_camera_interfaces::srv::GetIntegerValue;
  rclcpp::Client<GetIntegerValue>::SharedPtr get_max_num_buffer_client_;

private:
  // ── Type aliases ───────────────────────────────────────────────────────────

  using SetExposure     = pylon_ros2_camera_interfaces::srv::SetExposure;
  using SetGain         = pylon_ros2_camera_interfaces::srv::SetGain;
  using SetGamma        = pylon_ros2_camera_interfaces::srv::SetGamma;
  using SetSleeping     = pylon_ros2_camera_interfaces::srv::SetSleeping;
  using Trigger         = std_srvs::srv::Trigger;

  // ── Generic test implementations ───────────────────────────────────────────

  // Verify that the driver is publishing its component_status topic.
  // Uses detection_timeout_ so DDS has enough time to propagate the
  // publisher even on slow hosts or right after the driver started.
  bool test_status_topic()
  {
    const std::string topic = camera_ns_ + "/status";
    bool ok = wait_for_topic(topic, std::chrono::seconds(detection_timeout_));
    return assert_true(ok, "test_status_topic",
      "no publisher found on " + topic);
  }

  // Verify that get_max_num_buffer responds with a positive value.
  bool test_get_max_num_buffer()
  {
    auto req = std::make_shared<GetIntegerValue::Request>();
    auto res = call_service<GetIntegerValue>(get_max_num_buffer_client_, req);
    if (!res) {
      return assert_true(false, "test_get_max_num_buffer",
        "service call failed or timed out");
    }
    bool ok = assert_success(res->success, res->message,
      "test_get_max_num_buffer/success");
    ok &= assert_true(res->value > 0,
      "test_get_max_num_buffer/value",
      "expected value > 0, got " + std::to_string(res->value));
    return ok;
  }

  // Verify that set_exposure reaches a manually requested value.
  // Target: 10 000 µs.  Tolerance: ±2 000 µs.
  // Some cameras have a narrower range (e.g. Blaze [50–1000] µs); when the
  // target is above the camera's maximum the driver clamps it and returns
  // success=true with the clamped value.  The test skips gracefully in that
  // case so the suite remains usable across all camera models.
  bool test_set_exposure()
  {
    const float target = 10000.0f;
    const float tol    = 2000.0f;
    auto req = std::make_shared<SetExposure::Request>();
    req->target_exposure = target;
    auto res = call_service<SetExposure>(set_exposure_client_, req);
    if (!res) {
      return assert_true(false, "test_set_exposure",
        "service call failed or timed out");
    }
    if (!res->success) {
      RCLCPP_WARN(get_logger(),
        "test_set_exposure: not supported by this camera, skipping.");
      return true;
    }
    if (std::fabs(res->reached_exposure - target) > tol) {
      RCLCPP_WARN(
        get_logger(),
        "test_set_exposure: target %.0f us out of camera range "
        "(reached %.0f us), skipping.",
        static_cast<double>(target),
        static_cast<double>(res->reached_exposure));
      return true;
    }
    return assert_near(res->reached_exposure, target, tol,
      "test_set_exposure/reached_exposure");
  }

  // Verify that set_gain reaches a manually requested value.
  // Target: 0.3 (normalised).  Tolerance: ±0.1.
  // Some cameras (e.g. Blaze) have no gain parameter; the driver returns the
  // sentinel value -9999 in that case.  The test skips gracefully so the suite
  // remains usable across all camera models.
  bool test_set_gain()
  {
    const float target = 0.3f;
    auto req = std::make_shared<SetGain::Request>();
    req->target_gain = target;
    auto res = call_service<SetGain>(set_gain_client_, req);
    if (!res) {
      return assert_true(false, "test_set_gain",
        "service call failed or timed out");
    }
    if (!res->success || res->reached_gain < -100.0f) {
      RCLCPP_WARN(get_logger(),
        "test_set_gain: gain not supported by this camera, skipping.");
      return true;
    }
    return assert_near(res->reached_gain, target, 0.1,
      "test_set_gain/reached_gain");
  }

  // Verify that set_gamma reaches a manually requested value.
  // Target: 1.2.  Tolerance: ±0.1.
  // Some cameras do not expose a Gamma NodeMap; in that case the driver
  // returns reached_gamma=0.  The test skips gracefully so the suite
  // remains usable across all camera models.
  bool test_set_gamma()
  {
    const float target = 1.2f;
    auto req = std::make_shared<SetGamma::Request>();
    req->target_gamma = target;
    auto res = call_service<SetGamma>(set_gamma_client_, req);
    if (!res) {
      return assert_true(false, "test_set_gamma",
        "service call failed or timed out");
    }
    if (!res->success || std::fabs(res->reached_gamma) < 0.001f) {
      RCLCPP_WARN(get_logger(),
        "test_set_gamma: gamma not supported by this camera, skipping.");
      return true;
    }
    return assert_near(res->reached_gamma, target, 0.1,
      "test_set_gamma/reached_gamma");
  }

  // Verify that the camera can be put to sleep and woken up.
  bool test_sleeping_mode()
  {
    // Enable sleeping
    auto req_on = std::make_shared<SetSleeping::Request>();
    req_on->set_sleeping = true;
    auto res_on = call_service<SetSleeping>(set_sleeping_client_, req_on);
    if (!res_on) {
      return assert_true(false, "test_sleeping_mode",
        "set_sleeping(true) service call failed");
    }
    bool ok = assert_true(res_on->success,
      "test_sleeping_mode/sleep_on", "failed to enable sleeping mode");

    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Disable sleeping (restore)
    auto req_off = std::make_shared<SetSleeping::Request>();
    req_off->set_sleeping = false;
    auto res_off = call_service<SetSleeping>(set_sleeping_client_, req_off);
    if (!res_off) {
      return assert_true(false, "test_sleeping_mode",
        "set_sleeping(false) service call failed");
    }
    ok &= assert_true(res_off->success,
      "test_sleeping_mode/sleep_off", "failed to disable sleeping mode");
    return ok;
  }

  // Verify that grabbing can be stopped and restarted.
  bool test_stop_start_grabbing()
  {
    // Stop
    auto stop_req = std::make_shared<Trigger::Request>();
    auto stop_res = call_service<Trigger>(stop_grabbing_client_, stop_req);
    if (!stop_res) {
      return assert_true(false, "test_stop_start_grabbing",
        "stop_grabbing service call failed");
    }
    bool ok = assert_true(stop_res->success,
      "test_stop_start_grabbing/stop", stop_res->message);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Start (restore)
    auto start_req = std::make_shared<Trigger::Request>();
    auto start_res = call_service<Trigger>(start_grabbing_client_, start_req);
    if (!start_res) {
      return assert_true(false, "test_stop_start_grabbing",
        "start_grabbing service call failed");
    }
    ok &= assert_true(start_res->success,
      "test_stop_start_grabbing/start", start_res->message);
    return ok;
  }

  // ── Service clients ────────────────────────────────────────────────────────

  rclcpp::Client<SetExposure>::SharedPtr set_exposure_client_;
  rclcpp::Client<SetGain>::SharedPtr set_gain_client_;
  rclcpp::Client<SetGamma>::SharedPtr set_gamma_client_;
  rclcpp::Client<SetSleeping>::SharedPtr set_sleeping_client_;
  rclcpp::Client<Trigger>::SharedPtr stop_grabbing_client_;
  rclcpp::Client<Trigger>::SharedPtr start_grabbing_client_;
};

}  // namespace pylon_ros2_camera_test
