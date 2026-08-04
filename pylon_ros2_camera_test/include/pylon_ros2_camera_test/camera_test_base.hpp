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
// CameraTestBase
//
// Base class for all camera integration tests.  Subclasses register test
// functions via register_test(), declare camera-type detection logic in
// detect_camera(), then call start_tests() at the end of their constructor.
//
// At runtime the test thread:
//   1. Calls detect_camera().  Returns false → logs "not detected" and exits.
//   2. Runs the registered tests sequentially.
//   3. Prints a PASS/FAIL summary and shuts down the ROS context.
//
// Helper facilities:
//   assert_true() / assert_near() / assert_success()   – assertion helpers
//   call_service<SrvT>()                               – blocking service call
//   wait_for_topic()                                   – publisher presence check
//   make_client<SrvT>()                                – convenience client factory
//
// Parameters (declared at construction, overridable from a launch file):
//   camera_id              (default "my_camera")
//   camera_node_name       (default "pylon_ros2_camera_node")
//   camera_detection_timeout  seconds (default 10)
// ─────────────────────────────────────────────────────────────────────────────

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <future>
#include <string>
#include <thread>
#include <vector>

namespace pylon_ros2_camera_test
{

// ---------------------------------------------------------------------------
// Internal: one entry in the test queue
// ---------------------------------------------------------------------------
struct TestEntry
{
  std::string name;
  std::function<bool()> fn;
};

// ---------------------------------------------------------------------------
// CameraTestBase
// ---------------------------------------------------------------------------
class CameraTestBase : public rclcpp::Node
{
public:
  explicit CameraTestBase(
    const std::string & node_name,
    const rclcpp::NodeOptions & options)
  : rclcpp::Node(node_name, options)
  {
    camera_id_ = this->declare_parameter<std::string>("camera_id", "my_camera");
    camera_node_name_ =
      this->declare_parameter<std::string>("camera_node_name", "pylon_ros2_camera_node");
    detection_timeout_ =
      this->declare_parameter<int>("camera_detection_timeout", 10);
    device_user_id_ =
      this->declare_parameter<std::string>("device_user_id", "");
    fail_on_no_camera_ =
      this->declare_parameter<bool>("fail_on_no_camera", false);

    camera_ns_ = "/" + camera_id_ + "/" + camera_node_name_;
  }

  virtual ~CameraTestBase() = default;

protected:
  // ── To be implemented by leaf classes ────────────────────────────────────

  // Return true when the target camera is available, false on timeout.
  // Called from the test thread (NOT from the constructor / executor thread).
  virtual bool detect_camera() = 0;

  // ── Test registration ─────────────────────────────────────────────────────

  void register_test(const std::string & name, std::function<bool()> fn)
  {
    test_queue_.push_back({name, fn});
  }

  // Spawn the background test thread.
  // Must be called at the END of the leaf-class constructor, after all
  // register_test() calls.
  void start_tests()
  {
    test_thread_ = std::thread([this]() { run_tests(); });
    test_thread_.detach();  // node lifecycle managed by shared_ptr / executor
  }

  // ── Assertion helpers ─────────────────────────────────────────────────────

  // Logs a warning on failure and returns the condition value.
  bool assert_true(
    bool condition,
    const std::string & description,
    const std::string & detail = "")
  {
    if (!condition) {
      std::string msg = "  -> FAIL: " + description;
      if (!detail.empty()) {
        msg += " (" + detail + ")";
      }
      RCLCPP_WARN_STREAM(get_logger(), msg);
    }
    return condition;
  }

  bool assert_near(
    double actual,
    double expected,
    double tol,
    const std::string & description)
  {
    bool ok = std::fabs(actual - expected) <= tol;
    return assert_true(
      ok, description,
      "expected " + std::to_string(expected) +
      " ± " + std::to_string(tol) +
      ", got " + std::to_string(actual));
  }

  bool assert_success(
    bool success,
    const std::string & srv_message,
    const std::string & description)
  {
    return assert_true(success, description,
      "driver reported: " + (srv_message.empty() ? "(no message)" : srv_message));
  }

  // ── ROS utilities ─────────────────────────────────────────────────────────

  // Block until a publisher appears on full_topic_name, or timeout elapses.
  bool wait_for_topic(
    const std::string & full_topic_name,
    std::chrono::seconds timeout)
  {
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while (rclcpp::ok()) {
      if (count_publishers(full_topic_name) > 0) {
        return true;
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    return false;
  }

  // Poll an action server in 500 ms increments so that a SIGINT received while
  // waiting causes a clean return (false) instead of an exception / crash.
  // Returns true if the server became available before the deadline.
  template<typename ActionT>
  bool wait_for_action_server(
    typename rclcpp_action::Client<ActionT>::SharedPtr & client,
    std::chrono::nanoseconds timeout)
  {
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while (rclcpp::ok()) {
      auto remaining = deadline - std::chrono::steady_clock::now();
      if (remaining <= std::chrono::nanoseconds(0)) {
        return false;
      }
      auto poll = std::min(remaining, std::chrono::nanoseconds(
        std::chrono::milliseconds(500)));
      if (client->wait_for_action_server(poll)) {
        return true;
      }
    }
    return false;  // rclcpp::ok() became false (SIGINT / shutdown)
  }

  // Create a service client whose name is resolved as:
  //   camera_ns_ + "/" + srv_name
  template<typename SrvT>
  typename rclcpp::Client<SrvT>::SharedPtr make_client(const std::string & srv_name)
  {
    return this->create_client<SrvT>(camera_ns_ + "/" + srv_name);
  }

  // Blocking service call: waits for the service to be available, sends the
  // request, and waits for the response.  Returns nullptr on timeout, error,
  // or if the ROS context is shut down while waiting.
  template<typename SrvT>
  std::shared_ptr<typename SrvT::Response> call_service(
    typename rclcpp::Client<SrvT>::SharedPtr & client,
    std::shared_ptr<typename SrvT::Request> request,
    std::chrono::seconds availability_timeout = std::chrono::seconds(10),
    std::chrono::seconds response_timeout = std::chrono::seconds(15),
    bool log_errors = true)
  {
    // Poll wait_for_service in short increments so SIGINT can interrupt it.
    {
      auto deadline = std::chrono::steady_clock::now() + availability_timeout;
      bool available = false;
      while (rclcpp::ok()) {
        auto remaining = deadline - std::chrono::steady_clock::now();
        if (remaining <= std::chrono::nanoseconds(0)) break;
        auto poll = std::min(remaining,
          std::chrono::nanoseconds(std::chrono::milliseconds(200)));
        if (client->wait_for_service(poll)) { available = true; break; }
      }
      if (!available) {
        if (rclcpp::ok() && log_errors) {
          RCLCPP_ERROR_STREAM(get_logger(),
            "Service not available: " << client->get_service_name());
        }
        return nullptr;
      }
    }

    auto future = client->async_send_request(request);

    // Poll future in short increments so SIGINT can interrupt it cleanly.
    try {
      auto deadline = std::chrono::steady_clock::now() + response_timeout;
      while (rclcpp::ok()) {
        auto remaining = deadline - std::chrono::steady_clock::now();
        if (remaining <= std::chrono::nanoseconds(0)) {
          if (log_errors) {
            RCLCPP_ERROR_STREAM(get_logger(),
              "Service call timed out: " << client->get_service_name());
          }
          return nullptr;
        }
        auto poll = std::min(remaining,
          std::chrono::nanoseconds(std::chrono::milliseconds(200)));
        if (future.wait_for(poll) == std::future_status::ready) {
          return future.get();
        }
      }
    } catch (const std::exception &) {
      // Promise destroyed during shutdown — treat as no response.
    }
    return nullptr;
  }

  // ── Shared state accessible by subclasses ─────────────────────────────────

  std::string camera_ns_;       // e.g. "/my_camera/pylon_ros2_camera_node"
  int detection_timeout_;       // seconds

  // Set this to true inside detect_camera() when the camera is reachable but
  // is the wrong type for this test node (e.g. 3D camera → 2D node).  This
  // suppresses the "not reachable" fatal error so the node exits silently.
  bool is_wrong_camera_type_{false};

private:
  // ── Core test runner (executed in test_thread_) ───────────────────────────

  void run_tests()
  {
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Waiting for camera (timeout: " << detection_timeout_ << " s)...");

    if (!detect_camera()) {
      // If rclcpp is no longer OK, we were shut down externally (e.g. SIGINT
      // from another test node exiting).  Exit silently without printing an
      // error — the other node already reported the real cause.
      if (!rclcpp::ok()) {
        _Exit(EXIT_SUCCESS);
      }
      if (!device_user_id_.empty() && fail_on_no_camera_ && !is_wrong_camera_type_) {
        RCLCPP_ERROR_STREAM(
          get_logger(),
          "Camera with device_user_id '" << device_user_id_ << "' was not reachable "
          "within the detection timeout (" << detection_timeout_ << " s). "
          "Please check that:\n"
          "  1. The device_user_id is correct (specified: '" << device_user_id_ << "')\n"
          "  2. The camera is powered on and connected to the network/USB.");
        // _Exit avoids running C++ destructors from a detached thread,
        // which would otherwise cause SIGABRT due to concurrent cleanup.
        _Exit(EXIT_FAILURE);  // non-zero -> launch file triggers immediate stop
      } else {
        RCLCPP_INFO(get_logger(),
          "Camera not detected within timeout. Skipping all tests.");
        rclcpp::shutdown();
      }
      return;
    }

    RCLCPP_INFO_STREAM(
      get_logger(),
      "\n========================================="
      "\n  Starting test run (" << test_queue_.size() << " tests)"
      "\n=========================================");

    for (auto & entry : test_queue_) {
      RCLCPP_INFO_STREAM(get_logger(), "[ RUN  ] " << entry.name);
      bool ok = false;
      try {
        ok = entry.fn();
      } catch (const std::exception & e) {
        RCLCPP_ERROR_STREAM(get_logger(), "  Exception: " << e.what());
      }
      if (ok) {
        ++pass_count_;
        RCLCPP_INFO_STREAM(get_logger(), "[ PASS ] " << entry.name);
      } else {
        ++fail_count_;
        RCLCPP_WARN_STREAM(get_logger(), "[ FAIL ] " << entry.name);
      }
    }

    report_and_shutdown();
  }

  void report_and_shutdown()
  {
    int total = pass_count_ + fail_count_;
    RCLCPP_INFO_STREAM(
      get_logger(),
      "\n========================================="
      "\n  RESULTS: " << pass_count_ << " / " << total << " passed"
      "  (" << fail_count_ << " failed)"
      "\n=========================================");
    if (fail_count_ == 0 && total > 0) {
      RCLCPP_INFO(get_logger(), "  All tests PASSED.");
    }
    // rclcpp::shutdown() is the last operation – do not access 'this' after.
    rclcpp::shutdown();
  }

  // ── Members ───────────────────────────────────────────────────────────────

  std::string camera_id_;
  std::string camera_node_name_;
  std::string device_user_id_;
  bool fail_on_no_camera_;
  std::vector<TestEntry> test_queue_;
  std::thread test_thread_;
  int pass_count_{0};
  int fail_count_{0};
};

}  // namespace pylon_ros2_camera_test
