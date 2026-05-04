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
// CameraTest2D
//
// Tests specific to 2D cameras (GigE, USB, Dart, …).
// Inherits and auto-registers all generic tests from CameraTestGeneric, then
// adds:
//
//   test_grab_images_raw      – GrabImages action, gain_given=true
//   test_set_binning          – set 2×2 binning, verify, restore 1×1
//   test_set_roi              – set a small ROI, verify reached dimensions and
//                               camera_info.roi field, restore full sensor
//   test_set_image_encoding   – switch mono8↔bayer_rggb8, grab and verify
//                               image header encoding after each switch
//                               (regression test for the bit_shift_active_ cache)
//
// Camera detection: waits for the grab_images_raw action server.
//
// Adding a new 2D test:
//   1. Declare the method in this class (or a subclass of CameraTest2D).
//   2. Add a register_test(...) call inside the constructor, before start_tests().
//   3. Implement the method in camera_test_2d.cpp (or in the subclass source).
//
// For a new 2D camera family with its own specific behaviour:
//   class CameraTestMyGigE : public CameraTest2D { … };
// ─────────────────────────────────────────────────────────────────────────────

#include "pylon_ros2_camera_test/camera_test_generic.hpp"

#include <pylon_ros2_camera_interfaces/action/grab_blaze_data.hpp>
#include <pylon_ros2_camera_interfaces/action/grab_images.hpp>
#include <pylon_ros2_camera_interfaces/srv/set_binning.hpp>
#include <pylon_ros2_camera_interfaces/srv/set_roi.hpp>
#include <pylon_ros2_camera_interfaces/srv/set_string_value.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>

namespace pylon_ros2_camera_test
{

class CameraTest2D : public CameraTestGeneric
{
public:
  explicit CameraTest2D(const rclcpp::NodeOptions & options);

protected:
  bool detect_camera() override;

  // ── 2D-specific test declarations ─────────────────────────────────────────

  virtual bool test_grab_images_raw();
  virtual bool test_set_binning();
  virtual bool test_set_roi();
  virtual bool test_set_image_encoding();

  // ── Helpers ────────────────────────────────────────────────────────────────

  // Grab one frame via the GrabImages action and return its encoding string.
  // Returns an empty string on any failure.
  std::string grab_current_encoding();

  // Subscribe to camera_info and return the roi field from the next message.
  // Returns a default-constructed (all-zeros) ROI on timeout.
  sensor_msgs::msg::RegionOfInterest get_camera_info_roi(
    std::chrono::seconds timeout = std::chrono::seconds(5));

  // ── Type aliases ───────────────────────────────────────────────────────────

  using GrabBlazeDataAction = pylon_ros2_camera_interfaces::action::GrabBlazeData;
  using GrabImagesAction    = pylon_ros2_camera_interfaces::action::GrabImages;
  using GrabImagesGoalHdl   = rclcpp_action::ClientGoalHandle<GrabImagesAction>;
  using SetBinning          = pylon_ros2_camera_interfaces::srv::SetBinning;
  using SetROI              = pylon_ros2_camera_interfaces::srv::SetROI;
  using SetStringValue      = pylon_ros2_camera_interfaces::srv::SetStringValue;

  // ── Clients ────────────────────────────────────────────────────────────────

  rclcpp_action::Client<GrabBlazeDataAction>::SharedPtr blaze_detect_client_;
  rclcpp_action::Client<GrabImagesAction>::SharedPtr grab_images_client_;
  rclcpp::Client<SetBinning>::SharedPtr set_binning_client_;
  rclcpp::Client<SetROI>::SharedPtr set_roi_client_;
  rclcpp::Client<SetStringValue>::SharedPtr set_image_encoding_client_;
};

}  // namespace pylon_ros2_camera_test
