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
// CameraTest3D
//
// Tests specific to 3D cameras (currently Basler Blaze, extensible to future
// 3D models).
// Inherits and auto-registers all generic tests from CameraTestGeneric, then
// adds:
//
//   test_grab_3d_data          – Grab3DData action, exposure_given=true
//   test_set_depth_range       – set depth_min + depth_max, verify, restore
//   test_enable_spatial_filter – enable / disable round-trip
//   test_enable_temporal_filter – enable / disable round-trip
//   test_set_brightness        – set_brightness, skip if unsupported
//
// Camera detection: waits for the grab_3d_data action server.
// Note: "grab_3d_data" is the current driver action name; the test class
//       is intentionally named CameraTest3D to stay model-agnostic.
//
// Adding a new 3D test:
//   1. Declare the method in this class (or a subclass).
//   2. Add a register_test(...) call inside the constructor, before start_tests().
//   3. Implement the method in camera_test_3d.cpp (or in the subclass source).
//
// For a future 3D camera family with different capabilities:
//   class CameraTestMyNew3D : public CameraTest3D { … };
// ─────────────────────────────────────────────────────────────────────────────

#include "pylon_ros2_camera_test/camera_test_generic.hpp"

#include <pylon_ros2_camera_interfaces/action/grab3_d_data.hpp>
#include <pylon_ros2_camera_interfaces/srv/set_integer_value.hpp>
#include <pylon_ros2_camera_interfaces/srv/set_float_value.hpp>
#include <pylon_ros2_camera_interfaces/srv/set_brightness.hpp>
#include <pylon_ros2_camera_interfaces/msg/current_params.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace pylon_ros2_camera_test
{

class CameraTest3D : public CameraTestGeneric
{
public:
  explicit CameraTest3D(const rclcpp::NodeOptions & options);

protected:
  bool detect_camera() override;

  // ── Type aliases ───────────────────────────────────────────────────────────

  using Grab3DDataAction  = pylon_ros2_camera_interfaces::action::Grab3DData;
  using Grab3DDataGoalHdl =
    rclcpp_action::ClientGoalHandle<Grab3DDataAction>;
  using SetIntegerValue      = pylon_ros2_camera_interfaces::srv::SetIntegerValue;
  using SetFloatValue        = pylon_ros2_camera_interfaces::srv::SetFloatValue;
  using SetBrightness        = pylon_ros2_camera_interfaces::srv::SetBrightness;
  using SetBool              = std_srvs::srv::SetBool;
  using CurrentParams        = pylon_ros2_camera_interfaces::msg::CurrentParams;

  // ── 3D-specific test declarations ─────────────────────────────────────────

  virtual bool test_grab_3d_data();
  virtual bool test_set_depth_range();
  virtual bool test_enable_spatial_filter();
  virtual bool test_enable_temporal_filter();
  virtual bool test_set_brightness();
  virtual bool test_set_confidence_threshold();
  virtual bool test_enable_hdr_mode();
  virtual bool test_set_illumination_mode();
  virtual bool test_set_depth_quality();
  virtual bool test_enable_static_scene();
  virtual bool test_enable_depth_smooth();
  virtual bool test_set_depth_fill();
  virtual bool test_set_depth_seg();
  virtual bool test_enable_projector();
  virtual bool test_set_projector_level();
  virtual bool test_set_depth_preset();

  // Read one current_params message from the driver. Returns false if none
  // arrives within a few seconds. Used to detect which 3D features a camera
  // supports (unavailable features report -1) and to restore original values.
  bool read_current_params(CurrentParams & out);

  // Read the camera's current working depth range from the current_params
  // topic. The units are camera-native (mm for blaze/mini, meters for the
  // stereo ace). Returns false if no message arrives within a few seconds.
  bool read_current_depth_range(float & depth_min, float & depth_max);

  // ── Clients ────────────────────────────────────────────────────────────────

  rclcpp_action::Client<Grab3DDataAction>::SharedPtr grab_3d_client_;
  rclcpp::Client<SetFloatValue>::SharedPtr set_depth_min_client_;
  rclcpp::Client<SetFloatValue>::SharedPtr set_depth_max_client_;
  rclcpp::Client<SetBrightness>::SharedPtr set_brightness_client_;
  rclcpp::Client<SetBool>::SharedPtr enable_spatial_filter_client_;
  rclcpp::Client<SetBool>::SharedPtr enable_temporal_filter_client_;
  rclcpp::Client<SetFloatValue>::SharedPtr set_confidence_threshold_client_;
  rclcpp::Client<SetBool>::SharedPtr enable_hdr_mode_client_;
  rclcpp::Client<SetBool>::SharedPtr enable_static_scene_client_;
  rclcpp::Client<SetIntegerValue>::SharedPtr set_illumination_mode_client_;
  rclcpp::Client<SetIntegerValue>::SharedPtr set_depth_quality_client_;
  rclcpp::Client<SetBool>::SharedPtr enable_depth_smooth_client_;
  rclcpp::Client<SetIntegerValue>::SharedPtr set_depth_fill_client_;
  rclcpp::Client<SetIntegerValue>::SharedPtr set_depth_seg_client_;
  rclcpp::Client<SetBool>::SharedPtr enable_projector_client_;
  rclcpp::Client<SetIntegerValue>::SharedPtr set_projector_level_client_;
  rclcpp::Client<SetIntegerValue>::SharedPtr set_operating_mode_client_;
};

}  // namespace pylon_ros2_camera_test
