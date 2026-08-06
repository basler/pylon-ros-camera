/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (C) 2022, Basler AG. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * No contributors' name may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
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

#include <cstring>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "internal/impl/pylon_ros2_camera_3d.hpp"

#include <pylon/StereoAceInstantCamera.h>


namespace pylon_ros2_camera
{

namespace
{
    static const rclcpp::Logger LOGGER_STEREO_ACE = rclcpp::get_logger("basler.pylon.ros2.pylon_ros2_stereo_ace_camera");
}

/**
 * Basler Stereo ace integration.
 *
 * The Stereo ace delivers raw disparity (Coord3D_C16) together with an RGB8
 * IntensityCombined image (left stacked over right) and a Confidence8 map.
 * This class enables those components at startup, reconstructs an organized XYZ
 * point cloud host-side from the disparity and the Scan3d* calibration, and
 * exposes the left/right intensity images separately.
 *
 * Detection: enumerated devices report device class
 * "BaslerGTC/Basler/basler_xw" (see detectPylonCamType()).
 *
 * Disparity-to-XYZ reconstruction (grab3D()):
 *     calibrated_d = disp_raw * Scan3dCoordinateScale + Scan3dCoordinateOffset
 *     Z = 1000 * Scan3dBaseline * Scan3dFocalLength / calibrated_d
 *     X = (u - Scan3dPrincipalPointU) * Z / Scan3dFocalLength
 *     Y = (v - Scan3dPrincipalPointV) * Z / Scan3dFocalLength
 *
 * Illumination: BslIlluminationMode (AlternateActive / AlwaysActive / Off) is
 * set from the 'stereo_ace_illumination_mode' parameter (default AlternateActive).
 *
 * Hardware-validated: Scan3dBaseline is in meters (~0.100 m), depth range is in
 * meters [0.1, 100], and the reconstruction formula above matches measured output.
 */
class PylonROS2StereoAceCamera : public PylonROS23DCamera
{
public:
    explicit PylonROS2StereoAceCamera(Pylon::IPylonDevice* device);
    virtual ~PylonROS2StereoAceCamera();

    virtual bool registerCameraConfiguration() override;
    virtual bool openCamera() override;
    virtual bool applyCamSpecificStartupSettings(const PylonROS2CameraParameter& parameters) override;

    virtual bool startGrabbing(const PylonROS2CameraParameter& parameters) override;
    virtual bool setExposure(const float& target_exposure, float& reached_exposure) override;

    virtual bool grab3D(sensor_msgs::msg::PointCloud2& cloud_msg,
                        sensor_msgs::msg::Image& intensity_map_msg,
                        sensor_msgs::msg::Image& depth_map_msg,
                        sensor_msgs::msg::Image& depth_map_color_msg,
                        sensor_msgs::msg::Image& confidence_map_msg) override;
            bool grab3D(Pylon::CGrabResultPtr& grab_result);

    virtual void getInitialCameraInfo(sensor_msgs::msg::CameraInfo& cam_info_msg) override;

    virtual int imagePixelDepth() const override;

    virtual bool hasExtraIntensityImages() const override { return true; }
    virtual const sensor_msgs::msg::Image& extraIntensityLeft()  const override { return intensity_left_msg_; }
    virtual const sensor_msgs::msg::Image& extraIntensityRight() const override { return intensity_right_msg_; }

    // --- Option B service interface -----------------------------------------
    // The inherited base/profile implementations act on the never-opened
    // universal cam_ (or return "not available"); the following retarget every
    // feature the Stereo ace hardware actually supports to stereo_ace_cam_ and
    // return clear stubs for the nodes it does not expose.

    // 2D image controls the profile neutralizes but the Stereo ace supports.
    virtual bool setGain(const float& target_gain, float& reached_gain) override;
    virtual bool setGamma(const float& target_gamma, float& reached_gamma) override;
    virtual bool setBrightness(const int& target_brightness,
                               const float& current_brightness,
                               const bool& exposure_auto,
                               const bool& gain_auto) override;
    virtual std::string setWhiteBalance(const double& redValue, const double& greenValue, const double& blueValue) override;
    virtual std::string setBalanceWhiteAuto(const int& mode) override;
    virtual std::string setAcquisitionFrameRate(const float& framerate) override;
    virtual std::string enableAcquisitionFrameRate(const bool& enable) override;
    virtual std::string setAcquisitionFrameCount(const int& frameCount) override;

    // Real ROI / binning (Width/Height/Offset/Binning exist on the Stereo ace).
    virtual bool setROI(const sensor_msgs::msg::RegionOfInterest target_roi,
                        sensor_msgs::msg::RegionOfInterest& reached_roi) override;
    virtual bool setBinningX(const size_t& target_binning_x, size_t& reached_binning_x) override;
    virtual bool setBinningY(const size_t& target_binning_y, size_t& reached_binning_y) override;
    virtual std::string setOffsetXY(const int& offsetValue, bool xAxis) override;

    // Trigger controls.
    virtual std::string setTriggerSelector(const int& mode) override;
    virtual std::string setTriggerSource(const int& source) override;
    virtual std::string setTriggerMode(const bool& value) override;
    virtual std::string setTriggerActivation(const int& value) override;
    virtual std::string setTriggerDelay(const float& delayValue) override;
    virtual std::string executeSoftwareTrigger() override;
    virtual std::string triggerDeviceReset() override;

    // Digital I/O lines.
    virtual std::string setLineSelector(const int& value) override;
    virtual std::string setLineMode(const int& value) override;
    virtual std::string setLineSource(const int& value) override;
    virtual std::string setLineInverter(const bool& value) override;

    // Device link throughput.
    virtual std::string setDeviceLinkThroughputLimitMode(const bool& turnOn) override;
    virtual std::string setDeviceLinkThroughputLimit(const int& limit) override;

    // ace-style chunk data.
    virtual std::string setChunkModeActive(const bool& enable) override;
    virtual std::string setChunkSelector(const int& value) override;
    virtual std::string setChunkEnable(const bool& enable) override;
    virtual std::string setChunkExposureTime(const float& value) override;

    // Working depth range maps to BslDepthMinDepth / BslDepthMaxDepth.
    virtual std::string setDepthMin(const double& depth_min) override;
    virtual std::string setDepthMax(const double& depth_max) override;
    virtual double getDepthMin() override;
    virtual double getDepthMax() override;

    // Feature persistence (pfs) via the Stereo ace node map.
    virtual std::pair<std::string, std::string> getPfs() override;
    virtual std::string savePfs(const std::string& fileName) override;
    virtual std::string loadPfs(const std::string& fileName) override;

    // PTP (the ace exposes PtpEnable / PtpStatus / PtpServoStatus).
    virtual std::string enablePTP(const bool& value) override;
    virtual std::string getPTPStatus(int64_t& offset_from_master, std::string& status, std::string& servo_status) override;

    // Buffer / statistics retargeted to stereo_ace_cam_.
    virtual std::string setMaxNumBuffer(const int& size) override;
    virtual std::string setOutputQueueSize(const int& size) override;
    virtual int getMaxNumBuffer() override;
    virtual int getStatisticTotalBufferCount() override;
    virtual int getStatisticFailedBufferCount() override;
    virtual int getStatisticBufferUnderrunCount() override;
    virtual int getStatisticFailedPacketCount() override;
    virtual int getStatisticResendRequestCount() override;
    virtual int getStatisticMissedFrameCount() override;
    virtual int getStatisticResynchronizationCount() override;

    // Stubs for features the Stereo ace SDK does not expose.
    virtual std::string setLineDebouncerTime(const float& value) override;
    virtual std::string gammaEnable(const bool& enable) override;
    virtual std::string setUserSetSelector(const int& set) override;
    virtual std::string saveUserSet() override;
    virtual std::string loadUserSet() override;
    virtual std::string setUserSetDefaultSelector(const int& set) override;
    virtual std::string setTimerSelector(const int& selector) override;
    virtual std::string setTimerTriggerSource(const int& source) override;
    virtual std::string setTimerDuration(const float& duration) override;
    virtual std::string setMaxTransferSize(const int& maxTransferSize) override;
    virtual std::string setPTPPriority(const int& value) override;
    virtual std::string setPTPProfile(const int& value) override;
    virtual std::string setPTPNetworkMode(const int& value) override;
    virtual std::string setPTPUCPortAddressIndex(const int& value) override;
    virtual std::string setPTPUCPortAddress(const int& value) override;
    virtual std::string enablePTPManagementProtocol(const bool& value) override;
    virtual std::string enablePTPTwoStepOperation(const bool& value) override;
    virtual std::string setPeriodicSignalPeriod(const float& value) override;
    virtual std::string setPeriodicSignalDelay(const float& value) override;
    virtual std::string setSyncFreeRunTimerStartTimeLow(const int& value) override;
    virtual std::string setSyncFreeRunTimerStartTimeHigh(const int& value) override;
    virtual std::string setSyncFreeRunTimerTriggerRateAbs(const float& value) override;
    virtual std::string enableSyncFreeRunTimer(const bool& value) override;
    virtual std::string updateSyncFreeRunTimer() override;
    virtual std::string setActionTriggerConfiguration(const int& action_device_key, const int& action_group_key, const unsigned int& action_group_mask,
                                                      const int& registration_mode, const int& cleanup) override;
    virtual std::string issueActionCommand(const int& device_key, const int& group_key, const unsigned int& group_mask, const std::string& broadcast_address) override;
    virtual std::string issueScheduledActionCommand(const int& device_key, const int& group_key, const unsigned int& group_mask, const int64_t& action_time_ns_from_current_timestamp, const std::string& broadcast_address) override;
    virtual bool setAutoflash(const std::map<int, bool> flash_on_lines) override;
    virtual bool setUserOutput(const int& output_id, const bool& value) override;

protected:
    // The stereo ace grabs through stereo_ace_cam_; the profile uses this for
    // acquisition start/stop and device-removal detection.
    Pylon::CInstantCamera& activeCamera() const override { return *stereo_ace_cam_; }

public:
    Pylon::CStereoAceInstantCamera* stereo_ace_cam_;

private:
    // Scan3D reconstruction parameters (read at startup with ComponentSelector=Disparity)
    float sta_coordinate_scale_{0.0625f};
    float sta_coordinate_offset_{0.0f};
    float sta_baseline_{0.0f};     // meters
    float sta_focal_length_{0.0f}; // pixels
    float sta_cx_{0.0f};           // Scan3dPrincipalPointU
    float sta_cy_{0.0f};           // Scan3dPrincipalPointV

    // Extra intensity images (left / right, from IntensityCombined, top/bottom halves)
    sensor_msgs::msg::Image intensity_left_msg_;
    sensor_msgs::msg::Image intensity_right_msg_;

    // Re-reads img_cols_/img_rows_ from the IntensityCombined component after a
    // resolution-changing setting (ROI / binning).
    void relearnImageDimensions();
    // Reads an integer statistics counter from the camera node map; -1 if absent.
    int readStatisticCounter(const char* node_name);
};

PylonROS2StereoAceCamera::PylonROS2StereoAceCamera(Pylon::IPylonDevice* device) :
    PylonROS23DCamera(device),
    stereo_ace_cam_(new Pylon::CStereoAceInstantCamera(device))
{
    // Keep only the latest frame by default (matches the previous fixed
    // GrabStrategy_LatestImageOnly behavior); user-changeable via
    // set_grabbing_strategy.
    grab_strategy_ = 1;
}

PylonROS2StereoAceCamera::~PylonROS2StereoAceCamera()
{
    try
    {
        if (stereo_ace_cam_->IsOpen())
        {
            stereo_ace_cam_->Close();
        }
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_DEBUG_STREAM(LOGGER_STEREO_ACE, "Destructor (Stereo ace): Failed to close camera: " << e.GetDescription());
    }

    if (stereo_ace_cam_)
    {
        this->detachBaseDevice();
        delete stereo_ace_cam_;
        stereo_ace_cam_ = nullptr;
    }
}

bool PylonROS2StereoAceCamera::registerCameraConfiguration()
{
    return true;
}

bool PylonROS2StereoAceCamera::openCamera()
{
    try
    {
        stereo_ace_cam_->Open();
        RCLCPP_DEBUG_STREAM(LOGGER_STEREO_ACE, "Connected to camera " << stereo_ace_cam_->GetDeviceInfo().GetFriendlyName());
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_ACE, "An exception occurred while opening camera: " << e.GetDescription());
        return false;
    }
    return true;
}

bool PylonROS2StereoAceCamera::applyCamSpecificStartupSettings(const PylonROS2CameraParameter& parameters __attribute__((unused)))
{
    try
    {
        RCLCPP_DEBUG_STREAM(LOGGER_STEREO_ACE, "-> Model name: " << stereo_ace_cam_->GetDeviceInfo().GetModelName().c_str());
        RCLCPP_DEBUG_STREAM(LOGGER_STEREO_ACE, "-> Serial number: " << stereo_ace_cam_->GetDeviceInfo().GetSerialNumber().c_str());

        // Disable Intensity -- IntensityCombined delivers both left and right
        // rectified images stacked vertically in one buffer; using it avoids
        // transmitting a redundant third stream.
        stereo_ace_cam_->ComponentSelector.FromString("Intensity");
        stereo_ace_cam_->ComponentEnable.SetValue(false);

        // Enable IntensityCombined (top half = left, bottom half = right, RGB8).
        stereo_ace_cam_->ComponentSelector.FromString("IntensityCombined");
        stereo_ace_cam_->ComponentEnable.SetValue(true);
        stereo_ace_cam_->PixelFormat.TrySetValue("RGB8");

        // Enable disparity output (raw uint16 disparity, Coord3D_C16).
        stereo_ace_cam_->ComponentSelector.FromString("Disparity");
        stereo_ace_cam_->ComponentEnable.SetValue(true);

        // Enable confidence map (Confidence8).
        stereo_ace_cam_->ComponentSelector.FromString("Confidence");
        stereo_ace_cam_->ComponentEnable.SetValue(true);

        // Illumination mode: configurable via 'stereo_ace_illumination_mode' ROS parameter
        // (set in profile_3d.yaml or as a launch argument).
        // AlternateActive (default): clean intensity images (projector alternates exposures).
        // AlwaysActive: IR pattern visible in intensity images.
        // Measured: no ROS output-rate difference between modes (pipeline capped ~2 Hz host-side).
        const std::string illum_mode = parameters.stereo_ace_illumination_mode_.empty()
            ? "AlternateActive" : parameters.stereo_ace_illumination_mode_;
        stereo_ace_cam_->BslIlluminationMode.FromString(illum_mode.c_str());

        // Read Scan3D reconstruction parameters from the Disparity component.
        // ComponentSelector must be set to Disparity before reading these nodes.
        stereo_ace_cam_->ComponentSelector.FromString("Disparity");
        sta_coordinate_scale_  = static_cast<float>(stereo_ace_cam_->Scan3dCoordinateScale.GetValue());
        sta_coordinate_offset_ = static_cast<float>(stereo_ace_cam_->Scan3dCoordinateOffset.GetValue());
        sta_baseline_          = static_cast<float>(stereo_ace_cam_->Scan3dBaseline.GetValue());
        sta_focal_length_      = static_cast<float>(stereo_ace_cam_->Scan3dFocalLength.GetValue());
        sta_cx_                = static_cast<float>(stereo_ace_cam_->Scan3dPrincipalPointU.GetValue());
        sta_cy_                = static_cast<float>(stereo_ace_cam_->Scan3dPrincipalPointV.GetValue());

        RCLCPP_INFO_STREAM(LOGGER_STEREO_ACE,
            "Stereo ace configured: IntensityCombined=RGB8 (left/right), "
            "Disparity=Coord3D_C16, Confidence8, BslIlluminationMode=" << illum_mode << ". "
            "Reconstruction params: focal=" << sta_focal_length_
            << " baseline=" << sta_baseline_ << " m"
            << " scale=" << sta_coordinate_scale_
            << " cx=" << sta_cx_ << " cy=" << sta_cy_);
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_ACE, "An exception occurred while applying startup settings: " << e.GetDescription());
        return false;
    }
    return true;
}

bool PylonROS2StereoAceCamera::startGrabbing(const PylonROS2CameraParameter& parameters)
{
    try
    {
        this->grabbingStarting();

        device_user_id_ = stereo_ace_cam_->GetDeviceInfo().GetUserDefinedName().c_str();
        grab_timeout_ = std::max(parameters.grab_timeout_, 5000);
        RCLCPP_DEBUG_STREAM_ONCE(LOGGER_STEREO_ACE, "Grab timeout for Stereo ace: " << grab_timeout_);

        Pylon::CGrabResultPtr grab_result;
        if (this->grab3D(grab_result) && grab_result.IsValid())
        {
            // Use the IntensityCombined component for image size.
            // IntensityCombined stacks left and right vertically: each individual
            // image is width × (combined_height / 2).
            for (int idx = 0; idx < (int)grab_result->GetDataComponentCount(); ++idx)
            {
                const auto c = grab_result->GetDataComponent(idx);
                if (c.GetComponentType() == Pylon::ComponentType_IntensityCombined_STA)
                {
                    img_cols_ = static_cast<size_t>(c.GetWidth());
                    img_rows_ = static_cast<size_t>(c.GetHeight()) / 2; // each individual image
                    img_size_byte_ = img_cols_ * img_rows_ * imagePixelDepth();
                    is_ready_ = true;
                    break;
                }
            }
            if (!is_ready_)
            {
                RCLCPP_ERROR(LOGGER_STEREO_ACE, "Initial grab returned no IntensityCombined component");
            }
        }
        else
        {
            RCLCPP_ERROR(LOGGER_STEREO_ACE, "PylonROS2StereoAceCamera not ready: initial grab failed");
        }
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_ACE, "An exception occurred while starting image grabbing: " << e.GetDescription());
        return false;
    }
    return true;
}

bool PylonROS2StereoAceCamera::grab3D(Pylon::CGrabResultPtr& grab_result)
{
    if (!stereo_ace_cam_->IsGrabbing())
        return false;

    try
    {
        stereo_ace_cam_->RetrieveResult(grab_timeout_, grab_result, Pylon::TimeoutHandling_ThrowException);
    }
    catch (const GenICam::GenericException& e)
    {
        if (stereo_ace_cam_->IsCameraDeviceRemoved())
            RCLCPP_ERROR(LOGGER_STEREO_ACE, "Lost connection to the camera...");
        else
            RCLCPP_ERROR_STREAM(LOGGER_STEREO_ACE, "Image grabbing exception: " << e.GetDescription());
        return false;
    }
    catch (...)
    {
        RCLCPP_ERROR(LOGGER_STEREO_ACE, "An unspecified image grabbing exception occurred");
        return false;
    }

    if (!grab_result->GrabSucceeded())
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_ACE, "Error: " << grab_result->GetErrorCode() << " " << grab_result->GetErrorDescription());
        return false;
    }
    return true;
}

bool PylonROS2StereoAceCamera::grab3D(sensor_msgs::msg::PointCloud2& cloud_msg,
                                      sensor_msgs::msg::Image& intensity_map_msg,
                                      sensor_msgs::msg::Image& depth_map_msg,
                                      sensor_msgs::msg::Image& depth_map_color_msg,
                                      sensor_msgs::msg::Image& confidence_map_msg)
{
    Pylon::CGrabResultPtr ptr_grab_result;
    if (!this->grab3D(ptr_grab_result))
    {
        RCLCPP_ERROR(LOGGER_STEREO_ACE, "Grabbing with Stereo ace failed");
        return false;
    }

    // Locate IntensityCombined, Disparity, and Confidence components.
    int idxCombined = -1, idxDisparity = -1, idxConfidence = -1;
    for (int i = 0; i < (int)ptr_grab_result->GetDataComponentCount(); ++i)
    {
        const auto type = ptr_grab_result->GetDataComponent(i).GetComponentType();
        if (type == Pylon::ComponentType_IntensityCombined_STA) idxCombined  = i;
        else if (type == Pylon::ComponentType_Disparity)         idxDisparity = i;
        else if (type == Pylon::ComponentType_Confidence)        idxConfidence = i;
    }

    // --- IntensityCombined → intensity_3d (left) + intensity_left_3d + intensity_right_3d ---
    const uint8_t* left_data = nullptr;
    const uint8_t* int_data = nullptr; // pointer into the left half, used for point cloud coloring
    int iw = 0, ih = 0; // individual half dimensions
    if (idxCombined >= 0)
    {
        const auto comb = ptr_grab_result->GetDataComponent(idxCombined);
        const int cw  = static_cast<int>(comb.GetWidth());
        const int ch  = static_cast<int>(comb.GetHeight());
        const int half_h = ch / 2;
        const uint8_t* src = static_cast<const uint8_t*>(comb.GetData());
        const size_t row_bytes = static_cast<size_t>(cw) * 3u;
        left_data = src;                                   // top half = left
        const uint8_t* right_data = src + half_h * row_bytes; // bottom half = right
        iw = cw; ih = half_h;

        // intensity_3d = left image
        intensity_map_msg.encoding = sensor_msgs::image_encodings::RGB8;
        intensity_map_msg.height   = static_cast<uint32_t>(half_h);
        intensity_map_msg.width    = static_cast<uint32_t>(cw);
        intensity_map_msg.step     = static_cast<uint32_t>(cw * 3);
        intensity_map_msg.is_bigendian = false;
        intensity_map_msg.data.assign(left_data, left_data + half_h * row_bytes);

        // intensity_left_3d (same as intensity_3d)
        intensity_left_msg_ = intensity_map_msg;

        // intensity_right_3d = right image
        intensity_right_msg_.encoding = sensor_msgs::image_encodings::RGB8;
        intensity_right_msg_.height   = static_cast<uint32_t>(half_h);
        intensity_right_msg_.width    = static_cast<uint32_t>(cw);
        intensity_right_msg_.step     = static_cast<uint32_t>(cw * 3);
        intensity_right_msg_.is_bigendian = false;
        intensity_right_msg_.data.assign(right_data, right_data + half_h * row_bytes);

        // Expose left data for point cloud coloring below.
        int_data = left_data;
        iw = cw; ih = half_h;
    }

    // --- Disparity → point cloud + depth maps ---
    if (idxDisparity < 0)
    {
        RCLCPP_WARN_ONCE(LOGGER_STEREO_ACE,
            "No Disparity component in grab result. Cloud and depth maps will be empty.");
        return true;
    }

    const auto disp_comp = ptr_grab_result->GetDataComponent(idxDisparity);
    const int dw = static_cast<int>(disp_comp.GetWidth());   // 1224
    const int dh = static_cast<int>(disp_comp.GetHeight());  // 1024
    const uint16_t* disp = static_cast<const uint16_t*>(disp_comp.GetData());

    // Compute Z buffer in millimetres (0 = invalid pixel).
    // Formula from Basler sample (SavePointcloud.cpp):
    //   calibrated_d = raw * Scan3dCoordinateScale + Scan3dCoordinateOffset
    //   Z [mm] = 1000 * Scan3dBaseline[m] * Scan3dFocalLength[px] / calibrated_d[px]
    std::vector<float> z_buf(dw * dh, 0.0f);
    for (int v = 0; v < dh; ++v)
    {
        for (int u = 0; u < dw; ++u)
        {
            const uint16_t raw = disp[v * dw + u];
            if (raw == 0) continue; // invalid
            const float d = raw * sta_coordinate_scale_ + sta_coordinate_offset_;
            if (d <= 0.0f) continue;
            z_buf[v * dw + u] = 1000.0f * sta_baseline_ * sta_focal_length_ / d;
        }
    }

    // --- PointCloud2 (XYZ in metres + RGBA colour) ---
    cloud_msg.width = static_cast<uint32_t>(dw);
    cloud_msg.height = static_cast<uint32_t>(dh);
    cloud_msg.is_dense = false;
    cloud_msg.fields.resize(4);
    cloud_msg.fields[0].name = "x";    cloud_msg.fields[0].offset = 0;  cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32; cloud_msg.fields[0].count = 1;
    cloud_msg.fields[1].name = "y";    cloud_msg.fields[1].offset = 4;  cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32; cloud_msg.fields[1].count = 1;
    cloud_msg.fields[2].name = "z";    cloud_msg.fields[2].offset = 8;  cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32; cloud_msg.fields[2].count = 1;
    cloud_msg.fields[3].name = "rgba"; cloud_msg.fields[3].offset = 12; cloud_msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32; cloud_msg.fields[3].count = 1;
    cloud_msg.point_step = 16;
    cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
    cloud_msg.data.resize(cloud_msg.row_step * cloud_msg.height, 0);

    const float nan = std::numeric_limits<float>::quiet_NaN();
    for (int v = 0; v < dh; ++v)
    {
        for (int u = 0; u < dw; ++u)
        {
            const float z_mm = z_buf[v * dw + u];
            uint8_t* pt = cloud_msg.data.data() + static_cast<size_t>(v * dw + u) * 16;
            if (z_mm > 0.0f)
            {
                const float z_m = z_mm * 0.001f;
                const float x_m = (static_cast<float>(u) - sta_cx_) * z_mm * 0.001f / sta_focal_length_;
                const float y_m = (static_cast<float>(v) - sta_cy_) * z_mm * 0.001f / sta_focal_length_;
                memcpy(pt,     &x_m, 4);
                memcpy(pt + 4, &y_m, 4);
                memcpy(pt + 8, &z_m, 4);
            }
            else
            {
                memcpy(pt,     &nan, 4);
                memcpy(pt + 4, &nan, 4);
                memcpy(pt + 8, &nan, 4);
            }
            // RGB colour from intensity image (pylon RGB8packed: R,G,B order).
            uint8_t r = 128, g = 128, b = 128;
            if (int_data && iw > 0 && ih > 0)
            {
                const int iu = static_cast<int>(u * static_cast<double>(iw) / dw);
                const int iv = static_cast<int>(v * static_cast<double>(ih) / dh);
                if (iu < iw && iv < ih)
                {
                    const uint8_t* c = int_data + (iv * iw + iu) * 3;
                    r = c[0]; g = c[1]; b = c[2];
                }
            }
            pt[12] = r; pt[13] = g; pt[14] = b; pt[15] = 255u;
        }
    }

    // --- Depth map (mono16, Z directly in mm, 0 = invalid) ---
    depth_map_msg.encoding = sensor_msgs::image_encodings::MONO16;
    depth_map_msg.height = static_cast<uint32_t>(dh);
    depth_map_msg.width  = static_cast<uint32_t>(dw);
    depth_map_msg.step   = static_cast<uint32_t>(dw * 2);
    depth_map_msg.is_bigendian = false;
    depth_map_msg.data.resize(dh * dw * 2, 0);
    uint16_t* dm16 = reinterpret_cast<uint16_t*>(depth_map_msg.data.data());
    for (int i = 0; i < dh * dw; ++i)
    {
        const float z = z_buf[i];
        dm16[i] = (z > 0.0f && z < 65535.0f) ? static_cast<uint16_t>(z) : 0u;
    }

    // --- Depth map colour (bgr8 false-colour: near=blue, far=red, 5 m range) ---
    const float depth_range_mm = 5000.0f;
    depth_map_color_msg.encoding = sensor_msgs::image_encodings::BGR8;
    depth_map_color_msg.height = static_cast<uint32_t>(dh);
    depth_map_color_msg.width  = static_cast<uint32_t>(dw);
    depth_map_color_msg.step   = static_cast<uint32_t>(dw * 3);
    depth_map_color_msg.is_bigendian = false;
    depth_map_color_msg.data.resize(dh * dw * 3, 0);
    uint8_t* dmc = depth_map_color_msg.data.data();
    for (int i = 0; i < dh * dw; ++i)
    {
        const float z = z_buf[i];
        if (z > 0.0f)
        {
            const float frac = std::min(1.0f, z / depth_range_mm);
            dmc[i * 3 + 0] = static_cast<uint8_t>((1.0f - frac) * 255.0f); // B (near)
            dmc[i * 3 + 1] = 0u;
            dmc[i * 3 + 2] = static_cast<uint8_t>(frac * 255.0f);           // R (far)
        }
    }

    // --- Confidence map ---
    if (idxConfidence >= 0)
        this->buildConfidenceImage(ptr_grab_result->GetDataComponent(idxConfidence), confidence_map_msg);

    return true;
}

void PylonROS2StereoAceCamera::getInitialCameraInfo(sensor_msgs::msg::CameraInfo& cam_info_msg)
{
    // Read intrinsics from the Disparity component's Scan3d nodes.
    stereo_ace_cam_->ComponentSelector.FromString("Disparity");
    this->populateCameraInfoFromScan3d(stereo_ace_cam_->GetNodeMap(),
                                       static_cast<int>(this->imageCols()),
                                       static_cast<int>(this->imageRows()),
                                       cam_info_msg);
}

int PylonROS2StereoAceCamera::imagePixelDepth() const
{
    return 3; // RGB8 intensity
}

bool PylonROS2StereoAceCamera::setExposure(const float& target_exposure, float& reached_exposure)
{
    // Changing ExposureTime while stereo_ace_cam_ is grabbing can block
    // RetrieveResult(). Stop grabbing, change the exposure, then restart.
    try
    {
        const bool was_grabbing = stereo_ace_cam_->IsGrabbing();
        if (was_grabbing)
            stereo_ace_cam_->StopGrabbing();

        GenApi::INodeMap& nm = stereo_ace_cam_->GetNodeMap();
        GenApi::CEnumerationPtr ea = nm.GetNode("ExposureAuto");
        if (ea.IsValid() && GenApi::IsWritable(ea))
            ea->FromString("Off");

        GenApi::CFloatPtr et = nm.GetNode("ExposureTime");
        if (et.IsValid() && GenApi::IsWritable(et))
        {
            const float min_exp = static_cast<float>(et->GetMin());
            const float max_exp = static_cast<float>(et->GetMax());
            float exposure_to_set = target_exposure;
            if (exposure_to_set < min_exp)
            {
                RCLCPP_WARN_STREAM(LOGGER_STEREO_ACE, "Desired exposure (" << exposure_to_set
                    << ") unreachable! Setting to lower limit: " << min_exp);
                exposure_to_set = min_exp;
            }
            else if (exposure_to_set > max_exp)
            {
                RCLCPP_WARN_STREAM(LOGGER_STEREO_ACE, "Desired exposure (" << exposure_to_set
                    << ") unreachable! Setting to upper limit: " << max_exp);
                exposure_to_set = max_exp;
            }
            et->SetValue(exposure_to_set);
            reached_exposure = static_cast<float>(et->GetValue());
        }
        else
        {
            reached_exposure = target_exposure;
        }

        if (was_grabbing)
            stereo_ace_cam_->StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_ACE, "An exception while setting target exposure to "
            << target_exposure << " occurred: " << e.GetDescription());
        reached_exposure = target_exposure;
        return false;
    }
    return true;
}

// Shorthand for the Stereo ace enumeration tokens.
namespace StaParams = Pylon::StereoAceCameraParams_Params;

// --- Image controls the Stereo ace supports --------------------------------

bool PylonROS2StereoAceCamera::setGain(const float& target_gain, float& reached_gain)
{
    // The Stereo ace has no GainAuto; Gain is always manual. target_gain is a
    // fraction [0.0 - 1.0] mapped onto the Gain range selected by GainSelector.
    try
    {
        stereo_ace_cam_->GainSelector.TrySetValue(StaParams::GainSelector_All);

        float truncated_gain = target_gain;
        if (truncated_gain < 0.0f) truncated_gain = 0.0f;
        else if (truncated_gain > 1.0f) truncated_gain = 1.0f;

        const float min_gain = static_cast<float>(stereo_ace_cam_->Gain.GetMin());
        const float max_gain = static_cast<float>(stereo_ace_cam_->Gain.GetMax());
        stereo_ace_cam_->Gain.SetValue(min_gain + truncated_gain * (max_gain - min_gain));

        const float reached_abs = static_cast<float>(stereo_ace_cam_->Gain.GetValue());
        reached_gain = (max_gain > min_gain) ? (reached_abs - min_gain) / (max_gain - min_gain) : 0.0f;
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_ACE, "An exception while setting target gain to "
            << target_gain << " occurred: " << e.GetDescription());
        return false;
    }
    return true;
}

bool PylonROS2StereoAceCamera::setGamma(const float& target_gamma, float& reached_gamma)
{
    try
    {
        float gamma_to_set = target_gamma;
        const float min_gamma = static_cast<float>(stereo_ace_cam_->Gamma.GetMin());
        const float max_gamma = static_cast<float>(stereo_ace_cam_->Gamma.GetMax());
        if (gamma_to_set < min_gamma) gamma_to_set = min_gamma;
        else if (gamma_to_set > max_gamma) gamma_to_set = max_gamma;
        stereo_ace_cam_->Gamma.SetValue(gamma_to_set);
        reached_gamma = static_cast<float>(stereo_ace_cam_->Gamma.GetValue());
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_ACE, "An exception while setting target gamma to "
            << target_gamma << " occurred: " << e.GetDescription());
        return false;
    }
    return true;
}

bool PylonROS2StereoAceCamera::setBrightness(const int& target_brightness,
                                             const float& current_brightness __attribute__((unused)),
                                             const bool& exposure_auto,
                                             const bool& gain_auto __attribute__((unused)))
{
    // Direct analog BslBrightness control; target_brightness [1..255] mapped
    // linearly onto the BslBrightness range.
    try
    {
        if (exposure_auto)
            stereo_ace_cam_->ExposureAuto.TrySetValue(StaParams::ExposureAuto_Continuous);

        const float clamped = static_cast<float>(std::min(255, std::max(1, target_brightness)));
        const float min_b = static_cast<float>(stereo_ace_cam_->BslBrightness.GetMin());
        const float max_b = static_cast<float>(stereo_ace_cam_->BslBrightness.GetMax());
        stereo_ace_cam_->BslBrightness.SetValue(min_b + (clamped - 1.0f) / 254.0f * (max_b - min_b));
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_ACE, "An exception while setting brightness occurred: " << e.GetDescription());
        return false;
    }
    return true;
}

std::string PylonROS2StereoAceCamera::setWhiteBalance(const double& redValue, const double& greenValue, const double& blueValue)
{
    try
    {
        stereo_ace_cam_->BalanceWhiteAuto.TrySetValue(StaParams::BalanceWhiteAuto_Off);

        stereo_ace_cam_->BalanceRatioSelector.SetValue(StaParams::BalanceRatioSelector_Red);
        stereo_ace_cam_->BalanceRatio.SetValue(redValue);
        stereo_ace_cam_->BalanceRatioSelector.SetValue(StaParams::BalanceRatioSelector_Green);
        stereo_ace_cam_->BalanceRatio.SetValue(greenValue);
        stereo_ace_cam_->BalanceRatioSelector.SetValue(StaParams::BalanceRatioSelector_Blue);
        stereo_ace_cam_->BalanceRatio.SetValue(blueValue);
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_ACE, "An exception while setting the white balance occurred: " << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

std::string PylonROS2StereoAceCamera::setBalanceWhiteAuto(const int& mode)
{
    try
    {
        switch (mode)
        {
            case 0: stereo_ace_cam_->BalanceWhiteAuto.SetValue(StaParams::BalanceWhiteAuto_Off); break;
            case 1: stereo_ace_cam_->BalanceWhiteAuto.SetValue(StaParams::BalanceWhiteAuto_Once); break;
            case 2: stereo_ace_cam_->BalanceWhiteAuto.SetValue(StaParams::BalanceWhiteAuto_Continuous); break;
            default: return "Error: unknown value (0=Off, 1=Once, 2=Continuous)";
        }
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_ACE, "An exception while changing the balance white auto occurred: " << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

std::string PylonROS2StereoAceCamera::setAcquisitionFrameRate(const float& framerate)
{
    try
    {
        if (stereo_ace_cam_->AcquisitionFrameRateEnable.GetValue())
            stereo_ace_cam_->AcquisitionFrameRate.SetValue(framerate);
        else
            return "To change the acquisition frame rate, it must first be enabled";
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_ACE, "An exception while changing the acquisition frame rate occurred: " << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

std::string PylonROS2StereoAceCamera::enableAcquisitionFrameRate(const bool& enable)
{
    try
    {
        stereo_ace_cam_->AcquisitionFrameRateEnable.SetValue(enable);
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_ACE, "An exception while enabling/disabling the acquisition frame rate occurred: " << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

std::string PylonROS2StereoAceCamera::setAcquisitionFrameCount(const int& frameCount)
{
    try
    {
        // AcquisitionFrameCount is locked while grabbing; stop and restart around the change.
        this->grabbingStopping();
        stereo_ace_cam_->AcquisitionFrameCount.SetValue(frameCount);
        this->grabbingStarting();
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_ACE, "An exception while setting the acquisition frame count occurred: " << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

// --- ROI / binning ----------------------------------------------------------

bool PylonROS2StereoAceCamera::setROI(const sensor_msgs::msg::RegionOfInterest target_roi,
                                      sensor_msgs::msg::RegionOfInterest& reached_roi)
{
    try
    {
        this->grabbingStopping();

        const int64_t max_w = stereo_ace_cam_->Width.GetMax();
        const int64_t min_w = stereo_ace_cam_->Width.GetMin();
        const int64_t max_h = stereo_ace_cam_->Height.GetMax();
        const int64_t min_h = stereo_ace_cam_->Height.GetMin();
        const int64_t w_inc = std::max<int64_t>(1, stereo_ace_cam_->Width.GetInc());
        const int64_t h_inc = std::max<int64_t>(1, stereo_ace_cam_->Height.GetInc());

        // Reset offsets before resizing to avoid range conflicts.
        stereo_ace_cam_->OffsetX.TrySetValue(0);
        stereo_ace_cam_->OffsetY.TrySetValue(0);

        int64_t width_to_set    = std::min(max_w, std::max(min_w, static_cast<int64_t>(target_roi.width)));
        int64_t height_to_set   = std::min(max_h, std::max(min_h, static_cast<int64_t>(target_roi.height)));
        int64_t offset_x_to_set = static_cast<int64_t>(target_roi.x_offset);
        int64_t offset_y_to_set = static_cast<int64_t>(target_roi.y_offset);
        offset_x_to_set -= offset_x_to_set % w_inc;
        offset_y_to_set -= offset_y_to_set % h_inc;
        if (width_to_set + offset_x_to_set > max_w)   offset_x_to_set = max_w - width_to_set;
        if (height_to_set + offset_y_to_set > max_h)  offset_y_to_set = max_h - height_to_set;

        stereo_ace_cam_->Width.SetValue(width_to_set);
        stereo_ace_cam_->Height.SetValue(height_to_set);
        stereo_ace_cam_->OffsetX.SetValue(offset_x_to_set);
        stereo_ace_cam_->OffsetY.SetValue(offset_y_to_set);

        reached_roi.width    = static_cast<uint32_t>(stereo_ace_cam_->Width.GetValue());
        reached_roi.height   = static_cast<uint32_t>(stereo_ace_cam_->Height.GetValue());
        reached_roi.x_offset = static_cast<uint32_t>(stereo_ace_cam_->OffsetX.GetValue());
        reached_roi.y_offset = static_cast<uint32_t>(stereo_ace_cam_->OffsetY.GetValue());

        this->grabbingStarting();
        this->relearnImageDimensions();
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_ACE, "An exception while setting the ROI occurred: " << e.GetDescription());
        return false;
    }
    return true;
}

bool PylonROS2StereoAceCamera::setBinningX(const size_t& target_binning_x, size_t& reached_binning_x)
{
    try
    {
        this->grabbingStopping();
        int64_t v = std::min(stereo_ace_cam_->BinningHorizontal.GetMax(),
                             std::max(stereo_ace_cam_->BinningHorizontal.GetMin(), static_cast<int64_t>(target_binning_x)));
        stereo_ace_cam_->BinningHorizontal.SetValue(v);
        reached_binning_x = static_cast<size_t>(stereo_ace_cam_->BinningHorizontal.GetValue());
        this->grabbingStarting();
        this->relearnImageDimensions();
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_ACE, "An exception while setting the horizontal binning occurred: " << e.GetDescription());
        return false;
    }
    return true;
}

bool PylonROS2StereoAceCamera::setBinningY(const size_t& target_binning_y, size_t& reached_binning_y)
{
    try
    {
        this->grabbingStopping();
        int64_t v = std::min(stereo_ace_cam_->BinningVertical.GetMax(),
                             std::max(stereo_ace_cam_->BinningVertical.GetMin(), static_cast<int64_t>(target_binning_y)));
        stereo_ace_cam_->BinningVertical.SetValue(v);
        reached_binning_y = static_cast<size_t>(stereo_ace_cam_->BinningVertical.GetValue());
        this->grabbingStarting();
        this->relearnImageDimensions();
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_ACE, "An exception while setting the vertical binning occurred: " << e.GetDescription());
        return false;
    }
    return true;
}

std::string PylonROS2StereoAceCamera::setOffsetXY(const int& offsetValue, bool xAxis)
{
    try
    {
        if (xAxis) stereo_ace_cam_->OffsetX.SetValue(offsetValue);
        else       stereo_ace_cam_->OffsetY.SetValue(offsetValue);
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_ACE, "An exception while setting the offset occurred: " << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

// --- Trigger controls -------------------------------------------------------

std::string PylonROS2StereoAceCamera::setTriggerSelector(const int& mode)
{
    try
    {
        if (mode == 0)
        {
            stereo_ace_cam_->TriggerSelector.SetValue(StaParams::TriggerSelector_FrameStart);
            return "done";
        }
        return "Error: the Stereo ace only supports trigger selector 0 (FrameStart)";
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_ACE, "An exception while setting the trigger selector occurred: " << e.GetDescription());
        return e.GetDescription();
    }
}

std::string PylonROS2StereoAceCamera::setTriggerSource(const int& source)
{
    try
    {
        switch (source)
        {
            case 0: stereo_ace_cam_->TriggerSource.SetValue(StaParams::TriggerSource_Software); break;
            case 1: stereo_ace_cam_->TriggerSource.SetValue(StaParams::TriggerSource_In1); break;
            default: return "Error: the Stereo ace supports trigger source 0 (Software) or 1 (In1)";
        }
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_ACE, "An exception while setting the trigger source occurred: " << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

std::string PylonROS2StereoAceCamera::setTriggerMode(const bool& value)
{
    try
    {
        stereo_ace_cam_->TriggerMode.SetValue(value ? StaParams::TriggerMode_On : StaParams::TriggerMode_Off);
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_ACE, "An exception while setting the trigger mode occurred: " << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

std::string PylonROS2StereoAceCamera::setTriggerActivation(const int& value)
{
    try
    {
        switch (value)
        {
            case 0: stereo_ace_cam_->TriggerActivation.SetValue(StaParams::TriggerActivation_RisingEdge); break;
            case 1: stereo_ace_cam_->TriggerActivation.SetValue(StaParams::TriggerActivation_FallingEdge); break;
            case 2: stereo_ace_cam_->TriggerActivation.SetValue(StaParams::TriggerActivation_AnyEdge); break;
            default: return "Error: unknown value (0=RisingEdge, 1=FallingEdge, 2=AnyEdge)";
        }
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_ACE, "An exception while setting the trigger activation occurred: " << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

std::string PylonROS2StereoAceCamera::setTriggerDelay(const float& delayValue)
{
    try
    {
        stereo_ace_cam_->TriggerDelay.SetValue(delayValue);
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_ACE, "An exception while setting the trigger delay occurred: " << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

std::string PylonROS2StereoAceCamera::executeSoftwareTrigger()
{
    try
    {
        if (!stereo_ace_cam_->CanWaitForFrameTriggerReady())
        {
            stereo_ace_cam_->ExecuteSoftwareTrigger();
        }
        else if (stereo_ace_cam_->WaitForFrameTriggerReady(grab_timeout_, Pylon::TimeoutHandling_Return))
        {
            stereo_ace_cam_->ExecuteSoftwareTrigger();
        }
        else
        {
            return "Camera not ready to accept a software trigger";
        }
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_ACE, "An exception while executing the software trigger occurred: " << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

std::string PylonROS2StereoAceCamera::triggerDeviceReset()
{
    try
    {
        stereo_ace_cam_->DeviceReset.Execute();
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_ACE, "An exception while triggering the device reset occurred: " << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

// --- Digital I/O lines ------------------------------------------------------

std::string PylonROS2StereoAceCamera::setLineSelector(const int& value)
{
    // The Stereo ace exposes In1..In4 and Out1..Out4: 1-4 select inputs, 5-8 outputs.
    try
    {
        switch (value)
        {
            case 1: stereo_ace_cam_->LineSelector.SetValue(StaParams::LineSelector_In1); break;
            case 2: stereo_ace_cam_->LineSelector.SetValue(StaParams::LineSelector_In2); break;
            case 3: stereo_ace_cam_->LineSelector.SetValue(StaParams::LineSelector_In3); break;
            case 4: stereo_ace_cam_->LineSelector.SetValue(StaParams::LineSelector_In4); break;
            case 5: stereo_ace_cam_->LineSelector.SetValue(StaParams::LineSelector_Out1); break;
            case 6: stereo_ace_cam_->LineSelector.SetValue(StaParams::LineSelector_Out2); break;
            case 7: stereo_ace_cam_->LineSelector.SetValue(StaParams::LineSelector_Out3); break;
            case 8: stereo_ace_cam_->LineSelector.SetValue(StaParams::LineSelector_Out4); break;
            default: return "Error: unknown value (1-4=In1-In4, 5-8=Out1-Out4)";
        }
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_ACE, "An exception while setting the line selector occurred: " << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

std::string PylonROS2StereoAceCamera::setLineMode(const int& value)
{
    try
    {
        stereo_ace_cam_->LineMode.SetValue(value == 0 ? StaParams::LineMode_Input : StaParams::LineMode_Output);
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_ACE, "An exception while setting the line mode occurred: " << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

std::string PylonROS2StereoAceCamera::setLineSource(const int& value)
{
    try
    {
        switch (value)
        {
            case 0: stereo_ace_cam_->LineSource.SetValue(StaParams::LineSource_ExposureActive); break;
            case 1: stereo_ace_cam_->LineSource.SetValue(StaParams::LineSource_ExposureAlternateActive); break;
            case 2: stereo_ace_cam_->LineSource.SetValue(StaParams::LineSource_High); break;
            case 3: stereo_ace_cam_->LineSource.SetValue(StaParams::LineSource_Low); break;
            default: return "Error: unknown value (0=ExposureActive, 1=ExposureAlternateActive, 2=High, 3=Low)";
        }
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_ACE, "An exception while setting the line source occurred: " << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

std::string PylonROS2StereoAceCamera::setLineInverter(const bool& value)
{
    try
    {
        stereo_ace_cam_->LineInverter.SetValue(value);
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_ACE, "An exception while setting the line inverter occurred: " << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

// --- Device link throughput -------------------------------------------------

std::string PylonROS2StereoAceCamera::setDeviceLinkThroughputLimitMode(const bool& turnOn)
{
    try
    {
        stereo_ace_cam_->DeviceLinkThroughputLimitMode.FromString(turnOn ? "On" : "Off");
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_ACE, "An exception while changing the device link throughput limit mode occurred: " << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

std::string PylonROS2StereoAceCamera::setDeviceLinkThroughputLimit(const int& limit)
{
    try
    {
        stereo_ace_cam_->DeviceLinkThroughputLimit.SetValue(limit);
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_ACE, "An exception while changing the device link throughput limit occurred (throughput limit mode must be On): " << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

// --- Chunk data -------------------------------------------------------------

std::string PylonROS2StereoAceCamera::setChunkModeActive(const bool& enable)
{
    try
    {
        stereo_ace_cam_->ChunkModeActive.SetValue(enable);
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_ACE, "An exception while setting chunk mode active occurred: " << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

std::string PylonROS2StereoAceCamera::setChunkSelector(const int& value)
{
    // The driver-internal timestamp workflow calls setChunkSelector(29); map that
    // and FrameID to the Stereo ace ChunkSelector entries.
    try
    {
        switch (value)
        {
            case 29: stereo_ace_cam_->ChunkSelector.SetValue(StaParams::ChunkSelector_Timestamp); break;
            case 7:  stereo_ace_cam_->ChunkSelector.SetValue(StaParams::ChunkSelector_FrameID); break;
            default: return "Error: unsupported chunk selector for the Stereo ace (7=FrameID, 29=Timestamp)";
        }
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_ACE, "An exception while setting the chunk selector occurred: " << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

std::string PylonROS2StereoAceCamera::setChunkEnable(const bool& enable)
{
    try
    {
        stereo_ace_cam_->ChunkEnable.SetValue(enable);
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_ACE, "An exception while setting chunk enable occurred: " << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

std::string PylonROS2StereoAceCamera::setChunkExposureTime(const float& value)
{
    try
    {
        stereo_ace_cam_->ChunkExposureTime.SetValue(value);
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_ACE, "An exception while setting the chunk exposure time occurred: " << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

// --- Working depth range ----------------------------------------------------

std::string PylonROS2StereoAceCamera::setDepthMin(const double& depth_min)
{
    // The Stereo ace clamps working depth via BslDepthMinDepth/BslDepthMaxDepth
    // (float nodes in meters, range [0, 100]).
    try
    {
        stereo_ace_cam_->BslDepthMinDepth.SetValue(depth_min);
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_ACE, "An exception while setting the minimum depth occurred: " << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

std::string PylonROS2StereoAceCamera::setDepthMax(const double& depth_max)
{
    try
    {
        stereo_ace_cam_->BslDepthMaxDepth.SetValue(depth_max);
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_ACE, "An exception while setting the maximum depth occurred: " << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

double PylonROS2StereoAceCamera::getDepthMin()
{
    try
    {
        if (stereo_ace_cam_->BslDepthMinDepth.IsReadable())
            return stereo_ace_cam_->BslDepthMinDepth.GetValue();
    }
    catch (const GenICam::GenericException&) {}
    return -1.0;
}

double PylonROS2StereoAceCamera::getDepthMax()
{
    try
    {
        if (stereo_ace_cam_->BslDepthMaxDepth.IsReadable())
            return stereo_ace_cam_->BslDepthMaxDepth.GetValue();
    }
    catch (const GenICam::GenericException&) {}
    return -1.0;
}

// --- Feature persistence (pfs) ----------------------------------------------

std::pair<std::string, std::string> PylonROS2StereoAceCamera::getPfs()
{
    std::pair<std::string, std::string> result;
    try
    {
        this->grabbingStopping();
        Pylon::String_t pfs;
        Pylon::CFeaturePersistence::SaveToString(pfs, &stereo_ace_cam_->GetNodeMap());
        result.second = pfs;
        this->grabbingStarting();
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_ACE, "An exception while getting the camera configuration as pfs occurred: " << e.GetDescription());
        result.first = e.GetDescription();
        return result;
    }
    result.first = "done";
    return result;
}

std::string PylonROS2StereoAceCamera::savePfs(const std::string& fileName)
{
    try
    {
        this->grabbingStopping();
        Pylon::CFeaturePersistence::Save(fileName.c_str(), &stereo_ace_cam_->GetNodeMap());
        this->grabbingStarting();
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_ACE, "An exception while saving the pfs file occurred: " << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

std::string PylonROS2StereoAceCamera::loadPfs(const std::string& fileName)
{
    try
    {
        this->grabbingStopping();
        Pylon::CFeaturePersistence::Load(fileName.c_str(), &stereo_ace_cam_->GetNodeMap(), true);
        this->grabbingStarting();
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_ACE, "An exception while loading the pfs file occurred: " << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

// --- PTP --------------------------------------------------------------------

std::string PylonROS2StereoAceCamera::enablePTP(const bool& value)
{
    try
    {
        stereo_ace_cam_->PtpEnable.SetValue(value);
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_ACE, "An exception while enabling/disabling PTP occurred: " << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

std::string PylonROS2StereoAceCamera::getPTPStatus(int64_t& offset_from_master, std::string& status, std::string& servo_status)
{
    offset_from_master = -1;
    status = "no_status";
    servo_status = "no_servo_status";
    try
    {
        stereo_ace_cam_->PtpDataSetLatch.Execute();
        offset_from_master = stereo_ace_cam_->PtpOffsetFromMaster.GetValue();
        status = stereo_ace_cam_->PtpStatus.ToString().c_str();
        servo_status = stereo_ace_cam_->PtpServoStatus.ToString().c_str();
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_ACE, "An exception while getting the PTP status occurred: " << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

// --- Buffers / statistics ---------------------------------------------------

std::string PylonROS2StereoAceCamera::setMaxNumBuffer(const int& size)
{
    try
    {
        this->grabbingStopping();
        stereo_ace_cam_->MaxNumBuffer.SetValue(size);
        this->grabbingStarting();
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_ACE, "An exception while setting the maximum number of buffers occurred: " << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

std::string PylonROS2StereoAceCamera::setOutputQueueSize(const int& size)
{
    try
    {
        const int max_num_buffer = static_cast<int>(stereo_ace_cam_->MaxNumBuffer.GetValue());
        if (size < 0 || size > max_num_buffer)
            return "The requested output queue size is outside the limits of : 0-" + std::to_string(max_num_buffer);
        stereo_ace_cam_->OutputQueueSize.SetValue(size);
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_ACE, "An exception while setting the output queue size occurred: " << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

int PylonROS2StereoAceCamera::getMaxNumBuffer()
{
    try
    {
        return static_cast<int>(stereo_ace_cam_->MaxNumBuffer.GetValue());
    }
    catch (const GenICam::GenericException&)
    {
        return -2;
    }
}

int PylonROS2StereoAceCamera::readStatisticCounter(const char* node_name)
{
    try
    {
        GenApi::CIntegerPtr node(stereo_ace_cam_->GetNodeMap().GetNode(node_name));
        if (node.IsValid() && GenApi::IsReadable(node))
            return static_cast<int>(node->GetValue());
    }
    catch (const GenICam::GenericException&) {}
    return -1;
}

int PylonROS2StereoAceCamera::getStatisticTotalBufferCount()       { return readStatisticCounter("Statistic_Total_Buffer_Count"); }
int PylonROS2StereoAceCamera::getStatisticFailedBufferCount()      { return readStatisticCounter("Statistic_Failed_Buffer_Count"); }
int PylonROS2StereoAceCamera::getStatisticBufferUnderrunCount()    { return readStatisticCounter("Statistic_Buffer_Underrun_Count"); }
int PylonROS2StereoAceCamera::getStatisticFailedPacketCount()      { return readStatisticCounter("Statistic_Failed_Packet_Count"); }
int PylonROS2StereoAceCamera::getStatisticResendRequestCount()     { return readStatisticCounter("Statistic_Resend_Request_Count"); }
int PylonROS2StereoAceCamera::getStatisticMissedFrameCount()       { return -1; } // not exposed by the Stereo ace
int PylonROS2StereoAceCamera::getStatisticResynchronizationCount() { return -1; } // not exposed by the Stereo ace

// --- Stubs for features the Stereo ace SDK does not expose -------------------

std::string PylonROS2StereoAceCamera::setLineDebouncerTime(const float& /*value*/) { return "Feature not available for this camera type"; }
std::string PylonROS2StereoAceCamera::gammaEnable(const bool& /*enable*/) { return "Feature not available for this camera type"; }
std::string PylonROS2StereoAceCamera::setUserSetSelector(const int& /*set*/) { return "Feature not available for this camera type"; }
std::string PylonROS2StereoAceCamera::saveUserSet() { return "Feature not available for this camera type"; }
std::string PylonROS2StereoAceCamera::loadUserSet() { return "Feature not available for this camera type"; }
std::string PylonROS2StereoAceCamera::setUserSetDefaultSelector(const int& /*set*/) { return "Feature not available for this camera type"; }
std::string PylonROS2StereoAceCamera::setTimerSelector(const int& /*selector*/) { return "Feature not available for this camera type"; }
std::string PylonROS2StereoAceCamera::setTimerTriggerSource(const int& /*source*/) { return "Feature not available for this camera type"; }
std::string PylonROS2StereoAceCamera::setTimerDuration(const float& /*duration*/) { return "Feature not available for this camera type"; }
std::string PylonROS2StereoAceCamera::setMaxTransferSize(const int& /*maxTransferSize*/) { return "Feature not available for this camera type"; }
std::string PylonROS2StereoAceCamera::setPTPPriority(const int& /*value*/) { return "Feature not available for this camera type"; }
std::string PylonROS2StereoAceCamera::setPTPProfile(const int& /*value*/) { return "Feature not available for this camera type"; }
std::string PylonROS2StereoAceCamera::setPTPNetworkMode(const int& /*value*/) { return "Feature not available for this camera type"; }
std::string PylonROS2StereoAceCamera::setPTPUCPortAddressIndex(const int& /*value*/) { return "Feature not available for this camera type"; }
std::string PylonROS2StereoAceCamera::setPTPUCPortAddress(const int& /*value*/) { return "Feature not available for this camera type"; }
std::string PylonROS2StereoAceCamera::enablePTPManagementProtocol(const bool& /*value*/) { return "Feature not available for this camera type"; }
std::string PylonROS2StereoAceCamera::enablePTPTwoStepOperation(const bool& /*value*/) { return "Feature not available for this camera type"; }
std::string PylonROS2StereoAceCamera::setPeriodicSignalPeriod(const float& /*value*/) { return "Feature not available for this camera type"; }
std::string PylonROS2StereoAceCamera::setPeriodicSignalDelay(const float& /*value*/) { return "Feature not available for this camera type"; }
std::string PylonROS2StereoAceCamera::setSyncFreeRunTimerStartTimeLow(const int& /*value*/) { return "Feature not available for this camera type"; }
std::string PylonROS2StereoAceCamera::setSyncFreeRunTimerStartTimeHigh(const int& /*value*/) { return "Feature not available for this camera type"; }
std::string PylonROS2StereoAceCamera::setSyncFreeRunTimerTriggerRateAbs(const float& /*value*/) { return "Feature not available for this camera type"; }
std::string PylonROS2StereoAceCamera::enableSyncFreeRunTimer(const bool& /*value*/) { return "Feature not available for this camera type"; }
std::string PylonROS2StereoAceCamera::updateSyncFreeRunTimer() { return "Feature not available for this camera type"; }
std::string PylonROS2StereoAceCamera::setActionTriggerConfiguration(const int& /*action_device_key*/, const int& /*action_group_key*/, const unsigned int& /*action_group_mask*/,
                                                                    const int& /*registration_mode*/, const int& /*cleanup*/) { return "Feature not available for this camera type"; }
std::string PylonROS2StereoAceCamera::issueActionCommand(const int& /*device_key*/, const int& /*group_key*/, const unsigned int& /*group_mask*/, const std::string& /*broadcast_address*/) { return "Feature not available for this camera type"; }
std::string PylonROS2StereoAceCamera::issueScheduledActionCommand(const int& /*device_key*/, const int& /*group_key*/, const unsigned int& /*group_mask*/, const int64_t& /*action_time_ns_from_current_timestamp*/, const std::string& /*broadcast_address*/) { return "Feature not available for this camera type"; }
bool PylonROS2StereoAceCamera::setAutoflash(const std::map<int, bool> /*flash_on_lines*/) { return false; }
bool PylonROS2StereoAceCamera::setUserOutput(const int& /*output_id*/, const bool& /*value*/) { return false; }

// --- Helpers ----------------------------------------------------------------

void PylonROS2StereoAceCamera::relearnImageDimensions()
{
    Pylon::CGrabResultPtr grab_result;
    if (this->grab3D(grab_result) && grab_result.IsValid())
    {
        for (int idx = 0; idx < (int)grab_result->GetDataComponentCount(); ++idx)
        {
            const auto component = grab_result->GetDataComponent(idx);
            if (component.GetComponentType() == Pylon::ComponentType_IntensityCombined_STA)
            {
                img_cols_ = static_cast<size_t>(component.GetWidth());
                img_rows_ = static_cast<size_t>(component.GetHeight()) / 2;
                img_size_byte_ = img_cols_ * img_rows_ * imagePixelDepth();
                break;
            }
        }
    }
}

} // namespace pylon_ros2_camera
