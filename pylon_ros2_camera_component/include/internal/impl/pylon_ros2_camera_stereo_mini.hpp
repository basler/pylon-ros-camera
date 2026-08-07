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

#include <cmath>
#include <string>
#include <vector>
#include <algorithm>

#include "internal/impl/pylon_ros2_camera_3d.hpp"

#include <pylon/StereoMiniInstantCamera.h>


namespace pylon_ros2_camera
{

namespace
{
    static const rclcpp::Logger LOGGER_STEREO_MINI = rclcpp::get_logger("basler.pylon.ros2.pylon_ros2_stereo_mini_camera");

    // Stereo mini working depth range fallback (millimeters) used if DepthMin/
    // DepthMax cannot be read from the camera. The connected STM-951mg reports
    // DepthMin=0, DepthMax=16000.
    constexpr int s_stm_fallback_depth_min = 0;
    constexpr int s_stm_fallback_depth_max = 16000;
}

/**
 * Basler Stereo mini integration.
 *
 * The Stereo mini is a stereo 3D camera. Like the blaze it delivers, in a single
 * grab result, an organized point cloud (Range, Coord3D_ABC32f, in millimeters),
 * an intensity image and a confidence image. Differences with the blaze that are
 * relevant here (verified on an STM-951mg):
 *   - components are selected per Source: Source1 = left IR, Source2 = right IR,
 *     Source3 = color; this integration publishes the Source3 color intensity and
 *     the left/right IR intensity images from Source1/Source2,
 *   - color intensity is RGBa8; left/right IR intensity is Mono8 at 848x480,
 *   - the confidence component is Confidence8 (8-bit) instead of Confidence16,
 *   - ChunkModeActive must be disabled to receive a grab result,
 *   - acquisition is continuous (free-run) rather than software triggered.
 *
 * The heavy conversions (point cloud, depth maps, intensity/confidence images,
 * camera info) are provided by the generic 3D profile (PylonROS23DCamera) and
 * shared with other 3D cameras.
 */
class PylonROS2StereoMiniCamera : public PylonROS23DCamera
{
public:
    explicit PylonROS2StereoMiniCamera(Pylon::IPylonDevice* device);
    virtual ~PylonROS2StereoMiniCamera();

    virtual bool registerCameraConfiguration() override;
    virtual bool openCamera() override;
    virtual bool applyCamSpecificStartupSettings(const PylonROS2CameraParameter& parameters) override;

    virtual bool startGrabbing(const PylonROS2CameraParameter& parameters) override;

    virtual bool grab3D(sensor_msgs::msg::PointCloud2& cloud_msg,
                        sensor_msgs::msg::Image& intensity_map_msg,
                        sensor_msgs::msg::Image& depth_map_msg,
                        sensor_msgs::msg::Image& depth_map_color_msg,
                        sensor_msgs::msg::Image& confidence_map_msg) override;
            bool grab3D(Pylon::CGrabResultPtr& grab_result);

    // Left and right IR intensity images (Mono8, 848x480).
    virtual bool hasExtraIntensityImages() const override { return true; }
    virtual const sensor_msgs::msg::Image& extraIntensityLeft() const override { return intensity_ir_left_msg_; }
    virtual const sensor_msgs::msg::Image& extraIntensityRight() const override { return intensity_ir_right_msg_; }

    virtual int imagePixelDepth() const override;

    // Override setExposure to use stereo_mini_cam_->ExposureTime directly.
    // Using the inherited cam_->ExposureTime while stereo_mini_cam_ is grabbing
    // causes the grab queue to flush and RetrieveResult() to block indefinitely.
    virtual bool setExposure(const float& target_exposure, float& reached_exposure) override;

    virtual std::string setAcquisitionFrameRate(const float& framerate) override;
    virtual std::string enableAcquisitionFrameRate(const bool& enable) override;
    virtual std::string setTriggerSelector(const int& mode) override;
    virtual std::string setTriggerSource(const int& source) override;
    virtual std::string setTriggerMode(const bool& value) override;
    virtual std::string executeSoftwareTrigger() override;
    virtual std::string setDepthMin(const double& depth_min) override;
    virtual std::string setDepthMax(const double& depth_max) override;
    // Selects a BslDepthPreset by index into the entries the camera reports as available at runtime.
    virtual std::string setOperatingMode(const int& mode) override;

    // Overrides for features the stereo mini hardware actually supports. The
    // inherited base implementations use the (never-opened) cam_ device and
    // would throw "Camera is not open"; these use stereo_mini_cam_ instead.
    virtual bool setGain(const float& target_gain, float& reached_gain) override;
    virtual bool setGamma(const float& target_gamma, float& reached_gamma) override;
    virtual bool setBrightness(const int& target_brightness,
                               const float& current_brightness,
                               const bool& exposure_auto,
                               const bool& gain_auto) override;
    virtual std::string setWhiteBalance(const double& redValue, const double& greenValue, const double& blueValue) override;
    virtual std::string setBalanceWhiteAuto(const int& mode) override;
    virtual std::string setAcquisitionFrameCount(const int& frameCount) override;
    virtual std::string setTriggerDelay(const float& delayValue) override;
    virtual std::string triggerDeviceReset() override;

    // Buffer / statistics services retargeted to stereo_mini_cam_. The inherited
    // base versions act on the never-opened cam_ (wrong object; setMaxNumBuffer
    // even stops/restarts nothing useful, and the statistic getters read cam_'s
    // stream grabber outside a try block => potential uncaught exception).
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

    // Stubs for features the stereo mini SDK does not expose. The inherited
    // base implementations use the never-opened cam_ device and would report a
    // confusing "Camera is not open"; these return a clear message instead.
    virtual std::string setTriggerActivation(const int& value) override;
    virtual std::string setLineSelector(const int& value) override;
    virtual std::string setLineMode(const int& value) override;
    virtual std::string setLineSource(const int& value) override;
    virtual std::string setLineInverter(const bool& value) override;
    virtual std::string setLineDebouncerTime(const float& value) override;
    virtual std::string setDeviceLinkThroughputLimitMode(const bool& turnOn) override;
    virtual std::string setDeviceLinkThroughputLimit(const int& limit) override;
    virtual std::string gammaEnable(const bool& enable) override;

    // User set (configuration set) services - no UserSetSelector node.
    virtual std::string setUserSetSelector(const int& set) override;
    virtual std::string saveUserSet() override;
    virtual std::string loadUserSet() override;
    virtual std::string setUserSetDefaultSelector(const int& set) override;

    // Feature persistence (pfs) services - base uses the never-opened cam_.
    virtual std::pair<std::string, std::string> getPfs() override;
    virtual std::string savePfs(const std::string& fileName) override;
    virtual std::string loadPfs(const std::string& fileName) override;

    // ace-style chunk services - the stereo mini uses a different chunk model.
    virtual std::string setChunkModeActive(const bool& enable) override;
    virtual std::string setChunkSelector(const int& value) override;
    virtual std::string setChunkEnable(const bool& enable) override;
    virtual std::string setChunkExposureTime(const float& value) override;

    // Timer services - no Timer* nodes.
    virtual std::string setTimerSelector(const int& selector) override;
    virtual std::string setTimerTriggerSource(const int& source) override;
    virtual std::string setTimerDuration(const float& duration) override;

    // USB transfer tuning - no MaxTransferSize node.
    virtual std::string setMaxTransferSize(const int& maxTransferSize) override;

    // PTP / IEEE 1588 services - no PTP nodes.
    virtual std::string setPTPPriority(const int& value) override;
    virtual std::string setPTPProfile(const int& value) override;
    virtual std::string setPTPNetworkMode(const int& value) override;
    virtual std::string setPTPUCPortAddressIndex(const int& value) override;
    virtual std::string setPTPUCPortAddress(const int& value) override;
    virtual std::string enablePTPManagementProtocol(const bool& value) override;
    virtual std::string enablePTPTwoStepOperation(const bool& value) override;
    virtual std::string enablePTP(const bool& value) override;
    virtual std::string getPTPStatus(int64_t& offset_from_master, std::string& status, std::string& servo_status) override;

    // Periodic signal / synchronous free run services - no such nodes.
    virtual std::string setPeriodicSignalPeriod(const float& value) override;
    virtual std::string setPeriodicSignalDelay(const float& value) override;
    virtual std::string setSyncFreeRunTimerStartTimeLow(const int& value) override;
    virtual std::string setSyncFreeRunTimerStartTimeHigh(const int& value) override;
    virtual std::string setSyncFreeRunTimerTriggerRateAbs(const float& value) override;
    virtual std::string enableSyncFreeRunTimer(const bool& value) override;
    virtual std::string updateSyncFreeRunTimer() override;

    // GigE action command services - no action command nodes.
    virtual std::string setActionTriggerConfiguration(const int& action_device_key, const int& action_group_key, const unsigned int& action_group_mask,
                                                      const int& registration_mode, const int& cleanup) override;
    virtual std::string issueActionCommand(const int& device_key, const int& group_key, const unsigned int& group_mask, const std::string& broadcast_address) override;
    virtual std::string issueScheduledActionCommand(const int& device_key, const int& group_key, const unsigned int& group_mask, const int64_t& action_time_ns_from_current_timestamp, const std::string& broadcast_address) override;

    // bool-returning stubs the stereo mini SDK does not expose (flash / output
    // lines); return false to signal "not set".
    virtual bool setAutoflash(const std::map<int, bool> flash_on_lines) override;
    virtual bool setUserOutput(const int& output_id, const bool& value) override;

protected:
    // The stereo mini grabs through stereo_mini_cam_; the profile uses this for
    // acquisition start/stop and device-removal detection.
    Pylon::CInstantCamera& activeCamera() const override { return *stereo_mini_cam_; }

public:
    Pylon::CStereoMiniInstantCamera* stereo_mini_cam_;

private:
    // Cached left and right IR intensity images, filled by grab3D().
    sensor_msgs::msg::Image intensity_ir_left_msg_;
    sensor_msgs::msg::Image intensity_ir_right_msg_;
    // SourceIDValue values read at startup; used to identify components.
    uint64_t src_id_left_  = 0;  // Source1 (left IR)
    uint64_t src_id_right_ = 0;  // Source2 (right IR)
    uint64_t src_id_color_ = 0;  // Source3 (color)
};

PylonROS2StereoMiniCamera::PylonROS2StereoMiniCamera(Pylon::IPylonDevice* device) :
    PylonROS23DCamera(device),
    stereo_mini_cam_(new Pylon::CStereoMiniInstantCamera(device))
{
    // Default to keeping only the latest stereo frame; user-changeable via
    // set_grabbing_strategy (takes effect on the next grab (re)start).
    grab_strategy_ = 1;
}

PylonROS2StereoMiniCamera::~PylonROS2StereoMiniCamera()
{
    try
    {
        if (stereo_mini_cam_->IsOpen())
        {
            stereo_mini_cam_->Close();
        }
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_DEBUG_STREAM(LOGGER_STEREO_MINI, "Destructor (Stereo mini): Failed to properly close camera: " << e.GetDescription());
    }

    if (stereo_mini_cam_)
    {
        // stereo_mini_cam_ and the inherited base cam_ wrap the same IPylonDevice.
        // Detach the device from the base cam_ (without destroying it) before
        // stereo_mini_cam_ destroys it, to avoid a double DestroyDevice().
        this->detachBaseDevice();

        delete stereo_mini_cam_;
        stereo_mini_cam_ = nullptr;
    }
}

bool PylonROS2StereoMiniCamera::registerCameraConfiguration()
{
    // The Stereo mini streams continuously; no special instant camera
    // configuration is registered (unlike the blaze default configuration).
    return true;
}

bool PylonROS2StereoMiniCamera::openCamera()
{
    try
    {
        stereo_mini_cam_->Open();
        RCLCPP_DEBUG_STREAM(LOGGER_STEREO_MINI, "Connected to camera " << stereo_mini_cam_->GetDeviceInfo().GetFriendlyName());
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_MINI, "An exception occurred while opening camera:" << e.GetDescription());
        return false;
    }

    return true;
}

bool PylonROS2StereoMiniCamera::applyCamSpecificStartupSettings(const PylonROS2CameraParameter& parameters __attribute__((unused)))
{
    try
    {
        RCLCPP_DEBUG_STREAM(LOGGER_STEREO_MINI, "Infos about connected camera:");
        RCLCPP_DEBUG_STREAM(LOGGER_STEREO_MINI, "-> Model name:        " << stereo_mini_cam_->GetDeviceInfo().GetModelName().c_str());
        RCLCPP_DEBUG_STREAM(LOGGER_STEREO_MINI, "-> Serial number:     " << stereo_mini_cam_->GetDeviceInfo().GetSerialNumber().c_str());

        // Enable the color intensity from Source3.
        stereo_mini_cam_->SourceSelector.FromString("Source3");
        stereo_mini_cam_->ComponentSelector.FromString("Intensity");
        stereo_mini_cam_->ComponentEnable.SetValue(true);

        // Enable the range output as direct XYZ (Coord3D_ABC32f, millimeters).
        // The Range component is enabled by default; selecting the pixel format
        // is sufficient.
        stereo_mini_cam_->SourceSelector.FromString("Source1");
        stereo_mini_cam_->ComponentSelector.FromString("Range");
        stereo_mini_cam_->PixelFormat.FromString("Coord3D_ABC32f");

        // Enable the confidence map.
        stereo_mini_cam_->ComponentSelector.FromString("Confidence");
        stereo_mini_cam_->ComponentEnable.TrySetValue(true);

        // Enable left IR intensity from Source1 (Mono8, 848x480) and read its SourceIDValue.
        stereo_mini_cam_->SourceSelector.FromString("Source1");
        stereo_mini_cam_->ComponentSelector.FromString("Intensity");
        stereo_mini_cam_->ComponentEnable.SetValue(true);
        src_id_left_ = static_cast<uint64_t>(stereo_mini_cam_->SourceIDValue.GetValue());

        // Enable right IR intensity from Source2 (Mono8, 848x480) and read its SourceIDValue.
        stereo_mini_cam_->SourceSelector.FromString("Source2");
        stereo_mini_cam_->ComponentSelector.FromString("Intensity");
        stereo_mini_cam_->ComponentEnable.SetValue(true);
        src_id_right_ = static_cast<uint64_t>(stereo_mini_cam_->SourceIDValue.GetValue());

        // Read the Source3 (color) SourceIDValue for completeness.
        stereo_mini_cam_->SourceSelector.FromString("Source3");
        src_id_color_ = static_cast<uint64_t>(stereo_mini_cam_->SourceIDValue.GetValue());

        // Disabling chunk data is essential to receive a grab result.
        stereo_mini_cam_->ChunkModeActive.SetValue(false);

        RCLCPP_INFO_STREAM(LOGGER_STEREO_MINI, "Stereo mini configured: Range=Coord3D_ABC32f (mm), Intensity=Source3 (color) + Source1/Source2 (IR left/right), Confidence enabled.");
        RCLCPP_DEBUG_STREAM(LOGGER_STEREO_MINI, "Source IDs: left=" << src_id_left_ << " right=" << src_id_right_ << " color=" << src_id_color_);
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_MINI, "An exception occurred while applying specific startup settings:" << e.GetDescription());
        return false;
    }

    return true;
}

bool PylonROS2StereoMiniCamera::startGrabbing(const PylonROS2CameraParameter& parameters)
{
    try
    {
        this->grabbingStarting();

        device_user_id_ = stereo_mini_cam_->GetDeviceInfo().GetUserDefinedName().c_str();
        // The Stereo mini needs more time than a 2D camera to deliver its first
        // (and subsequent) stereo-processed frames. Use a generous grab timeout
        // (at least 5 s), independent of the smaller 2D default.
        grab_timeout_ = std::max(parameters.grab_timeout_, 5000);
        RCLCPP_DEBUG_STREAM_ONCE(LOGGER_STEREO_MINI, "Grab timeout for Stereo mini: " << grab_timeout_);

        // Perform an initial grab to determine the image dimensions and confirm
        // the camera is delivering data.
        // When trigger mode is active no frame arrives until the user issues a
        // trigger, so a blocking startup grab would time out. Instead read the
        // image dimensions from the GenICam Width/Height nodes and defer.
        try
        {
            if (stereo_mini_cam_->TriggerMode.GetValue() ==
                Pylon::StereoMiniCameraParams_Params::TriggerModeEnums::TriggerMode_On)
            {
                GenApi::CIntegerPtr width_node(stereo_mini_cam_->GetNodeMap().GetNode("Width"));
                GenApi::CIntegerPtr height_node(stereo_mini_cam_->GetNodeMap().GetNode("Height"));
                if (width_node.IsValid() && GenApi::IsReadable(width_node) &&
                    height_node.IsValid() && GenApi::IsReadable(height_node))
                {
                    img_cols_ = static_cast<size_t>(width_node->GetValue());
                    img_rows_ = static_cast<size_t>(height_node->GetValue());
                    img_size_byte_ = img_cols_ * img_rows_ * imagePixelDepth();
                }
                RCLCPP_INFO(LOGGER_STEREO_MINI, "Trigger mode active — deferring initial grab; use execute_software_trigger to acquire frames");
                is_ready_ = true;
                return true;
            }
        }
        catch (const GenICam::GenericException& e)
        {
            RCLCPP_WARN_STREAM(LOGGER_STEREO_MINI, "Could not read dimensions in trigger mode: " << e.GetDescription());
            is_ready_ = true;
            return true;
        }

        Pylon::CGrabResultPtr grab_result;
        if (this->grab3D(grab_result) && grab_result.IsValid())
        {
            const auto range_list = grab_result->GetDataComponent(Pylon::ComponentType_Range);
            if (!range_list.empty())
            {
                img_cols_ = static_cast<size_t>(range_list[0].GetWidth());
                img_rows_ = static_cast<size_t>(range_list[0].GetHeight());
                img_size_byte_ = img_cols_ * img_rows_ * imagePixelDepth();
                is_ready_ = true;
            }
            else
            {
                RCLCPP_ERROR(LOGGER_STEREO_MINI, "Initial grab returned no Range component");
            }
        }
        else
        {
            RCLCPP_ERROR(LOGGER_STEREO_MINI, "PylonROS2StereoMiniCamera not ready because the result of the initial grab is invalid");
        }
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_MINI, "An exception occurred while starting image grabbing:" << e.GetDescription());
        return false;
    }

    return true;
}

bool PylonROS2StereoMiniCamera::grab3D(Pylon::CGrabResultPtr& grab_result)
{
    if (!stereo_mini_cam_->IsGrabbing())
    {
        return false;
    }

    try
    {
        stereo_mini_cam_->RetrieveResult(grab_timeout_, grab_result, Pylon::TimeoutHandling_ThrowException);
    }
    catch (const GenICam::GenericException& e)
    {
        if (stereo_mini_cam_->IsCameraDeviceRemoved())
        {
            RCLCPP_ERROR(LOGGER_STEREO_MINI, "Lost connection to the camera...");
        }
        else
        {
            RCLCPP_ERROR_STREAM(LOGGER_STEREO_MINI, "An image grabbing exception in pylon camera occurred: " << e.GetDescription());
        }
        return false;
    }
    catch (...)
    {
        RCLCPP_ERROR(LOGGER_STEREO_MINI, "An unspecified image grabbing exception occurred");
        return false;
    }

    if (!grab_result->GrabSucceeded())
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_MINI, "Error: " << grab_result->GetErrorCode() << " " << grab_result->GetErrorDescription());
        return false;
    }

    return true;
}

bool PylonROS2StereoMiniCamera::grab3D(sensor_msgs::msg::PointCloud2& cloud_msg,
                                       sensor_msgs::msg::Image& intensity_map_msg,
                                       sensor_msgs::msg::Image& depth_map_msg,
                                       sensor_msgs::msg::Image& depth_map_color_msg,
                                       sensor_msgs::msg::Image& confidence_map_msg)
{
    Pylon::CGrabResultPtr ptr_grab_result;
    if (!this->grab3D(ptr_grab_result))
    {
        RCLCPP_ERROR(LOGGER_STEREO_MINI, "Grabbing with Stereo mini failed");
        return false;
    }

    // Locate the components (all delivered in a single grab result).
    const auto range_list = ptr_grab_result->GetDataComponent(Pylon::ComponentType_Range);
    const auto intensity_list = ptr_grab_result->GetDataComponent(Pylon::ComponentType_Intensity);
    const auto confidence_list = ptr_grab_result->GetDataComponent(Pylon::ComponentType_Confidence);

    if (range_list.empty())
    {
        RCLCPP_ERROR(LOGGER_STEREO_MINI, "Grab result contains no Range component");
        return false;
    }

    const auto range_component = range_list[0];
    const bool has_intensity = !intensity_list.empty();
    const bool has_confidence = !confidence_list.empty();

    const int width = range_component.GetWidth();
    const int height = range_component.GetHeight();

    // Point cloud (colored from the intensity component when available).
    this->buildPointCloud(range_component, has_intensity ? &intensity_list[0] : nullptr, cloud_msg);

    // Depth maps: the Z coordinate is already in millimeters (coordinate scale 1.0).
    int min_depth = 0, max_depth = 0;
    this->readDepthRange(stereo_mini_cam_->GetNodeMap(), s_stm_fallback_depth_min, s_stm_fallback_depth_max, min_depth, max_depth);

    std::vector<uint16_t> depth_buffer(static_cast<size_t>(width) * height);
    this->calculateDepthMap(range_component, 1.0, min_depth, max_depth, depth_buffer.data());
    {
        cv_bridge::CvImage depth_cv_img;
        depth_cv_img.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        depth_cv_img.image = cv::Mat(height, width, CV_16UC1, depth_buffer.data());
        const auto msg = depth_cv_img.toImageMsg();
        depth_map_msg.header = msg->header;
        depth_map_msg.height = msg->height;
        depth_map_msg.width = msg->width;
        depth_map_msg.encoding = msg->encoding;
        depth_map_msg.is_bigendian = msg->is_bigendian;
        depth_map_msg.step = msg->step;
        depth_map_msg.data = msg->data;
    }

    std::vector<BGR> depth_color_buffer(static_cast<size_t>(width) * height);
    this->calculateDepthMapColor(range_component, min_depth, max_depth, depth_color_buffer.data());
    {
        cv_bridge::CvImage depth_color_cv_img;
        depth_color_cv_img.encoding = sensor_msgs::image_encodings::BGR8;
        depth_color_cv_img.image = cv::Mat(height, width, CV_8UC3, depth_color_buffer.data());
        const auto msg = depth_color_cv_img.toImageMsg();
        depth_map_color_msg.header = msg->header;
        depth_map_color_msg.height = msg->height;
        depth_map_color_msg.width = msg->width;
        depth_map_color_msg.encoding = msg->encoding;
        depth_map_color_msg.is_bigendian = msg->is_bigendian;
        depth_map_color_msg.step = msg->step;
        depth_map_color_msg.data = msg->data;
    }

    // Intensity and confidence images.
    // intensity_list may contain up to 3 components (Source3 color, Source1 IR
    // left, Source2 IR right). Identify each by its sourceId.
    if (has_intensity)
    {
        for (const auto& comp : intensity_list)
        {
            const uint64_t sid = static_cast<uint64_t>(comp.GetSourceId());
            if (sid == src_id_color_)
            {
                this->buildIntensityImage(comp, intensity_map_msg);
            }
            else if (sid == src_id_left_)
            {
                this->buildIntensityImage(comp, intensity_ir_left_msg_);
            }
            else if (sid == src_id_right_)
            {
                this->buildIntensityImage(comp, intensity_ir_right_msg_);
            }
        }
    }
    if (has_confidence)
    {
        this->buildConfidenceImage(confidence_list[0], confidence_map_msg);
    }

    return true;
}

int PylonROS2StereoMiniCamera::imagePixelDepth() const
{
    // The published color intensity image is RGBa8 (4 bytes per pixel).
    return 4;
}

bool PylonROS2StereoMiniCamera::setExposure(const float& target_exposure, float& reached_exposure)
{
    // SourceSelector must be Source3 (color sensor) before changing ExposureTime.
    // Changing the stereo-pair sources (Source1/Source2) while grabbing blocks
    // RetrieveResult() indefinitely — confirmed with Basler support.
    // SourceSelector is intentionally left on Source3 after this call.
    // TODO: expose SourceSelector as a user-accessible service parameter.
    try
    {
        stereo_mini_cam_->SourceSelector.SetValue(
            Pylon::StereoMiniCameraParams_Params::SourceSelectorEnums::SourceSelector_Source3);

        stereo_mini_cam_->ExposureAuto.TrySetValue(
            Pylon::StereoMiniCameraParams_Params::ExposureAutoEnums::ExposureAuto_Off);

        float exposure_to_set = target_exposure;
        const float min_exp = static_cast<float>(stereo_mini_cam_->ExposureTime.GetMin());
        const float max_exp = static_cast<float>(stereo_mini_cam_->ExposureTime.GetMax());

        if (exposure_to_set < min_exp)
        {
            RCLCPP_WARN_STREAM(LOGGER_STEREO_MINI, "Desired exposure (" << exposure_to_set
                << ") unreachable! Setting to lower limit: " << min_exp);
            exposure_to_set = min_exp;
        }
        else if (exposure_to_set > max_exp)
        {
            RCLCPP_WARN_STREAM(LOGGER_STEREO_MINI, "Desired exposure (" << exposure_to_set
                << ") unreachable! Setting to upper limit: " << max_exp);
            exposure_to_set = max_exp;
        }

        stereo_mini_cam_->ExposureTime.SetValue(exposure_to_set);
        reached_exposure = static_cast<float>(stereo_mini_cam_->ExposureTime.GetValue());
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_MINI, "An exception while setting target exposure to "
            << target_exposure << " occurred: " << e.GetDescription());
        reached_exposure = target_exposure;
        return false;
    }
    return true;
}

std::string PylonROS2StereoMiniCamera::setAcquisitionFrameRate(const float& framerate)
{
    try
    {
        if (stereo_mini_cam_->AcquisitionFrameRateEnable.GetValue())
        {
            stereo_mini_cam_->AcquisitionFrameRate.SetValue(framerate);
            RCLCPP_DEBUG_STREAM(LOGGER_STEREO_MINI, "Acquisition frame rate set to " << framerate);
        }
        else
        {
            RCLCPP_ERROR_STREAM(LOGGER_STEREO_MINI, "To change acquisition frame rate, it must first be enabled");
            return "To change acquisition frame rate, it must first be enabled";
        }
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_MINI, "An exception while changing the acquisition frame rate occurred: " << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

std::string PylonROS2StereoMiniCamera::enableAcquisitionFrameRate(const bool& enable)
{
    try
    {
        stereo_mini_cam_->AcquisitionFrameRateEnable.SetValue(enable);
        RCLCPP_DEBUG_STREAM(LOGGER_STEREO_MINI, "Acquisition frame rate " << (enable ? "enabled" : "disabled"));
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_MINI, "An exception while enabling/disabling acquisition frame rate occurred: " << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

std::string PylonROS2StereoMiniCamera::setTriggerSelector(const int& mode)
{
    try
    {
        if (GenApi::IsAvailable(stereo_mini_cam_->TriggerSelector))
        {
            switch (mode)
            {
                case 0:
                    stereo_mini_cam_->TriggerSelector.SetValue(
                        Pylon::StereoMiniCameraParams_Params::TriggerSelectorEnums::TriggerSelector_FrameStart);
                    RCLCPP_INFO_STREAM(LOGGER_STEREO_MINI, "Trigger selector: Frame Start");
                    break;
                case 1:
                    stereo_mini_cam_->TriggerSelector.SetValue(
                        Pylon::StereoMiniCameraParams_Params::TriggerSelectorEnums::TriggerSelector_AcquisitionStart);
                    RCLCPP_INFO_STREAM(LOGGER_STEREO_MINI, "Trigger selector: Acquisition Start");
                    break;
                default:
                    return "Error: unknown trigger selector value (0=FrameStart, 1=AcquisitionStart)";
            }
        }
        else
        {
            RCLCPP_ERROR_STREAM(LOGGER_STEREO_MINI, "TriggerSelector not available on this camera");
            return "The connected camera does not support this feature";
        }
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_MINI, "An exception while setting trigger selector occurred: " << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

std::string PylonROS2StereoMiniCamera::setTriggerSource(const int& source)
{
    try
    {
        if (GenApi::IsAvailable(stereo_mini_cam_->TriggerSource))
        {
            switch (source)
            {
                case 0:
                    stereo_mini_cam_->TriggerSource.SetValue(
                        Pylon::StereoMiniCameraParams_Params::TriggerSourceEnums::TriggerSource_Software);
                    RCLCPP_INFO_STREAM(LOGGER_STEREO_MINI, "Trigger source: Software");
                    break;
                case 1:
                    stereo_mini_cam_->TriggerSource.SetValue(
                        Pylon::StereoMiniCameraParams_Params::TriggerSourceEnums::TriggerSource_Line1);
                    RCLCPP_INFO_STREAM(LOGGER_STEREO_MINI, "Trigger source: Line1");
                    break;
                case 2:
                    stereo_mini_cam_->TriggerSource.SetValue(
                        Pylon::StereoMiniCameraParams_Params::TriggerSourceEnums::TriggerSource_Primary);
                    RCLCPP_INFO_STREAM(LOGGER_STEREO_MINI, "Trigger source: Primary");
                    break;
                case 3:
                    stereo_mini_cam_->TriggerSource.SetValue(
                        Pylon::StereoMiniCameraParams_Params::TriggerSourceEnums::TriggerSource_Secondary_synced);
                    RCLCPP_INFO_STREAM(LOGGER_STEREO_MINI, "Trigger source: Secondary synced");
                    break;
                default:
                    return "Error: unknown trigger source (0=Software, 1=Line1, 2=Primary, 3=Secondary_synced)";
            }
        }
        else
        {
            RCLCPP_ERROR_STREAM(LOGGER_STEREO_MINI, "TriggerSource not available on this camera");
            return "The connected camera does not support this feature";
        }
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_MINI, "An exception while setting trigger source occurred: " << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

std::string PylonROS2StereoMiniCamera::setTriggerMode(const bool& value)
{
    try
    {
        if (GenApi::IsAvailable(stereo_mini_cam_->TriggerMode))
        {
            stereo_mini_cam_->TriggerMode.SetValue(
                value ? Pylon::StereoMiniCameraParams_Params::TriggerModeEnums::TriggerMode_On
                      : Pylon::StereoMiniCameraParams_Params::TriggerModeEnums::TriggerMode_Off);
            RCLCPP_INFO_STREAM(LOGGER_STEREO_MINI, "Trigger mode: " << (value ? "On" : "Off"));
        }
        else
        {
            RCLCPP_ERROR_STREAM(LOGGER_STEREO_MINI, "TriggerMode not available on this camera");
            return "The connected camera does not support this feature";
        }
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_MINI, "An exception while setting trigger mode occurred: " << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

std::string PylonROS2StereoMiniCamera::executeSoftwareTrigger()
{
    try
    {
        if (!stereo_mini_cam_->CanWaitForFrameTriggerReady())
        {
            stereo_mini_cam_->ExecuteSoftwareTrigger();
        }
        else if (stereo_mini_cam_->WaitForFrameTriggerReady(grab_timeout_, Pylon::TimeoutHandling_Return))
        {
            stereo_mini_cam_->ExecuteSoftwareTrigger();
        }
        else
        {
            RCLCPP_ERROR(LOGGER_STEREO_MINI, "WaitForFrameTriggerReady timed out, cannot execute software trigger");
            return "Camera not ready to accept software trigger";
        }
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_MINI, "An exception while executing software trigger occurred: " << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

std::string PylonROS2StereoMiniCamera::setDepthMin(const double& depth_min)
{
    try
    {
        // The stereo mini DepthMin node is an integer in mm; round the requested value.
        stereo_mini_cam_->DepthMin.SetValue(static_cast<int64_t>(std::llround(depth_min)));
        RCLCPP_DEBUG_STREAM(LOGGER_STEREO_MINI, "Depth min set to " << depth_min);
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_MINI, "An exception while setting depth min occurred: " << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

std::string PylonROS2StereoMiniCamera::setDepthMax(const double& depth_max)
{
    try
    {
        // The stereo mini DepthMax node is an integer in mm; round the requested value.
        stereo_mini_cam_->DepthMax.SetValue(static_cast<int64_t>(std::llround(depth_max)));
        RCLCPP_DEBUG_STREAM(LOGGER_STEREO_MINI, "Depth max set to " << depth_max);
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_MINI, "An exception while setting depth max occurred: " << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

std::string PylonROS2StereoMiniCamera::setOperatingMode(const int& mode)
{
    // The stereo mini exposes depth tuning through the BslDepthPreset enum. The set of
    // presets depends on model/firmware, so enumerate the available entries at runtime and
    // select by index rather than relying on a fixed enum mapping. BslDepthPreset is locked
    // while grabbing, so stop/start around the write.
    try
    {
        if (!GenApi::IsAvailable(stereo_mini_cam_->BslDepthPreset))
        {
            RCLCPP_ERROR(LOGGER_STEREO_MINI, "BslDepthPreset is not available on this camera");
            return "BslDepthPreset is not available on this camera";
        }

        GenApi::NodeList_t entries;
        stereo_mini_cam_->BslDepthPreset.GetEntries(entries);
        std::vector<std::string> available;
        for (GenApi::NodeList_t::iterator it = entries.begin(); it != entries.end(); ++it)
        {
            if (!GenApi::IsAvailable(*it))
                continue;
            GenApi::CEnumEntryPtr entry(*it);
            if (entry.IsValid())
                available.push_back(std::string(entry->GetSymbolic().c_str()));
        }

        if (available.empty())
        {
            RCLCPP_ERROR(LOGGER_STEREO_MINI, "No BslDepthPreset entries are available on this camera");
            return "No BslDepthPreset entries are available on this camera";
        }

        if (mode < 0 || mode >= static_cast<int>(available.size()))
        {
            std::ostringstream ss;
            ss << "Depth preset index " << mode << " is out of range. Available presets (" << available.size() << "): ";
            for (size_t i = 0; i < available.size(); ++i)
                ss << i << "=" << available[i] << (i + 1 < available.size() ? ", " : "");
            RCLCPP_ERROR_STREAM(LOGGER_STEREO_MINI, ss.str());
            return ss.str();
        }

        this->grabbingStopping();
        stereo_mini_cam_->BslDepthPreset.FromString(available[mode].c_str());
        this->grabbingStarting();
        RCLCPP_DEBUG_STREAM(LOGGER_STEREO_MINI, "Depth preset set to " << available[mode]);
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_MINI, "An exception while setting the depth preset occurred: " << e.GetDescription());
        this->grabbingStarting();
        return e.GetDescription();
    }
    return "done";
}

bool PylonROS2StereoMiniCamera::setGain(const float& target_gain, float& reached_gain)
{
    // Gain is selected by SourceSelector; operate on the color sensor (Source3),
    // consistent with setExposure. target_gain is a fraction [0.0 - 1.0].
    try
    {
        stereo_mini_cam_->SourceSelector.SetValue(
            Pylon::StereoMiniCameraParams_Params::SourceSelectorEnums::SourceSelector_Source3);

        // The color source has no separate GainAuto; its auto-exposure function
        // also owns Gain and keeps the Gain node read-only while active. Disable
        // ExposureAuto before writing a manual gain (mirrors setExposure).
        stereo_mini_cam_->ExposureAuto.TrySetValue(
            Pylon::StereoMiniCameraParams_Params::ExposureAutoEnums::ExposureAuto_Off);

        float truncated_gain = target_gain;
        if (truncated_gain < 0.0f)
        {
            RCLCPP_WARN_STREAM(LOGGER_STEREO_MINI, "Desired gain (" << target_gain
                << ") out of range [0.0 - 1.0]! Setting to lower limit: 0.0");
            truncated_gain = 0.0f;
        }
        else if (truncated_gain > 1.0f)
        {
            RCLCPP_WARN_STREAM(LOGGER_STEREO_MINI, "Desired gain (" << target_gain
                << ") out of range [0.0 - 1.0]! Setting to upper limit: 1.0");
            truncated_gain = 1.0f;
        }

        const float min_gain = static_cast<float>(stereo_mini_cam_->Gain.GetMin());
        const float max_gain = static_cast<float>(stereo_mini_cam_->Gain.GetMax());
        const float gain_to_set = min_gain + truncated_gain * (max_gain - min_gain);
        stereo_mini_cam_->Gain.SetValue(gain_to_set);

        const float reached_abs = static_cast<float>(stereo_mini_cam_->Gain.GetValue());
        reached_gain = (max_gain > min_gain) ? (reached_abs - min_gain) / (max_gain - min_gain) : 0.0f;
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_MINI, "An exception while setting target gain to "
            << target_gain << " occurred: " << e.GetDescription());
        return false;
    }
    return true;
}

bool PylonROS2StereoMiniCamera::setGamma(const float& target_gamma, float& reached_gamma)
{
    // Gamma is selected by SourceSelector; operate on the color sensor (Source3).
    try
    {
        stereo_mini_cam_->SourceSelector.SetValue(
            Pylon::StereoMiniCameraParams_Params::SourceSelectorEnums::SourceSelector_Source3);

        float gamma_to_set = target_gamma;
        const float min_gamma = static_cast<float>(stereo_mini_cam_->Gamma.GetMin());
        const float max_gamma = static_cast<float>(stereo_mini_cam_->Gamma.GetMax());
        if (gamma_to_set < min_gamma)
        {
            RCLCPP_WARN_STREAM(LOGGER_STEREO_MINI, "Desired gamma (" << target_gamma
                << ") unreachable! Setting to lower limit: " << min_gamma);
            gamma_to_set = min_gamma;
        }
        else if (gamma_to_set > max_gamma)
        {
            RCLCPP_WARN_STREAM(LOGGER_STEREO_MINI, "Desired gamma (" << target_gamma
                << ") unreachable! Setting to upper limit: " << max_gamma);
            gamma_to_set = max_gamma;
        }

        stereo_mini_cam_->Gamma.SetValue(gamma_to_set);
        reached_gamma = static_cast<float>(stereo_mini_cam_->Gamma.GetValue());
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_MINI, "An exception while setting target gamma to "
            << target_gamma << " occurred: " << e.GetDescription());
        return false;
    }
    return true;
}

bool PylonROS2StereoMiniCamera::setBrightness(const int& target_brightness,
                                              const float& current_brightness __attribute__((unused)),
                                              const bool& exposure_auto,
                                              const bool& gain_auto __attribute__((unused)))
{
    // The stereo mini has no auto-brightness search. It exposes a direct analog
    // BslBrightness control (selected by SourceSelector). target_brightness
    // [1..255] is mapped linearly onto the BslBrightness range.
    try
    {
        stereo_mini_cam_->SourceSelector.SetValue(
            Pylon::StereoMiniCameraParams_Params::SourceSelectorEnums::SourceSelector_Source3);

        if (exposure_auto)
        {
            stereo_mini_cam_->ExposureAuto.TrySetValue(
                Pylon::StereoMiniCameraParams_Params::ExposureAutoEnums::ExposureAuto_Continuous);
        }

        const float clamped = static_cast<float>(std::min(255, std::max(1, target_brightness)));
        const float min_b = static_cast<float>(stereo_mini_cam_->BslBrightness.GetMin());
        const float max_b = static_cast<float>(stereo_mini_cam_->BslBrightness.GetMax());
        const float value = min_b + (clamped - 1.0f) / 254.0f * (max_b - min_b);
        stereo_mini_cam_->BslBrightness.SetValue(value);
        RCLCPP_DEBUG_STREAM(LOGGER_STEREO_MINI, "BslBrightness set to " << value
            << " (from target brightness " << target_brightness << ")");
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_MINI, "An exception while setting brightness occurred: " << e.GetDescription());
        return false;
    }
    return true;
}

std::string PylonROS2StereoMiniCamera::setWhiteBalance(const double& redValue, const double& greenValue, const double& blueValue)
{
    // BalanceRatio is applied per channel via the (runtime) BalanceRatioSelector
    // node, which the typed StereoMiniCameraParams header does not expose; access
    // it generically. Requires the color sensor source (Source3).
    try
    {
        stereo_mini_cam_->SourceSelector.SetValue(
            Pylon::StereoMiniCameraParams_Params::SourceSelectorEnums::SourceSelector_Source3);

        GenApi::INodeMap& node_map = stereo_mini_cam_->GetNodeMap();
        GenApi::CEnumerationPtr wb_auto(node_map.GetNode("BalanceWhiteAuto"));
        if (wb_auto.IsValid() && GenApi::IsWritable(wb_auto))
        {
            wb_auto->FromString("Off");
        }

        GenApi::CEnumerationPtr selector(node_map.GetNode("BalanceRatioSelector"));
        GenApi::CFloatPtr ratio(node_map.GetNode("BalanceRatio"));
        if (!ratio.IsValid() || !GenApi::IsWritable(ratio))
        {
            RCLCPP_ERROR_STREAM(LOGGER_STEREO_MINI, "BalanceRatio not writable on this camera");
            return "Feature not available for this camera type";
        }

        if (selector.IsValid() && GenApi::IsWritable(selector))
        {
            selector->FromString("Red");
            ratio->SetValue(redValue);
            selector->FromString("Green");
            ratio->SetValue(greenValue);
            selector->FromString("Blue");
            ratio->SetValue(blueValue);
        }
        else
        {
            // No per-channel selector: apply a single ratio (red channel value).
            ratio->SetValue(redValue);
        }
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_MINI, "An exception while setting the white balance occurred: " << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

std::string PylonROS2StereoMiniCamera::setBalanceWhiteAuto(const int& mode)
{
    // The typed BalanceWhiteAuto enum only carries a placeholder value; use the
    // runtime enumeration node so real Off/Once/Continuous entries can be set.
    try
    {
        stereo_mini_cam_->SourceSelector.SetValue(
            Pylon::StereoMiniCameraParams_Params::SourceSelectorEnums::SourceSelector_Source3);

        GenApi::CEnumerationPtr wb_auto(stereo_mini_cam_->GetNodeMap().GetNode("BalanceWhiteAuto"));
        if (!wb_auto.IsValid() || !GenApi::IsWritable(wb_auto))
        {
            RCLCPP_ERROR_STREAM(LOGGER_STEREO_MINI, "BalanceWhiteAuto not writable on this camera");
            return "Feature not available for this camera type";
        }

        switch (mode)
        {
            case 0: wb_auto->FromString("Off"); break;
            case 1: wb_auto->FromString("Once"); break;
            case 2: wb_auto->FromString("Continuous"); break;
            default: return "Error: unknown value (0=Off, 1=Once, 2=Continuous)";
        }
        RCLCPP_DEBUG_STREAM(LOGGER_STEREO_MINI, "Balance white auto set to mode " << mode);
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_MINI, "An exception while changing the balance white auto occurred: " << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

std::string PylonROS2StereoMiniCamera::setAcquisitionFrameCount(const int& frameCount)
{
    try
    {
        // AcquisitionFrameCount is locked while grabbing; stop and restart around the change.
        this->grabbingStopping();
        stereo_mini_cam_->AcquisitionFrameCount.SetValue(frameCount);
        this->grabbingStarting();
        RCLCPP_DEBUG_STREAM(LOGGER_STEREO_MINI, "Acquisition frame count set to " << frameCount);
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_MINI, "An exception while setting the acquisition frame count occurred: " << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

// --- Stubs for features the stereo mini SDK does not expose -----------------

std::string PylonROS2StereoMiniCamera::setTriggerActivation(const int& /*value*/)
{
    return "Feature not available for this camera type";
}

std::string PylonROS2StereoMiniCamera::setLineSelector(const int& /*value*/)
{
    return "Feature not available for this camera type";
}

std::string PylonROS2StereoMiniCamera::setLineMode(const int& /*value*/)
{
    return "Feature not available for this camera type";
}

std::string PylonROS2StereoMiniCamera::setLineSource(const int& /*value*/)
{
    return "Feature not available for this camera type";
}

std::string PylonROS2StereoMiniCamera::setLineInverter(const bool& /*value*/)
{
    return "Feature not available for this camera type";
}

std::string PylonROS2StereoMiniCamera::setLineDebouncerTime(const float& /*value*/)
{
    return "Feature not available for this camera type";
}

std::string PylonROS2StereoMiniCamera::setDeviceLinkThroughputLimitMode(const bool& /*turnOn*/)
{
    return "Feature not available for this camera type";
}

std::string PylonROS2StereoMiniCamera::setDeviceLinkThroughputLimit(const int& /*limit*/)
{
    return "Feature not available for this camera type";
}

std::string PylonROS2StereoMiniCamera::gammaEnable(const bool& /*enable*/)
{
    return "Feature not available for this camera type";
}

// --- Overrides for features backed by stereo mini SDK nodes -----------------

std::string PylonROS2StereoMiniCamera::setTriggerDelay(const float& delayValue)
{
    try
    {
        if (GenApi::IsWritable(stereo_mini_cam_->TriggerDelay))
        {
            stereo_mini_cam_->TriggerDelay.SetValue(delayValue);
            RCLCPP_DEBUG_STREAM(LOGGER_STEREO_MINI, "Trigger delay set to " << delayValue << " us");
        }
        else
        {
            return "Error: TriggerDelay is not writable";
        }
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_MINI, "An exception while setting the trigger delay occurred: " << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

std::string PylonROS2StereoMiniCamera::triggerDeviceReset()
{
    try
    {
        stereo_mini_cam_->DeviceReset.Execute();
        RCLCPP_DEBUG_STREAM(LOGGER_STEREO_MINI, "Device reset triggered");
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_MINI, "An exception while triggering the device reset occurred: " << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

// --- Buffer / statistics services (retargeted to stereo_mini_cam_) ----------

std::string PylonROS2StereoMiniCamera::setMaxNumBuffer(const int& size)
{
    if (!GenApi::IsAvailable(stereo_mini_cam_->MaxNumBuffer))
    {
        return "The connected Camera not supporting this feature";
    }
    try
    {
        // MaxNumBuffer is locked while grabbing; stop and restart around the change.
        this->grabbingStopping();
        stereo_mini_cam_->MaxNumBuffer.SetValue(size);
        this->grabbingStarting();
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_MINI, "An exception while setting the maximum number of buffers occurred: " << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

std::string PylonROS2StereoMiniCamera::setOutputQueueSize(const int& size)
{
    try
    {
        const int max_num_buffer = static_cast<int>(stereo_mini_cam_->MaxNumBuffer.GetValue());
        if (size < 0 || size > max_num_buffer)
        {
            return "requested output queue size is out side the limits of : 0-" + std::to_string(max_num_buffer);
        }
        stereo_mini_cam_->OutputQueueSize.SetValue(size);
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_MINI, "An exception while setting the output queue size occurred: " << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

int PylonROS2StereoMiniCamera::getMaxNumBuffer()
{
    if (!GenApi::IsAvailable(stereo_mini_cam_->MaxNumBuffer))
    {
        return -1;  // Not supported
    }
    try
    {
        return static_cast<int>(stereo_mini_cam_->MaxNumBuffer.GetValue());
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_MINI, "An exception while getting the maximum number of buffers occurred: " << e.GetDescription());
        return -2;  // Error
    }
}

// The stereo mini stream grabber exposes no Statistic_* counters, so all of
// these report "not supported" rather than touching the never-opened cam_.
int PylonROS2StereoMiniCamera::getStatisticTotalBufferCount()      { return -1; }
int PylonROS2StereoMiniCamera::getStatisticFailedBufferCount()     { return -1; }
int PylonROS2StereoMiniCamera::getStatisticBufferUnderrunCount()   { return -1; }
int PylonROS2StereoMiniCamera::getStatisticFailedPacketCount()     { return -1; }
int PylonROS2StereoMiniCamera::getStatisticResendRequestCount()    { return -1; }
int PylonROS2StereoMiniCamera::getStatisticMissedFrameCount()      { return -1; }
int PylonROS2StereoMiniCamera::getStatisticResynchronizationCount(){ return -1; }

// --- Additional stubs for services with no matching stereo mini SDK node ----

std::string PylonROS2StereoMiniCamera::setUserSetSelector(const int& /*set*/)
{
    return "Feature not available for this camera type";
}

std::string PylonROS2StereoMiniCamera::saveUserSet()
{
    return "Feature not available for this camera type";
}

std::string PylonROS2StereoMiniCamera::loadUserSet()
{
    return "Feature not available for this camera type";
}

std::string PylonROS2StereoMiniCamera::setUserSetDefaultSelector(const int& /*set*/)
{
    return "Feature not available for this camera type";
}

std::pair<std::string, std::string> PylonROS2StereoMiniCamera::getPfs()
{
    return {"Feature not available for this camera type", ""};
}

std::string PylonROS2StereoMiniCamera::savePfs(const std::string& /*fileName*/)
{
    return "Feature not available for this camera type";
}

std::string PylonROS2StereoMiniCamera::loadPfs(const std::string& /*fileName*/)
{
    return "Feature not available for this camera type";
}

std::string PylonROS2StereoMiniCamera::setChunkModeActive(const bool& /*enable*/)
{
    return "Feature not available for this camera type";
}

std::string PylonROS2StereoMiniCamera::setChunkSelector(const int& /*value*/)
{
    return "Feature not available for this camera type";
}

std::string PylonROS2StereoMiniCamera::setChunkEnable(const bool& /*enable*/)
{
    return "Feature not available for this camera type";
}

std::string PylonROS2StereoMiniCamera::setChunkExposureTime(const float& /*value*/)
{
    return "Feature not available for this camera type";
}

std::string PylonROS2StereoMiniCamera::setTimerSelector(const int& /*selector*/)
{
    return "Feature not available for this camera type";
}

std::string PylonROS2StereoMiniCamera::setTimerTriggerSource(const int& /*source*/)
{
    return "Feature not available for this camera type";
}

std::string PylonROS2StereoMiniCamera::setTimerDuration(const float& /*duration*/)
{
    return "Feature not available for this camera type";
}

std::string PylonROS2StereoMiniCamera::setMaxTransferSize(const int& /*maxTransferSize*/)
{
    return "Feature not available for this camera type";
}

std::string PylonROS2StereoMiniCamera::setPTPPriority(const int& /*value*/)
{
    return "Feature not available for this camera type";
}

std::string PylonROS2StereoMiniCamera::setPTPProfile(const int& /*value*/)
{
    return "Feature not available for this camera type";
}

std::string PylonROS2StereoMiniCamera::setPTPNetworkMode(const int& /*value*/)
{
    return "Feature not available for this camera type";
}

std::string PylonROS2StereoMiniCamera::setPTPUCPortAddressIndex(const int& /*value*/)
{
    return "Feature not available for this camera type";
}

std::string PylonROS2StereoMiniCamera::setPTPUCPortAddress(const int& /*value*/)
{
    return "Feature not available for this camera type";
}

std::string PylonROS2StereoMiniCamera::enablePTPManagementProtocol(const bool& /*value*/)
{
    return "Feature not available for this camera type";
}

std::string PylonROS2StereoMiniCamera::enablePTPTwoStepOperation(const bool& /*value*/)
{
    return "Feature not available for this camera type";
}

std::string PylonROS2StereoMiniCamera::enablePTP(const bool& /*value*/)
{
    return "Feature not available for this camera type";
}

std::string PylonROS2StereoMiniCamera::getPTPStatus(int64_t& /*offset_from_master*/, std::string& /*status*/, std::string& /*servo_status*/)
{
    return "Feature not available for this camera type";
}

std::string PylonROS2StereoMiniCamera::setPeriodicSignalPeriod(const float& /*value*/)
{
    return "Feature not available for this camera type";
}

std::string PylonROS2StereoMiniCamera::setPeriodicSignalDelay(const float& /*value*/)
{
    return "Feature not available for this camera type";
}

std::string PylonROS2StereoMiniCamera::setSyncFreeRunTimerStartTimeLow(const int& /*value*/)
{
    return "Feature not available for this camera type";
}

std::string PylonROS2StereoMiniCamera::setSyncFreeRunTimerStartTimeHigh(const int& /*value*/)
{
    return "Feature not available for this camera type";
}

std::string PylonROS2StereoMiniCamera::setSyncFreeRunTimerTriggerRateAbs(const float& /*value*/)
{
    return "Feature not available for this camera type";
}

std::string PylonROS2StereoMiniCamera::enableSyncFreeRunTimer(const bool& /*value*/)
{
    return "Feature not available for this camera type";
}

std::string PylonROS2StereoMiniCamera::updateSyncFreeRunTimer()
{
    return "Feature not available for this camera type";
}

std::string PylonROS2StereoMiniCamera::setActionTriggerConfiguration(const int& /*action_device_key*/, const int& /*action_group_key*/, const unsigned int& /*action_group_mask*/,
                                                                     const int& /*registration_mode*/, const int& /*cleanup*/)
{
    return "Feature not available for this camera type";
}

std::string PylonROS2StereoMiniCamera::issueActionCommand(const int& /*device_key*/, const int& /*group_key*/, const unsigned int& /*group_mask*/, const std::string& /*broadcast_address*/)
{
    return "Feature not available for this camera type";
}

std::string PylonROS2StereoMiniCamera::issueScheduledActionCommand(const int& /*device_key*/, const int& /*group_key*/, const unsigned int& /*group_mask*/, const int64_t& /*action_time_ns_from_current_timestamp*/, const std::string& /*broadcast_address*/)
{
    return "Feature not available for this camera type";
}

bool PylonROS2StereoMiniCamera::setAutoflash(const std::map<int, bool> /*flash_on_lines*/)
{
    return false;
}

bool PylonROS2StereoMiniCamera::setUserOutput(const int& /*output_id*/, const bool& /*value*/)
{
    return false;
}

} // namespace pylon_ros2_camera
