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

#include <string>
#include <vector>

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
    virtual std::string grabbingStarting();
    virtual std::string grabbingStopping() override;
    virtual bool isCamRemoved() override;

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

    virtual void getInitialCameraInfo(sensor_msgs::msg::CameraInfo& cam_info_msg) override;

    virtual int imagePixelDepth() const override;
    virtual float maxPossibleFramerate() override;

    // Override setExposure to use stereo_mini_cam_->ExposureTime directly.
    // Using the inherited cam_->ExposureTime while stereo_mini_cam_ is grabbing
    // causes the grab queue to flush and RetrieveResult() to block indefinitely.
    virtual bool setExposure(const float& target_exposure, float& reached_exposure) override;

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

std::string PylonROS2StereoMiniCamera::grabbingStarting()
{
    try
    {
        // Continuous (free-run) acquisition, keeping only the latest image.
        stereo_mini_cam_->StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_MINI, "An exception occurred while starting image grabbing:" << e.GetDescription());
        return e.GetDescription();
    }

    return "done";
}

std::string PylonROS2StereoMiniCamera::grabbingStopping()
{
    try
    {
        stereo_mini_cam_->StopGrabbing();
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_MINI, "An exception occurred while stopping image grabbing:" << e.GetDescription());
        return e.GetDescription();
    }

    return "done";
}

bool PylonROS2StereoMiniCamera::isCamRemoved()
{
    return cam_->IsCameraDeviceRemoved();
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

void PylonROS2StereoMiniCamera::getInitialCameraInfo(sensor_msgs::msg::CameraInfo& cam_info_msg)
{
    this->populateCameraInfoFromScan3d(stereo_mini_cam_->GetNodeMap(),
                                       static_cast<int>(this->imageCols()),
                                       static_cast<int>(this->imageRows()),
                                       cam_info_msg);
}

int PylonROS2StereoMiniCamera::imagePixelDepth() const
{
    // The published color intensity image is RGBa8 (4 bytes per pixel).
    return 4;
}

float PylonROS2StereoMiniCamera::maxPossibleFramerate()
{
    try
    {
        GenApi::CFloatPtr frame_rate(stereo_mini_cam_->GetNodeMap().GetNode("AcquisitionFrameRate"));
        if (frame_rate.IsValid() && GenApi::IsReadable(frame_rate))
        {
            return static_cast<float>(frame_rate->GetValue());
        }
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_DEBUG_STREAM(LOGGER_STEREO_MINI, "maxPossibleFramerate: could not read AcquisitionFrameRate: " << e.GetDescription());
    }
    return 30.0f;
}

bool PylonROS2StereoMiniCamera::setExposure(const float& target_exposure, float& reached_exposure)
{
    // The Stereo mini stops delivering grab results when ExposureTime is
    // changed while continuously grabbing — RetrieveResult() blocks
    // indefinitely after the change. Stopping and restarting the grab cycle
    // around the parameter change works around this camera behaviour.
    try
    {
        const bool was_grabbing = stereo_mini_cam_->IsGrabbing();
        if (was_grabbing)
            stereo_mini_cam_->StopGrabbing();

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

        if (was_grabbing)
            stereo_mini_cam_->StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);
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

} // namespace pylon_ros2_camera
