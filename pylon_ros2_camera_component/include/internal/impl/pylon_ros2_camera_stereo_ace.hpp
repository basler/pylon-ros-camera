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

#include <pylon/StereoAceInstantCamera.h>


namespace pylon_ros2_camera
{

namespace
{
    static const rclcpp::Logger LOGGER_STEREO_ACE = rclcpp::get_logger("basler.pylon.ros2.pylon_ros2_stereo_ace_camera");
}

/**
 * Basler Stereo ace integration scaffold.
 *
 * STATUS: SCAFFOLD — compiles but is NOT hardware-verified.
 *         The following MUST be resolved before this class can be used:
 *
 *   (a) DETECTION (pylon_ros2_camera.cpp):
 *       The Stereo ace has no static device class string (calling
 *       CStereoAceInstantCamera::DeviceClass() throws). The interface device
 *       class is BaslerGenTlStaDeviceClass = "BaslerGTC/Basler/basler_xw", but
 *       it is not confirmed whether enumerated devices report this string via
 *       device_info.GetDeviceClass(). The detection branch in detectPylonCamType()
 *       is currently commented out with a placeholder. Verify with hardware and
 *       fill in the actual device class string.
 *
 *   (b) DISPARITY-TO-XYZ RECONSTRUCTION (extractPointCloudXYZ):
 *       Unlike the blaze and Stereo mini, the Stereo ace delivers raw disparity
 *       (Coord16) instead of direct XYZ coordinates. Point cloud generation
 *       requires a host-side triangulation:
 *           calibrated_d = disp_raw * Scan3dCoordinateScale + Scan3dCoordinateOffset
 *           Z = 1000 * Scan3dBaseline * Scan3dFocalLength / calibrated_d
 *           X = (u - Scan3dPrincipalPointU) * Z / Scan3dFocalLength
 *           Y = (v - Scan3dPrincipalPointV) * Z / Scan3dFocalLength
 *       The sign conventions and exact parameter semantics (in particular whether
 *       Z comes out in mm or m, and whether baseline is in mm or m) must be
 *       confirmed with the Stereo ace hardware before uncommenting the
 *       implementation in grab3D() below.
 *
 *   (c) ILLUMINATION MODE:
 *       BslIlluminationMode (AlternateActive / AlwaysActive / Off) controls
 *       whether the IR projector runs. AlternateActive provides clean intensity
 *       images without the IR pattern but halves the frame rate. The current
 *       value used during startup should be confirmed with the user/Basler.
 *
 *   (d) NO CONFIDENCE MAP:
 *       The Stereo ace does not provide a Confidence component. The confidence
 *       output of grab3D() will always be an empty message.
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
    virtual std::string grabbingStarting();
    virtual std::string grabbingStopping() override;
    virtual bool isCamRemoved() override;

    virtual bool grab3D(sensor_msgs::msg::PointCloud2& cloud_msg,
                        sensor_msgs::msg::Image& intensity_map_msg,
                        sensor_msgs::msg::Image& depth_map_msg,
                        sensor_msgs::msg::Image& depth_map_color_msg,
                        sensor_msgs::msg::Image& confidence_map_msg) override;
            bool grab3D(Pylon::CGrabResultPtr& grab_result);

    virtual void getInitialCameraInfo(sensor_msgs::msg::CameraInfo& cam_info_msg) override;

    virtual int imagePixelDepth() const override;
    virtual float maxPossibleFramerate() override;

public:
    Pylon::CStereoAceInstantCamera* stereo_ace_cam_;
};

PylonROS2StereoAceCamera::PylonROS2StereoAceCamera(Pylon::IPylonDevice* device) :
    PylonROS23DCamera(device),
    stereo_ace_cam_(new Pylon::CStereoAceInstantCamera(device))
{
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

        // Enable intensity output.
        stereo_ace_cam_->ComponentSelector.FromString("Intensity");
        stereo_ace_cam_->ComponentEnable.SetValue(true);
        // TODO: confirm whether RGB8 or Mono8 is preferred (depends on camera model).
        stereo_ace_cam_->PixelFormat.TrySetValue("RGB8");

        // Enable disparity output.
        stereo_ace_cam_->ComponentSelector.FromString("Disparity");
        stereo_ace_cam_->ComponentEnable.SetValue(true);

        // TODO: confirm illumination mode with user/Basler.
        //   AlwaysActive  — IR projector always on (best depth quality, IR pattern visible in intensity).
        //   AlternateActive — alternates with/without projector (clean intensity frames, half frame rate).
        //   Off           — passive stereo (no projector).
        stereo_ace_cam_->BslIlluminationMode.FromString("AlwaysActive");

        RCLCPP_INFO_STREAM(LOGGER_STEREO_ACE,
            "Stereo ace configured (SCAFFOLD — not hardware verified). "
            "Intensity enabled, Disparity enabled, BslIlluminationMode=AlwaysActive. "
            "Point cloud reconstruction requires implementation (see TODO in grab3D).");
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
            // Use the intensity component dimensions for image size (disparity may differ).
            for (int idx = 0; idx < (int)grab_result->GetDataComponentCount(); ++idx)
            {
                const auto c = grab_result->GetDataComponent(idx);
                if (c.GetComponentType() == Pylon::ComponentType_Intensity)
                {
                    img_cols_ = static_cast<size_t>(c.GetWidth());
                    img_rows_ = static_cast<size_t>(c.GetHeight());
                    img_size_byte_ = img_cols_ * img_rows_ * imagePixelDepth();
                    is_ready_ = true;
                    break;
                }
            }
            if (!is_ready_)
            {
                RCLCPP_ERROR(LOGGER_STEREO_ACE, "Initial grab returned no Intensity component");
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

std::string PylonROS2StereoAceCamera::grabbingStarting()
{
    try
    {
        stereo_ace_cam_->StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_ACE, "An exception while starting grabbing: " << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

std::string PylonROS2StereoAceCamera::grabbingStopping()
{
    try
    {
        stereo_ace_cam_->StopGrabbing();
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_STEREO_ACE, "An exception while stopping grabbing: " << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

bool PylonROS2StereoAceCamera::isCamRemoved()
{
    return cam_->IsCameraDeviceRemoved();
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
                                      sensor_msgs::msg::Image& confidence_map_msg __attribute__((unused)))
{
    Pylon::CGrabResultPtr ptr_grab_result;
    if (!this->grab3D(ptr_grab_result))
    {
        RCLCPP_ERROR(LOGGER_STEREO_ACE, "Grabbing with Stereo ace failed");
        return false;
    }

    // Locate the components.
    auto componentIdxIntensity = -1;
    auto componentIdxDisparity = -1;
    for (int idx = 0; idx < (int)ptr_grab_result->GetDataComponentCount(); ++idx)
    {
        switch (ptr_grab_result->GetDataComponent(idx).GetComponentType())
        {
            case Pylon::ComponentType_Intensity:  componentIdxIntensity  = idx; break;
            case Pylon::ComponentType_Disparity:  componentIdxDisparity  = idx; break;
            default: break;
        }
    }

    // Intensity image.
    if (componentIdxIntensity >= 0)
    {
        this->buildIntensityImage(ptr_grab_result->GetDataComponent(componentIdxIntensity), intensity_map_msg);
    }

    // TODO (b): Disparity -> XYZ point cloud and depth maps.
    //
    // The Stereo ace delivers raw disparity (component type Disparity, pixel
    // type Coord16) instead of direct XYZ coordinates. The triangulation
    // formulas below are taken from the Basler C++ samples but MUST be
    // verified with actual hardware before enabling:
    //
    //   camera.ComponentSelector.FromString("Disparity");
    //   double scale  = camera.Scan3dCoordinateScale.GetValue();
    //   double offset = camera.Scan3dCoordinateOffset.GetValue();
    //   double baseline    = camera.Scan3dBaseline.GetValue();     // meters?
    //   double focal_len   = camera.Scan3dFocalLength.GetValue();  // pixels?
    //   double cx = camera.Scan3dPrincipalPointU.GetValue();
    //   double cy = camera.Scan3dPrincipalPointV.GetValue();
    //
    //   For each pixel (u, v) with raw disparity disp_raw:
    //     calibrated_d = disp_raw * scale + offset
    //     Z = 1000.0 * baseline * focal_len / calibrated_d   [mm? m? TBD]
    //     X = (u - cx) * Z / focal_len
    //     Y = (v - cy) * Z / focal_len
    //
    // Once verified:
    //   1. Build an XYZ float array from the disparity component.
    //   2. Wrap it in a Pylon::CPylonDataComponent-like structure OR directly
    //      call buildPointCloud/calculateDepthMap from the shared profile
    //      (these expect a Coord3D_ABC32f component; adapting may be needed).
    //
    // OPEN QUESTIONS for Basler:
    //   - Units of Z (mm vs m), baseline (mm vs m)?
    //   - Sign convention for X, Y?
    //   - Is calibrated_d expected to be always > 0?
    //   - Does the Stereo ace also provide IntensityCombined (left+right stacked)?
    if (componentIdxDisparity >= 0)
    {
        RCLCPP_WARN_ONCE(LOGGER_STEREO_ACE,
            "Stereo ace point cloud and depth map generation is NOT implemented "
            "(TODO: disparity-to-XYZ reconstruction). Returning empty cloud/depth. "
            "See pylon_ros2_camera_stereo_ace.hpp for details.");
        // cloud_msg, depth_map_msg, depth_map_color_msg remain default-constructed (empty).
        (void)cloud_msg;
        (void)depth_map_msg;
        (void)depth_map_color_msg;
    }

    // The Stereo ace has no Confidence component; confidence_map_msg stays empty.

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
    // Mono8 or RGB8 intensity; default to 1 byte (Mono8).
    return 1;
}

float PylonROS2StereoAceCamera::maxPossibleFramerate()
{
    try
    {
        GenApi::CFloatPtr frame_rate(stereo_ace_cam_->GetNodeMap().GetNode("AcquisitionFrameRate"));
        if (frame_rate.IsValid() && GenApi::IsReadable(frame_rate))
            return static_cast<float>(frame_rate->GetValue());
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_DEBUG_STREAM(LOGGER_STEREO_ACE, "maxPossibleFramerate: " << e.GetDescription());
    }
    return 30.0f;
}

} // namespace pylon_ros2_camera
