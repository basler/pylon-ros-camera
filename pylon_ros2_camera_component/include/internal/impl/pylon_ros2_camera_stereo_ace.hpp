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
        // AlternateActive (default): clean intensity images, frame rate halved.
        // AlwaysActive: full frame rate, IR pattern visible in intensity images.
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
                if (c.GetComponentType() == static_cast<Pylon::EComponentType>(0xFF01)) // IntensityCombined_STA
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
        if (type == static_cast<Pylon::EComponentType>(0xFF01)) idxCombined  = i; // IntensityCombined_STA
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

} // namespace pylon_ros2_camera
