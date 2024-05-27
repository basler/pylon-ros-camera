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

#include "internal/impl/pylon_ros2_camera_usb.hpp"


namespace pylon_ros2_camera
{

namespace
{
    static const rclcpp::Logger LOGGER_DART = rclcpp::get_logger("basler.pylon.ros2.pylon_ros2_dart_camera");
}

class PylonROS2DARTCamera : public PylonROS2USBCamera
{
public:
    explicit PylonROS2DARTCamera(Pylon::IPylonDevice* device);
    virtual ~PylonROS2DARTCamera();

    virtual bool applyCamSpecificStartupSettings(const PylonROS2CameraParameter& params) override;
    virtual bool setUserOutput(const int& output_id, const bool& value) override;
    virtual std::string typeName() const override;

protected:
    virtual bool setupSequencer(const std::vector<float>& exposure_times,
                                std::vector<float>& exposure_times_set) override;
    bool grab(Pylon::CGrabResultPtr& grab_result);
};

PylonROS2DARTCamera::PylonROS2DARTCamera(Pylon::IPylonDevice* device) :
    PylonROS2USBCamera(device)
{}

PylonROS2DARTCamera::~PylonROS2DARTCamera()
{}

bool PylonROS2DARTCamera::applyCamSpecificStartupSettings(const PylonROS2CameraParameter& parameters)
{
    if ( !PylonROS2USBCamera::applyCamSpecificStartupSettings(parameters) )
    {
        return false;
    }
    return true;
}

bool PylonROS2DARTCamera::setUserOutput(const int& output_id, const bool& value)
{
    (void)output_id;
    (void)value;
    RCLCPP_ERROR(LOGGER_DART, "Dart camera has no digital output.");
    return false;
}

bool PylonROS2DARTCamera::setupSequencer(const std::vector<float>& exposure_times,
                                         std::vector<float>& exposure_times_set)
{
    (void)exposure_times;
    (void)exposure_times_set;
    RCLCPP_ERROR(LOGGER_DART, "Sequencer Mode for Dart Cameras not yet implemented");
    return false;
}

bool PylonROS2DARTCamera::grab(Pylon::CGrabResultPtr& grab_result)
{
    try
    {
        // /!\ The dart camera device does not support
        // 'waitForFrameTriggerReady'
        cam_->ExecuteSoftwareTrigger();
        cam_->RetrieveResult(grab_timeout_, grab_result,
                             Pylon::TimeoutHandling_ThrowException);
    }
    catch (const GenICam::GenericException &e)
    {
        if ( cam_->IsCameraDeviceRemoved() )
        {
            RCLCPP_ERROR(LOGGER_DART, "Camera was removed");
        }
        else
        {
            if (cam_->TriggerSource.GetValue() != TriggerSourceEnums::TriggerSource_Software)
            {
                RCLCPP_ERROR_STREAM(LOGGER_DART, "Waiting for Hardware Trigger");
            }
            else if (cam_->TriggerMode.GetValue() == TriggerModeEnums::TriggerMode_On)
            {
                RCLCPP_ERROR_STREAM(LOGGER_DART, "Waiting for Trigger signal");
            }
            else
            {
            RCLCPP_ERROR_STREAM(LOGGER_DART, "An image grabbing exception in pylon camera occurred: "
                    << e.GetDescription());
            }
        }
        return false;
    }
    catch ( ... )
    {
        RCLCPP_ERROR(LOGGER_DART, "An unspecified image grabbing exception in pylon camera occurred");
        return false;
    }
    if ( !grab_result->GrabSucceeded() )
    {
        RCLCPP_ERROR_STREAM(LOGGER_DART, "Error: " << grab_result->GetErrorCode()
                << " " << grab_result->GetErrorDescription());
        return false;
    }

    return true;
}

std::string PylonROS2DARTCamera::typeName() const
{
    return "DART";
}

}  // namespace pylon_ros2_camera
