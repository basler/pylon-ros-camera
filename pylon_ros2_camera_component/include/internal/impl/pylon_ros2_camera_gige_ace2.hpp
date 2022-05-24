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

#include "internal/impl/pylon_ros2_camera_gige.hpp"


namespace pylon_ros2_camera
{

namespace
{
    static const rclcpp::Logger LOGGER_GIGE_ACE2 = rclcpp::get_logger("basler.pylon.ros2.pylon_ros2_gige_ace2_camera");
}

class PylonROS2GigEAce2Camera : public PylonROS2GigECamera
{
public:
    explicit PylonROS2GigEAce2Camera(Pylon::IPylonDevice* device);
    virtual ~PylonROS2GigEAce2Camera();

    virtual bool applyCamSpecificStartupSettings(const PylonROS2CameraParameter& params);
    virtual GenApi::IFloat& gain();
    virtual GenApi::IFloat& autoGainLowerLimit();
    virtual GenApi::IFloat& autoGainUpperLimit();
    virtual std::string typeName() const;
    virtual GenApi::IFloat& resultingFrameRate();
    virtual float currentGain();
    virtual std::string setBlackLevel(const int& value);
    virtual int getBlackLevel();
    virtual std::string setNoiseReduction(const float& value);
    virtual float getNoiseReduction();
    virtual std::string setSharpnessEnhancement(const float& value);
    virtual float getSharpnessEnhancement();
    virtual std::string setTriggerDelay(const float& delayValue);
    virtual float getTriggerDelay();
    virtual std::string setAcquisitionFrameCount(const int& frameCount);
    virtual int getAcquisitionFrameCount();
    virtual std::string setDeviceLinkThroughputLimitMode(const bool& turnOn);
    virtual int getDeviceLinkThroughputLimitMode();
    virtual std::string setDeviceLinkThroughputLimit(const int& limit);
    virtual bool setGain(const float& target_gain, float& reached_gain);

};

PylonROS2GigEAce2Camera::PylonROS2GigEAce2Camera(Pylon::IPylonDevice* device) :
    PylonROS2GigECamera(device)
{}

PylonROS2GigEAce2Camera::~PylonROS2GigEAce2Camera()
{}

bool PylonROS2GigEAce2Camera::applyCamSpecificStartupSettings(const PylonROS2CameraParameter& parameters)
{
    try
    {
        //cam_->StartGrabbing();
        grabbingStarting();
        cam_->StopGrabbing();
        if (parameters.startup_user_set_ == "Default")
            {
                // Remove all previous settings (sequencer etc.)
                // Default Setting = Free-Running
                cam_->UserSetSelector.SetValue(Basler_UniversalCameraParams::UserSetSelector_Default);
                cam_->UserSetLoad.Execute();
                // UserSetSelector_Default overrides Software Trigger Mode !!
                cam_->TriggerSource.SetValue(Basler_UniversalCameraParams::TriggerSource_Software);
                cam_->TriggerMode.SetValue(Basler_UniversalCameraParams::TriggerMode_On);
                /* Thresholds for the AutoExposure Functions:
                 *  - lower limit can be used to get rid of changing light conditions
                 *    due to 50Hz lamps (-> 20ms cycle duration)
                 *  - upper limit is to prevent motion blur
                 */
                double upper_lim = std::min(parameters.auto_exposure_upper_limit_,
                                            cam_->ExposureTimeAbs.GetMax());
                cam_->AutoExposureTimeAbsLowerLimit.SetValue(cam_->ExposureTimeAbs.GetMin());
                cam_->AutoExposureTimeAbsUpperLimit.SetValue(upper_lim);
                // The gain auto function and the exposure auto function can be used at the
                // same time. In this case, however, you must also set the
                // Auto Function Profile feature.
                // acA1920-40gm does not support Basler_UniversalCameraParams::GainSelector_AnalogAll
                // has Basler_UniversalCameraParams::GainSelector_All instead
                // cam_->GainSelector.SetValue(Basler_UniversalCameraParams::GainSelector_AnalogAll);

                if ( GenApi::IsAvailable(cam_->BinningHorizontal) &&
                     GenApi::IsAvailable(cam_->BinningVertical) )
                {
                    RCLCPP_INFO_STREAM(LOGGER_GIGE_ACE2, "Cam has binning range: x(hz) = ["
                            << cam_->BinningHorizontal.GetMin() << " - "
                            << cam_->BinningHorizontal.GetMax() << "], y(vt) = ["
                            << cam_->BinningVertical.GetMin() << " - "
                            << cam_->BinningVertical.GetMax() << "].");
                }
                else
                {
                    RCLCPP_INFO_STREAM(LOGGER_GIGE_ACE2, "Cam does not support binning.");
                }

                RCLCPP_INFO_STREAM(LOGGER_GIGE_ACE2, "Cam has exposure time range: ["
                        << cam_->ExposureTimeAbs.GetMin()
                        << " - " << cam_->ExposureTimeAbs.GetMax()
                        << "] measured in microseconds.");
                RCLCPP_INFO_STREAM(LOGGER_GIGE_ACE2, "Cam has gain range: ["
                        << cam_->Gain.GetMin() << " - "
                        << cam_->Gain.GetMax()
                        << "] measured in device specific units.");

                // Check if gamma is available, print range
                if ( !GenApi::IsAvailable(cam_->Gamma) )
                {
                    RCLCPP_WARN(LOGGER_GIGE_ACE2, "Cam gamma not available, will keep the default (auto).");
                }
                else
                {
                    RCLCPP_INFO_STREAM(LOGGER_GIGE_ACE2, "Cam has gamma range: ["
                        << cam_->Gamma.GetMin() << " - "
                        << cam_->Gamma.GetMax() << "].");
                }

                RCLCPP_INFO_STREAM(LOGGER_GIGE_ACE2, "Cam has pylon auto brightness range: ["
                        << cam_->AutoTargetValue.GetMin() << " - "
                        << cam_->AutoTargetValue.GetMax()
                        << "] which is the average pixel intensity.");

                // raise inter-package delay (GevSCPD) for solving error:
                // 'the image buffer was incompletely grabbed'
                // also in ubuntu settings -> network -> options -> MTU Size
                // from 'automatic' to 3000 if card supports it
                // single-board computers have MTU = 1500, max value for some cards: 9000
                RCLCPP_WARN(LOGGER_GIGE_ACE2, "setting MTU");
                cam_->GevSCPSPacketSize.SetValue(parameters.mtu_size_);
                RCLCPP_WARN(LOGGER_GIGE_ACE2, "MTU Setted");
                if (parameters.auto_flash_)
                {
                    std::map<int, bool> flash_on_lines;
                    RCLCPP_INFO(LOGGER_GIGE_ACE2, "Flash 2: %i", parameters.auto_flash_line_2_);
                    RCLCPP_INFO(LOGGER_GIGE_ACE2, "Flash 3: %i", parameters.auto_flash_line_3_);

                    flash_on_lines[2] = parameters.auto_flash_line_2_;
                    flash_on_lines[3] = parameters.auto_flash_line_3_;
                    setAutoflash(flash_on_lines);
                }

                // http://www.baslerweb.com/media/documents/AW00064902000%20Control%20Packet%20Timing%20With%20Delays.pdf
                // inter package delay in ticks (? -> mathi said in nanosec) -> prevent lost frames
                // package size * n_cams + 5% overhead = inter package size
                // int n_cams = 1;
                // int inter_package_delay_in_ticks = n_cams * imageSize() * 1.05;
                cam_->GevSCPD.SetValue(parameters.inter_pkg_delay_);
                RCLCPP_WARN(LOGGER_GIGE_ACE2, "Default User Setting Loaded");
            }
        else if (parameters.startup_user_set_ == "UserSet1")
            {
                cam_->UserSetSelector.SetValue(Basler_UniversalCameraParams::UserSetSelector_UserSet1);
                cam_->UserSetLoad.Execute();
                cam_->GevSCPSPacketSize.SetValue(parameters.mtu_size_);
                RCLCPP_WARN(LOGGER_GIGE_ACE2, "User Set 1 Loaded");
            } 
        else if (parameters.startup_user_set_ == "UserSet2")
            {
                cam_->UserSetSelector.SetValue(Basler_UniversalCameraParams::UserSetSelector_UserSet2);
                cam_->UserSetLoad.Execute();
                cam_->GevSCPSPacketSize.SetValue(parameters.mtu_size_);
                RCLCPP_WARN(LOGGER_GIGE_ACE2, "User Set 2 Loaded");
            } 
        else if (parameters.startup_user_set_ == "UserSet3")
            {
                cam_->UserSetSelector.SetValue(Basler_UniversalCameraParams::UserSetSelector_UserSet3);
                cam_->UserSetLoad.Execute();
                cam_->GevSCPSPacketSize.SetValue(parameters.mtu_size_);
                RCLCPP_WARN(LOGGER_GIGE_ACE2, "User Set 3 Loaded");
            } 
        else if (parameters.startup_user_set_ == "CurrentSetting")
            {
                cam_->GevSCPSPacketSize.SetValue(parameters.mtu_size_);
                RCLCPP_WARN(LOGGER_GIGE_ACE2, "No user set is provided -> Camera current setting will be applied");
            }
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "Error applying camera specific startup setting for GigE cameras: "
                << e.GetDescription());
        return false;
    }
    return true;
}


GenApi::IFloat& PylonROS2GigEAce2Camera::gain()
{
    if ( GenApi::IsAvailable(cam_->Gain) )
    {
        return cam_->Gain; // GigE ace 1
    } else
    {
        throw std::runtime_error("Error while accessing Gain in PylonROS2GigEAce2Camera");
    }
}

GenApi::IFloat& PylonROS2GigEAce2Camera::autoGainLowerLimit()
{
    if ( GenApi::IsAvailable(cam_->AutoGainLowerLimit) )
    {
        return cam_->AutoGainLowerLimit;
    }
    else
    {
        throw std::runtime_error("Error while accessing AutoGainLowerLimit in PylonROS2GigEAce2Camera");
    }
}


GenApi::IFloat& PylonROS2GigEAce2Camera::autoGainUpperLimit()
{
    if ( GenApi::IsAvailable(cam_->AutoGainUpperLimit) )
    {
        return cam_->AutoGainUpperLimit;
    }
    else
    {
        throw std::runtime_error("Error while accessing AutoGainUpperLimit in PylonROS2GigEAce2Camera");
    }
}


float PylonROS2GigEAce2Camera::currentGain()
{
    float curr_gain = (static_cast<float>(gain().GetValue()) - static_cast<float>(gain().GetMin())) /
        (static_cast<float>(gain().GetMax() - static_cast<float>(gain().GetMin())));
    return curr_gain;
}

GenApi::IFloat& PylonROS2GigEAce2Camera::resultingFrameRate()
{
    if ( GenApi::IsAvailable(cam_->ResultingFrameRate) )
    {
        return cam_->ResultingFrameRate;
    }
    else
    {
        throw std::runtime_error("Error while accessing ResultingFrameRate in PylonROS2GigECamera");
    }
}

std::string PylonROS2GigEAce2Camera::setBlackLevel(const int& value)
{
    try
    {
        if ( GenApi::IsAvailable(cam_->BlackLevel) )
        {
            cam_->BlackLevel.SetValue(value);   
            return "done";
        }
        else 
        {
             RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "Error while trying to change the image black level. The connected Camera not supporting this feature");
             return "The connected Camera not supporting this feature";
        }

    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "An exception while changing the image black level occurred:" << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}


int PylonROS2GigEAce2Camera::getBlackLevel()
{
    try
    {
        if ( GenApi::IsAvailable(cam_->BlackLevel) )
        {
            return static_cast<int>(cam_->BlackLevel.GetValue());     
        }
        else 
        { 
             return -10000;
        }

    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "An exception while getting the image black level occurred:" << e.GetDescription());
        return -10000;
    }
}


std::string PylonROS2GigEAce2Camera::setNoiseReduction(const float& value)
{
 try
    {

        if ( GenApi::IsAvailable(cam_->BslNoiseReduction) )
        {
            cam_->BslNoiseReduction.SetValue(value);
            return "done";
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "Error while trying to change the noise reduction value. The connected Camera not supporting this feature");
            return "The connected Camera not supporting this feature";
        }

    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "An exception while setting the noise reduction value occurred:" << e.GetDescription());
        return e.GetDescription();
    }
}


float PylonROS2GigEAce2Camera::getNoiseReduction()
{
 try
    {
        if ( GenApi::IsAvailable(cam_->BslNoiseReduction) )
        {
            return  static_cast<float>(cam_->BslNoiseReduction.GetValue());
        }
        else 
        {
            return -10000.0;
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        return -20000.0;
    }
}


std::string PylonROS2GigEAce2Camera::setSharpnessEnhancement(const float& value)
{
 try
    {

        if ( GenApi::IsAvailable(cam_->BslSharpnessEnhancement) )
        {
            cam_->BslSharpnessEnhancement.SetValue(value);
            return "done";
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "Error while trying to change the sharpness enhancement value. The connected Camera not supporting this feature");
            return "The connected Camera not supporting this feature";
        }

    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "An exception while setting the sharpness enhancement value occurred:" << e.GetDescription());
        return e.GetDescription();
    }
}


float PylonROS2GigEAce2Camera::getSharpnessEnhancement()
{
 try
    {
        if ( GenApi::IsAvailable(cam_->BslSharpnessEnhancement) )
        {
            return static_cast<float>(cam_->BslSharpnessEnhancement.GetValue());
        }
        else 
        {
            return -10000.0;
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        return -20000.0;
    }
}


std::string PylonROS2GigEAce2Camera::setTriggerDelay(const float& delayValue)
{
    try
    {   if ( GenApi::IsAvailable(cam_->TriggerDelay) )
        {

            cam_->TriggerDelay.SetValue(delayValue);
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "Error while trying to change the trigger delay. The connected Camera not supporting this feature");
            return "The connected Camera not supporting this feature";
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "An exception while setting the trigger delay occurred:" << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

float PylonROS2GigEAce2Camera::getTriggerDelay()
{
    try
    {   if ( GenApi::IsAvailable(cam_->TriggerDelay) )
        {

             return static_cast<float>(cam_->TriggerDelay.GetValue());
        }
        else 
        {
            return -10000.0; // Not available
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        return -20000.0; // Error
    }
}


std::string PylonROS2GigEAce2Camera::setAcquisitionFrameCount(const int& frameCount)
{
    try
    {
        if ( GenApi::IsAvailable(cam_->AcquisitionFrameRate) && GenApi::IsAvailable(cam_->AcquisitionFrameRateEnable))
        {  
            cam_->AcquisitionFrameRateEnable.SetValue(true);
            cam_->AcquisitionFrameRate.SetValue(static_cast<float>(frameCount));   
            return "done";
        }
        else 
        {
             RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "Error while trying to change the Acquisition frame count. The connected Camera not supporting this feature");
             return "The connected Camera not supporting this feature";
        }

    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "An exception while changing Acquisition frame count occurred:" << e.GetDescription());
        return e.GetDescription();
    }
}


int PylonROS2GigEAce2Camera::getAcquisitionFrameCount()
{
    try
    {
        if ( GenApi::IsAvailable(cam_->AcquisitionFrameRate) )
        {  
            return static_cast<int>(cam_->AcquisitionFrameRate.GetValue());
        }
        else 
        {
             return -10000; // Not available
        }

    }
    catch ( const GenICam::GenericException &e )
    {
        return -20000; // Error
    }
}


std::string PylonROS2GigEAce2Camera::setDeviceLinkThroughputLimitMode(const bool& turnOn)
{
    try
    {
        if ( GenApi::IsAvailable(cam_->DeviceLinkThroughputLimitMode) )
        {  
            if (turnOn)
            {
            cam_->DeviceLinkThroughputLimitMode.SetValue( Basler_UniversalCameraParams::DeviceLinkThroughputLimitModeEnums::DeviceLinkThroughputLimitMode_On);   
            return "done";
            }
            else 
            {
            cam_->DeviceLinkThroughputLimitMode.SetValue( Basler_UniversalCameraParams::DeviceLinkThroughputLimitModeEnums::DeviceLinkThroughputLimitMode_Off);    
            return "done";
            }
        }
        else 
        {
             RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "Error while trying to change the device link throughput limit mode. The connected Camera not supporting this feature");
             return "The connected Camera not supporting this feature";
        }

    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "An exception while changing the device link throughput limit mode occurred:" << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}


int PylonROS2GigEAce2Camera::getDeviceLinkThroughputLimitMode()
{
    try
    {
        if ( GenApi::IsAvailable(cam_->DeviceLinkThroughputLimitMode) )
        {  
            if (cam_->DeviceLinkThroughputLimitMode.GetValue() ==  Basler_UniversalCameraParams::DeviceLinkThroughputLimitModeEnums::DeviceLinkThroughputLimitMode_On)
            {
                return 0; // On
            }
            else if (cam_->DeviceLinkThroughputLimitMode.GetValue() ==  Basler_UniversalCameraParams::DeviceLinkThroughputLimitModeEnums::DeviceLinkThroughputLimitMode_Off)
            {
            return 1; // Off
            }
            else 
            {
            return -3; // Unknown
            }
        }
        else 
        {
             return -1; // Not Available
        }

    }
    catch ( const GenICam::GenericException &e )
    {
        return -2; // Error
    }
}


std::string PylonROS2GigEAce2Camera::setDeviceLinkThroughputLimit(const int& limit)
{
    try
    {
        if ( GenApi::IsAvailable(cam_->DeviceLinkThroughputLimit) )
        {  
            if (cam_->DeviceLinkThroughputLimitMode.GetValue() ==  Basler_UniversalCameraParams::DeviceLinkThroughputLimitModeEnums::DeviceLinkThroughputLimitMode_On)
            {
                cam_->DeviceLinkThroughputLimit.SetValue(limit);     
                return "done";
            }
            else 
            {
                return "Error : device link throughput limit mode is off";
            }
        }
        else 
        {
             RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "Error while trying to change the device link throughput limit. The connected Camera not supporting this feature");
             return "The connected Camera not supporting this feature";
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "An exception while changing the device link throughput limit occurred:" << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}


bool PylonROS2GigEAce2Camera::setGain(const float& target_gain, float& reached_gain)
{
    try
    {
        cam_->GainAuto.SetValue(GainAutoEnums::GainAuto_Off);
        float truncated_gain = target_gain;
        if ( truncated_gain < 0.0 )
        {
            RCLCPP_WARN_STREAM(LOGGER_GIGE_ACE2, "Desired gain (" << target_gain << ") in "
                << "percent out of range [0.0 - 1.0]! Setting to lower "
                << "limit: 0.0");
            truncated_gain = 0.0;
        }
        else if ( truncated_gain > 1.0 )
        {
            RCLCPP_WARN_STREAM(LOGGER_GIGE_ACE2, "Desired gain (" << target_gain << ") in "
                << "percent out of range [0.0 - 1.0]! Setting to upper "
                << "limit: 1.0");
            truncated_gain = 1.0;
        }

        float gain_to_set = gain().GetMin() +
                            truncated_gain * (gain().GetMax() - gain().GetMin());
        gain().SetValue(gain_to_set);
        reached_gain = currentGain();
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "An exception while setting target gain to "
               << target_gain << " occurred: " << e.GetDescription());
        return false;
    }
    return true;
}

std::string PylonROS2GigEAce2Camera::typeName() const
{
    return "GIGE2";
}

}  // namespace pylon_ros2_camera
