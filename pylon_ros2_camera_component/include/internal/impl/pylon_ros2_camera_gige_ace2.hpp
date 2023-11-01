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
    virtual std::string setTriggerSource(const int& source);
    virtual std::string setTimerSelector(const int& selector);
    virtual std::string setPTPPriority(const int& value);
    virtual std::string setPTPProfile(const int& value);
    virtual std::string setPTPNetworkMode(const int& value);
    virtual std::string setPTPUCPortAddressIndex(const int& value);
    virtual std::string setPTPUCPortAddress(const int& value);
    virtual std::string setPeriodicSignalPeriod(const float& value);
    virtual std::string setPeriodicSignalDelay(const float& value);
    virtual std::string enablePTPManagementProtocol(const bool& value);
    virtual std::string enablePTPTwoStepOperation(const bool& value);
    virtual std::string enablePTP(const bool& value);
    virtual std::string setActionTriggerConfiguration(const int& action_device_key, const int& action_group_key, const unsigned int& action_group_mask,
                                                      const int& registration_mode, const int& cleanup);
    virtual std::string issueActionCommand(const int& device_key, const int& group_key, const unsigned int& group_mask, const std::string& broadcast_address);
    virtual std::string issueScheduledActionCommand(const int& device_key, const int& group_key, const unsigned int& group_mask, const int64_t& action_time_ns_from_current_timestamp,
                                                    const std::string& broadcast_address);
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

        RCLCPP_INFO_STREAM(LOGGER_GIGE_ACE2, "Startup user profile set to " << parameters.startup_user_set_);
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
            double upper_lim = std::min(parameters.auto_exposure_upper_limit_, cam_->ExposureTimeAbs.GetMax());
            cam_->AutoExposureTimeAbsLowerLimit.SetValue(cam_->ExposureTimeAbs.GetMin());
            cam_->AutoExposureTimeAbsUpperLimit.SetValue(upper_lim);
            RCLCPP_INFO_STREAM(LOGGER_GIGE_ACE2, "Cam has upper exposure value limit range: ["
                    << cam_->ExposureTimeAbs.GetMin()
                    << " - " << upper_lim << " (max possible value from cam is " << cam_->ExposureTimeAbs.GetMax() << ")"
                    << "].");
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
            cam_->GevSCPSPacketSize.SetValue(parameters.mtu_size_);
            
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

            // frame transmission delay
            cam_->GevSCFTD.SetValue(parameters.frame_transmission_delay_);

            RCLCPP_WARN(LOGGER_GIGE_ACE2, "Default User Setting Loaded");
        }
        else if (parameters.startup_user_set_ == "UserSet1")
        {
            cam_->UserSetSelector.SetValue(Basler_UniversalCameraParams::UserSetSelector_UserSet1);
            cam_->UserSetLoad.Execute();

            cam_->GevSCPSPacketSize.SetValue(parameters.mtu_size_);
            cam_->GevSCPD.SetValue(parameters.inter_pkg_delay_);
            cam_->GevSCFTD.SetValue(parameters.frame_transmission_delay_);
            
            RCLCPP_WARN(LOGGER_GIGE_ACE2, "User Set 1 Loaded");
        } 

        else if (parameters.startup_user_set_ == "UserSet2")
        {
            cam_->UserSetSelector.SetValue(Basler_UniversalCameraParams::UserSetSelector_UserSet2);
            cam_->UserSetLoad.Execute();

            cam_->GevSCPSPacketSize.SetValue(parameters.mtu_size_);
            cam_->GevSCPD.SetValue(parameters.inter_pkg_delay_);
            cam_->GevSCFTD.SetValue(parameters.frame_transmission_delay_);
            
            RCLCPP_WARN(LOGGER_GIGE_ACE2, "User Set 2 Loaded");
        } 

        else if (parameters.startup_user_set_ == "UserSet3")
        {
            cam_->UserSetSelector.SetValue(Basler_UniversalCameraParams::UserSetSelector_UserSet3);
            cam_->UserSetLoad.Execute();

            cam_->GevSCPSPacketSize.SetValue(parameters.mtu_size_);
            cam_->GevSCPD.SetValue(parameters.inter_pkg_delay_);
            cam_->GevSCFTD.SetValue(parameters.frame_transmission_delay_);
            
            RCLCPP_WARN(LOGGER_GIGE_ACE2, "User Set 3 Loaded");
        }
        else if (parameters.startup_user_set_ == "CurrentSetting")
        {
            cam_->GevSCPSPacketSize.SetValue(parameters.mtu_size_);
            cam_->GevSCPD.SetValue(parameters.inter_pkg_delay_);
            cam_->GevSCFTD.SetValue(parameters.frame_transmission_delay_);
            
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
    if (GenApi::IsAvailable(cam_->AutoGainUpperLimit))
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
        cam_->GainAuto.TrySetValue(GainAutoEnums::GainAuto_Off);
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

std::string PylonROS2GigEAce2Camera::setTriggerSource(const int& source)
{
    try
    {   if (GenApi::IsAvailable(cam_->TriggerSource))
        {
            switch (source)
            {
                case 0:
                    cam_->TriggerSource.SetValue(TriggerSourceEnums::TriggerSource_Software);
                    RCLCPP_INFO_STREAM(LOGGER_GIGE_ACE2, "Trigger source: Software");
                    break;
                case 1:
                    cam_->TriggerSource.SetValue(TriggerSourceEnums::TriggerSource_Line1);
                    RCLCPP_INFO_STREAM(LOGGER_GIGE_ACE2, "Trigger source: Line 1");
                    break;
                case 2:
                    cam_->TriggerSource.SetValue(TriggerSourceEnums::TriggerSource_Line3);
                    RCLCPP_INFO_STREAM(LOGGER_GIGE_ACE2, "Trigger source: Line 3");
                    break;
                case 3:
                    cam_->TriggerSource.SetValue(TriggerSourceEnums::TriggerSource_Line4);
                    RCLCPP_INFO_STREAM(LOGGER_GIGE_ACE2, "Trigger source: Line 4");
                    break;
                case 4:
                    cam_->TriggerSource.SetValue(TriggerSourceEnums::TriggerSource_Action1);
                    RCLCPP_INFO_STREAM(LOGGER_GIGE_ACE2, "Trigger source: Action 1");
                    break;
                case 5:
                    cam_->TriggerSource.SetValue(TriggerSourceEnums::TriggerSource_PeriodicSignal1);
                    RCLCPP_INFO_STREAM(LOGGER_GIGE_ACE2, "Trigger source: Periodic Signal 1");
                    break;
                default:
                    RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "Trigger source value is invalid! Please choose between 0 -> Trigger Software / 1 -> Line 1 / 2 -> Line 3 / 3 -> Line 4 / 4 -> Action 1 / 5 -> Periodic Signal 1");
                    return "Error: unknown value for trigger source";
            }
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "Error while trying to change the trigger source. The connected camera does not support this feature");
            return "The connected camera does not support this feature";
        }

    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "An exception while setting the trigger source occurred:" << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

std::string PylonROS2GigEAce2Camera::setTimerSelector(const int& selector)
{
    try
    {   if (GenApi::IsAvailable(cam_->TimerSelector))
        {
            switch (selector)
            {
                case 1:
                    cam_->TimerSelector.SetValue(TimerSelectorEnums::TimerSelector_Timer1);
                    break;
                case 2:
                    cam_->TimerSelector.SetValue(TimerSelectorEnums::TimerSelector_Timer2);
                    break;
                default:
                    RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "Timer selector value is invalid! Please choose between 1 -> Timer 1 / 2 -> Timer 2");
                    return "Error: unknown value for timer selector";
            }
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "Error while trying to change the timer selector. The connected camera does not support this feature");
            return "The connected camera does not support this feature";
        }
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "An exception while setting the timer selector occurred:" << e.GetDescription());
        return e.GetDescription();
    }
    
    return "done";
}

std::string PylonROS2GigEAce2Camera::setPTPPriority(const int& value)
{
    try
    {
        if (GenApi::IsAvailable(cam_->BslPtpPriority1))
        {
            cam_->BslPtpPriority1.SetValue(value);   
            return "done";
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "Error while trying to set PTP priority. The connected camera does not support this feature.");
            return "The connected camera does not support this feature";
        }

    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "An exception while setting PTP priority:" << e.GetDescription());
        return e.GetDescription();
    }
}

std::string PylonROS2GigEAce2Camera::setPTPProfile(const int& value)
{
    try
    {
        if (GenApi::IsAvailable(cam_->BslPtpProfile))
        {
            switch(value)
            {
                case 1:
                    cam_->BslPtpProfile.SetValue(Basler_UniversalCameraParams::BslPtpProfileEnums::BslPtpProfile_DelayRequestResponseDefaultProfile);
                    return "done";
                    break;
                case 2:
                    cam_->BslPtpProfile.SetValue(Basler_UniversalCameraParams::BslPtpProfileEnums::BslPtpProfile_PeerToPeerDefaultProfile);
                    return "done";
                    break;
                default:
                    return "Error: Unknown ptp profile index";
                    break;
            }
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "Error while trying to set PTP profile. The connected camera does not support this feature.");
            return "The connected camera does not support this feature";
        }

    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "An exception while setting PTP profile:" << e.GetDescription());
        return e.GetDescription();
    }
}

std::string PylonROS2GigEAce2Camera::setPTPNetworkMode(const int& value)
{
    try
    {
        if (GenApi::IsAvailable(cam_->BslPtpNetworkMode))
        {
            switch(value)
            {
                case 1:
                    cam_->BslPtpNetworkMode.SetValue(Basler_UniversalCameraParams::BslPtpNetworkModeEnums::BslPtpNetworkMode_Hybrid);
                    return "done";
                    break;
                case 2:
                    cam_->BslPtpNetworkMode.SetValue(Basler_UniversalCameraParams::BslPtpNetworkModeEnums::BslPtpNetworkMode_Multicast);
                    return "done";
                    break;
                case 3:
                    cam_->BslPtpNetworkMode.SetValue(Basler_UniversalCameraParams::BslPtpNetworkModeEnums::BslPtpNetworkMode_Unicast);
                    return "done";
                    break;        
                default:
                    return "Error: Unknown ptp network mode";
                    break;
            }
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "Error while trying to set PTP network mode. The connected camera does not support this feature.");
            return "The connected camera does not support this feature";
        }

    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "An exception while setting PTP network mode:" << e.GetDescription());
        return e.GetDescription();
    }
}

std::string PylonROS2GigEAce2Camera::setPTPUCPortAddressIndex(const int& value)
{
    try
    {
        if (GenApi::IsAvailable(cam_->BslPtpUcPortAddrIndex))
        {
            cam_->BslPtpUcPortAddrIndex.SetValue(value);   
            return "done";
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "Error while trying to set PTP UC address index. The connected camera does not support this feature.");
            return "The connected camera does not support this feature";
        }

    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "An exception while setting PTP UC address index:" << e.GetDescription());
        return e.GetDescription();
    }
}

std::string PylonROS2GigEAce2Camera::setPTPUCPortAddress(const int& value)
{
    try
    {
        if (GenApi::IsAvailable(cam_->BslPtpUcPortAddr))
        {
            cam_->BslPtpUcPortAddr.SetValue(value);   
            return "done";
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "Error while trying to set PTP UC address. The connected camera does not support this feature.");
            return "The connected camera does not support this feature";
        }

    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "An exception while setting PTP UC address:" << e.GetDescription());
        return e.GetDescription();
    }
}

std::string PylonROS2GigEAce2Camera::setPeriodicSignalPeriod(const float& value)
{
    try
    {
        if (GenApi::IsAvailable(cam_->BslPeriodicSignalPeriod))
        {
            cam_->BslPeriodicSignalPeriod.SetValue(value);
            return "done";
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "Error while trying to set periodic signal period. The connected camera does not support this feature.");
            return "The connected camera does not support this feature";
        }

    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "An exception while setting periodic signal period:" << e.GetDescription());
        return e.GetDescription();
    }
}

std::string PylonROS2GigEAce2Camera::setPeriodicSignalDelay(const float& value)
{
    try
    {
        if (GenApi::IsAvailable(cam_->BslPeriodicSignalDelay))
        {
            cam_->BslPeriodicSignalDelay.SetValue(value);
            return "done";
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "Error while trying to set periodic signal delay. The connected camera does not support this feature.");
            return "The connected camera does not support this feature";
        }

    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "An exception while setting periodic signal delay:" << e.GetDescription());
        return e.GetDescription();
    }
}

std::string PylonROS2GigEAce2Camera::enablePTPManagementProtocol(const bool& value)
{
    try
    {
        if (GenApi::IsAvailable(cam_->BslPtpManagementEnable))
        {
            cam_->BslPtpManagementEnable.SetValue(value);   
            return "done";
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "Error while trying to enable/disable PTP management protocol. The connected camera does not support this feature.");
            return "The connected camera does not support this feature";
        }

    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "An exception while enabling/disabling PTP management protocol:" << e.GetDescription());
        return e.GetDescription();
    }
}

std::string PylonROS2GigEAce2Camera::enablePTPTwoStepOperation(const bool& value)
{
    try
    {
        if (GenApi::IsAvailable(cam_->BslPtpTwoStep))
        {
            cam_->BslPtpTwoStep.SetValue(value);   
            return "done";
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "Error while trying to enable/disable PTP two step operation. The connected camera does not support this feature.");
            return "The connected camera does not support this feature";
        }

    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "An exception while enabling/disabling PTP two step operation:" << e.GetDescription());
        return e.GetDescription();
    }
}

std::string PylonROS2GigEAce2Camera::enablePTP(const bool& value)
{
    try
    {
        if (GenApi::IsAvailable(cam_->PtpEnable))
        {
            cam_->PtpEnable.SetValue(value);   
            return "done";
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "Error while trying to enable/disable PTP. The connected camera does not support this feature.");
            return "The connected camera does not support this feature";
        }

    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "An exception while enabling/disabling PTP occurred:" << e.GetDescription());
        return e.GetDescription();
    }
}

std::string PylonROS2GigEAce2Camera::setActionTriggerConfiguration(const int& action_device_key, const int& action_group_key, const unsigned int& action_group_mask,
                                                                   const int& registration_mode, const int& cleanup)
{
    try
    {
        Pylon::ERegistrationMode reg_mode;
        switch (registration_mode)
        {
            case 1:
                reg_mode = Pylon::ERegistrationMode::RegistrationMode_Append;
                break;
            case 2:
                reg_mode = Pylon::ERegistrationMode::RegistrationMode_ReplaceAll;
                break;
            default:
                return "The registration mode value specified by the user is invalid";
        }

        Pylon::ECleanup cu;
        switch (cleanup)
        {
            case 1:
                cu = Pylon::ECleanup::Cleanup_None;
                break;
            case 2:
                cu = Pylon::ECleanup::Cleanup_Delete;
                break;
            default:
                return "The cleanup value specified by the user is invalid";
        }

        cam_->RegisterConfiguration(new Pylon::CActionTriggerConfiguration(action_device_key, action_group_key, action_group_mask), reg_mode, cu);

        RCLCPP_INFO_STREAM(LOGGER_GIGE_ACE2, "Action trigger configuration has been set successfully: " 
                                          << "Device key -> " << action_device_key << ", "
                                          << "Group key -> " << action_group_key << ", "
                                          << "Group mask -> " << std::hex << action_group_mask << ", "
                                          << "Registration mode -> " << (reg_mode == Pylon::ERegistrationMode::RegistrationMode_Append ? "RegistrationMode_Append" : "RegistrationMode_ReplaceAll") << ", "
                                          << "Cleanup -> " << (cu == Pylon::ECleanup::Cleanup_None ? "Cleanup_None" : "Cleanup_Delete"));
        
        return "done";
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "An exception while setting action trigger connfiguration occurred: " << e.GetDescription());
        RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "The connected camera may not support this feature");
        return e.GetDescription();
    }
}

std::string PylonROS2GigEAce2Camera::issueActionCommand(const int& device_key, const int& group_key, const unsigned int& group_mask, const std::string& broadcast_address)
{
    try
    {
        // get transport layer
        Pylon::CTlFactory & factory(Pylon::CTlFactory::GetInstance());
        Pylon::IGigETransportLayer* pTl = dynamic_cast<Pylon::IGigETransportLayer*>(factory.CreateTl(Pylon::BaslerGigEDeviceClass));

        // Send an action command to the cameras
        pTl->IssueActionCommand(device_key, group_key, group_mask, broadcast_address.c_str());

        RCLCPP_INFO_STREAM(LOGGER_GIGE_ACE2, "Action command has been issued successfully: " 
                                        << "Device key -> " << device_key << ", "
                                        << "Group key -> " << group_key << ", "
                                        << "Group mask -> " << std::hex << group_mask << ", "
                                        << "Broadcast address -> " << broadcast_address);
        
        return "done";
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "An exception while issuing an action command occurred: " << e.GetDescription());
        RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "The connected camera may not support this feature");
        return e.GetDescription();
    }
}

std::string PylonROS2GigEAce2Camera::issueScheduledActionCommand(const int& device_key, const int& group_key, const unsigned int& group_mask, const int64_t& action_time_ns_from_current_timestamp, const std::string& broadcast_address)
{
    try
    {
        // get transport layer
        Pylon::CTlFactory & factory(Pylon::CTlFactory::GetInstance());
        Pylon::IGigETransportLayer* pTl = dynamic_cast<Pylon::IGigETransportLayer*>(factory.CreateTl(Pylon::BaslerGigEDeviceClass));

        // Get the current timestamp of the first camera
        // NOTE: All cameras must be synchronized via Precision Time Protocol
        cam_->GevTimestampControlLatch.Execute();
        int64_t current_timestamp = cam_->GevTimestampValue.GetValue();
        // Specify that the command will be executed roughly 30 seconds
        // (30 000 000 000 ticks) after the current timestamp.
        int64_t action_time = current_timestamp + action_time_ns_from_current_timestamp;
        // Send a scheduled action command to the cameras
        pTl->IssueScheduledActionCommand(device_key, group_key, group_mask, action_time, broadcast_address.c_str());

        RCLCPP_INFO_STREAM(LOGGER_GIGE_ACE2, "Scheduled action command has been issued successfully: " 
                                        << "Device key -> " << device_key << ", "
                                        << "Group key -> " << group_key << ", "
                                        << "Group mask -> " << std::hex << group_mask << ", "
                                        << "Action time from current timestamp -> " << (double)action_time_ns_from_current_timestamp << " ns, "
                                        << "Broadcast address -> " << broadcast_address);
        
        return "done";
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "An exception while issuing a scheduled action command occurred: " << e.GetDescription());
        RCLCPP_ERROR_STREAM(LOGGER_GIGE_ACE2, "The connected camera may not support this feature");
        return e.GetDescription();
    }
}

}  // namespace pylon_ros2_camera
