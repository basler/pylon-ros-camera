/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (C) 2016, Magazino GmbH. All rights reserved.
 *
 * Improved by drag and bot GmbH (www.dragandbot.com), 2019
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Magazino GmbH nor the names of its
 *     contributors may be used to endorse or promote products derived from
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

#ifndef PYLON_CAMERA_INTERNAL_GIGE_ACE2_H_
#define PYLON_CAMERA_INTERNAL_GIGE_ACE2_H_

#include <string>
#include <vector>

#include <pylon_camera/internal/impl/pylon_camera_gige.hpp>

namespace pylon_camera
{

class PylonGigEAce2Camera : public PylonGigECamera
{
public:
    explicit PylonGigEAce2Camera(Pylon::IPylonDevice* device);
    virtual ~PylonGigEAce2Camera();

    virtual bool applyCamSpecificStartupSettings(const PylonCameraParameter& params);
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

PylonGigEAce2Camera::PylonGigEAce2Camera(Pylon::IPylonDevice* device) :
    PylonGigECamera(device)
{}

PylonGigEAce2Camera::~PylonGigEAce2Camera()
{}

bool PylonGigEAce2Camera::applyCamSpecificStartupSettings(const PylonCameraParameter& parameters)
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
                double upper_lim = std::min(parameters.auto_exp_upper_lim_,
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
                    ROS_INFO_STREAM("Cam has binning range: x(hz) = ["
                            << cam_->BinningHorizontal.GetMin() << " - "
                            << cam_->BinningHorizontal.GetMax() << "], y(vt) = ["
                            << cam_->BinningVertical.GetMin() << " - "
                            << cam_->BinningVertical.GetMax() << "].");
                }
                else
                {
                    ROS_INFO_STREAM("Cam does not support binning.");
                }

                ROS_INFO_STREAM("Cam has exposure time range: ["
                        << cam_->ExposureTimeAbs.GetMin()
                        << " - " << cam_->ExposureTimeAbs.GetMax()
                        << "] measured in microseconds.");
                ROS_INFO_STREAM("Cam has gain range: ["
                        << cam_->Gain.GetMin() << " - "
                        << cam_->Gain.GetMax()
                        << "] measured in device specific units.");

                // Check if gamma is available, print range
                if ( !GenApi::IsAvailable(cam_->Gamma) )
                {
                    ROS_WARN("Cam gamma not available, will keep the default (auto).");
                }
                else
                {
                    ROS_INFO_STREAM("Cam has gamma range: ["
                        << cam_->Gamma.GetMin() << " - "
                        << cam_->Gamma.GetMax() << "].");
                }

                ROS_INFO_STREAM("Cam has pylon auto brightness range: ["
                        << cam_->AutoTargetValue.GetMin() << " - "
                        << cam_->AutoTargetValue.GetMax()
                        << "] which is the average pixel intensity.");

                // raise inter-package delay (GevSCPD) for solving error:
                // 'the image buffer was incompletely grabbed'
                // also in ubuntu settings -> network -> options -> MTU Size
                // from 'automatic' to 3000 if card supports it
                // Raspberry PI has MTU = 1500, max value for some cards: 9000
                ROS_WARN("setting MTU");
                cam_->GevSCPSPacketSize.SetValue(parameters.mtu_size_);
                ROS_WARN("MTU Setted");
                if (parameters.auto_flash_)
                {
                    std::map<int, bool> flash_on_lines;
                    ROS_INFO("Flash 2: %i", parameters.auto_flash_line_2_);
                    ROS_INFO("Flash 3: %i", parameters.auto_flash_line_3_);

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
                ROS_WARN("Default User Setting Loaded");
            }
        else if (parameters.startup_user_set_ == "UserSet1")
            {
                cam_->UserSetSelector.SetValue(Basler_UniversalCameraParams::UserSetSelector_UserSet1);
                cam_->UserSetLoad.Execute();
                cam_->GevSCPSPacketSize.SetValue(parameters.mtu_size_);
                ROS_WARN("User Set 1 Loaded");
            } 
        else if (parameters.startup_user_set_ == "UserSet2")
            {
                cam_->UserSetSelector.SetValue(Basler_UniversalCameraParams::UserSetSelector_UserSet2);
                cam_->UserSetLoad.Execute();
                cam_->GevSCPSPacketSize.SetValue(parameters.mtu_size_);
                ROS_WARN("User Set 2 Loaded");
            } 
        else if (parameters.startup_user_set_ == "UserSet3")
            {
                cam_->UserSetSelector.SetValue(Basler_UniversalCameraParams::UserSetSelector_UserSet3);
                cam_->UserSetLoad.Execute();
                cam_->GevSCPSPacketSize.SetValue(parameters.mtu_size_);
                ROS_WARN("User Set 3 Loaded");
            } 
        else if (parameters.startup_user_set_ == "CurrentSetting")
            {
                cam_->GevSCPSPacketSize.SetValue(parameters.mtu_size_);
                ROS_WARN("No User Set Is selected, Camera current setting will be used");
            }
    }
    catch ( const GenICam::GenericException &e )
    {
        ROS_ERROR_STREAM("Error applying cam specific startup setting for GigE cameras: "
                << e.GetDescription());
        return false;
    }
    return true;
}


GenApi::IFloat& PylonGigEAce2Camera::gain()
{
    if ( GenApi::IsAvailable(cam_->Gain) )
    {
        return cam_->Gain; // GigE ace 1
    } else
    {
        throw std::runtime_error("Error while accessing Gain in PylonGigEAce2Camera");
    }
}

GenApi::IFloat& PylonGigEAce2Camera::autoGainLowerLimit()
{
    if ( GenApi::IsAvailable(cam_->AutoGainLowerLimit) )
    {
        return cam_->AutoGainLowerLimit;
    }
    else
    {
        throw std::runtime_error("Error while accessing AutoGainLowerLimit in PylonGigEAce2Camera");
    }
}


GenApi::IFloat& PylonGigEAce2Camera::autoGainUpperLimit()
{
    if ( GenApi::IsAvailable(cam_->AutoGainUpperLimit) )
    {
        return cam_->AutoGainUpperLimit;
    }
    else
    {
        throw std::runtime_error("Error while accessing AutoGainUpperLimit in PylonGigEAce2Camera");
    }
}


float PylonGigEAce2Camera::currentGain()
{
    float curr_gain = (static_cast<float>(gain().GetValue()) - static_cast<float>(gain().GetMin())) /
        (static_cast<float>(gain().GetMax() - static_cast<float>(gain().GetMin())));
    return curr_gain;
}

GenApi::IFloat& PylonGigEAce2Camera::resultingFrameRate()
{
    if ( GenApi::IsAvailable(cam_->ResultingFrameRate) )
    {
        return cam_->ResultingFrameRate;
    }
    else
    {
        throw std::runtime_error("Error while accessing ResultingFrameRate in PylonGigECamera");
    }
}

std::string PylonGigEAce2Camera::setBlackLevel(const int& value)
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
             ROS_ERROR_STREAM("Error while trying to change the image black level. The connected Camera not supporting this feature");
             return "The connected Camera not supporting this feature";
        }

    }
    catch ( const GenICam::GenericException &e )
    {
        ROS_ERROR_STREAM("An exception while changing the image black level occurred:" << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}


int PylonGigEAce2Camera::getBlackLevel()
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
        ROS_ERROR_STREAM("An exception while getting the image black level occurred:" << e.GetDescription());
        return -10000;
    }
}


std::string PylonGigEAce2Camera::setNoiseReduction(const float& value)
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
            ROS_ERROR_STREAM("Error while trying to change the noise reduction value. The connected Camera not supporting this feature");
            return "The connected Camera not supporting this feature";
        }

    }
    catch ( const GenICam::GenericException &e )
    {
        ROS_ERROR_STREAM("An exception while setting the noise reduction value occurred:" << e.GetDescription());
        return e.GetDescription();
    }
}


float PylonGigEAce2Camera::getNoiseReduction()
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


std::string PylonGigEAce2Camera::setSharpnessEnhancement(const float& value)
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
            ROS_ERROR_STREAM("Error while trying to change the sharpness enhancement value. The connected Camera not supporting this feature");
            return "The connected Camera not supporting this feature";
        }

    }
    catch ( const GenICam::GenericException &e )
    {
        ROS_ERROR_STREAM("An exception while setting the sharpness enhancement value occurred:" << e.GetDescription());
        return e.GetDescription();
    }
}


float PylonGigEAce2Camera::getSharpnessEnhancement()
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


std::string PylonGigEAce2Camera::setTriggerDelay(const float& delayValue)
{
    try
    {   if ( GenApi::IsAvailable(cam_->TriggerDelay) )
        {

            cam_->TriggerDelay.SetValue(delayValue);
        }
        else 
        {
            ROS_ERROR_STREAM("Error while trying to change the trigger delay. The connected Camera not supporting this feature");
            return "The connected Camera not supporting this feature";
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        ROS_ERROR_STREAM("An exception while setting the trigger delay occurred:" << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

float PylonGigEAce2Camera::getTriggerDelay()
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


std::string PylonGigEAce2Camera::setAcquisitionFrameCount(const int& frameCount)
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
             ROS_ERROR_STREAM("Error while trying to change the Acquisition frame count. The connected Camera not supporting this feature");
             return "The connected Camera not supporting this feature";
        }

    }
    catch ( const GenICam::GenericException &e )
    {
        ROS_ERROR_STREAM("An exception while changing Acquisition frame count occurred:" << e.GetDescription());
        return e.GetDescription();
    }
}


int PylonGigEAce2Camera::getAcquisitionFrameCount()
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


std::string PylonGigEAce2Camera::setDeviceLinkThroughputLimitMode(const bool& turnOn)
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
             ROS_ERROR_STREAM("Error while trying to change the device link throughput limit mode. The connected Camera not supporting this feature");
             return "The connected Camera not supporting this feature";
        }

    }
    catch ( const GenICam::GenericException &e )
    {
        ROS_ERROR_STREAM("An exception while changing the device link throughput limit mode occurred:" << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}


int PylonGigEAce2Camera::getDeviceLinkThroughputLimitMode()
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


std::string PylonGigEAce2Camera::setDeviceLinkThroughputLimit(const int& limit)
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
             ROS_ERROR_STREAM("Error while trying to change the device link throughput limit. The connected Camera not supporting this feature");
             return "The connected Camera not supporting this feature";
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        ROS_ERROR_STREAM("An exception while changing the device link throughput limit occurred:" << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}


bool PylonGigEAce2Camera::setGain(const float& target_gain, float& reached_gain)
{
    try
    {
        cam_->GainAuto.SetValue(GainAutoEnums::GainAuto_Off);
        float truncated_gain = target_gain;
        if ( truncated_gain < 0.0 )
        {
            ROS_WARN_STREAM("Desired gain (" << target_gain << ") in "
                << "percent out of range [0.0 - 1.0]! Setting to lower "
                << "limit: 0.0");
            truncated_gain = 0.0;
        }
        else if ( truncated_gain > 1.0 )
        {
            ROS_WARN_STREAM("Desired gain (" << target_gain << ") in "
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
        ROS_ERROR_STREAM("An exception while setting target gain to "
               << target_gain << " occurred: " << e.GetDescription());
        return false;
    }
    return true;
}

std::string PylonGigEAce2Camera::typeName() const
{
    return "GIGE2";
}

}  // namespace pylon_camera

#endif  // PYLON_CAMERA_INTERNAL_GIGE_ACE2_H_