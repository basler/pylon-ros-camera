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

#include "internal/pylon_ros2_camera_impl.hpp"
#include <pylon/BaslerUniversalInstantCamera.h>


namespace pylon_ros2_camera
{

namespace
{
    static const rclcpp::Logger LOGGER_USB = rclcpp::get_logger("basler.pylon.ros2.pylon_ros2_usb_camera");
}

struct USBCameraTrait
{
    typedef Pylon::CBaslerUniversalInstantCamera CBaslerInstantCameraT;
    typedef Basler_UniversalCameraParams::ExposureAutoEnums ExposureAutoEnums;
    typedef Basler_UniversalCameraParams::GainAutoEnums GainAutoEnums;
    typedef Basler_UniversalCameraParams::PixelFormatEnums PixelFormatEnums;
    typedef Basler_UniversalCameraParams::PixelSizeEnums PixelSizeEnums;
    typedef GenApi::IFloat AutoTargetBrightnessType;
    typedef GenApi::IFloat GainType;
    typedef double AutoTargetBrightnessValueType;
    typedef Basler_UniversalCameraParams::ShutterModeEnums ShutterModeEnums;
    typedef Basler_UniversalCameraParams::UserOutputSelectorEnums UserOutputSelectorEnums;
    typedef Basler_UniversalCameraParams::AcquisitionStatusSelectorEnums AcquisitionStatusSelectorEnums;
    typedef Basler_UniversalCameraParams::SensorReadoutModeEnums SensorReadoutModeEnums;
    typedef Basler_UniversalCameraParams::TriggerSelectorEnums TriggerSelectorEnums;
    typedef Basler_UniversalCameraParams::TriggerModeEnums TriggerModeEnums;
    typedef Basler_UniversalCameraParams::TriggerSourceEnums TriggerSourceEnums;
    typedef Basler_UniversalCameraParams::TriggerActivationEnums TriggerActivationEnums;
    typedef Basler_UniversalCameraParams::LineSelectorEnums LineSelectorEnums;
    typedef Basler_UniversalCameraParams::LineModeEnums LineModeEnums;
    typedef Basler_UniversalCameraParams::DeviceLinkThroughputLimitModeEnums DeviceLinkThroughputLimitModeEnums;
    typedef Basler_UniversalCameraParams::AutoFunctionROISelectorEnums AutoFunctionROISelectorEnums;
    typedef Basler_UniversalCameraParams::BalanceWhiteAutoEnums BalanceWhiteAutoEnums;
    typedef Basler_UniversalCameraParams::LightSourcePresetEnums LightSourcePresetEnums;
    typedef Basler_UniversalCameraParams::LineSourceEnums LineSourceEnums;
    typedef Basler_UniversalCameraParams::DemosaicingModeEnums DemosaicingModeEnums;
    typedef Basler_UniversalCameraParams::PgiModeEnums PgiModeEnums;
    typedef Basler_UniversalCameraParams::UserSetSelectorEnums UserSetSelectorEnums;
    typedef Basler_UniversalCameraParams::UserSetDefaultEnums UserSetDefaultSelectorEnums;
    typedef Basler_UniversalCameraParams::LineFormatEnums LineFormatEnums;
    typedef Basler_UniversalCameraParams::BalanceRatioSelectorEnums BalanceRatioSelectorEnums;
    typedef Basler_UniversalCameraParams::TimerSelectorEnums TimerSelectorEnums;
    typedef Basler_UniversalCameraParams::TimerTriggerSourceEnums TimerTriggerSourceEnums;

    static inline AutoTargetBrightnessValueType convertBrightness(const int& value)
    {
        return value / 255.0;
    }
};

typedef PylonROS2CameraImpl<USBCameraTrait> PylonROS2USBCamera;

template <>
bool PylonROS2USBCamera::applyCamSpecificStartupSettings(const PylonROS2CameraParameter& parameters)
{
    try
    {
        //cam_->StartGrabbing();
        grabbingStarting();
        cam_->StopGrabbing();

        RCLCPP_INFO_STREAM(LOGGER_USB, "Startup user profile set to \"" << parameters.startup_user_set_ << "\"");
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
                                        cam_->ExposureTime.GetMax());
            cam_->AutoExposureTimeLowerLimit.SetValue(cam_->ExposureTime.GetMin());
            cam_->AutoExposureTimeUpperLimit.SetValue(upper_lim);
            RCLCPP_INFO_STREAM(LOGGER_USB, "Cam has upper exposure value limit range: ["
                    << cam_->ExposureTimeAbs.GetMin()
                    << " - " << upper_lim << " (max possible value from cam is " << cam_->ExposureTimeAbs.GetMax() << ")"
                    << "].");

            cam_->AutoGainLowerLimit.SetValue(cam_->Gain.GetMin());
            cam_->AutoGainUpperLimit.SetValue(cam_->Gain.GetMax());

            // The gain auto function and the exposure auto function can be used at the same time. In this case,
            // however, you must also set the Auto Function Profile feature.
            //  cam_->AutoFunctionProfile.SetValue(Basler_UniversalCameraParams::AutoFunctionProfile_MinimizeGain);

            if ( GenApi::IsAvailable(cam_->BinningHorizontal) &&
                 GenApi::IsAvailable(cam_->BinningVertical) )
            {
                RCLCPP_INFO_STREAM(LOGGER_USB, "Cam has binning range: x(hz) = ["
                        << cam_->BinningHorizontal.GetMin() << " - "
                        << cam_->BinningHorizontal.GetMax() << "], y(vt) = ["
                        << cam_->BinningVertical.GetMin() << " - "
                        << cam_->BinningVertical.GetMax() << "].");
            }
            else
            {
                RCLCPP_INFO_STREAM(LOGGER_USB, "Cam does not support binning.");
            }

            RCLCPP_INFO_STREAM(LOGGER_USB, "Cam has exposure time range: [" << cam_->ExposureTime.GetMin()
                    << " - " << cam_->ExposureTime.GetMax()
                    << "] measured in microseconds.");
            RCLCPP_INFO_STREAM(LOGGER_USB, "Cam has gain range: [" << cam_->Gain.GetMin()
                    << " - " << cam_->Gain.GetMax()
                    << "] measured in dB.");
            RCLCPP_INFO_STREAM(LOGGER_USB, "Cam has gammma range: ["
                    << cam_->Gamma.GetMin() << " - "
                    << cam_->Gamma.GetMax() << "].");
            RCLCPP_INFO_STREAM(LOGGER_USB, "Cam has pylon auto brightness range: ["
                    << cam_->AutoTargetBrightness.GetMin() * 255 << " - "
                    << cam_->AutoTargetBrightness.GetMax() * 255
                    << "] which is the average pixel intensity.");
        }
        else if (parameters.startup_user_set_ == "UserSet1")
        {
            cam_->UserSetSelector.SetValue(Basler_UniversalCameraParams::UserSetSelector_UserSet1);
            cam_->UserSetLoad.Execute();
            RCLCPP_INFO(LOGGER_USB, "User Set 1 Loaded");
        }
        else if (parameters.startup_user_set_ == "UserSet2")
        {
            cam_->UserSetSelector.SetValue(Basler_UniversalCameraParams::UserSetSelector_UserSet2);
            cam_->UserSetLoad.Execute();
            RCLCPP_INFO(LOGGER_USB, "User Set 2 Loaded");
        }
        else if (parameters.startup_user_set_ == "UserSet3")
        {
            cam_->UserSetSelector.SetValue(Basler_UniversalCameraParams::UserSetSelector_UserSet3);
            cam_->UserSetLoad.Execute();
            RCLCPP_INFO(LOGGER_USB, "User Set 3 Loaded");
        }
        else if (parameters.startup_user_set_ == "CurrentSetting")
        {
            RCLCPP_INFO(LOGGER_USB, "No user set is provided -> Camera current setting will be applied");
        }
        else
        {
            RCLCPP_WARN_STREAM(LOGGER_USB, "Unsupported startup user profile \"" << parameters.startup_user_set_ << "\", ignoring");
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_USB, "Error applying camera specific startup setting for USB camera: "
                << e.GetDescription());
        return false;
    }
    return true;
}

template <>
bool PylonROS2USBCamera::setupSequencer(const std::vector<float>& exposure_times,
                                    std::vector<float>& exposure_times_set)
{
    try
    {
        // Runtime Sequencer: cam_->IsGrabbing() ? cam_->StopGrabbing(); //10ms
        if ( GenApi::IsWritable(cam_->SequencerMode) )
        {
            cam_->SequencerMode.SetValue(Basler_UniversalCameraParams::SequencerMode_Off);
        }
        else
        {
            RCLCPP_ERROR(LOGGER_USB, "Sequencer Mode not writable");
        }

        cam_->SequencerConfigurationMode.SetValue(Basler_UniversalCameraParams::SequencerConfigurationMode_On);

        // **** valid for all sets: reset on software signal 1 ****
        int64_t initial_set = cam_->SequencerSetSelector.GetMin();

        cam_->SequencerSetSelector.SetValue(initial_set);
        cam_->SequencerPathSelector.SetValue(0);
        cam_->SequencerSetNext.SetValue(initial_set);
        cam_->SequencerTriggerSource.SetValue(Basler_UniversalCameraParams::SequencerTriggerSource_SoftwareSignal1);
        // advance on Frame Start
        cam_->SequencerPathSelector.SetValue(1);
        cam_->SequencerTriggerSource.SetValue(Basler_UniversalCameraParams::SequencerTriggerSource_FrameStart);
        // ********************************************************

        for ( std::size_t i = 0; i < exposure_times.size(); ++i )
        {
            if ( i > 0 )
            {
                cam_->SequencerSetSelector.SetValue(i);
            }

            if ( i == exposure_times.size() - 1 )  // last frame
            {
                cam_->SequencerSetNext.SetValue(0);
            }
            else
            {
                cam_->SequencerSetNext.SetValue(i + 1);
            }
            float reached_exposure;
            setExposure(exposure_times.at(i), reached_exposure);
            exposure_times_set.push_back(reached_exposure / 1000000.);
            cam_->SequencerSetSave.Execute();
        }

        // config finished
        cam_->SequencerConfigurationMode.SetValue(Basler_UniversalCameraParams::SequencerConfigurationMode_Off);
        cam_->SequencerMode.SetValue(Basler_UniversalCameraParams::SequencerMode_On);
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_USB, "ERROR while initializing pylon sequencer: "
                << e.GetDescription());
        return false;
    }
    return true;
}

template <>
GenApi::IFloat& PylonROS2USBCamera::exposureTime()
{
    if ( GenApi::IsAvailable(cam_->ExposureTime) )
    {
        return cam_->ExposureTime;
    }
    else
    {
        throw std::runtime_error("Error while accessing ExposureTime in PylonROS2USBCamera");
    }
}

template <>
USBCameraTrait::GainType& PylonROS2USBCamera::gain()
{
    if ( GenApi::IsAvailable(cam_->Gain) )
    {
        return cam_->Gain;
    }
    else
    {
        throw std::runtime_error("Error while accessing Gain in PylonROS2USBCamera");
    }
}

template <>
bool PylonROS2USBCamera::setGamma(const float& target_gamma, float& reached_gamma)
{
    if ( !GenApi::IsAvailable(cam_->Gamma) )
    {
        RCLCPP_ERROR_STREAM(LOGGER_USB, "Error while trying to set gamma: cam.Gamma NodeMap is"
               << " not available!");
        return false;
    }

    try
    {
        float gamma_to_set = target_gamma;
        if ( gamma().GetMin() > gamma_to_set )
        {
            gamma_to_set = gamma().GetMin();
            RCLCPP_WARN_STREAM(LOGGER_USB, "Desired gamma unreachable! Setting to lower limit: "
                                  << gamma_to_set);
        }
        else if ( gamma().GetMax() < gamma_to_set )
        {
            gamma_to_set = gamma().GetMax();
            RCLCPP_WARN_STREAM(LOGGER_USB, "Desired gamma unreachable! Setting to upper limit: "
                                  << gamma_to_set);
        }
        gamma().SetValue(gamma_to_set);
        reached_gamma = currentGamma();
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_USB, "An exception while setting target gamma to "
                << target_gamma << " occurred: " << e.GetDescription());
        return false;
    }
    return true;
}

template <>
GenApi::IFloat& PylonROS2USBCamera::autoExposureTimeLowerLimit()
{
    if ( GenApi::IsAvailable(cam_->AutoExposureTimeLowerLimit) )
    {
        return cam_->AutoExposureTimeLowerLimit;
    }
    else
    {
        throw std::runtime_error("Error while accessing AutoExposureTimeLowerLimit in PylonROS2USBCamera");
    }
}

template <>
GenApi::IFloat& PylonROS2USBCamera::autoExposureTimeUpperLimit()
{
    if ( GenApi::IsAvailable(cam_->AutoExposureTimeUpperLimit) )
    {
        return cam_->AutoExposureTimeUpperLimit;
    }
    else
    {
        throw std::runtime_error("Error while accessing AutoExposureTimeUpperLimit in PylonROS2USBCamera");
    }
}

template <>
USBCameraTrait::GainType& PylonROS2USBCamera::autoGainLowerLimit()
{
    if ( GenApi::IsAvailable(cam_->AutoGainLowerLimit) )
    {
        return cam_->AutoGainLowerLimit;
    }
    else
    {
        throw std::runtime_error("Error while accessing AutoGainLowerLimit in PylonROS2USBCamera");
    }
}

template <>
USBCameraTrait::GainType& PylonROS2USBCamera::autoGainUpperLimit()
{
    if ( GenApi::IsAvailable(cam_->AutoGainUpperLimit) )
    {
        return cam_->AutoGainUpperLimit;
    }
    else
    {
        throw std::runtime_error("Error while accessing AutoGainUpperLimit in PylonROS2USBCamera");
    }
}

template <>
GenApi::IFloat& PylonROS2USBCamera::resultingFrameRate()
{
    if ( GenApi::IsAvailable(cam_->ResultingFrameRate) )
    {
        return cam_->ResultingFrameRate;
    }
    else
    {
        throw std::runtime_error("Error while accessing ResultingFrameRate in PylonROS2USBCamera");
    }
}

template <>
USBCameraTrait::AutoTargetBrightnessType& PylonROS2USBCamera::autoTargetBrightness()
{
    if ( GenApi::IsAvailable(cam_->AutoTargetBrightness) )
    {
        return cam_->AutoTargetBrightness;
    }
    else
    {
        throw std::runtime_error("Error while accessing AutoTargetBrightness in PylonROS2USBCamera");
    }
}

template <>
std::string PylonROS2USBCamera::typeName() const
{
    return "USB";
}

template <>
std::string PylonROS2USBCamera::setAcquisitionFrameCount(const int& frameCount)
{
    try
    {
        if ( GenApi::IsAvailable(cam_->AcquisitionBurstFrameCount) )
        {
            cam_->AcquisitionBurstFrameCount.SetValue(frameCount);
            return "done";
        }
        else if ( GenApi::IsAvailable(cam_->AcquisitionFrameRate) )
        {
            cam_->AcquisitionFrameRate.SetValue(frameCount);
            return "done";
        }
        else
        {
             RCLCPP_ERROR_STREAM(LOGGER_USB, "Error while trying to change the Acquisition frame count. The connected Camera not supporting this feature");
             return "The connected Camera not supporting this feature";
        }

    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_USB, "An exception while changing Acquisition frame count occurred:" << e.GetDescription());
        return e.GetDescription();
    }
}

template <>
int PylonROS2USBCamera::getAcquisitionFrameCount()
{
    try
    {
        if ( GenApi::IsAvailable(cam_->AcquisitionBurstFrameCount) )
        {
            return static_cast<int>(cam_->AcquisitionBurstFrameCount.GetValue());
        }
        else if ( GenApi::IsAvailable(cam_->AcquisitionFrameRate) )
        {
            return static_cast<int>(cam_->AcquisitionFrameRate.GetValue());
        }
        else
        {
             return -10000;
        }

    }
    catch ( const GenICam::GenericException &e )
    {
        return -20000;
    }
}

template <>
std::string PylonROS2USBCamera::setGammaSelector(const int& gammaSelector)
{
    (void)gammaSelector;
    return "Error, the connect camera not supporting this feature";
}

template <>
std::string PylonROS2USBCamera::gammaEnable(const bool& enable)
{
    (void)enable;
    return "Error, the connect camera not supporting this feature";
}

template <>
float PylonROS2USBCamera::getTemperature(){
    try
    {
        if ( GenApi::IsAvailable(cam_->DeviceTemperature) )
        {
            return static_cast<float>(cam_->DeviceTemperature.GetValue());
        }
        else
        {
             return 0.0;
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        return 0.0;
    }
}

template <>
std::string PylonROS2USBCamera::setTimerSelector(const int& selector)
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
                    RCLCPP_ERROR_STREAM(LOGGER_USB, "Timer selector value is invalid! Please choose between 1 -> Timer 1 / 2 -> Timer 2");
                    return "Error: unknown value for timer selector";
            }
        }
        else
        {
            RCLCPP_ERROR_STREAM(LOGGER_USB, "Error while trying to change the timer selector. The connected camera does not support this feature");
            return "The connected camera does not support this feature";
        }
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_USB, "An exception while setting the timer selector occurred:" << e.GetDescription());
        return e.GetDescription();
    }

    return "done";
}

}  // namespace pylon_ros2_camera
