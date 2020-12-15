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

#ifndef PYLON_CAMERA_INTERNAL_USB_H_
#define PYLON_CAMERA_INTERNAL_USB_H_

#include <string>
#include <vector>

#include <pylon_camera/internal/pylon_camera.h>

#include <pylon/usb/BaslerUsbInstantCamera.h>

namespace pylon_camera
{

struct USBCameraTrait
{
    typedef Pylon::CBaslerUsbInstantCamera CBaslerInstantCameraT;
    typedef Basler_UsbCameraParams::ExposureAutoEnums ExposureAutoEnums;
    typedef Basler_UsbCameraParams::GainAutoEnums GainAutoEnums;
    typedef Basler_UsbCameraParams::PixelFormatEnums PixelFormatEnums;
    typedef Basler_UsbCameraParams::PixelSizeEnums PixelSizeEnums;
    typedef GenApi::IFloat AutoTargetBrightnessType;
    typedef GenApi::IFloat GainType;
    typedef double AutoTargetBrightnessValueType;
    typedef Basler_UsbCameraParams::ShutterModeEnums ShutterModeEnums;
    typedef Basler_UsbCameraParams::UserOutputSelectorEnums UserOutputSelectorEnums;
    typedef Basler_UsbCameraParams::AcquisitionStatusSelectorEnums AcquisitionStatusSelectorEnums;
    typedef Basler_UsbCameraParams::SensorReadoutModeEnums SensorReadoutModeEnums;
    typedef Basler_UsbCameraParams::TriggerSelectorEnums TriggerSelectorEnums;
    typedef Basler_UsbCameraParams::TriggerModeEnums TriggerModeEnums;
    typedef Basler_UsbCameraParams::TriggerSourceEnums TriggerSourceEnums;
    typedef Basler_UsbCameraParams::TriggerActivationEnums TriggerActivationEnums;
    typedef Basler_UsbCameraParams::LineSelectorEnums LineSelectorEnums;
    typedef Basler_UsbCameraParams::LineModeEnums LineModeEnums;
    typedef Basler_UsbCameraParams::DeviceLinkThroughputLimitModeEnums DeviceLinkThroughputLimitModeEnums;
    typedef Basler_UsbCameraParams::AutoFunctionROISelectorEnums AutoFunctionROISelectorEnums;
    typedef Basler_UsbCameraParams::BalanceWhiteAutoEnums BalanceWhiteAutoEnums;
    typedef Basler_UsbCameraParams::LightSourcePresetEnums LightSourcePresetEnums;
    typedef Basler_UsbCameraParams::LineSourceEnums LineSourceEnums;
    typedef Basler_UsbCameraParams::DemosaicingModeEnums DemosaicingModeEnums;
    typedef Basler_UsbCameraParams::PgiModeEnums PgiModeEnums;
    typedef Basler_UsbCameraParams::UserSetSelectorEnums UserSetSelectorEnums;
    typedef Basler_UsbCameraParams::UserSetDefaultEnums UserSetDefaultSelectorEnums;
    typedef Basler_UsbCameraParams::LineFormatEnums LineFormatEnums;
    typedef Basler_UsbCameraParams::BalanceRatioSelectorEnums BalanceRatioSelectorEnums;


    static inline AutoTargetBrightnessValueType convertBrightness(const int& value)
    {
        return value / 255.0;
    }
};

typedef PylonCameraImpl<USBCameraTrait> PylonUSBCamera;

template <>
bool PylonUSBCamera::applyCamSpecificStartupSettings(const PylonCameraParameter& parameters)
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
            cam_->UserSetSelector.SetValue(Basler_UsbCameraParams::UserSetSelector_Default);
            cam_->UserSetLoad.Execute();
            // UserSetSelector_Default overrides Software Trigger Mode !!
            cam_->TriggerSource.SetValue(Basler_UsbCameraParams::TriggerSource_Software);
            cam_->TriggerMode.SetValue(Basler_UsbCameraParams::TriggerMode_On);

             /* Thresholds for the AutoExposure Functions:
              *  - lower limit can be used to get rid of changing light conditions
              *    due to 50Hz lamps (-> 20ms cycle duration)
              *  - upper limit is to prevent motion blur
              */
            double upper_lim = std::min(parameters.auto_exp_upper_lim_,
                                        cam_->ExposureTime.GetMax());
            cam_->AutoExposureTimeLowerLimit.SetValue(cam_->ExposureTime.GetMin());
            cam_->AutoExposureTimeUpperLimit.SetValue(upper_lim);

            cam_->AutoGainLowerLimit.SetValue(cam_->Gain.GetMin());
            cam_->AutoGainUpperLimit.SetValue(cam_->Gain.GetMax());

            // The gain auto function and the exposure auto function can be used at the same time. In this case,
            // however, you must also set the Auto Function Profile feature.
            //  cam_->AutoFunctionProfile.SetValue(Basler_UsbCameraParams::AutoFunctionProfile_MinimizeGain);

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

            ROS_INFO_STREAM("Cam has exposure time range: [" << cam_->ExposureTime.GetMin()
                    << " - " << cam_->ExposureTime.GetMax()
                    << "] measured in microseconds.");
            ROS_INFO_STREAM("Cam has gain range: [" << cam_->Gain.GetMin()
                    << " - " << cam_->Gain.GetMax()
                    << "] measured in dB.");
            ROS_INFO_STREAM("Cam has gammma range: ["
                    << cam_->Gamma.GetMin() << " - "
                    << cam_->Gamma.GetMax() << "].");
            ROS_INFO_STREAM("Cam has pylon auto brightness range: ["
                    << cam_->AutoTargetBrightness.GetMin() * 255 << " - "
                    << cam_->AutoTargetBrightness.GetMax() * 255
                    << "] which is the average pixel intensity.");
            }
        else if (parameters.startup_user_set_ == "UserSet1")
            {
                cam_->UserSetSelector.SetValue(Basler_UsbCameraParams::UserSetSelector_UserSet1);
                cam_->UserSetLoad.Execute();
                ROS_WARN("User Set 1 Loaded");
            } 
        else if (parameters.startup_user_set_ == "UserSet2")
            {
                cam_->UserSetSelector.SetValue(Basler_UsbCameraParams::UserSetSelector_UserSet2);
                cam_->UserSetLoad.Execute();
                ROS_WARN("User Set 2 Loaded");
            } 
        else if (parameters.startup_user_set_ == "UserSet3")
            {
                cam_->UserSetSelector.SetValue(Basler_UsbCameraParams::UserSetSelector_UserSet3);
                cam_->UserSetLoad.Execute();
                ROS_WARN("User Set 3 Loaded");
            } 
        else if (parameters.startup_user_set_ == "CurrentSetting")
            {
                ROS_WARN("No User Set Is selected, Camera current setting will be used");
            }
    }
    catch ( const GenICam::GenericException &e )
    {
        ROS_ERROR_STREAM("Error applying cam specific startup setting for USB cameras: "
                << e.GetDescription());
        return false;
    }
    return true;
}

template <>
bool PylonUSBCamera::setupSequencer(const std::vector<float>& exposure_times,
                                    std::vector<float>& exposure_times_set)
{
    try
    {
        // Runtime Sequencer: cam_->IsGrabbing() ? cam_->StopGrabbing(); //10ms
        if ( GenApi::IsWritable(cam_->SequencerMode) )
        {
            cam_->SequencerMode.SetValue(Basler_UsbCameraParams::SequencerMode_Off);
        }
        else
        {
            ROS_ERROR("Sequencer Mode not writable");
        }

        cam_->SequencerConfigurationMode.SetValue(Basler_UsbCameraParams::SequencerConfigurationMode_On);

        // **** valid for all sets: reset on software signal 1 ****
        int64_t initial_set = cam_->SequencerSetSelector.GetMin();

        cam_->SequencerSetSelector.SetValue(initial_set);
        cam_->SequencerPathSelector.SetValue(0);
        cam_->SequencerSetNext.SetValue(initial_set);
        cam_->SequencerTriggerSource.SetValue(Basler_UsbCameraParams::SequencerTriggerSource_SoftwareSignal1);
        // advance on Frame Start
        cam_->SequencerPathSelector.SetValue(1);
        cam_->SequencerTriggerSource.SetValue(Basler_UsbCameraParams::SequencerTriggerSource_FrameStart);
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
        cam_->SequencerConfigurationMode.SetValue(Basler_UsbCameraParams::SequencerConfigurationMode_Off);
        cam_->SequencerMode.SetValue(Basler_UsbCameraParams::SequencerMode_On);
    }
    catch ( const GenICam::GenericException &e )
    {
        ROS_ERROR_STREAM("ERROR while initializing pylon sequencer: "
                << e.GetDescription());
        return false;
    }
    return true;
}

template <>
GenApi::IFloat& PylonUSBCamera::exposureTime()
{
    if ( GenApi::IsAvailable(cam_->ExposureTime) )
    {
        return cam_->ExposureTime;
    }
    else
    {
        throw std::runtime_error("Error while accessing ExposureTime in PylonUSBCamera");
    }
}

template <>
USBCameraTrait::GainType& PylonUSBCamera::gain()
{
    if ( GenApi::IsAvailable(cam_->Gain) )
    {
        return cam_->Gain;
    }
    else
    {
        throw std::runtime_error("Error while accessing Gain in PylonUSBCamera");
    }
}

template <>
bool PylonUSBCamera::setGamma(const float& target_gamma, float& reached_gamma)
{
    if ( !GenApi::IsAvailable(cam_->Gamma) )
    {
        ROS_ERROR_STREAM("Error while trying to set gamma: cam.Gamma NodeMap is"
               << " not available!");
        return false;
    }

    try
    {
        float gamma_to_set = target_gamma;
        if ( gamma().GetMin() > gamma_to_set )
        {
            gamma_to_set = gamma().GetMin();
            ROS_WARN_STREAM("Desired gamma unreachable! Setting to lower limit: "
                                  << gamma_to_set);
        }
        else if ( gamma().GetMax() < gamma_to_set )
        {
            gamma_to_set = gamma().GetMax();
            ROS_WARN_STREAM("Desired gamma unreachable! Setting to upper limit: "
                                  << gamma_to_set);
        }
        gamma().SetValue(gamma_to_set);
        reached_gamma = currentGamma();
    }
    catch ( const GenICam::GenericException &e )
    {
        ROS_ERROR_STREAM("An exception while setting target gamma to "
                << target_gamma << " occurred: " << e.GetDescription());
        return false;
    }
    return true;
}

template <>
GenApi::IFloat& PylonUSBCamera::autoExposureTimeLowerLimit()
{
    if ( GenApi::IsAvailable(cam_->AutoExposureTimeLowerLimit) )
    {
        return cam_->AutoExposureTimeLowerLimit;
    }
    else
    {
        throw std::runtime_error("Error while accessing AutoExposureTimeLowerLimit in PylonUSBCamera");
    }
}

template <>
GenApi::IFloat& PylonUSBCamera::autoExposureTimeUpperLimit()
{
    if ( GenApi::IsAvailable(cam_->AutoExposureTimeUpperLimit) )
    {
        return cam_->AutoExposureTimeUpperLimit;
    }
    else
    {
        throw std::runtime_error("Error while accessing AutoExposureTimeUpperLimit in PylonUSBCamera");
    }
}

template <>
USBCameraTrait::GainType& PylonUSBCamera::autoGainLowerLimit()
{
    if ( GenApi::IsAvailable(cam_->AutoGainLowerLimit) )
    {
        return cam_->AutoGainLowerLimit;
    }
    else
    {
        throw std::runtime_error("Error while accessing AutoGainLowerLimit in PylonUSBCamera");
    }
}

template <>
USBCameraTrait::GainType& PylonUSBCamera::autoGainUpperLimit()
{
    if ( GenApi::IsAvailable(cam_->AutoGainUpperLimit) )
    {
        return cam_->AutoGainUpperLimit;
    }
    else
    {
        throw std::runtime_error("Error while accessing AutoGainUpperLimit in PylonUSBCamera");
    }
}

template <>
GenApi::IFloat& PylonUSBCamera::resultingFrameRate()
{
    if ( GenApi::IsAvailable(cam_->ResultingFrameRate) )
    {
        return cam_->ResultingFrameRate;
    }
    else
    {
        throw std::runtime_error("Error while accessing ResultingFrameRate in PylonUSBCamera");
    }
}

template <>
USBCameraTrait::AutoTargetBrightnessType& PylonUSBCamera::autoTargetBrightness()
{
    if ( GenApi::IsAvailable(cam_->AutoTargetBrightness) )
    {
        return cam_->AutoTargetBrightness;
    }
    else
    {
        throw std::runtime_error("Error while accessing AutoTargetBrightness in PylonUSBCamera");
    }
}

template <>
std::string PylonUSBCamera::typeName() const
{
    return "USB";
}

template <>
std::string PylonUSBCamera::setAcquisitionFrameCount(const int& frameCount)
{
    try
    {
        if ( GenApi::IsAvailable(cam_->AcquisitionBurstFrameCount) )
        {  
            cam_->AcquisitionBurstFrameCount.SetValue(frameCount);   
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

template <>
int PylonUSBCamera::getAcquisitionFrameCount()
{
    try
    {
        if ( GenApi::IsAvailable(cam_->AcquisitionBurstFrameCount) )
        {  
            return static_cast<int>(cam_->AcquisitionBurstFrameCount.GetValue());
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
std::string PylonUSBCamera::setGammaSelector(const int& gammaSelector)
{
    return "Error, the connect camera not supporting this feature";
}

template <>
std::string PylonUSBCamera::gammaEnable(const bool& enable)
{
    return "Error, the connect camera not supporting this feature";
}

template <> 
float PylonUSBCamera::getTemperature(){
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

}  // namespace pylon_camera

#endif  // PYLON_CAMERA_INTERNAL_USB_H_
