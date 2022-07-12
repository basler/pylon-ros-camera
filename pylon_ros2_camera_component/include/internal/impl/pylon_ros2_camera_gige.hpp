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
#include <pylon/gige/GigETransportLayer.h>
#include <pylon/gige/ActionTriggerConfiguration.h>


namespace pylon_ros2_camera
{

namespace
{
    static const rclcpp::Logger LOGGER_GIGE = rclcpp::get_logger("basler.pylon.ros2.pylon_ros2_gige_camera");
}

struct GigECameraTrait
{
    typedef Pylon::CBaslerUniversalInstantCamera CBaslerInstantCameraT;
    typedef Basler_UniversalCameraParams::ExposureAutoEnums ExposureAutoEnums;
    typedef Basler_UniversalCameraParams::GainAutoEnums GainAutoEnums;
    typedef Basler_UniversalCameraParams::PixelFormatEnums PixelFormatEnums;
    typedef Basler_UniversalCameraParams::PixelSizeEnums PixelSizeEnums;
    typedef GenApi::IInteger AutoTargetBrightnessType;

    // In GigE ace 1, gain is defined through GainRaw as integer (dB). With the new GigE ace2, it is a float.
    // Therefore now both of them will use floats and convert at the end to integer when necessary
    typedef GenApi::IInteger GainType; 

    typedef int64_t AutoTargetBrightnessValueType;
    typedef Basler_UniversalCameraParams::ShutterModeEnums ShutterModeEnums;
    typedef Basler_UniversalCameraParams::UserOutputSelectorEnums UserOutputSelectorEnums;
    typedef Basler_UniversalCameraParams::LineSelectorEnums LineSelectorEnums;
    typedef Basler_UniversalCameraParams::LineModeEnums LineModeEnums;
    typedef Basler_UniversalCameraParams::LineSourceEnums LineSourceEnums;
    typedef Basler_UniversalCameraParams::AcquisitionStatusSelectorEnums AcquisitionStatusSelectorEnums;
    typedef Basler_UniversalCameraParams::SensorReadoutModeEnums SensorReadoutModeEnums;
    typedef Basler_UniversalCameraParams::TriggerSelectorEnums TriggerSelectorEnums;
    typedef Basler_UniversalCameraParams::TriggerModeEnums TriggerModeEnums;
    typedef Basler_UniversalCameraParams::TriggerSourceEnums TriggerSourceEnums;
    typedef Basler_UniversalCameraParams::TriggerActivationEnums TriggerActivationEnums;
    typedef Basler_UniversalCameraParams::LineSourceEnums DeviceLinkThroughputLimitModeEnums;
    typedef Basler_UniversalCameraParams::AutoFunctionAOISelectorEnums AutoFunctionROISelectorEnums;
    typedef Basler_UniversalCameraParams::BalanceWhiteAutoEnums BalanceWhiteAutoEnums;
    typedef Basler_UniversalCameraParams::LightSourceSelectorEnums LightSourcePresetEnums;
    typedef Basler_UniversalCameraParams::DemosaicingModeEnums DemosaicingModeEnums;
    typedef Basler_UniversalCameraParams::PgiModeEnums PgiModeEnums;
    typedef Basler_UniversalCameraParams::UserSetSelectorEnums UserSetSelectorEnums;
    typedef Basler_UniversalCameraParams::UserSetDefaultSelectorEnums UserSetDefaultSelectorEnums;
    typedef Basler_UniversalCameraParams::LineFormatEnums LineFormatEnums;
    typedef Basler_UniversalCameraParams::GammaSelectorEnums GammaSelectorEnums;
    typedef Basler_UniversalCameraParams::BalanceRatioSelectorEnums BalanceRatioSelectorEnums;
    typedef Basler_UniversalCameraParams::TimerSelectorEnums TimerSelectorEnums;
    typedef Basler_UniversalCameraParams::TimerTriggerSourceEnums TimerTriggerSourceEnums;

    static inline AutoTargetBrightnessValueType convertBrightness(const int& value)
    {
        return value;
    }
};

typedef PylonROS2CameraImpl<GigECameraTrait> PylonROS2GigECamera;

template <>
bool PylonROS2GigECamera::setAutoflash(const std::map<int, bool> flash_on_lines)
{
    // bool acc_auto_flash = false;
    for (const std::pair<int, bool> p : flash_on_lines)
    {
        try
        {
            //cam_->StartGrabbing();
            grabbingStarting();
            cam_->StopGrabbing();
            RCLCPP_INFO(LOGGER_GIGE, "Executing SetAutoFlash: %i -> %i", p.first, p.second);
            if (p.first == 2)
            {
                if (p.second)
                {
                    // Set to flash
                    cam_->LineSelector.SetValue(Basler_UniversalCameraParams::LineSelector_Line2);
                    cam_->LineMode.SetValue(Basler_UniversalCameraParams::LineMode_Output);
                    cam_->LineSource.SetValue(Basler_UniversalCameraParams::LineSource_ExposureActive);
                }
                else
                {
                    // Set to default
                    cam_->LineSelector.SetValue(Basler_UniversalCameraParams::LineSelector_Line2);
                    cam_->LineMode.SetValue(Basler_UniversalCameraParams::LineMode_Output);
                    cam_->LineSource.SetValue(Basler_UniversalCameraParams::LineSource_UserOutput1);
                }
                continue;
            }
            if (p.first == 3)
            {
                if (p.second)
                {
                    // Set to flash
                    cam_->LineSelector.SetValue(Basler_UniversalCameraParams::LineSelector_Line3);
                    cam_->LineMode.SetValue(Basler_UniversalCameraParams::LineMode_Output);
                    cam_->LineSource.SetValue(Basler_UniversalCameraParams::LineSource_ExposureActive);
                }
                else
                {
                    // Set to default
                    cam_->LineSelector.SetValue(Basler_UniversalCameraParams::LineSelector_Line3);
                    cam_->LineMode.SetValue(Basler_UniversalCameraParams::LineMode_Input);
                }
                continue;
           }
           RCLCPP_WARN(LOGGER_GIGE, "Got request to set Flash for line %i, but only 2 and 3 are implemented!", p.first);
        }
        catch ( const GenICam::GenericException &e )
        {
            RCLCPP_ERROR_STREAM(LOGGER_GIGE, "Error applying cam specific startup setting for GigE cameras: "
                    << e.GetDescription());
        }
    }
    return true;
}

template <>
bool PylonROS2GigECamera::applyCamSpecificStartupSettings(const PylonROS2CameraParameter& parameters)
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

                //cam_->AutoGainRawLowerLimit.SetValue(cam_->GainRaw.GetMin());
                //cam_->AutoGainRawUpperLimit.SetValue(cam_->GainRaw.GetMax());

                // The gain auto function and the exposure auto function can be used at the
                // same time. In this case, however, you must also set the
                // Auto Function Profile feature.
                // acA1920-40gm does not support Basler_UniversalCameraParams::GainSelector_AnalogAll
                // has Basler_UniversalCameraParams::GainSelector_All instead
                // cam_->GainSelector.SetValue(Basler_UniversalCameraParams::GainSelector_AnalogAll);

                if ( GenApi::IsAvailable(cam_->BinningHorizontal) &&
                     GenApi::IsAvailable(cam_->BinningVertical) )
                {
                    RCLCPP_INFO_STREAM(LOGGER_GIGE, "Cam has binning range: x(hz) = ["
                            << cam_->BinningHorizontal.GetMin() << " - "
                            << cam_->BinningHorizontal.GetMax() << "], y(vt) = ["
                            << cam_->BinningVertical.GetMin() << " - "
                            << cam_->BinningVertical.GetMax() << "].");
                }
                else
                {
                    RCLCPP_INFO_STREAM(LOGGER_GIGE, "Cam does not support binning.");
                }

                RCLCPP_INFO_STREAM(LOGGER_GIGE, "Cam has exposure time range: ["
                        << cam_->ExposureTimeAbs.GetMin()
                        << " - " << cam_->ExposureTimeAbs.GetMax()
                        << "] measured in microseconds.");
                RCLCPP_INFO_STREAM(LOGGER_GIGE, "Cam has gain range: ["
                        << cam_->GainRaw.GetMin() << " - "
                        << cam_->GainRaw.GetMax()
                        << "] measured in device specific units.");

                // Check if gamma is available, print range
                if ( !GenApi::IsAvailable(cam_->Gamma) )
                {
                    RCLCPP_WARN(LOGGER_GIGE, "Cam gamma not available, will keep the default (auto).");
                }
                else
                {
                    RCLCPP_INFO_STREAM(LOGGER_GIGE, "Cam has gamma range: ["
                        << cam_->Gamma.GetMin() << " - "
                        << cam_->Gamma.GetMax() << "].");
                }
                if ( GenApi::IsAvailable(cam_->AutoTargetValue) )
                {
                    RCLCPP_INFO_STREAM(LOGGER_GIGE, "Cam has pylon auto brightness range: ["
                        << cam_->AutoTargetValue.GetMin() << " - "
                        << cam_->AutoTargetValue.GetMax()
                        << "] which is the average pixel intensity.");
                } 
                else if ( GenApi::IsAvailable(cam_->AutoTargetBrightness) )
                {
                    RCLCPP_INFO_STREAM(LOGGER_GIGE, "Cam has pylon auto brightness range: ["
                        << cam_->AutoTargetBrightness.GetMin() << " - "
                        << cam_->AutoTargetBrightness.GetMax()
                        << "] which is the average pixel intensity.");
                }
                

                // raise inter-package delay (GevSCPD) for solving error:
                // 'the image buffer was incompletely grabbed'
                // also in ubuntu settings -> network -> options -> MTU Size
                // from 'automatic' to 3000 if card supports it
                // single-board computers have MTU = 1500, max value for some cards: 9000
                RCLCPP_WARN(LOGGER_GIGE, "setting MTU");
                cam_->GevSCPSPacketSize.SetValue(parameters.mtu_size_);
                RCLCPP_WARN(LOGGER_GIGE, "MTU Setted");
                if (parameters.auto_flash_)
                {
                    std::map<int, bool> flash_on_lines;
                    RCLCPP_INFO(LOGGER_GIGE, "Flash 2: %i", parameters.auto_flash_line_2_);
                    RCLCPP_INFO(LOGGER_GIGE, "Flash 3: %i", parameters.auto_flash_line_3_);

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
                RCLCPP_WARN(LOGGER_GIGE, "Default User Setting Loaded");
            }
        else if (parameters.startup_user_set_ == "UserSet1")
            {
                cam_->UserSetSelector.SetValue(Basler_UniversalCameraParams::UserSetSelector_UserSet1);
                cam_->UserSetLoad.Execute();
                cam_->GevSCPSPacketSize.SetValue(parameters.mtu_size_);
                RCLCPP_WARN(LOGGER_GIGE, "User Set 1 Loaded");
            } 
        else if (parameters.startup_user_set_ == "UserSet2")
            {
                cam_->UserSetSelector.SetValue(Basler_UniversalCameraParams::UserSetSelector_UserSet2);
                cam_->UserSetLoad.Execute();
                cam_->GevSCPSPacketSize.SetValue(parameters.mtu_size_);
                RCLCPP_WARN(LOGGER_GIGE, "User Set 2 Loaded");
            } 
        else if (parameters.startup_user_set_ == "UserSet3")
            {
                cam_->UserSetSelector.SetValue(Basler_UniversalCameraParams::UserSetSelector_UserSet3);
                cam_->UserSetLoad.Execute();
                cam_->GevSCPSPacketSize.SetValue(parameters.mtu_size_);
                RCLCPP_WARN(LOGGER_GIGE, "User Set 3 Loaded");
            } 
        else if (parameters.startup_user_set_ == "CurrentSetting")
            {
                cam_->GevSCPSPacketSize.SetValue(parameters.mtu_size_);
                RCLCPP_WARN(LOGGER_GIGE, "No user set is provided -> Camera current setting will be applied");
            }
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE, "Error applying camera specific startup setting for GigE cameras: "
                << e.GetDescription());
        return false;
    }
    return true;
}

template <>
bool PylonROS2GigECamera::setupSequencer(const std::vector<float>& exposure_times,
                                     std::vector<float>& exposure_times_set)
{
    try
    {
        if ( GenApi::IsWritable(cam_->SequenceEnable) )
        {
            cam_->SequenceEnable.SetValue(false);
        }
        else
        {
            RCLCPP_ERROR(LOGGER_GIGE, "Sequence mode not enabled.");
            return false;
        }

        cam_->SequenceAdvanceMode = Basler_UniversalCameraParams::SequenceAdvanceMode_Auto;
        cam_->SequenceSetTotalNumber = exposure_times.size();

        for ( std::size_t i = 0; i < exposure_times.size(); ++i )
        {
            // Set parameters for each step
            cam_->SequenceSetIndex = i;
            float reached_exposure;
            setExposure(exposure_times.at(i), reached_exposure);
            exposure_times_set.push_back(reached_exposure / 1000000.);
            cam_->SequenceSetStore.Execute();
        }

        // config finished
        cam_->SequenceEnable.SetValue(true);
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR(LOGGER_GIGE, "%s", e.GetDescription());
        return false;
    }
    return true;
}

template <>
GenApi::IFloat& PylonROS2GigECamera::exposureTime()
{
    if ( GenApi::IsAvailable(cam_->ExposureTimeAbs) )
    {
        return cam_->ExposureTimeAbs; // GigE ace 1
    } 
    else if (GenApi::IsAvailable(cam_->ExposureTime)) 
    {
        return cam_->ExposureTime; // GigE ace 2 (pylon 6)
    }
    else
    {
        throw std::runtime_error("Error while accessing ExposureTimeAbs in PylonROS2GigECamera");
    }
}

template <>
GigECameraTrait::GainType& PylonROS2GigECamera::gain()
{
    if ( GenApi::IsAvailable(cam_->GainRaw) )
    {
        return cam_->GainRaw; // GigE ace 1
    }
    /*else if ( GenApi::IsAvailable(cam_->Gain) )
    {
        return cam_->Gain; // GigE ace 2 (pylon 6)
    }*/
    else
    {
        throw std::runtime_error("Error while accessing GainRaw in PylonROS2GigECamera");
    }
}

/**
 * @override
 * Overrides the base implementation as the Gamma object might not be available
 * for some GigE color cameras when the 'AUTO GAMMA' is activated (see setGamma()).
 *
 * @returns -1 if Gamma is set to AUTO, returns gamma value if Gamma is set to USER.
 */
template <>
float PylonROS2GigECamera::currentGamma()
{
    if ( !GenApi::IsAvailable(cam_->Gamma) )
    {
        //RCLCPP_WARN_STREAM(LOGGER_GIGE, "Error while trying to access gamma: cam.Gamma NodeMap"<< " is not available!");
        // return -1 in case of Gamma selector set to SRGB
        return -1.;
    }
    else
    {
        return static_cast<float>(gamma().GetValue());
    }
}

template <>
bool PylonROS2GigECamera::setGamma(const float& target_gamma, float& reached_gamma)
{
    // for GigE cameras you have to enable gamma first
    if ( GenApi::IsAvailable(cam_->GammaEnable) )
    {
        cam_->GammaEnable.SetValue(true);
    }

    if ( !GenApi::IsAvailable(cam_->Gamma) )
    {
        RCLCPP_WARN_STREAM(LOGGER_GIGE, "Error while trying to set gamma: cam.Gamma NodeMap is"
                << " not available!");
        return true;
    }

    if ( GenApi::IsAvailable(cam_->GammaSelector) )
    {
        // set gamma selector to USER, so that the gamma value has an influence
        try
        {
            cam_->GammaSelector.SetValue(Basler_UniversalCameraParams::GammaSelector_User);
        }
        catch ( const GenICam::GenericException &e )
        {
            RCLCPP_ERROR_STREAM(LOGGER_GIGE, "An exception while setting gamma selector to"
                    << " USER occurred: " << e.GetDescription());
            return false;
        }
    }
    
    try
    {
        float gamma_to_set = target_gamma;
        if ( gamma().GetMin() > gamma_to_set )
        {
            gamma_to_set = gamma().GetMin();
            RCLCPP_WARN_STREAM(LOGGER_GIGE, "Desired gamma unreachable! Setting to lower limit: "
                                  << gamma_to_set);
        }
        else if ( gamma().GetMax() < gamma_to_set )
        {
            gamma_to_set = gamma().GetMax();
            RCLCPP_WARN_STREAM(LOGGER_GIGE, "Desired gamma unreachable! Setting to upper limit: "
                                  << gamma_to_set);
        }
        gamma().SetValue(gamma_to_set);
        reached_gamma = currentGamma();
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE, "An exception while setting target gamma to "
                << target_gamma << " occurred: " << e.GetDescription());
        return false;
    }
    return true;
}

template <>
GenApi::IFloat& PylonROS2GigECamera::autoExposureTimeLowerLimit()
{
    if ( GenApi::IsAvailable(cam_->AutoExposureTimeAbsLowerLimit) )
    {
        return cam_->AutoExposureTimeAbsLowerLimit;
    }
    else
    {
        throw std::runtime_error("Error while accessing AutoExposureTimeAbsLowerLimit in PylonROS2GigECamera");
    }
}

template <>
GenApi::IFloat& PylonROS2GigECamera::autoExposureTimeUpperLimit()
{
    if ( GenApi::IsAvailable(cam_->AutoExposureTimeAbsUpperLimit) )
    {
        return cam_->AutoExposureTimeAbsUpperLimit;
    }
    else
    {
        throw std::runtime_error("Error while accessing AutoExposureTimeAbsUpperLimit in PylonROS2GigECamera");
    }
}

template <>
GenApi::IInteger& PylonROS2GigECamera::autoGainLowerLimit()
{
    if ( GenApi::IsAvailable(cam_->AutoGainRawLowerLimit) )
    {
        return cam_->AutoGainRawLowerLimit;
    }
    else
    {
        throw std::runtime_error("Error while accessing AutoGainRawLowerLimit in PylonROS2GigECamera");
    }
}

template <>
GenApi::IInteger& PylonROS2GigECamera::autoGainUpperLimit()
{
    if ( GenApi::IsAvailable(cam_->AutoGainRawUpperLimit) )
    {
        return cam_->AutoGainRawUpperLimit;
    }
    else
    {
        throw std::runtime_error("Error while accessing AutoGainRawUpperLimit in PylonROS2GigECamera");
    }
}

template <>
GenApi::IFloat& PylonROS2GigECamera::resultingFrameRate()
{
    if ( GenApi::IsAvailable(cam_->ResultingFrameRate) )
    {
        return cam_->ResultingFrameRate;
    }
    else if ( GenApi::IsAvailable(cam_->ResultingFrameRateAbs) )
    {
        return cam_->ResultingFrameRateAbs;
    }
    else
    {
        throw std::runtime_error("Error while accessing ResultingFrameRateAbs in PylonROS2GigECamera");
    }
}

template <>
GigECameraTrait::AutoTargetBrightnessType& PylonROS2GigECamera::autoTargetBrightness()
{
    if ( GenApi::IsAvailable(cam_->AutoTargetValue) )
    {
        return cam_->AutoTargetValue;
    }
    /**else if ( GenApi::IsAvailable(cam_->AutoTargetBrightness) )
    {
        return cam_->AutoTargetBrightness;
    }**/
    else
    {
        throw std::runtime_error("Error while accessing AutoTargetValue/AutoTargetBrightness in PylonROS2GigECamera");
    }
}

template <>
std::string PylonROS2GigECamera::typeName() const
{
    return "GigE";
}

template <>
std::string PylonROS2GigECamera::setBlackLevel(const int& value)
{
    try
    {
        if ( GenApi::IsAvailable(cam_->BlackLevelRaw) )
        {
            cam_->BlackLevelRaw.SetValue(value);   
            return "done";
        }
        else 
        {
             RCLCPP_ERROR_STREAM(LOGGER_GIGE, "Error while trying to change the image black level. The connected Camera not supporting this feature");
             return "The connected Camera not supporting this feature";
        }

    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE, "An exception while changing the image black level occurred:" << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

template <>
int PylonROS2GigECamera::getBlackLevel()
{
    try
    {
        if ( GenApi::IsAvailable(cam_->BlackLevelRaw) )
        {
            return static_cast<int>(cam_->BlackLevelRaw.GetValue());     
        }
        else 
        { 
             return -10000;
        }

    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE, "An exception while getting the image black level occurred:" << e.GetDescription());
        return -10000;
    }
}

template <>
std::string PylonROS2GigECamera::setNoiseReduction(const float& value)
{
 try
    {
        if ( GenApi::IsAvailable(cam_->PgiMode) )
        {
            if (cam_->PgiMode.GetValue() == PgiModeEnums::PgiMode_On)
            {
                if ( GenApi::IsAvailable(cam_->NoiseReductionRaw) )
                {
                    cam_->NoiseReductionRaw.SetValue(value);
                    return "done";
                }
                else 
                {
                    RCLCPP_ERROR_STREAM(LOGGER_GIGE, "Error while trying to change the noise reduction value. The connected Camera not supporting this feature");
                    return "The connected Camera not supporting this feature";
                }
            }
            else
            {
                return "Error : Noise Reduction feature not available while PGI mode is off";
            }
        }
        else 
            {
                RCLCPP_ERROR_STREAM(LOGGER_GIGE, "Error while trying to change the noise reduction value. The connected Camera not supporting this feature");
                return "The connected Camera not supporting this feature";
            } 
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE, "An exception while setting the noise reduction value occurred:" << e.GetDescription());
        return e.GetDescription();
    }
}

template <>
float PylonROS2GigECamera::getNoiseReduction()
{
 try
    {
        if ( GenApi::IsAvailable(cam_->NoiseReductionRaw) )
        {
            return  static_cast<float>(cam_->NoiseReductionRaw.GetValue());
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

template <>
std::string PylonROS2GigECamera::setSharpnessEnhancement(const float& value)
{
 try
    {
        if ( GenApi::IsAvailable(cam_->PgiMode) )
        {
            if (cam_->PgiMode.GetValue() == PgiModeEnums::PgiMode_On)
            {
                if ( GenApi::IsAvailable(cam_->SharpnessEnhancementRaw) )
                {
                    cam_->SharpnessEnhancementRaw.SetValue(value);
                    return "done";
                }
                else 
                {
                    RCLCPP_ERROR_STREAM(LOGGER_GIGE, "Error while trying to change the sharpness enhancement value. The connected Camera not supporting this feature");
                    return "The connected Camera not supporting this feature";
                }
            }
            else
            {
                return "Error : Sharpness Enhancement feature not available while PGI mode is off";
            }
        }
        else 
            {
                RCLCPP_ERROR_STREAM(LOGGER_GIGE, "Error while trying to change the sharpness enhancement value. The connected Camera not supporting this feature");
                return "The connected Camera not supporting this feature";
            } 
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE, "An exception while setting the sharpness enhancement value occurred:" << e.GetDescription());
        return e.GetDescription();
    }
}

template <>
float PylonROS2GigECamera::getSharpnessEnhancement()
{
 try
    {
        if ( GenApi::IsAvailable(cam_->SharpnessEnhancementRaw) )
        {
            return static_cast<float>(cam_->SharpnessEnhancementRaw.GetValue());
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

template <>
std::string PylonROS2GigECamera::setAcquisitionFrameCount(const int& frameCount)
{
    try
    {
        if ( GenApi::IsAvailable(cam_->AcquisitionFrameCount) )
        {  
            cam_->AcquisitionFrameCount.SetValue(frameCount);   
            return "done";
        }
        else 
        {
             RCLCPP_ERROR_STREAM(LOGGER_GIGE, "Error while trying to change the Acquisition frame count. The connected Camera not supporting this feature");
             return "The connected Camera not supporting this feature";
        }

    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE, "An exception while changing Acquisition frame count occurred:" << e.GetDescription());
        return e.GetDescription();
    }
}

template <>
int PylonROS2GigECamera::getAcquisitionFrameCount()
{
    try
    {
        if ( GenApi::IsAvailable(cam_->AcquisitionFrameCount) )
        {  
            return static_cast<int>(cam_->AcquisitionFrameCount.GetValue());
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

template <>
std::string PylonROS2GigECamera::setTriggerSelector(const int& mode)
{
    try
    {
        if ( GenApi::IsAvailable(cam_->TriggerSelector) )
        {  
            if (mode == 0)
            {
                cam_->TriggerSelector.SetValue(TriggerSelectorEnums::TriggerSelector_FrameStart);
                RCLCPP_INFO_STREAM(LOGGER_GIGE, "Trigger selector: Frame Start");
                return "done";
            }
            else if (mode == 1)
            {
                cam_->TriggerSelector.SetValue(TriggerSelectorEnums::TriggerSelector_AcquisitionStart);
                RCLCPP_INFO_STREAM(LOGGER_GIGE, "Trigger selector: Acquisition Start");
                return "done";
            }
            else
            {
                return "Error: unknown value";
            }
        }
        else 
        {
             RCLCPP_ERROR_STREAM(LOGGER_GIGE, "Error while trying to change the Acquisition frame count. The connected Camera not supporting this feature");
             return "The connected Camera not supporting this feature";
        }

    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE, "An exception while changing Acquisition frame count occurred:" << e.GetDescription());
        return e.GetDescription();
    }
}

template <>
int PylonROS2GigECamera::getTriggerSelector()
{
    try
    {
        if ( GenApi::IsAvailable(cam_->TriggerSelector) )
        {  
            if (cam_->TriggerSelector.GetValue() == TriggerSelectorEnums::TriggerSelector_FrameStart)
            {
            return 0; // FrameStart
            }
            else if (cam_->TriggerSelector.GetValue() == TriggerSelectorEnums::TriggerSelector_AcquisitionStart)
            { 
            return 1; // AcquisitionStart
            }
            else
            {
                return -3; // Unknown
            }
        }
        else 
        {
             return -1; // Not avalibale 
        }

    }
    catch ( const GenICam::GenericException &e )
    {
        return -2; // Error
    }
}

template <>
std::string PylonROS2GigECamera::setTriggerSource(const int& source)
{
    try
    {   if (GenApi::IsAvailable(cam_->TriggerSource))
        {
            switch (source)
            {
                case 0:
                    cam_->TriggerSource.SetValue(TriggerSourceEnums::TriggerSource_Software);
                    RCLCPP_INFO_STREAM(LOGGER_GIGE, "Trigger source: Software");
                    break;
                case 1:
                    cam_->TriggerSource.SetValue(TriggerSourceEnums::TriggerSource_Line1);
                    RCLCPP_INFO_STREAM(LOGGER_GIGE, "Trigger source: Line 1");
                    break;
                case 2:
                    cam_->TriggerSource.SetValue(TriggerSourceEnums::TriggerSource_Line3);
                    RCLCPP_INFO_STREAM(LOGGER_GIGE, "Trigger source: Line 3");
                    break;
                case 3:
                    cam_->TriggerSource.SetValue(TriggerSourceEnums::TriggerSource_Line4);
                    RCLCPP_INFO_STREAM(LOGGER_GIGE, "Trigger source: Line 4");
                    break;
                case 4:
                    cam_->TriggerSource.SetValue(TriggerSourceEnums::TriggerSource_Action1);
                    RCLCPP_INFO_STREAM(LOGGER_GIGE, "Trigger source: Action 1");
                    break;
                default:
                    RCLCPP_ERROR_STREAM(LOGGER_GIGE, "Trigger source value is invalid! Please choose between 0 -> Trigger Software / 1 -> Line 1 / 2 -> Line 3 / 3 -> Line 4 / 4 -> Action 1");
                    return "Error: unknown value for trigger source";
            }
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_GIGE, "Error while trying to change the trigger source. The connected camera does not support this feature");
            return "The connected camera does not support this feature";
        }

    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE, "An exception while setting the trigger source occurred:" << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

template <>
int PylonROS2GigECamera::getTriggerSource()
{
    try
    {   if ( GenApi::IsAvailable(cam_->TriggerSource) )
        {
            if (cam_->TriggerSource.GetValue() == TriggerSourceEnums::TriggerSource_Software)
            {
                return 0; // Software
            }
            else if (cam_->TriggerSource.GetValue() == TriggerSourceEnums::TriggerSource_Line1)
            {
                return 1; // Line1
            }
            else if (cam_->TriggerSource.GetValue() == TriggerSourceEnums::TriggerSource_Line3)
            {
                return 2; // Line3
            }
            else if (cam_->TriggerSource.GetValue() == TriggerSourceEnums::TriggerSource_Line4)
            {
                return 3; // Line4
            }
            else if (cam_->TriggerSource.GetValue() == TriggerSourceEnums::TriggerSource_Action1)
            {
                return 4 ; // Action1
            }
            else 
            {
                return -3; // Unknown
            }
        }
        else 
        {
            return -1; // Not available
        }

    }
    catch ( const GenICam::GenericException &e )
    {
        return -2; // Error
    }
}

template <>
std::string PylonROS2GigECamera::setTriggerDelay(const float& delayValue)
{
    try
    {   if ( GenApi::IsAvailable(cam_->TriggerDelayAbs) )
        {

            cam_->TriggerDelayAbs.SetValue(delayValue);
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_GIGE, "Error while trying to change the trigger delay. The connected Camera not supporting this feature");
            return "The connected Camera not supporting this feature";
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE, "An exception while setting the trigger delay occurred:" << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

template <>
float PylonROS2GigECamera::getTriggerDelay()
{
    try
    {   if ( GenApi::IsAvailable(cam_->TriggerDelayAbs) )
        {

             return static_cast<float>(cam_->TriggerDelayAbs.GetValue());
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

template <>
std::string PylonROS2GigECamera::setLineMode(const int& value)
{
    try
    {   if ( (cam_->LineFormat.GetValue() == LineFormatEnums::LineFormat_TTL) )
        {
            if (value == 0)
            {
                cam_->LineMode.SetValue(LineModeEnums::LineMode_Input);
            }
            else if (value == 1)
            {
                cam_->LineMode.SetValue(LineModeEnums::LineMode_Output);
            }
            else 
            {
                return "Error: unknown value";
            }
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_GIGE, "Error : the selected line number dose not have change line mode feature");
            return "Error : the selected line number dose not have change line mode feature";
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE, "An exception while setting the line mode occurred:" << e.GetDescription());
        return e.GetDescription(); 
    }
    return "done";
}

template <>
std::string PylonROS2GigECamera::setLineDebouncerTime(const float& value)
{
    try
    {   if ( GenApi::IsAvailable(cam_->LineDebouncerTimeAbs) )
        {   
            if ( cam_->LineMode.GetValue() == Basler_UniversalCameraParams::LineMode_Input)
            {
                cam_->LineDebouncerTimeAbs.SetValue(value);
            }
            else 
            {
                return "Error: can't set the line debouncer time, is the selected line number is an input";
            }
            
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_GIGE, "Error while trying to set the line debouncer time. The connected Camera not supporting this feature");
            return "The connected Camera not supporting this feature";
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE, "An exception while setting the line debouncer time occurred:" << e.GetDescription());
        return e.GetDescription(); 
    }
    return "done";
}

template <>
std::string PylonROS2GigECamera::setDeviceLinkThroughputLimitMode(const bool& turnOn)
{
    (void) turnOn;
    return "Trying to change the device link throughput limit mode. The connected Camera not supporting this feature";
}

template <>
int PylonROS2GigECamera::getDeviceLinkThroughputLimitMode()
{
    return -1; // Not available
}

template <>
std::string PylonROS2GigECamera::setDeviceLinkThroughputLimit(const int& limit)
{
    (void) limit;
    return "Trying to change the device link throughput limit. The connected Camera not supporting this feature";
}

template <>
std::string PylonROS2GigECamera::setBalanceWhiteAuto(const int& mode)
{
    try
    {
        if ( GenApi::IsAvailable(cam_->BalanceWhiteAuto))
        {
            if (mode == 0)
            {
                cam_->BalanceWhiteAuto.SetValue(BalanceWhiteAutoEnums::BalanceWhiteAuto_Off);
            }  
            else if (mode == 1)
            {
                cam_->BalanceWhiteAuto.SetValue(BalanceWhiteAutoEnums::BalanceWhiteAuto_Once);
            } 
            else if (mode == 2)
            {
                cam_->BalanceWhiteAuto.SetValue(BalanceWhiteAutoEnums::BalanceWhiteAuto_Continuous);
            } 
            else 
            {
                return "Error: unknown value";
            }
        }
        else 
        {
             RCLCPP_ERROR_STREAM(LOGGER_GIGE, "Error while trying to change the balance white auto. The connected Camera not supporting this feature");
             return "The connected Camera not supporting this feature";
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE, "An exception while changing balance white auto occurred:" << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

template <>
int PylonROS2GigECamera::getBalanceWhiteAuto()
{
    try
    {
        if ( GenApi::IsAvailable(cam_->AutoFunctionAOISelector) && GenApi::IsAvailable(cam_->BalanceWhiteAuto) && cam_->AutoFunctionAOISelector.GetValue() == AutoFunctionROISelectorEnums::AutoFunctionAOISelector_AOI2)
        {  
            if (cam_->BalanceWhiteAuto.GetValue() == BalanceWhiteAutoEnums::BalanceWhiteAuto_Off)
            {
                return 0; // Off
            }  
            else if (cam_->BalanceWhiteAuto.GetValue() == BalanceWhiteAutoEnums::BalanceWhiteAuto_Once)
            {
                return 1; // Once
            } 
            else if (cam_->BalanceWhiteAuto.GetValue() == BalanceWhiteAutoEnums::BalanceWhiteAuto_Continuous)
            {
                return 2; // Continuous
            } 
            else 
            {
                return -3; // Unknown
            }
        }
        else 
        {
             return -1; //Not available
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        return -2; // Error
    }
}

template <>
std::string PylonROS2GigECamera::setLightSourcePreset(const int& mode)
{
    try
    {
        if ( GenApi::IsAvailable(cam_->LightSourceSelector))
        {  
            if (mode == 0)
            {
                cam_->LightSourceSelector.SetValue(LightSourcePresetEnums::LightSourceSelector_Off);
            }  
            else if (mode == 1)
            {
                cam_->LightSourceSelector.SetValue(LightSourcePresetEnums::LightSourceSelector_Daylight); //(at about 5000K)
            } 
            else if (mode == 2)
            {
                cam_->LightSourceSelector.SetValue(LightSourcePresetEnums::LightSourceSelector_Daylight6500K);
            } 
            else if (mode == 3)
            {
                cam_->LightSourceSelector.SetValue(LightSourcePresetEnums::LightSourceSelector_Tungsten);
            } 
            else 
            {
                return "Error: unknown value";
            }
        }
        else if ( GenApi::IsAvailable(cam_->BslLightSourcePreset))
        {  
            if (mode == 0)
            {
                cam_->BslLightSourcePreset.SetValue(Basler_UniversalCameraParams::BslLightSourcePresetEnums::BslLightSourcePreset_Off );
            }  
            else if (mode == 1)
            {
                cam_->BslLightSourcePreset.SetValue(Basler_UniversalCameraParams::BslLightSourcePresetEnums::BslLightSourcePreset_Daylight5000K); //(at about 5000K)
            } 
            else if (mode == 2)
            {
                cam_->BslLightSourcePreset.SetValue(Basler_UniversalCameraParams::BslLightSourcePresetEnums::BslLightSourcePreset_Daylight6500K );
            } 
            else if (mode == 3)
            {
                cam_->BslLightSourcePreset.SetValue(Basler_UniversalCameraParams::BslLightSourcePresetEnums::BslLightSourcePreset_Tungsten );
            } 
            else 
            {
                return "Error: unknown value";
            }
        }
        else 
        {
             RCLCPP_ERROR_STREAM(LOGGER_GIGE, "Error while trying to change the light source preset. The connected Camera not supporting this feature");
             return "The connected Camera not supporting this feature";
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE, "An exception while changing light source preset occurred:" << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

template <>
int PylonROS2GigECamera::getLightSourcePreset()
{
    try
    {
        if ( GenApi::IsAvailable(cam_->LightSourceSelector))
        {  
            if (cam_->LightSourceSelector.GetValue() == LightSourcePresetEnums::LightSourceSelector_Off)
            {
                return 0; // Off
            }  
            else if (cam_->LightSourceSelector.GetValue() == LightSourcePresetEnums::LightSourceSelector_Daylight)
            {
                return 1; //Daylight(~ Daylight5000K)
            } 
            else if (cam_->LightSourceSelector.GetValue() == LightSourcePresetEnums::LightSourceSelector_Daylight6500K)
            {
                return 2; //Daylight6500K
            } 
            else if (cam_->LightSourceSelector.GetValue() == LightSourcePresetEnums::LightSourceSelector_Tungsten)
            {
                return 3;
            } 
            else 
            {
                return -3; // Unknown
            }
        } 
        else if ( GenApi::IsAvailable(cam_->BslLightSourcePreset))
        {  
            if (cam_->BslLightSourcePreset.GetValue() == Basler_UniversalCameraParams::BslLightSourcePresetEnums::BslLightSourcePreset_Off)
            {
                return 0; // Off
            }  
            else if (cam_->BslLightSourcePreset.GetValue() == Basler_UniversalCameraParams::BslLightSourcePresetEnums::BslLightSourcePreset_Daylight5000K)
            {
                return 1; //Daylight(~ Daylight5000K)
            } 
            else if (cam_->BslLightSourcePreset.GetValue() == Basler_UniversalCameraParams::BslLightSourcePresetEnums::BslLightSourcePreset_Daylight6500K)
            {
                return 2; //Daylight6500K
            } 
            else if (cam_->BslLightSourcePreset.GetValue() == Basler_UniversalCameraParams::BslLightSourcePresetEnums::BslLightSourcePreset_Tungsten)
            {
                return 3;
            } 
            else 
            {
                return -3; // Unknown
            }
        }
        else 
        {
            return -1; // Not available
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        return -2; // Error
    }
}

template <>
std::string PylonROS2GigECamera::setUserSetDefaultSelector(const int& set)
{
    try 
    {
        if ( GenApi::IsAvailable(cam_->UserSetDefaultSelector)) 
        {  
            if (set == 0) 
            {
                cam_->UserSetDefaultSelector.SetValue(UserSetDefaultSelectorEnums::UserSetDefaultSelector_Default);
            } 
            else if (set == 1) 
            {
                cam_->UserSetDefaultSelector.SetValue(UserSetDefaultSelectorEnums::UserSetDefaultSelector_UserSet1);
            } 
            else if (set == 2) 
            {
                cam_->UserSetDefaultSelector.SetValue(UserSetDefaultSelectorEnums::UserSetDefaultSelector_UserSet2);
            } 
            else if (set == 3) 
            {
                cam_->UserSetDefaultSelector.SetValue(UserSetDefaultSelectorEnums::UserSetDefaultSelector_UserSet3);
            } 
            else if (set == 4) 
            {
                cam_->UserSetDefaultSelector.SetValue(UserSetDefaultSelectorEnums::UserSetDefaultSelector_HighGain);
            } 
            else if (set == 5) 
            {
                cam_->UserSetDefaultSelector.SetValue(UserSetDefaultSelectorEnums::UserSetDefaultSelector_AutoFunctions);
            } 
            else if (set == 6) 
            {
                cam_->UserSetDefaultSelector.SetValue(UserSetDefaultSelectorEnums::UserSetDefaultSelector_ColorRaw);
            } 
            else 
            {
                return "Error: unknown value";
            }
        } 
        else if ( GenApi::IsAvailable(cam_->UserSetDefault))
        {  
            if (set == 0)
            {
                grabbingStopping();
                cam_->UserSetDefault.SetValue(Basler_UniversalCameraParams::UserSetDefaultEnums::UserSetDefault_Default);
                grabbingStarting();
            }  
            else if (set == 1)
            {
                grabbingStopping();
                cam_->UserSetDefault.SetValue(Basler_UniversalCameraParams::UserSetDefaultEnums::UserSetDefault_UserSet1);
                grabbingStarting();
            } 
            else if (set == 2)
            {
                grabbingStopping();
                cam_->UserSetDefault.SetValue(Basler_UniversalCameraParams::UserSetDefaultEnums::UserSetDefault_UserSet2);
                grabbingStarting();
            } 
            else if (set == 3)
            {
                grabbingStopping();
                cam_->UserSetDefault.SetValue(Basler_UniversalCameraParams::UserSetDefaultEnums::UserSetDefault_UserSet3);
                grabbingStarting();
            } 
            else if (set == 4)
            {
                grabbingStopping();
                cam_->UserSetDefault.SetValue(Basler_UniversalCameraParams::UserSetDefaultEnums::UserSetDefault_HighGain);
                grabbingStarting();
            } 
            else if (set == 5)
            {
                grabbingStopping();
                cam_->UserSetDefault.SetValue(Basler_UniversalCameraParams::UserSetDefaultEnums::UserSetDefault_AutoFunctions);
                grabbingStarting();
            } 
            else if (set == 6)
            {
                grabbingStopping();
                cam_->UserSetDefault.SetValue(Basler_UniversalCameraParams::UserSetDefaultEnums::UserSetDefault_ColorRaw);
                grabbingStarting();
            } 
            else 
            {
                return "Error: unknown value";
            }
        } 
        else 
        {
             RCLCPP_ERROR_STREAM(LOGGER_GIGE, "Error while trying to select the user default set. The connected Camera not supporting this feature");
             return "The connected Camera not supporting this feature";
        }
    }
    catch ( const GenICam::GenericException &e ) 
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE, "An exception while selecting the user default set occurred:" << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

template <>
int PylonROS2GigECamera::getUserSetDefaultSelector()
{
    try
    {
        if ( GenApi::IsAvailable(cam_->UserSetDefaultSelector))
        {  
            if (cam_->UserSetDefaultSelector.GetValue() == UserSetDefaultSelectorEnums::UserSetDefaultSelector_Default)
            {
                return 0;
            }  
            else if (cam_->UserSetDefaultSelector.GetValue() == UserSetDefaultSelectorEnums::UserSetDefaultSelector_UserSet1)
            {
                return 1; // UserSet1
            } 
            else if (cam_->UserSetDefaultSelector.GetValue() == UserSetDefaultSelectorEnums::UserSetDefaultSelector_UserSet2)
            {
                return 2; // UserSet2
            } 
            else if (cam_->UserSetDefaultSelector.GetValue() == UserSetDefaultSelectorEnums::UserSetDefaultSelector_UserSet3)
            {
                return 3; // UserSet3
            } 
            else if (cam_->UserSetDefaultSelector.GetValue() == UserSetDefaultSelectorEnums::UserSetDefaultSelector_HighGain)
            {
                return 4; // HighGain
            } 
            else if (cam_->UserSetDefaultSelector.GetValue() == UserSetDefaultSelectorEnums::UserSetDefaultSelector_AutoFunctions)
            {
                return 5; // AutoFunctions
            } 
            else if (cam_->UserSetDefaultSelector.GetValue() == UserSetDefaultSelectorEnums::UserSetDefaultSelector_ColorRaw)
            {
                return 6; // ColorRaw
            } 
            else 
            {
                return -3; // Unknown
            }
        } 
        else if ( GenApi::IsAvailable(cam_->UserSetDefault)) 
        {  
            if (cam_->UserSetDefault.GetValue() == Basler_UniversalCameraParams::UserSetDefaultEnums::UserSetDefault_Default) 
            {
                return 0; // Default
            } 
            else if (cam_->UserSetDefault.GetValue() == Basler_UniversalCameraParams::UserSetDefaultEnums::UserSetDefault_UserSet1) 
            {
                return 1; // UserSet1
            } 
            else if (cam_->UserSetDefault.GetValue() == Basler_UniversalCameraParams::UserSetDefaultEnums::UserSetDefault_UserSet2) 
            {
                return 2; // UserSet2
            } 
            else if (cam_->UserSetDefault.GetValue() == Basler_UniversalCameraParams::UserSetDefaultEnums::UserSetDefault_UserSet3) 
            {
                return 3; // UserSet3
            } 
            else if (cam_->UserSetDefault.GetValue() == Basler_UniversalCameraParams::UserSetDefaultEnums::UserSetDefault_HighGain) 
            {
                return 4; // HighGain
            } 
            else if (cam_->UserSetDefault.GetValue() == Basler_UniversalCameraParams::UserSetDefaultEnums::UserSetDefault_AutoFunctions) 
            {
                return 5; // AutoFunctions
            } 
            else if (cam_->UserSetDefault.GetValue() == Basler_UniversalCameraParams::UserSetDefaultEnums::UserSetDefault_ColorRaw) 
            {
                return 6; // ColorRaw
            } 
            else 
            {
                return -3; // Unknown
            }
        }
        else 
        {
             return -1; // Not available
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        return -2; // Error
    }
}

template <>
std::string PylonROS2GigECamera::setMaxTransferSize(const int& maxTransferSize)
{
    (void) maxTransferSize;
    return "Error, this feature supported by USB Cameras only";

}

template <>
std::string PylonROS2GigECamera::setGammaSelector(const int& gammaSelector)
{
try
    {
        if ( GenApi::IsAvailable(cam_->GammaSelector))
        {  
            if (gammaSelector == 0)
            {
                cam_-> GammaSelector.SetValue(Basler_UniversalCameraParams::GammaSelector_User);
                return "done";
            }  
            else if (gammaSelector == 1)
            {
                cam_-> GammaSelector.SetValue(Basler_UniversalCameraParams::GammaSelector_sRGB );
                return "done";
            } 
            else 
            {
                return "Error: unknown value";
            }
        }
        else 
        {
             RCLCPP_ERROR_STREAM(LOGGER_GIGE, "Error while trying to set the gamma selector. The connected Camera not supporting this feature");
             return "The connected Camera not supporting this feature";
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE, "An exception while setting the gamma selector occurred:" << e.GetDescription());
        return e.GetDescription();
    }
}

template <>
std::string PylonROS2GigECamera::gammaEnable(const bool& enable)
{
    try
    {
        if (GenApi::IsAvailable(cam_->GammaEnable))
        {
            cam_->GammaEnable.SetValue(enable);
            return "done";
        }
        else
        {
            RCLCPP_ERROR_STREAM(LOGGER_GIGE, "Error while trying to enable/disable the gamma. The connected Camera not supporting this feature");
            return "The connected Camera not supporting this feature";
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE, "An exception while to change gamma enable occurred:" << e.GetDescription());
        grabbingStarting();
        return e.GetDescription();
    }
}

template <> 
float PylonROS2GigECamera::getTemperature()
{
    try
    {
        if ( GenApi::IsAvailable(cam_->TemperatureAbs) )
        {  
            return static_cast<float>(cam_->TemperatureAbs.GetValue());   
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
std::string PylonROS2GigECamera::setWhiteBalance(const double& redValue, const double& greenValue, const double& blueValue)
{
    try
    {
        if (GenApi::IsAvailable(cam_->BalanceWhiteAuto) && GenApi::IsAvailable(cam_->BalanceRatioAbs)) 
        {
            cam_->BalanceWhiteAuto.SetValue(BalanceWhiteAutoEnums::BalanceWhiteAuto_Off);
            cam_->BalanceRatioSelector.SetValue(BalanceRatioSelectorEnums::BalanceRatioSelector_Red);
            cam_->BalanceRatioAbs.SetValue(redValue);
            cam_->BalanceRatioSelector.SetValue(BalanceRatioSelectorEnums::BalanceRatioSelector_Green);
            cam_->BalanceRatioAbs.SetValue(greenValue);
            cam_->BalanceRatioSelector.SetValue(BalanceRatioSelectorEnums::BalanceRatioSelector_Blue);
            cam_->BalanceRatioAbs.SetValue(blueValue);

            return "done";
        } 
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_GIGE, "Error while trying to set the white balance. The connected Camera not supporting this feature");
            return "The connected Camera not supporting this feature";
        }
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE, "An exception while setting the white balance occurred:" << e.GetDescription());
        return e.GetDescription();
    }
}

template <>
std::string PylonROS2GigECamera::setTimerSelector(const int& selector)
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
                case 3:
                    cam_->TimerSelector.SetValue(TimerSelectorEnums::TimerSelector_Timer3);
                    break;
                case 4:
                    cam_->TimerSelector.SetValue(TimerSelectorEnums::TimerSelector_Timer4);
                    break;
                default:
                    RCLCPP_ERROR_STREAM(LOGGER_GIGE, "Timer selector value is invalid! Please choose between 1 -> Timer 1 / 2 -> Timer 2 / 3 -> Timer 3 / 4 -> Timer 4");
                    return "Error: unknown value for timer selector";
            }
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_GIGE, "Error while trying to change the timer selector. The connected camera does not support this feature");
            return "The connected camera does not support this feature";
        }
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE, "An exception while setting the timer selector occurred:" << e.GetDescription());
        return e.GetDescription();
    }
    
    return "done";
}

template <>
std::string PylonROS2GigECamera::setSyncFreeRunTimerStartTimeLow(const int& value)
{
    try
    {
        if (GenApi::IsAvailable(cam_->SyncFreeRunTimerStartTimeLow))
        {
            cam_->SyncFreeRunTimerStartTimeLow.SetValue(value);   
            return "done";
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_GIGE, "Error while trying to set sync. free run timer start time low. The connected camera does not support this feature.");
            return "The connected camera does not support this feature";
        }

    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE, "An exception while setting sync. free run timer start time low:" << e.GetDescription());
        return e.GetDescription();
    }
}

template <>
std::string PylonROS2GigECamera::setSyncFreeRunTimerStartTimeHigh(const int& value)
{
    try
    {
        if (GenApi::IsAvailable(cam_->SyncFreeRunTimerStartTimeHigh))
        {
            cam_->SyncFreeRunTimerStartTimeHigh.SetValue(value);   
            return "done";
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_GIGE, "Error while trying to set sync. free run timer start time high. The connected camera does not support this feature.");
            return "The connected camera does not support this feature";
        }

    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE, "An exception while setting sync. free run timer start time high:" << e.GetDescription());
        return e.GetDescription();
    }
}

template <>
std::string PylonROS2GigECamera::setSyncFreeRunTimerTriggerRateAbs(const float& value)
{
    try
    {
        if (GenApi::IsAvailable(cam_->SyncFreeRunTimerTriggerRateAbs))
        {
            cam_->SyncFreeRunTimerTriggerRateAbs.SetValue(value);
            return "done";
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_GIGE, "Error while trying to set sync. free run timer trigger rate abs. The connected camera does not support this feature.");
            return "The connected camera does not support this feature";
        }

    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE, "An exception while setting sync. free run timer trigger rate abs:" << e.GetDescription());
        return e.GetDescription();
    }
}

template <> 
std::string PylonROS2GigECamera::enablePTP(const bool& value)
{
    try
    {
        if (GenApi::IsAvailable(cam_->GevIEEE1588))
        {
            cam_->GevIEEE1588.SetValue(value);
            return "done";
        }
        else
        {
            RCLCPP_ERROR_STREAM(LOGGER_GIGE, "Error while trying to enable/disable PTP. The connected camera does not support this feature.");
            return "The connected camera does not support this feature";
        }
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE, "An exception while enabling/disabling PTP occurred: " << e.GetDescription());
        return e.GetDescription();
    }
}

template <> 
std::string PylonROS2GigECamera::enableSyncFreeRunTimer(const bool& value)
{
    try
    {
        if (GenApi::IsAvailable(cam_->SyncFreeRunTimerEnable))
        {
            cam_->SyncFreeRunTimerEnable.SetValue(value);
            return "done";
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_GIGE, "Error while trying to enable/disable sync. free run timer. The connected camera does not support this feature.");
            return "The connected camera does not support this feature";
        }
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE, "An exception while enabling/disabling sync. free run timer:" << e.GetDescription());
        return e.GetDescription();
    }
}

template <> 
std::string PylonROS2GigECamera::updateSyncFreeRunTimer()
{
    try
    {
        if (GenApi::IsAvailable(cam_->SyncFreeRunTimerUpdate))
        {
            cam_->SyncFreeRunTimerUpdate.Execute();
            return "done";
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_GIGE, "Error while trying to update sync. free run timer. The connected camera does not support this feature.");
            return "The connected camera does not support this feature";
        }
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE, "An exception while updating sync. free run timer:" << e.GetDescription());
        return e.GetDescription();
    }
}

template <> 
std::string PylonROS2GigECamera::setActionTriggerConfiguration(const int& action_device_key, const int& action_group_key, const unsigned int& action_group_mask,
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

        RCLCPP_INFO_STREAM(LOGGER_GIGE, "Action trigger configuration has been set successfully: " 
                                        << "Device key -> " << action_device_key << ", "
                                        << "Group key -> " << action_group_key << ", "
                                        << "Group mask -> " << std::hex << action_group_mask << ", "
                                        << "Registration mode -> " << (reg_mode == Pylon::ERegistrationMode::RegistrationMode_Append ? "RegistrationMode_Append" : "RegistrationMode_ReplaceAll") << ", "
                                        << "Cleanup -> " << (cu == Pylon::ECleanup::Cleanup_None ? "Cleanup_None" : "Cleanup_Delete"));
        
        return "done";
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE, "An exception while setting action trigger connfiguration occurred: " << e.GetDescription());
        RCLCPP_ERROR_STREAM(LOGGER_GIGE, "The connected camera may not support this feature");
        return e.GetDescription();
    }
}

template <> 
std::string PylonROS2GigECamera::issueActionCommand(const int& device_key, const int& group_key, const unsigned int& group_mask, const std::string& broadcast_address)
{
    try
    {
        // get transport layer
        Pylon::CTlFactory & factory(Pylon::CTlFactory::GetInstance());
        Pylon::IGigETransportLayer* pTl = dynamic_cast<Pylon::IGigETransportLayer*>(factory.CreateTl(Pylon::BaslerGigEDeviceClass));

        // Send an action command to the cameras
        pTl->IssueActionCommand(device_key, group_key, group_mask, broadcast_address.c_str());

        RCLCPP_INFO_STREAM(LOGGER_GIGE, "Action command has been issued successfully: " 
                                        << "Device key -> " << device_key << ", "
                                        << "Group key -> " << group_key << ", "
                                        << "Group mask -> " << std::hex << group_mask << ", "
                                        << "Broadcast address -> " << broadcast_address);
        
        return "done";
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE, "An exception while issuing an action command occurred: " << e.GetDescription());
        RCLCPP_ERROR_STREAM(LOGGER_GIGE, "The connected camera may not support this feature");
        return e.GetDescription();
    }
}

template <>
std::string PylonROS2GigECamera::issueScheduledActionCommand(const int& device_key, const int& group_key, const unsigned int& group_mask, const int64_t& action_time_ns_from_current_timestamp, const std::string& broadcast_address)
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

        RCLCPP_INFO_STREAM(LOGGER_GIGE, "Scheduled action command has been issued successfully: " 
                                        << "Device key -> " << device_key << ", "
                                        << "Group key -> " << group_key << ", "
                                        << "Group mask -> " << std::hex << group_mask << ", "
                                        << "Action time from current timestamp -> " << (double)action_time_ns_from_current_timestamp << " ns, "
                                        << "Broadcast address -> " << broadcast_address);
        
        return "done";
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_GIGE, "An exception while issuing a scheduled action command occurred: " << e.GetDescription());
        RCLCPP_ERROR_STREAM(LOGGER_GIGE, "The connected camera may not support this feature");
        return e.GetDescription();
    }
}

}  // namespace pylon_ros2_camera
