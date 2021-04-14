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

#ifndef PYLON_CAMERA_INTERNAL_GIGE_H_
#define PYLON_CAMERA_INTERNAL_GIGE_H_

#include <string>
#include <vector>

#include <pylon_camera/internal/pylon_camera.h>
#include <pylon/BaslerUniversalInstantCamera.h>

namespace pylon_camera
{

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


    static inline AutoTargetBrightnessValueType convertBrightness(const int& value)
    {
        return value;
    }
};

typedef PylonCameraImpl<GigECameraTrait> PylonGigECamera;


template <>
bool PylonGigECamera::setAutoflash(const std::map<int, bool> flash_on_lines)
{
    // bool acc_auto_flash = false;
    for (const std::pair<int, bool> p : flash_on_lines)
    {
        try
        {
            //cam_->StartGrabbing();
            grabbingStarting();
            cam_->StopGrabbing();
            ROS_INFO("Executing SetAutoFlash: %i -> %i", p.first, p.second);
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
           ROS_WARN("Got request to set Flash for line %i, but only 2 and 3 are implemented!", p.first);
        }
        catch ( const GenICam::GenericException &e )
        {
            ROS_ERROR_STREAM("Error applying cam specific startup setting for GigE cameras: "
                    << e.GetDescription());
        }
    }
    return true;
}

template <>
bool PylonGigECamera::applyCamSpecificStartupSettings(const PylonCameraParameter& parameters)
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
                        << cam_->GainRaw.GetMin() << " - "
                        << cam_->GainRaw.GetMax()
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
                if ( GenApi::IsAvailable(cam_->AutoTargetValue) )
                {
                    ROS_INFO_STREAM("Cam has pylon auto brightness range: ["
                        << cam_->AutoTargetValue.GetMin() << " - "
                        << cam_->AutoTargetValue.GetMax()
                        << "] which is the average pixel intensity.");
                } 
                else if ( GenApi::IsAvailable(cam_->AutoTargetBrightness) )
                {
                    ROS_INFO_STREAM("Cam has pylon auto brightness range: ["
                        << cam_->AutoTargetBrightness.GetMin() << " - "
                        << cam_->AutoTargetBrightness.GetMax()
                        << "] which is the average pixel intensity.");
                }
                

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


template <>
bool PylonGigECamera::setupSequencer(const std::vector<float>& exposure_times,
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
            ROS_ERROR("Sequence mode not enabled.");
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
        ROS_ERROR("%s", e.GetDescription());
        return false;
    }
    return true;
}

template <>
GenApi::IFloat& PylonGigECamera::exposureTime()
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
        throw std::runtime_error("Error while accessing ExposureTimeAbs in PylonGigECamera");
    }
}


template <>
GigECameraTrait::GainType& PylonGigECamera::gain()
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
        throw std::runtime_error("Error while accessing GainRaw in PylonGigECamera");
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
float PylonGigECamera::currentGamma()
{
    if ( !GenApi::IsAvailable(cam_->Gamma) )
    {
        //ROS_WARN_STREAM("Error while trying to access gamma: cam.Gamma NodeMap"<< " is not available!");
        // return -1 in case of Gamma selector set to SRGB
        return -1.;
    }
    else
    {
        return static_cast<float>(gamma().GetValue());
    }
}

template <>
bool PylonGigECamera::setGamma(const float& target_gamma, float& reached_gamma)
{
    // for GigE cameras you have to enable gamma first
    if ( GenApi::IsAvailable(cam_->GammaEnable) )
    {
        cam_->GammaEnable.SetValue(true);
    }

    if ( !GenApi::IsAvailable(cam_->Gamma) )
    {
        ROS_WARN_STREAM("Error while trying to set gamma: cam.Gamma NodeMap is"
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
            ROS_ERROR_STREAM("An exception while setting gamma selector to"
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
GenApi::IFloat& PylonGigECamera::autoExposureTimeLowerLimit()
{
    if ( GenApi::IsAvailable(cam_->AutoExposureTimeAbsLowerLimit) )
    {
        return cam_->AutoExposureTimeAbsLowerLimit;
    }
    else
    {
        throw std::runtime_error("Error while accessing AutoExposureTimeAbsLowerLimit in PylonGigECamera");
    }
}

template <>
GenApi::IFloat& PylonGigECamera::autoExposureTimeUpperLimit()
{
    if ( GenApi::IsAvailable(cam_->AutoExposureTimeAbsUpperLimit) )
    {
        return cam_->AutoExposureTimeAbsUpperLimit;
    }
    else
    {
        throw std::runtime_error("Error while accessing AutoExposureTimeAbsUpperLimit in PylonGigECamera");
    }
}

template <>
GenApi::IInteger& PylonGigECamera::autoGainLowerLimit()
{
    if ( GenApi::IsAvailable(cam_->AutoGainRawLowerLimit) )
    {
        return cam_->AutoGainRawLowerLimit;
    }
    else
    {
        throw std::runtime_error("Error while accessing AutoGainRawLowerLimit in PylonGigECamera");
    }
}

template <>
GenApi::IInteger& PylonGigECamera::autoGainUpperLimit()
{
    if ( GenApi::IsAvailable(cam_->AutoGainRawUpperLimit) )
    {
        return cam_->AutoGainRawUpperLimit;
    }
    else
    {
        throw std::runtime_error("Error while accessing AutoGainRawUpperLimit in PylonGigECamera");
    }
}

template <>
GenApi::IFloat& PylonGigECamera::resultingFrameRate()
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
        throw std::runtime_error("Error while accessing ResultingFrameRateAbs in PylonGigECamera");
    }
}

template <>
GigECameraTrait::AutoTargetBrightnessType& PylonGigECamera::autoTargetBrightness()
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
        throw std::runtime_error("Error while accessing AutoTargetValue/AutoTargetBrightness in PylonGigECamera");
    }
}

template <>
std::string PylonGigECamera::typeName() const
{
    return "GigE";
}

template <>
std::string PylonGigECamera::setBlackLevel(const int& value)
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

template <>
int PylonGigECamera::getBlackLevel()
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
        ROS_ERROR_STREAM("An exception while getting the image black level occurred:" << e.GetDescription());
        return -10000;
    }
}

template <>
std::string PylonGigECamera::setNoiseReduction(const float& value)
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
                    ROS_ERROR_STREAM("Error while trying to change the noise reduction value. The connected Camera not supporting this feature");
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

template <>
float PylonGigECamera::getNoiseReduction()
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
std::string PylonGigECamera::setSharpnessEnhancement(const float& value)
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
                    ROS_ERROR_STREAM("Error while trying to change the sharpness enhancement value. The connected Camera not supporting this feature");
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

template <>
float PylonGigECamera::getSharpnessEnhancement()
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
std::string PylonGigECamera::setAcquisitionFrameCount(const int& frameCount)
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
int PylonGigECamera::getAcquisitionFrameCount()
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
std::string PylonGigECamera::setTriggerSelector(const int& mode)
{
    try
    {
        if ( GenApi::IsAvailable(cam_->TriggerSelector) )
        {  
            if (mode == 0)
            {
            cam_->TriggerSelector.SetValue(TriggerSelectorEnums::TriggerSelector_FrameStart);   
            return "done";
            }
            else if (mode == 1)
            {
            cam_->TriggerSelector.SetValue(TriggerSelectorEnums::TriggerSelector_AcquisitionStart);   
            return "done";
            }
            else
            {
                return "Error: unknown value";
            }
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
int PylonGigECamera::getTriggerSelector()
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
std::string PylonGigECamera::setTriggerSource(const int& source)
{
    try
    {   if ( GenApi::IsAvailable(cam_->TriggerSource) )
        {
            if (source == 0)
            {
                cam_->TriggerSource.SetValue(TriggerSourceEnums::TriggerSource_Software);
            }
            else if (source == 1)
            {
                cam_->TriggerSource.SetValue(TriggerSourceEnums::TriggerSource_Line1);
            }
            else if (source == 2)
            {
                cam_->TriggerSource.SetValue(TriggerSourceEnums::TriggerSource_Line3);
            }
            else if (source == 3)
            {
                cam_->TriggerSource.SetValue(TriggerSourceEnums::TriggerSource_Line4);
            }
            else if (source == 4)
            {
                cam_->TriggerSource.SetValue(TriggerSourceEnums::TriggerSource_Action1);
            }
            else 
            {
                return "Error: unknown value";
            }
        }
        else 
        {
            ROS_ERROR_STREAM("Error while trying to change the trigger mode. The connected Camera not supporting this feature");
            return "The connected Camera not supporting this feature";
        }

    }
    catch ( const GenICam::GenericException &e )
    {
        ROS_ERROR_STREAM("An exception while setting the trigger mode occurred:" << e.GetDescription());
        return e.GetDescription(); // + " Check the TriggerSelector setting";
    }
    return "done";
}

template <>
int PylonGigECamera::getTriggerSource()
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
std::string PylonGigECamera::setTriggerDelay(const float& delayValue)
{
    try
    {   if ( GenApi::IsAvailable(cam_->TriggerDelayAbs) )
        {

            cam_->TriggerDelayAbs.SetValue(delayValue);
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

template <>
float PylonGigECamera::getTriggerDelay()
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
std::string PylonGigECamera::setLineMode(const int& value)
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
            ROS_ERROR_STREAM("Error : the selected line number dose not have change line mode feature");
            return "Error : the selected line number dose not have change line mode feature";
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        ROS_ERROR_STREAM("An exception while setting the line mode occurred:" << e.GetDescription());
        return e.GetDescription(); 
    }
    return "done";
}

template <>
std::string PylonGigECamera::setLineDebouncerTime(const float& value)
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
            ROS_ERROR_STREAM("Error while trying to set the line debouncer time. The connected Camera not supporting this feature");
            return "The connected Camera not supporting this feature";
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        ROS_ERROR_STREAM("An exception while setting the line debouncer time occurred:" << e.GetDescription());
        return e.GetDescription(); 
    }
    return "done";
}

template <>
std::string PylonGigECamera::setDeviceLinkThroughputLimitMode(const bool& turnOn)
{
    return "Trying to change the device link throughput limit mode. The connected Camera not supporting this feature";
}

template <>
int PylonGigECamera::getDeviceLinkThroughputLimitMode()
{
    return -1; // Not available
}

template <>
std::string PylonGigECamera::setDeviceLinkThroughputLimit(const int& limit)
{
    return "Trying to change the device link throughput limit. The connected Camera not supporting this feature";
}

template <>
std::string PylonGigECamera::setBalanceWhiteAuto(const int& mode)
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
             ROS_ERROR_STREAM("Error while trying to change the balance white auto. The connected Camera not supporting this feature");
             return "The connected Camera not supporting this feature";
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        ROS_ERROR_STREAM("An exception while changing balance white auto occurred:" << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

template <>
int PylonGigECamera::getBalanceWhiteAuto()
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
std::string PylonGigECamera::setLightSourcePreset(const int& mode)
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
             ROS_ERROR_STREAM("Error while trying to change the light source preset. The connected Camera not supporting this feature");
             return "The connected Camera not supporting this feature";
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        ROS_ERROR_STREAM("An exception while changing light source preset occurred:" << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

template <>
int PylonGigECamera::getLightSourcePreset()
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
std::string PylonGigECamera::setUserSetDefaultSelector(const int& set)
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
             ROS_ERROR_STREAM("Error while trying to select the user default set. The connected Camera not supporting this feature");
             return "The connected Camera not supporting this feature";
        }
    }
    catch ( const GenICam::GenericException &e ) 
    {
        ROS_ERROR_STREAM("An exception while selecting the user default set occurred:" << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

template <>
int PylonGigECamera::getUserSetDefaultSelector()
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
std::string PylonGigECamera::setMaxTransferSize(const int& maxTransferSize)
{
    return "Error, this feature supported by USB Cameras only";

}

template <>
std::string PylonGigECamera::setGammaSelector(const int& gammaSelector)
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
             ROS_ERROR_STREAM("Error while trying to set the gamma selector. The connected Camera not supporting this feature");
             return "The connected Camera not supporting this feature";
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        ROS_ERROR_STREAM("An exception while setting the gamma selector occurred:" << e.GetDescription());
        return e.GetDescription();
    }
}

template <>
std::string PylonGigECamera::gammaEnable(const bool& enable)
{
    try
    {
        if ( GenApi::IsAvailable(cam_->GammaEnable))
        {
            cam_->GammaEnable.SetValue(enable);
            return "done";
        }
        else
        {
            ROS_ERROR_STREAM("Error while trying to enable/disable the gamma. The connected Camera not supporting this feature");
            return "The connected Camera not supporting this feature";
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        ROS_ERROR_STREAM("An exception while to change gamma enable occurred:" << e.GetDescription());
        grabbingStarting();
        return e.GetDescription();
    }
}

template <> 
float PylonGigECamera::getTemperature(){
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
std::string PylonGigECamera::setWhiteBalance(const double& redValue, const double& greenValue, const double& blueValue){
    try
    {
        if ( GenApi::IsAvailable(cam_->BalanceWhiteAuto) && GenApi::IsAvailable(cam_->BalanceRatioAbs)) {
           cam_->BalanceWhiteAuto.SetValue(BalanceWhiteAutoEnums::BalanceWhiteAuto_Off);
            cam_->BalanceRatioSelector.SetValue(BalanceRatioSelectorEnums::BalanceRatioSelector_Red);
            cam_->BalanceRatioAbs.SetValue(redValue);
            cam_->BalanceRatioSelector.SetValue(BalanceRatioSelectorEnums::BalanceRatioSelector_Green);
            cam_->BalanceRatioAbs.SetValue(greenValue);
            cam_->BalanceRatioSelector.SetValue(BalanceRatioSelectorEnums::BalanceRatioSelector_Blue);
            cam_->BalanceRatioAbs.SetValue(blueValue);
            return "done"; 
        } else {
            ROS_ERROR_STREAM("Error while trying to set the white balance. The connected Camera not supporting this feature");
            return "The connected Camera not supporting this feature";
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        ROS_ERROR_STREAM("An exception while setting the white balance occurred:" << e.GetDescription());
        return e.GetDescription();
    }
}

}  // namespace pylon_camera

#endif  // PYLON_CAMERA_INTERNAL_GIGE_H_
