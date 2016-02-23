// Copyright 2015 <Magazino GmbH>

#ifndef PYLON_CAMERA_INTERNAL_GIGE_H_
#define PYLON_CAMERA_INTERNAL_GIGE_H_

#include <string>
#include <vector>

#include <pylon_camera/internal/pylon_camera.h>

#include <pylon/gige/BaslerGigEInstantCamera.h>

namespace pylon_camera
{

struct GigECameraTrait
{
    typedef Pylon::CBaslerGigEInstantCamera CBaslerInstantCameraT;
    typedef Basler_GigECameraParams::ExposureAutoEnums ExposureAutoEnums;
    typedef Basler_GigECameraParams::PixelFormatEnums PixelFormatEnums;
    typedef Basler_GigECameraParams::PixelSizeEnums PixelSizeEnums;
    typedef GenApi::IInteger AutoTargetBrightnessType;
    typedef GenApi::IInteger GainType;
    typedef int64_t AutoTargetBrightnessValueType;
    typedef Basler_GigECameraParams::ShutterModeEnums ShutterModeEnums;
    typedef Basler_GigECamera::UserOutputSelectorEnums UserOutputSelectorEnums;

    static inline AutoTargetBrightnessValueType convertBrightness(const int& value)
    {
        return value;
    }

};

typedef PylonCameraImpl<GigECameraTrait> PylonGigECamera;

template <>
bool PylonGigECamera::applyStartupSettings(const PylonCameraParameter& params)
{
    try
    {
        // Remove all previous settings (sequencer etc.)
        // Default Setting = Free-Running
        cam_->UserSetSelector.SetValue(Basler_GigECameraParams::UserSetSelector_Default);
        cam_->UserSetLoad.Execute();
        // UserSetSelector_Default overrides Software Trigger Mode !!
        cam_->TriggerSource.SetValue(Basler_GigECameraParams::TriggerSource_Software);
        cam_->TriggerMode.SetValue(Basler_GigECameraParams::TriggerMode_On);

        /* Thresholds for the AutoExposure Funcitons:
         *  - lower limit can be used to get rid of changing light conditions
         *    due to 50Hz lamps (-> 20ms cycle duration)
         *  - upper limit is to prevent motion blur
         */
        cam_->AutoExposureTimeAbsLowerLimit.SetValue(cam_->ExposureTimeAbs.GetMin());
        cam_->AutoExposureTimeAbsUpperLimit.SetValue(cam_->ExposureTimeAbs.GetMax());

        cam_->AutoGainRawLowerLimit.SetValue(cam_->GainRaw.GetMin());
        cam_->AutoGainRawUpperLimit.SetValue(cam_->GainRaw.GetMax());

        // The gain auto function and the exposure auto function can be used at the same time. In this case,
        // however, you must also set the Auto Function Profile feature.
        cam_->AutoFunctionProfile.SetValue(Basler_GigECameraParams::AutoFunctionProfile_GainMinimum);
        // acA1920-40gm does not suppert Basler_GigECameraParams::GainSelector_AnalogAll
        // has Basler_GigECameraParams::GainSelector_All instead
        // cam_->GainSelector.SetValue(Basler_GigECameraParams::GainSelector_AnalogAll);
        cam_->GainAuto.SetValue(Basler_GigECameraParams::GainAuto_Off);
        float reached_gain;
        setGain(params.gain_, reached_gain);

        ROS_INFO_STREAM("Cam has gain range: [" << cam_->GainRaw.GetMin() << " - " << cam_->GainRaw.GetMax()
                << "] measured in decive specific units. Initialiy setting to: " << cam_->GainRaw.GetValue());
        ROS_INFO_STREAM("Cam has exposure time range: [" << cam_->ExposureTimeAbs.GetMin() << " - "
                << cam_->ExposureTimeAbs.GetMax() << "] measured in microseconds. Initially setting to: "
                << cam_->ExposureTimeAbs.GetValue());

        ROS_INFO_STREAM("Cam has pylon auto brightness range: ["
                << cam_->AutoTargetValue.GetMin() << " - "
                << cam_->AutoTargetValue.GetMax() << "].");
        // Basler-Debug Day:  Read- & Write Retry-Counter should stay default (=2)
        // Linux does only support 1

        // raise inter-package delay (GevSCPD) for solving error: 'the image buffer was incompletely grabbed'
        // also in ubuntu settings -> network -> options -> MTU Size from 'automatic' to 3000 if card supports it
        // Raspberry PI has MTU = 1500, max value for some cards: 9000
        cam_->GevSCPSPacketSize.SetValue(params.mtu_size_);

        // http://www.baslerweb.com/media/documents/AW00064902000%20Control%20Packet%20Timing%20With%20Delays.pdf
        // inter package delay in ticks (? -> mathi said in nanosec) -> prevent lost frames
        // package size * n_cams + 5% overhead = inter package size
        // int n_cams = 1;
        // int inter_package_delay_in_ticks = n_cams * imageSize() * 1.05;
        // std::cout << "Inter-Package Delay" << inter_package_delay_in_ticks << std::endl;
        cam_->GevSCPD.SetValue(1000);
    }
    catch (const GenICam::GenericException &e)
    {
        ROS_ERROR("%s", e.GetDescription());
        return false;
    }
    return true;
}

template <>
bool PylonGigECamera::setupSequencer(const std::vector<float>& exposure_times, std::vector<float>& exposure_times_set)
{
    try
    {
        if (GenApi::IsWritable(cam_->SequenceEnable))
        {
            cam_->SequenceEnable.SetValue(false);
        }
        else
        {
            ROS_ERROR("Sequence mode not enabled.");
            return false;
        }

        cam_->SequenceAdvanceMode = Basler_GigECameraParams::SequenceAdvanceMode_Auto;
        cam_->SequenceSetTotalNumber = exposure_times.size();

        for (std::size_t i = 0; i < exposure_times.size(); ++i)
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
    catch (const GenICam::GenericException &e)
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
        return cam_->ExposureTimeAbs;
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
        return cam_->GainRaw;
    }
    else
    {
        throw std::runtime_error("Error while accessing GainRaw in PylonGigECamera");
    }
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
GigECameraTrait::GainType& PylonGigECamera::autoGainLowerLimit()
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
GigECameraTrait::GainType& PylonGigECamera::autoGainUpperLimit()
{
    if(GenApi::IsAvailable(cam_->AutoGainRawUpperLimit))
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
    if(GenApi::IsAvailable(cam_->ResultingFrameRateAbs))
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
    if(GenApi::IsAvailable(cam_->AutoTargetValue))
    {
        return cam_->AutoTargetValue;
    }
    else
    {
        throw std::runtime_error("Error while accessing AutoTargetValue in PylonGigECamera");
    }
}

template <>
std::string PylonGigECamera::typeName() const
{
    return "GigE";
}

}  // namespace pylon_camera

#endif  // PYLON_CAMERA_INTERNAL_GIGE_H_
