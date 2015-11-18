#ifndef PYLON_CAMERA_INTERNAL_GIGE_H_
#define PYLON_CAMERA_INTERNAL_GIGE_H_

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
    typedef int64_t AutoTargetBrightnessValueType;

    static inline AutoTargetBrightnessValueType convertBrightness(const int& value)
    {
        return value;
    }
};

typedef PylonCameraImpl<GigECameraTrait> PylonGigECamera;

template <>
bool PylonGigECamera::registerCameraConfiguration(const PylonCameraParameter& params)
{
    try
    {
        cam_->RegisterConfiguration(new Pylon::CSoftwareTriggerConfiguration,
                                    Pylon::RegistrationMode_ReplaceAll,
                                    Pylon::Cleanup_Delete);
        // TODO: Sinnvolle Werte ermitteln! -> default = 2
        cam_->GetTLParams().MaxRetryCountRead.SetValue(6);
        cam_->GetTLParams().MaxRetryCountWrite.SetValue(6);
        cam_->Open();

        // Remove all previous settings (sequencer etc.)
        cam_->UserSetSelector.SetValue(Basler_GigECameraParams::UserSetSelector_Default);
        cam_->UserSetLoad.Execute();
        // UserSetSelector_Default overrides Software Trigger Mode !!
        cam_->TriggerSource.SetValue(Basler_GigECameraParams::TriggerSource_Software);
        cam_->TriggerMode.SetValue(Basler_GigECameraParams::TriggerMode_On);

        // raise inter-package delay (GevSCPD) for solving error: 'the image buffer was incompletely grabbed'
        // also in ubuntu settings -> network -> options -> MTU Size from 'automatic' to 9000 (if card supports it, else 3000)
        cam_->GevStreamChannelSelector.SetValue(Basler_GigECameraParams::GevStreamChannelSelector_StreamChannel0);

        // TODO: Sinnvolle Werte ermitteln! Ideal: Maximum = 9000
        cam_->GevSCPSPacketSize.SetValue(params.mtu_size_);

        // http://www.baslerweb.com/media/documents/AW00064902000%20Control%20Packet%20Timing%20With%20Delays.pdf
        // inter package delay in ticks -> prevent lost frames
        // package size * n_cams + overhead = inter package size
        cam_->GevSCPD.SetValue(500);
    }
    catch (const GenICam::GenericException &e)
    {
        std::cerr << e.GetDescription() << std::endl;
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
            std::cerr << "Sequence mode not enabled." << std::endl;
            return false;
        }

        cam_->SequenceAdvanceMode = Basler_GigECameraParams::SequenceAdvanceMode_Auto;
        cam_->SequenceSetTotalNumber = exposure_times.size();

        for (std::size_t i = 0; i < exposure_times.size(); ++i)
        {
            // Set parameters for each step
            cam_->SequenceSetIndex = i;
            setExposure(exposure_times.at(i));
            exposure_times_set.push_back(cam_->ExposureTimeAbs.GetValue() / 1000000.);
            cam_->SequenceSetStore.Execute();
        }

        // config finished
        cam_->SequenceEnable.SetValue(true);
    }
    catch (const GenICam::GenericException &e)
    {
        std::cerr << e.GetDescription() << std::endl;
        return false;
    }
    return true;
}

template <>
GenApi::IFloat& PylonGigECamera::exposureTime()
{
    return cam_->ExposureTimeAbs;
}

template <>
GenApi::IFloat& PylonGigECamera::resultingFrameRate()
{
    return cam_->ResultingFrameRateAbs;
}

template <>
GigECameraTrait::AutoTargetBrightnessType& PylonGigECamera::autoTargetBrightness()
{
    return cam_->AutoTargetValue;
}

template <>
std::string PylonGigECamera::typeName() const
{
    return "GigE";
}

}

#endif
