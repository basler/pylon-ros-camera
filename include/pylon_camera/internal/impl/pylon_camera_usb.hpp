#ifndef PYLON_CAMERA_INTERNAL_USB_H_
#define PYLON_CAMERA_INTERNAL_USB_H_

#include <pylon_camera/internal/pylon_camera.h>

#include <pylon/usb/BaslerUsbInstantCamera.h>

namespace pylon_camera
{

struct USBCameraTrait
{
    typedef Pylon::CBaslerUsbInstantCamera CBaslerInstantCameraT;
    typedef Basler_UsbCameraParams::ExposureAutoEnums ExposureAutoEnums;
    typedef Basler_UsbCameraParams::PixelFormatEnums PixelFormatEnums;
    typedef Basler_UsbCameraParams::PixelSizeEnums PixelSizeEnums;
    typedef GenApi::IFloat AutoTargetBrightnessType;
    typedef double AutoTargetBrightnessValueType;

    static inline AutoTargetBrightnessValueType convertBrightness(const int& value)
    {
        return value / 255.0;
    }
};

typedef PylonCameraImpl<USBCameraTrait> PylonUSBCamera;

template <>
bool PylonUSBCamera::registerCameraConfiguration(const PylonCameraParameter& params)
{
    try
    {
        cam_->RegisterConfiguration(new Pylon::CSoftwareTriggerConfiguration,
                                        Pylon::RegistrationMode_ReplaceAll,
                                        Pylon::Cleanup_Delete);
        cam_->Open();
        // Remove all previous settings (sequencer etc.)
        cam_->UserSetSelector.SetValue(Basler_UsbCameraParams::UserSetSelector_Default);
        cam_->UserSetLoad.Execute();
        // UserSetSelector_Default overrides Software Trigger Mode !!
        cam_->TriggerSource.SetValue(Basler_UsbCameraParams::TriggerSource_Software);
        cam_->TriggerMode.SetValue(Basler_UsbCameraParams::TriggerMode_On);
    }
    catch (const GenICam::GenericException &e)
    {
        std::cerr << e.GetDescription() << std::endl;
        return false;
    }
    return true;
}

template <>
bool PylonUSBCamera::setupSequencer(const std::vector<float>& exposure_times, std::vector<float>& exposure_times_set)
{
    try
    {
    	// Runtime Sequencer: cam_->IsGrabbing() ? cam_->StopGrabbing(); //10ms
        if (GenApi::IsWritable(cam_->SequencerMode))
        {
            cam_->SequencerMode.SetValue(Basler_UsbCameraParams::SequencerMode_Off);
        } else
        {
            std::cerr << "Sequencer Mode not writable" << std::endl;
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

        for (std::size_t i = 0; i < exposure_times.size(); ++i)
        {
            if (i > 0)
            {
                cam_->SequencerSetSelector.SetValue(i);
            }

            if (i == exposure_times.size() - 1) // last frame
            {
                cam_->SequencerSetNext.SetValue(0);
            }
            else
            {
                cam_->SequencerSetNext.SetValue(i + 1);
            }

            setExposure(exposure_times.at(i));
            exposure_times_set.push_back(cam_->ExposureTime.GetValue() / 1000000.);
            cam_->SequencerSetSave.Execute();
        }

        // config finished
        cam_->SequencerConfigurationMode.SetValue(Basler_UsbCameraParams::SequencerConfigurationMode_Off);
        cam_->SequencerMode.SetValue(Basler_UsbCameraParams::SequencerMode_On);
    }
    catch (const GenICam::GenericException &e)
    {
        std::cerr << "ERROR while initializing pylon sequencer:" << std::endl;
        std::cerr << e.GetDescription() << std::endl;
        return false;
    }
    return true;
}

template <>
GenApi::IFloat& PylonUSBCamera::exposureTime()
{
    return cam_->ExposureTime;
}

template <>
GenApi::IFloat& PylonUSBCamera::resultingFrameRate()
{
    return cam_->ResultingFrameRate;
}

template <>
USBCameraTrait::AutoTargetBrightnessType& PylonUSBCamera::autoTargetBrightness()
{
    return cam_->AutoTargetBrightness;
}

template <>
std::string PylonUSBCamera::typeName() const
{
    return "USB";
}

}

#endif
