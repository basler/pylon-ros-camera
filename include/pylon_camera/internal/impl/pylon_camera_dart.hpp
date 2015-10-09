#ifndef PYLON_CAMERA_INTERNAL_DART_H_
#define PYLON_CAMERA_INTERNAL_DART_H_

#include <pylon_camera/internal/impl/pylon_camera_usb.hpp>

namespace pylon_camera
{

class PylonDARTCamera : public PylonUSBCamera
{
public:
    PylonDARTCamera(Pylon::IPylonDevice* device);
    virtual ~PylonDARTCamera();

    virtual std::string typeName() const;

    virtual bool registerCameraConfiguration(const PylonCameraParameter& params);

protected:

    virtual bool setupSequencer(const std::vector<float>& exposure_times, std::vector<float>& exposure_times_set);

    virtual bool grab(Pylon::CGrabResultPtr& grab_result);

};

PylonDARTCamera::PylonDARTCamera(Pylon::IPylonDevice* device) :
    PylonUSBCamera(device)
{
}

PylonDARTCamera::~PylonDARTCamera()
{
}

bool PylonDARTCamera::registerCameraConfiguration(const PylonCameraParameter& params)
{
    if (PylonUSBCamera::registerCameraConfiguration(params))
    {
        try
        {
            cam_->GainAuto.SetValue(Basler_UsbCameraParams::GainAuto_Off);
            cam_->Gain.SetValue(0.0);
            cam_->Gamma.SetValue(1.0);
            return true;
        }
        catch (const GenICam::GenericException &e)
        {
            std::cerr << e.GetDescription() << std::endl;
            return false;
        }
    }
    return false;
}

bool PylonDARTCamera::setupSequencer(const std::vector<float>& exposure_times, std::vector<float>& exposure_times_set)
{
    std::cerr << "SEQUENCER FOR DART CAMERAS NOT YET IMPLEMENTED!!!" << std::endl;
    return false;
}

bool PylonDARTCamera::grab(Pylon::CGrabResultPtr& grab_result)
{
    try
    {
        // /!\ The dart camera device does not support waiting for frame trigger ready
        cam_->ExecuteSoftwareTrigger();
        float timeout = 1.0 / cam_->ResultingFrameRate.GetValue() * 3000;
        cam_->RetrieveResult((int)timeout,
                             grab_result,
                             Pylon::TimeoutHandling_ThrowException);
        return true;
    }
    catch (const GenICam::GenericException &e)
    {
        if (cam_->IsCameraDeviceRemoved())
        {
            is_cam_removed_ = true;
        }
        else
        {
            std::cerr << "An image grabbing exception in pylon camera occurred: " << e.GetDescription()
                      << std::endl;
        }
    }
    return false;
}

std::string PylonDARTCamera::typeName() const
{
    return "DART";
}

}

#endif
