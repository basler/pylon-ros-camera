// Copyright 2015 <Magazino GmbH>

#ifndef PYLON_CAMERA_INTERNAL_BASE_HPP_
#define PYLON_CAMERA_INTERNAL_BASE_HPP_

#include <cmath>
#include <string>
#include <vector>

#include <pylon_camera/internal/pylon_camera.h>
#include <sensor_msgs/image_encodings.h>

namespace pylon_camera
{

template <typename CameraTraitT>
PylonCameraImpl<CameraTraitT>::PylonCameraImpl(Pylon::IPylonDevice* device) :
    PylonCamera(),
    cam_(new CBaslerInstantCameraT(device))
{}

template <typename CameraTraitT>
PylonCameraImpl<CameraTraitT>::~PylonCameraImpl()
{
    delete cam_;
    cam_ = NULL;
}

template <typename CameraTraitT>
bool PylonCameraImpl<CameraTraitT>::registerCameraConfiguration()
{
    try
    {
        cam_->RegisterConfiguration(new Pylon::CSoftwareTriggerConfiguration,
                                        Pylon::RegistrationMode_ReplaceAll,
                                        Pylon::Cleanup_Delete);
        return true;
    }
    catch (const GenICam::GenericException &e)
    {
        ROS_ERROR_STREAM(e.GetDescription());
        return false;
    }
}

template <typename CameraTraitT>
bool PylonCameraImpl<CameraTraitT>::openCamera()
{
    try
    {
        cam_->Open();
        return true;
    }
    catch (const GenICam::GenericException &e)
    {
        ROS_ERROR_STREAM(e.GetDescription());
        return false;
    }
}

template <typename CameraTraitT>
float PylonCameraImpl<CameraTraitT>::currentExposure()
{
    return static_cast<float>(exposureTime().GetValue());
}

template <typename CameraTraitT>
float PylonCameraImpl<CameraTraitT>::currentGain()
{
    float curr_gain = (static_cast<float>(gain().GetValue()) - static_cast<float>(gain().GetMin())) /
        (static_cast<float>(gain().GetMax() - static_cast<float>(gain().GetMin())));
    return curr_gain;
}

template <typename CameraTraitT>
float PylonCameraImpl<CameraTraitT>::currentAutoExposureTimeLowerLimit()
{
    return static_cast<float>(autoExposureTimeLowerLimit().GetValue());
}

template <typename CameraTraitT>
float PylonCameraImpl<CameraTraitT>::currentAutoExposureTimeUpperLimit()
{
    return static_cast<float>(autoExposureTimeUpperLimit().GetValue());
}

template <typename CameraTraitT>
float PylonCameraImpl<CameraTraitT>::currentAutoGainLowerLimit()
{
    return static_cast<float>(autoGainLowerLimit().GetValue());
}

template <typename CameraTraitT>
float PylonCameraImpl<CameraTraitT>::currentAutoGainUpperLimit()
{
    return static_cast<float>(autoGainUpperLimit().GetValue());
}

template <typename CameraTraitT>
bool PylonCameraImpl<CameraTraitT>::isPylonAutoBrightnessFunctionRunning()
{
    return (cam_->ExposureAuto.GetValue() != ExposureAutoEnums::ExposureAuto_Off);
}

template <typename CameraTraitT>
bool PylonCameraImpl<CameraTraitT>::isBrightnessSearchRunning()
{
    return isBinaryExposureSearchRunning() || (cam_->ExposureAuto.GetValue() != ExposureAutoEnums::ExposureAuto_Off);
}

template <typename CameraTraitT>
void PylonCameraImpl<CameraTraitT>::disableAllRunningAutoBrightessFunctions()
{
    is_binary_exposure_search_running_ = false;
    cam_->ExposureAuto.SetValue(ExposureAutoEnums::ExposureAuto_Off);
    if ( binary_exp_search_ )
    {
        delete binary_exp_search_;
        binary_exp_search_ = nullptr;
    }
}

template <typename CameraTraitT>
bool PylonCameraImpl<CameraTraitT>::setupSequencer(const std::vector<float>& exposure_times)
{
    std::vector<float> exposure_times_set;
    if (setupSequencer(exposure_times, exposure_times_set))
    {
        seq_exp_times_ = exposure_times_set;
        std::stringstream ss;
        ss << "Initialized sequencer with the following inverse exposure-times [1/s]: ";
        for (size_t i = 0; i < seq_exp_times_.size(); ++i)
        {
            ss << seq_exp_times_.at(i);
            if (i != seq_exp_times_.size() - 1)
            {
                ss << ", ";
            }
        }
        ROS_INFO_STREAM(ss.str());
        return true;
    }
    else
    {
        return false;
    }
}

template <typename CameraTraitT>
bool PylonCameraImpl<CameraTraitT>::startGrabbing(const PylonCameraParameter& parameters)
{
    try
    {
        if (GenApi::IsAvailable(cam_->ShutterMode))
        {
            setShutterMode(parameters.shutter_mode_);
        }

        cam_->BinningHorizontal.SetValue(parameters.binning_);
        cam_->BinningVertical.SetValue(parameters.binning_);

        cam_->StartGrabbing();

        img_rows_ = static_cast<size_t>(cam_->Height.GetValue());
        img_cols_ = static_cast<size_t>(cam_->Width.GetValue());
        image_encoding_ = cam_->PixelFormat.GetValue();
        image_pixel_depth_ = cam_->PixelSize.GetValue();
        img_size_byte_ =  img_cols_ * img_rows_ * imagePixelDepth();

        if (image_encoding_ != PixelFormatEnums::PixelFormat_Mono8)
        {
            cam_->PixelFormat.SetValue(PixelFormatEnums::PixelFormat_Mono8);
            image_encoding_ = cam_->PixelFormat.GetValue();
            ROS_WARN("Color Image support not yet implemented! Will switch to 8-Bit Mono");
        }

        grab_timeout_ = exposureTime().GetMax() * 1.05;

        // grab one image to be sure, that the communication is successful
        Pylon::CGrabResultPtr grab_result;
        grab(grab_result);
        if (grab_result.IsValid())
        {
            is_ready_ = true;
        }
        else
        {
            ROS_ERROR("PylonCamera not ready because the result of the initial grab is invalid");
        }
    }
    catch (const GenICam::GenericException &e)
    {
        ROS_ERROR_STREAM("startGrabbing: " << e.GetDescription());
        return false;
    }

    return true;
}

template <typename CameraTrait>
bool PylonCameraImpl<CameraTrait>::grab(std::vector<uint8_t>& image)
{
    Pylon::CGrabResultPtr ptr_grab_result;
    if (!grab(ptr_grab_result))
    {
        ROS_ERROR("Error: Grab was not successful");
        return false;
    }

    const uint8_t *pImageBuffer = reinterpret_cast<uint8_t*>(ptr_grab_result->GetBuffer());
    image.assign(pImageBuffer, pImageBuffer + img_size_byte_);

    if (!is_ready_)
        is_ready_ = true;

    return true;
}

template <typename CameraTrait>
bool PylonCameraImpl<CameraTrait>::grab(uint8_t* image)
{
    Pylon::CGrabResultPtr ptr_grab_result;
    if (!grab(ptr_grab_result))
    {
        ROS_ERROR("Error: Grab was not successful");
        return false;
    }

    memcpy(image, ptr_grab_result->GetBuffer(), img_size_byte_);

    return true;
}

template <typename CameraTrait>
bool PylonCameraImpl<CameraTrait>::grab(Pylon::CGrabResultPtr& grab_result)
{
    try
    {
        int timeout = 5000;  // ms

        // WaitForFrameTriggerReady to prevent trigger signal to get lost
        // this could happen, if 2xExecuteSoftwareTrigger() is only followed by 1xgrabResult()
        // -> 2nd trigger might get lost
        if (cam_->WaitForFrameTriggerReady(timeout, Pylon::TimeoutHandling_ThrowException))
        {
            cam_->ExecuteSoftwareTrigger();
        }
        else
        {
            ROS_ERROR("Error WaitForFrameTriggerReady() timed out, impossible to ExecuteSoftwareTrigger()");
            return false;
        }
        cam_->RetrieveResult(grab_timeout_, grab_result, Pylon::TimeoutHandling_ThrowException);
    }
    catch (const GenICam::GenericException &e)
    {
        if (cam_->IsCameraDeviceRemoved())
        {
            is_cam_removed_ = true;
            ROS_ERROR("Camera was removed");
        }
        else
        {
            ROS_ERROR_STREAM("An image grabbing exception in pylon camera occurred: " << e.GetDescription());
        }
        return false;
    }
    catch (...)
    {
        ROS_ERROR("An unspecified image grabbing exception in pylon camera occurred");
        return false;
    }

    if (!grab_result->GrabSucceeded())
    {
        ROS_ERROR_STREAM("Error: " << grab_result->GetErrorCode() << " " << grab_result->GetErrorDescription());
        return false;
    }

    return true;
}

template <typename CameraTraitT>
bool PylonCameraImpl<CameraTraitT>::setExposure(const float& target_exposure, float& reached_exposure)
{
    if ((target_exposure == -1.0 || target_exposure == 0.0) && !GenApi::IsAvailable(cam_->ExposureAuto) )
    {
        ROS_ERROR("Error while trying to set auto exposure properties: camera has no auto exposure function!");
        return false;
    }
    else if (!(target_exposure == -1.0 || target_exposure == 0.0) && target_exposure < 0.0)
    {
        ROS_ERROR_STREAM("Target Exposure " << target_exposure << " not in the allowed range");
        return false;
    }

    try
    {
        if (target_exposure == -1.0)
        {
            cam_->ExposureAuto.SetValue(ExposureAutoEnums::ExposureAuto_Continuous);
        }
        else if (target_exposure == 0.0)
        {
            cam_->ExposureAuto.SetValue(ExposureAutoEnums::ExposureAuto_Off);
        }
        else
        {
            cam_->ExposureAuto.SetValue(ExposureAutoEnums::ExposureAuto_Off);

            float exposure_to_set = target_exposure;
            if (exposureTime().GetMin() > exposure_to_set)
            {
                exposure_to_set = exposureTime().GetMin();
                ROS_WARN_STREAM("Desired exposure time unreachable! Setting to lower limit: "
                                  << exposure_to_set);
            }
            else if (exposureTime().GetMax() < exposure_to_set)
            {
                exposure_to_set = exposureTime().GetMax();
                ROS_WARN_STREAM("Desired exposure time unreachable! Setting to upper limit: "
                                  << exposure_to_set);
            }
            exposureTime().SetValue(exposure_to_set);
            reached_exposure = currentExposure();

            if ( fabs(reached_exposure - exposure_to_set) > exposureStep() )
            {
                // no success if the delta between target and reached exposure
                // is greater then the exposure step in ms
                return false;
            }
        }
    }
    catch (const GenICam::GenericException &e)
    {
        ROS_ERROR_STREAM("An exception while setting target exposure to "
                         << target_exposure << " occurred:"
                         << e.GetDescription());
        return false;
    }
    return true;
}

template <typename CameraTraitT>
bool PylonCameraImpl<CameraTraitT>::setGain(const float& target_gain, float& reached_gain)
{
    if ((target_gain == -1.0 || target_gain == -2.0) && !GenApi::IsAvailable(cam_->GainAuto) )
    {
        ROS_ERROR("Error while trying to set auto gain properties: camera has no auto gain function!");
        return false;
    }
    else if (!(target_gain == -1.0 || target_gain == -2.0) && target_gain < 0.0)
    {
        ROS_ERROR_STREAM("Target Gain " << target_gain << " not in the allowed range");
        return false;
    }

    try
    {
        if (target_gain == -1.0)
        {
            cam_->GainAuto.SetValue(GainAutoEnums::GainAuto_Continuous);
        }
        else if (target_gain == -2.0)
        {
            cam_->GainAuto.SetValue(GainAutoEnums::GainAuto_Off);
        }
        else
        {
            float gain_to_set = gain().GetMin() + target_gain * (gain().GetMax() - gain().GetMin());
            gain().SetValue(gain_to_set);
            ROS_INFO_STREAM("Gain to set: " << gain_to_set ); 
            reached_gain = currentGain();

          //  if ( fabs(reached_gain - gain_to_set) > exposureStep() )
          //  {
          //      // no success if the delta between target and reached exposure
          //      // is greater then the exposure step in ms
          //      return false;
          //  }
        }
    }
    catch (const GenICam::GenericException &e)
    {
        ROS_ERROR_STREAM("An exception while setting target gain to " << target_gain << " occurred:"
                         << e.GetDescription());
        return false;
    }
    return true;
}

template <typename CameraTraitT>
bool PylonCameraImpl<CameraTraitT>::setBrightness(const int& target_brightness,
                                                  const float& current_brightness)
{
    try
    {
        if ( (target_brightness == -1 || target_brightness == 0) && !GenApi::IsAvailable(cam_->ExposureAuto) )
        {
            ROS_ERROR("Error while trying to set auto brightness properties: camera has no auto exposure function!");
            return false;
        }
        else if ( !(target_brightness == -1 || target_brightness == 0) && target_brightness < 0 )
        {
            ROS_ERROR_STREAM("Target brightness " << target_brightness << " not in the allowed range");
            return false;
        }

        if ( target_brightness == 0 )
        {
            cam_->ExposureAuto.SetValue(ExposureAutoEnums::ExposureAuto_Off);
            return true;
        }

        // if the target brightness is greater 255, limit it to 255
        // the brightness_to_set is a float value, regardless of the current
        // pixel data output format, i.e., 0.0 -> black, 1.0 -> white.
        typename CameraTraitT::AutoTargetBrightnessValueType brightness_to_set =
            CameraTraitT::convertBrightness(std::min(255, target_brightness));

#if 0
        std::cout << "br = " << current_brightness << ", gain = "
            << currentGain() << ", exp = "
            << currentExposure()
            << ", curAutoExpLimits ["
            << currentAutoExposureTimeLowerLimit()
            << ", " << currentAutoExposureTimeUpperLimit()
            << "], autoBrightness [min: "
            << autoTargetBrightness().GetMin()*255 << ", max: "
            << autoTargetBrightness().GetMax()*255 << ", curr: "
            << autoTargetBrightness().GetValue()*255 << "]"
            << " curAutoGainLimits [min: "
            << currentAutoGainLowerLimit() << ", max: "
            << currentAutoGainUpperLimit() << "]" << std::endl;
#endif

        if ( isPylonAutoBrightnessFunctionRunning() )
        {
            // do nothing while the pylon-auto function is active and if the
            // desired brightness is inside the possible range
            return true;
        }

#if 0
        ROS_INFO("pylon auto finished . . .");
#endif

        if ( target_brightness == -1 )
        {
            cam_->ExposureAuto.SetValue(ExposureAutoEnums::ExposureAuto_Continuous);
            return true;
        }

        if ( autoTargetBrightness().GetMin() <= brightness_to_set &&
             autoTargetBrightness().GetMax() >= brightness_to_set )
        {
            // Use Pylon Auto Function, whenever in possible range
            // -> Own binary exposure search not necessary
            autoTargetBrightness().SetValue(brightness_to_set, true);
            cam_->ExposureAuto.SetValue(ExposureAutoEnums::ExposureAuto_Once);
        }
        else
        {
            if ( isBinaryExposureSearchRunning() )
            {
                // pre-control using the possible limits of the pylon auto function range
                // if they were reached, we continue with the extended brightness search
                if ( !setExtendedBrightness(std::min(255, target_brightness), current_brightness) )
                {
                    return false;
                }
            }
            else
            {
                // pre control to the possible pylon limits, no matter where
                // we are currently
                // This is not the best solution in case the current brightness
                // is e.g. 35 and we want to reach e.g. 33, because we first go
                // up to 50 and then start the binary exposure search to go down
                // again.
                // But in fact it's the only solution, because the exact exposure
                // times related to the min & max limit of the pylon auto function
                // are unknown, beacause they depend on the current light scene
                if ( brightness_to_set < autoTargetBrightness().GetMin() )
                {
                    // target < 50 -> pre control to 50
                    autoTargetBrightness().SetValue(autoTargetBrightness().GetMin(), true);
                    cam_->ExposureAuto.SetValue(ExposureAutoEnums::ExposureAuto_Once);
                }
                else  // target > 205 -> pre control to 205
                {
                    autoTargetBrightness().SetValue(autoTargetBrightness().GetMax(), true);
                    cam_->ExposureAuto.SetValue(ExposureAutoEnums::ExposureAuto_Once);
                }
                is_binary_exposure_search_running_ = true;
            }
        }
    }
    catch (const GenICam::GenericException &e)
    {
        ROS_ERROR_STREAM("An generic exception while setting target brightness to " << target_brightness << " (= "
                         << CameraTraitT::convertBrightness(std::min(255, target_brightness)) <<  ") occurred: "
                         << e.GetDescription());
        return false;
    }
    return true;
}

template <typename CameraTraitT>
bool PylonCameraImpl<CameraTraitT>::setExtendedBrightness(const int& target_brightness,
                                                          const float& current_brightness)
{
    assert(target_brightness > 0 && target_brightness <= 255);

    typename CameraTraitT::AutoTargetBrightnessValueType brightness_to_set =
        CameraTraitT::convertBrightness(target_brightness);

    if ( !binary_exp_search_ )
    {
        if ( brightness_to_set < autoTargetBrightness().GetMin() )  // Range from [0 - 49]
        {
            binary_exp_search_ = new BinaryExposureSearch(target_brightness,
                                                          exposureTime().GetMin(),
                                                          currentExposure(),
                                                          currentExposure());
        }
        else  // Range from [206-255]
        {
            binary_exp_search_ = new BinaryExposureSearch(target_brightness,
                                                          currentExposure(),
                                                          exposureTime().GetMax(),
                                                          currentExposure());
        }
    }

    if ( binary_exp_search_->isLimitReached() )
    {
        disableAllRunningAutoBrightessFunctions();
        ROS_ERROR_STREAM("BinaryExposureSearach reached the exposure limits that "
                      << "the camera is able to set, but the target_brightness "
                      << "was not yet reached.");
        return false;
    }

    if ( !binary_exp_search_->update(current_brightness, currentExposure()) )
    {
        disableAllRunningAutoBrightessFunctions();
        return false;
    }

    float reached_exposure;
    if ( !setExposure(binary_exp_search_->newExposure(), reached_exposure) )
    {
        return false;
    }

    // failure if we reached min or max exposure limit
    // but we return in the next cycle because maybe the current setting leads
    // to the target brightness
    if ( reached_exposure == exposureTime().GetMin() ||
         reached_exposure == exposureTime().GetMax() )
    {
        binary_exp_search_->limitReached(true);
    }
    return true;
}

template <typename CameraTraitT>
bool PylonCameraImpl<CameraTraitT>::setShutterMode(const SHUTTER_MODE &mode)
{
    try
    {
        switch (mode)
        {
        case pylon_camera::SM_ROLLING:
            cam_->ShutterMode.SetValue(ShutterModeEnums::ShutterMode_Rolling);
            break;
        case pylon_camera::SM_GLOBAL:
            cam_->ShutterMode.SetValue(ShutterModeEnums::ShutterMode_Global);
            break;
        case pylon_camera::SM_GLOBAL_RESET_RELEASE:
            cam_->ShutterMode.SetValue(ShutterModeEnums::ShutterMode_GlobalResetRelease);
            break;
        default:
            // keep default setting
            break;
        }
    }
    catch (const GenICam::GenericException &e)
    {
        ROS_ERROR_STREAM("An exception while setting shutter mode to " << mode <<
                         " occurred: " << e.GetDescription());
        return false;
    }

    /// TODO: read back value
    return true;
}

template <typename CameraTraitT>
std::string PylonCameraImpl<CameraTraitT>::imageEncoding() const
{
    switch (image_encoding_)
    {
        case PixelFormatEnums::PixelFormat_Mono8:
            return sensor_msgs::image_encodings::MONO8;
        default:
            throw std::runtime_error("Currently, only mono8 cameras are supported");
    }
}

template <typename CameraTraitT>
int PylonCameraImpl<CameraTraitT>::imagePixelDepth() const
{
    switch (image_pixel_depth_)
    {
        case PixelSizeEnums::PixelSize_Bpp8:
            return sizeof(uint8_t);
        default:
            throw std::runtime_error("Currently, only 8bit images are supported");
    }
}

template <typename CameraTraitT>
float PylonCameraImpl<CameraTraitT>::exposureStep()
{
    return static_cast<float>(exposureTime().GetMin());
}

template <typename CameraTraitT>
float PylonCameraImpl<CameraTraitT>::maxPossibleFramerate()
{
    return static_cast<float>(resultingFrameRate().GetValue());
}

template <typename CameraTraitT>
bool PylonCameraImpl<CameraTraitT>::setUserOutput(const int& output_id, const bool& value)
{
    ROS_INFO("PylonGigECamera: Setting output id %i to %i", output_id, value);

    try
    {
        cam_->UserOutputSelector.SetValue(static_cast<UserOutputSelectorEnums>(output_id));
        cam_->UserOutputValue.SetValue(value);
    }
    catch (const std::exception& ex)
    {
        ROS_ERROR("Could not set user output %i: %s", output_id, ex.what());
        return false;
    }

    if (value != cam_->UserOutputValue.GetValue())
    {
        ROS_ERROR("Value %i could not be set to output %i", value, output_id);
        return false;
    }

    return true;
}
}  // namespace pylon_camera

#endif  // PYLON_CAMERA_INTERNAL_BASE_HPP_
