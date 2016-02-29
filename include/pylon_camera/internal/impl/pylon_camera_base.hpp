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
{
    has_auto_exposure_ = GenApi::IsAvailable(cam_->ExposureAuto);
}

template <typename CameraTraitT>
PylonCameraImpl<CameraTraitT>::~PylonCameraImpl()
{
    delete cam_;
    cam_ = NULL;
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

template <typename CameraTraitT>
float PylonCameraImpl<CameraTraitT>::currentExposure()
{
    return static_cast<float>(exposureTime().GetValue());
}

template <typename CameraTraitT>
bool PylonCameraImpl<CameraTraitT>::isAutoBrightnessFunctionRunning()
{
    return (cam_->ExposureAuto.GetValue() != ExposureAutoEnums::ExposureAuto_Off);
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
void PylonCameraImpl<CameraTraitT>::setupExtendedBrightnessSearch(const int& brightness)
{
    const typename CameraTraitT::AutoTargetBrightnessValueType brightness_value =
            CameraTraitT::convertBrightness(brightness);
    if (autoTargetBrightness().GetMin() > brightness_value)
    {
        autoTargetBrightness().SetValue(autoTargetBrightness().GetMin(), false);
        cam_->ExposureAuto.SetValue(ExposureAutoEnums::ExposureAuto_Once);
        is_own_brightness_function_running_ = true;
    }
    else if (autoTargetBrightness().GetMax() < brightness_value)
    {
        autoTargetBrightness().SetValue(autoTargetBrightness().GetMax(), false);
        cam_->ExposureAuto.SetValue(ExposureAutoEnums::ExposureAuto_Once);
        is_own_brightness_function_running_ = true;
    }
    else
    {
        ROS_ERROR("ERROR unexpected brightness case");
    }
    cam_->GetNodeMap().InvalidateNodes();
}

template <typename CameraTraitT>
bool PylonCameraImpl<CameraTraitT>::startGrabbing(const PylonCameraParameter& params)
{
    try
    {
        if (GenApi::IsAvailable(cam_->ShutterMode))
        {
            setShutterMode(params.shutter_mode_);
        }
        else
        {
            ROS_INFO("Desired cam has no selectable ShutterMode, will keep the default setting.");
        }

        cam_->BinningHorizontal.SetValue(params.binning_);
        cam_->BinningVertical.SetValue(params.binning_);

        cam_->StartGrabbing();

        img_rows_ = static_cast<int>(cam_->Height.GetValue());
        img_cols_ = static_cast<int>(cam_->Width.GetValue());
        image_encoding_ = cam_->PixelFormat.GetValue();
        image_pixel_depth_ = cam_->PixelSize.GetValue();
        img_size_byte_ =  img_cols_ * img_rows_ * imagePixelDepth();
        has_auto_exposure_ = GenApi::IsAvailable(cam_->ExposureAuto);

        if (image_encoding_ != PixelFormatEnums::PixelFormat_Mono8)
        {
            cam_->PixelFormat.SetValue(PixelFormatEnums::PixelFormat_Mono8);
            image_encoding_ = cam_->PixelFormat.GetValue();
            ROS_WARN("Color Image support not yet implemented! Will switch to 8-Bit Mono");
        }

        cam_->ExposureAuto.SetValue(ExposureAutoEnums::ExposureAuto_Off);

        double truncated_start_exposure = params.start_exposure_;
        if (exposureTime().GetMin() > truncated_start_exposure)
        {
            truncated_start_exposure = exposureTime().GetMin();
            ROS_WARN_STREAM("Desired exposure time unreachable! Setting to lower limit: " << truncated_start_exposure);
        }
        else if (exposureTime().GetMax() < truncated_start_exposure)
        {
            truncated_start_exposure = exposureTime().GetMax();
            ROS_WARN_STREAM("Desired exposure time unreachable! Setting to upper limit: " << truncated_start_exposure);
        }
        exposureTime().SetValue(truncated_start_exposure, false);

        max_framerate_ = resultingFrameRate().GetValue();

        // grab one image to be sure, that the desired exposure is set for the first image being sent
        Pylon::CGrabResultPtr grab_result;
        grab(grab_result);
        if (grab_result.IsValid())
        {
            is_ready_ = true;
        }
        else
        {
            ROS_ERROR("PylonCamera not ready because the result of the inital grab is invalid");
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
        int timeout = 5000;  // ms -> 5s timeout

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
        cam_->RetrieveResult(timeout, grab_result, Pylon::TimeoutHandling_ThrowException);
    }
    catch (const GenICam::GenericException &e)
    {
        if (cam_->IsCameraDeviceRemoved())
        {
            is_cam_removed_ = true;
            ROS_ERROR("Camera was removed, trying to re-open");
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
bool PylonCameraImpl<CameraTraitT>::setExposure(const double& exposure)
{
    if ((exposure == -1.0 || exposure == 0.0) && !has_auto_exposure_)
    {
        ROS_ERROR("Error while trying to set auto exposure properties: camera has no auto exposure function!");
        return false;
    }
    else if (!(exposure == -1.0 || exposure == 0.0) && exposure < 0.0)
    {
        ROS_ERROR_STREAM("Target Exposure " << exposure << " not in the allowed range");
        return false;
    }

    try
    {
        if (exposure == -1.0)
        {
            cam_->ExposureAuto.SetValue(ExposureAutoEnums::ExposureAuto_Continuous);
        }
        else if (exposure == 0.0)
        {
            cam_->ExposureAuto.SetValue(ExposureAutoEnums::ExposureAuto_Off);
        }
        else
        {
            if (has_auto_exposure_)
            {
                cam_->ExposureAuto.SetValue(ExposureAutoEnums::ExposureAuto_Off);
            }

            double exposure_to_set = exposure;
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
            exposureTime().SetValue(exposure_to_set, false);
        }

        // Basler Debug Day: cam_->GetNodeMap().InvalidateAll(); would be better here
        cam_->GetNodeMap().InvalidateNodes();
    }
    catch (const GenICam::GenericException &e)
    {
        ROS_ERROR_STREAM("An exception while setting target exposure to " << exposure << " occurred:"
                         << e.GetDescription());
        return false;
    }

    // pylon interface is ready, if it has already grabbed one image
    if (!is_ready_)
        is_ready_ = true;

    return true;
}


template <typename CameraTraitT>
bool PylonCameraImpl<CameraTraitT>::setExtendedBrightness(int& brightness)
{
    if (!exp_search_params_.is_initialized_)
    {
        cam_->ExposureAuto.SetValue(ExposureAutoEnums::ExposureAuto_Off);
        if (!GenApi::IsWritable(exposureTime()))
        {
            ROS_ERROR("Pylon Exposure Auto Node not writable in own auto-exp-function!");
            return false;
        }

        if (autoTargetBrightness().GetMin() > CameraTraitT::convertBrightness(brightness))
        {
            exp_search_params_.initialize(brightness,
                                          exposureTime().GetMin(),
                                          exposureTime().GetValue(),
                                          exposureTime().GetValue(),
                                          exp_search_params_.current_brightness_);
        }
        else
        {
            if (brightness > 255)
            {
                brightness = 255;
            }
            exp_search_params_.initialize(brightness,
                                          exposureTime().GetValue(),
                                          exposureTime().GetMax(),
                                          exposureTime().GetValue(),
                                          exp_search_params_.current_brightness_);
        }
    }

    if (fabs(exp_search_params_.target_brightness_ - exp_search_params_.current_brightness_) < 2)
    {
        is_own_brightness_function_running_ = false;
        exp_search_params_.is_initialized_ = false;
        brightness = exp_search_params_.current_brightness_;
        ROS_DEBUG_STREAM("Own Auto Function: Success! Reached: " << brightness);
        return true;
    }
    if (exp_search_params_.last_unchanged_exposure_counter_ > 2)
    {
        is_own_brightness_function_running_ = false;
        exp_search_params_.is_initialized_ = false;
        exp_search_params_.last_unchanged_exposure_counter_ = 0;
        brightness = exp_search_params_.current_brightness_;
        ROS_WARN_STREAM("Own Auto Function: Fail! Last brightness = " << brightness);
        return false;
    }

    exp_search_params_.updateBinarySearch();

    // truncate desired exposure if out of range
    if (exp_search_params_.target_exposure_ < exposureTime().GetMin() ||
        exp_search_params_.target_exposure_ > exposureTime().GetMax())
    {
        if (exp_search_params_.target_exposure_ < exposureTime().GetMin())
        {
            ROS_WARN_STREAM("Desired mean brightness unreachable! Min possible exposure = "
                            << exposureTime().GetMin()
                            << ". Will limit to this value.");
            exp_search_params_.target_exposure_ = exposureTime().GetMin();
        }
        else if (exp_search_params_.target_exposure_ > exposureTime().GetMax())
        {
            ROS_WARN_STREAM("Desired mean brightness unreachable! Max possible exposure = "
                            << exposureTime().GetMax()
                            << ". Will limit to this value.");

            exp_search_params_.target_exposure_ = exposureTime().GetMax();
        }
    }
    // Current exposure  = min/max limit value -> auto function finished -> update brightness param
    if (exp_search_params_.current_exposure_ == exposureTime().GetMin() ||
        exp_search_params_.current_exposure_ == exposureTime().GetMax())
    {
        is_own_brightness_function_running_ = false;
        exp_search_params_.is_initialized_ = false;
        ROS_WARN("WILL USE SMALLES EXP POSSIBLE!!!");
        brightness = exp_search_params_.current_brightness_;
        return true;
    }

    // gige_cam_->ExposureAuto.SetValue(Basler_UsbCameraParams::ExposureAuto_Off);
    exposureTime().SetValue(exp_search_params_.target_exposure_);
    // Update GeniCam Cache with GetNodeMap().InvalidateNodes()
    cam_->GetNodeMap().InvalidateNodes();
    // Attention: Setting and Getting exposure not necessary the same: Difference of up to 35.0 ms
    exp_search_params_.last_exposure_ = exp_search_params_.current_exposure_;
    exp_search_params_.current_exposure_ = exposureTime().GetValue();

    return false;
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
bool PylonCameraImpl<CameraTraitT>::setBrightness(const int& brightness)
{
    if ((brightness == -1 || brightness == 0) && !has_auto_exposure_)
    {
        ROS_ERROR("Error while trying to set auto brightness properties: camera has no auto exposure function!");
        return false;
    }
    else if (!(brightness == -1 || brightness == 0) && brightness < 0)
    {
        ROS_ERROR_STREAM("Target brightness " << brightness << " not in the allowed range");
        return false;
    }

    try
    {
        if (brightness == -1)
        {
            cam_->ExposureAuto.SetValue(ExposureAutoEnums::ExposureAuto_Continuous);
        }
        else if (brightness == 0)
        {
            cam_->ExposureAuto.SetValue(ExposureAutoEnums::ExposureAuto_Off);
        }
        else
        {
            // cam_->ExposureAuto.SetValue(ExposureAutoEnums::ExposureAuto_Once);
            typename CameraTraitT::AutoTargetBrightnessValueType brightness_value =
                    CameraTraitT::convertBrightness(brightness);
            // Set the target value for luminance control. The value is always expressed
            // as an float value regardless of the current pixel data output format,
            // i.e., 0.0 -> black, 1.0 -> white.
            if (autoTargetBrightness().GetMin() <= brightness_value &&
                autoTargetBrightness().GetMax() >= brightness_value)
            {
                // Use Pylon Auto Function, whenever in possible range
                cam_->ExposureAuto.SetValue(ExposureAutoEnums::ExposureAuto_Once);
                autoTargetBrightness().SetValue(brightness_value, false);
                // cam_->GetNodeMap().InvalidateNodes();
                // cam_->AutoTargetBrightness.InvalidateNode("AutoTargetBrightness");
            }
            else
            {
                // Extended brightness search only available in PylonOpenCVInterface
                setupExtendedBrightnessSearch(brightness);
            }
        }
    }
    catch (const GenICam::GenericException &e)
    {
        ROS_ERROR_STREAM("An exception while setting target brightness to " << brightness <<
                         " occurred:" << e.GetDescription());
        return false;
    }
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

}  // namespace pylon_camera

#endif  // PYLON_CAMERA_INTERNAL_BASE_HPP_
