#ifndef PYLON_CAMERA_INTERNAL_BASE_HPP_
#define PYLON_CAMERA_INTERNAL_BASE_HPP_

#include <cmath>

#include <pylon_camera/internal/pylon_camera.h>

namespace pylon_camera
{

template <typename CameraTraitT>
PylonCameraImpl<CameraTraitT>::PylonCameraImpl(Pylon::IPylonDevice* device)
    : PylonCamera()
    , cam_(new CBaslerInstantCameraT(device))
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
float PylonCameraImpl<CameraTraitT>::currentExposure()
{
    return (float) exposureTime().GetValue();
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
        std::cout << "Initialized sequencer with the following inverse exposure-times [1/s]: ";
        for (size_t i = 0; i < seq_exp_times_.size(); ++i)
        {
            std::cout << seq_exp_times_.at(i);
            if (i != seq_exp_times_.size() - 1)
                std::cout << ", ";
            else
                std::cout << std::endl;
        }
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
    const typename CameraTraitT::AutoTargetBrightnessValueType brightness_value = CameraTraitT::convertBrightness(brightness);
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
        std::cerr << "ERROR unexpected brightness case" << std::endl;
    }
    cam_->GetNodeMap().InvalidateNodes();
}

template <typename CameraTraitT>
bool PylonCameraImpl<CameraTraitT>::startGrabbing(const PylonCameraParameter& params)
{
    try
    {
        cam_->StartGrabbing();
        img_rows_ = (int)cam_->Height.GetValue();
        img_cols_ = (int)cam_->Width.GetValue();
        image_encoding_ = cam_->PixelFormat.GetValue();
        image_pixel_depth_ = cam_->PixelSize.GetValue();
        height_aoi_ = 0.5 * img_rows_;
        width_aoi_ = 0.5 * img_cols_;
        offset_height_aoi_ = 0.25 * img_rows_;
        offset_width_aoi_ = 0.25 * img_cols_;
        has_auto_exposure_ = GenApi::IsAvailable(cam_->ExposureAuto);

        if (image_encoding_ != PixelFormatEnums::PixelFormat_Mono8)
        {
            cam_->PixelFormat.SetValue(PixelFormatEnums::PixelFormat_Mono8);
            image_encoding_ = cam_->PixelFormat.GetValue();
            std::cout << "Color Image support not yet implemented! Will switch to 8-Bit Mono" << std::endl;
        }

        if (!params.use_sequencer_)
        {
            cam_->ExposureAuto.SetValue(ExposureAutoEnums::ExposureAuto_Off);

            double truncated_start_exposure = params.start_exposure_;
            if (exposureTime().GetMin() > truncated_start_exposure)
            {
                truncated_start_exposure = exposureTime().GetMin();
                std::cout << "Desired exposure time unreachable! Setting to lower limit: "
                          << truncated_start_exposure
                          << std::endl;
            }
            else if (exposureTime().GetMax() < truncated_start_exposure)
            {
                truncated_start_exposure = exposureTime().GetMax();
                std::cout << "Desired exposure time unreachable! Setting to upper limit: "
                          << truncated_start_exposure
                          << std::endl;
            }
            exposureTime().SetValue(truncated_start_exposure, false);
        }

        max_framerate_ = resultingFrameRate().GetValue();
        try
        {
            float timeout = getFrameTimeout();
            if (cam_->WaitForFrameTriggerReady((int)timeout, Pylon::TimeoutHandling_ThrowException))
            {
                cam_->ExecuteSoftwareTrigger();
            }

        }
        catch (const GenICam::GenericException &e)
        {
            std::cerr << "Error while executing initial software trigger" << std::endl;
            std::cerr << e.GetDescription() << std::endl;
        }
    }
    catch (const GenICam::GenericException &e)
    {
        std::cerr << e.GetDescription() << std::endl;
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
        std::cout << "Error: Grab was not successful" << std::endl;
        return false;
    }

    const uint8_t *pImageBuffer = (uint8_t *)ptr_grab_result->GetBuffer();
    image = std::vector<uint8_t>(pImageBuffer, pImageBuffer + img_size_byte_);

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
        std::cout << "Error: Grab was not successful" << std::endl;
        return false;
    }

    //image = cv::Mat(img_rows_, img_cols_, CV_8UC1);
    memcpy(image, ptr_grab_result->GetBuffer(), img_size_byte_);

    return true;
}

template <typename CameraTrait>
bool PylonCameraImpl<CameraTrait>::grab(Pylon::CGrabResultPtr& grab_result)
{
    try
    {
        //double timeout = std::min(std::max(getFrameTimeout(), 200.0), 1000.0);
        double timeout = getFrameTimeout();
        //if (cam_->WaitForFrameTriggerReady((int)timeout, Pylon::TimeoutHandling_ThrowException))
        //{
            cam_->ExecuteSoftwareTrigger();
        //}
        //else
        //{
        //    return false;
        //}

        //if (cam_->GetGrabResultWaitObject().Wait((int)timeout))
        //{
            cam_->RetrieveResult((int)timeout,
                                 grab_result,
                                 Pylon::TimeoutHandling_ThrowException);
        //}
        //else
        //{
        //    return false;
        //}
    }
    catch (const GenICam::GenericException &e)
    {
        if (cam_->IsCameraDeviceRemoved())
        {
            is_cam_removed_ = true;
            std::cerr << "cam is removed" << std::endl;
        }
        else
        {
            std::cerr << "An image grabbing exception in pylon camera occurred: " << e.GetDescription()
                      << std::endl;
        }
        return false;
    }
    catch (...)
    {
        std::cerr << "An image grabbing exception in pylon camera occurred" << std::endl;
        return false;
    }

    if (!(grab_result->GrabSucceeded()))
    {
        std::cout << "Error: " << grab_result->GetErrorCode() << " "
                  << grab_result->GetErrorDescription()
                  << std::endl;
        return false;
    }

    return true;
}

template <typename CameraTraitT>
bool PylonCameraImpl<CameraTraitT>::setExposure(const double& exposure)
{
    // Exposure Auto is the 'automatic' counterpart to manually setting the Exposure Time Abs parameter.
    // It adjusts the Exposure Time Abs parameter value automatically within set limits until a target
    // average gray value for the pixel data of the related Auto Function AOI is reached
    // -2: AutoExposureOnce //!<Sets operation mode to 'once'.
    // -1: AutoExposureContinuous //!<Sets operation mode to 'continuous'.
    //  0: AutoExposureOff //!<Disables the Exposure Auto function.

    if ((exposure == -1.0 || exposure == -2.0 || exposure == 0.0) && !has_auto_exposure_)
    {
        std::cerr << "Error while trying to set auto exposure properties: camera has no auto exposure function!"
                  << std::endl;
        return false;
    }
    else if (!(exposure == -1.0 || exposure == -2.0 || exposure == 0.0) && exposure < 0.0)
    {
        std::cerr << "Target Exposure " << exposure << " not in the allowed range:" << std::endl;
        std::cerr << "-2: AutoExposureOnce, -1: AutoExposureContinuous, 0: AutoExposureOff, else: exposure in mus"
                  << std::endl;
        return false;
    }

    try
    {
        if (exposure == -2.0)
        {
            cam_->ExposureAuto.SetValue(ExposureAutoEnums::ExposureAuto_Once);
        }
        else if (exposure == -1.0)
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
                std::cout << "Desired exposure time unreachable! Setting to lower limit: "
                          << exposure_to_set
                          << std::endl;
            }
            else if (exposureTime().GetMax() < exposure_to_set)
            {
                exposure_to_set = exposureTime().GetMax();
                std::cout << "Desired exposure time unreachable! Setting to upper limit: "
                          << exposure_to_set
                          << std::endl;
            }
            exposureTime().SetValue(exposure_to_set, false);
        }

        // Basler Debug Day: cam_->GetNodeMap().InvalidateAll(); would be better here
        cam_->GetNodeMap().InvalidateNodes();

        last_exposure_val_ = exposure;
    }
    catch (const GenICam::GenericException &e)
    {
        std::cerr << "An exception while setting target exposure to " << exposure << " occurred:"
                  << std::endl;
        std::cerr << e.GetDescription() << std::endl;
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
            std::cerr << "Pylon Exposure Auto Node not writable in own auto-exp-function!" << std::endl;
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

    if (fabs(exp_search_params_.goal_brightness_ - exp_search_params_.current_brightness_) < 2)
    {
        is_own_brightness_function_running_ = false;
        exp_search_params_.is_initialized_ = false;
        brightness = exp_search_params_.current_brightness_;
        std::cout << "Own Auto Function: Success! Reached: " << brightness << std::endl;
        return true;
    }
    if (exp_search_params_.last_unchanged_exposure_counter_ > 2)
    {
        is_own_brightness_function_running_ = false;
        exp_search_params_.is_initialized_ = false;
        exp_search_params_.last_unchanged_exposure_counter_ = 0;
        brightness = exp_search_params_.current_brightness_;
        std::cout << "Own Auto Function: Fail! Last brightness = " << brightness << std::endl;
        return false;
    }

    exp_search_params_.updateBinarySearch();

    // truncate desired exposure if out of range
    if (exp_search_params_.desired_exposure_ < exposureTime().GetMin() ||
        exp_search_params_.desired_exposure_ > exposureTime().GetMax())
    {
        if (exp_search_params_.desired_exposure_ < exposureTime().GetMin())
        {
            std::cout << "Desired mean brightness unreachable! Min possible exposure = "
                      << exposureTime().GetMin()
                      << ". Will limit to this value." << std::endl;
            exp_search_params_.desired_exposure_ = exposureTime().GetMin();
        }
        else if (exp_search_params_.desired_exposure_ > exposureTime().GetMax())
        {
            std::cout << "Desired mean brightness unreachable! Max possible exposure = "
                      << exposureTime().GetMax()
                      << ". Will limit to this value." << std::endl;

            exp_search_params_.desired_exposure_ = exposureTime().GetMax();
        }
    }
    // Current exposure  = min/max limit value -> auto function finished -> update brightness param
    if (exp_search_params_.current_exposure_ == exposureTime().GetMin() ||
        exp_search_params_.current_exposure_ == exposureTime().GetMax())
    {
        is_own_brightness_function_running_ = false;
        exp_search_params_.is_initialized_ = false;
        std::cout << "WILL USE SMALLES EXP POSSIBLE!!!" << std::endl;
        brightness = exp_search_params_.current_brightness_;
        return true;
    }

    //gige_cam_->ExposureAuto.SetValue(Basler_UsbCameraParams::ExposureAuto_Off);
    exposureTime().SetValue(exp_search_params_.desired_exposure_);
    // Update GeniCam Cache with GetNodeMap().InvalidateNodes()
    cam_->GetNodeMap().InvalidateNodes();
    // Attention: Setting and Getting exposure not necessary the same: Difference of up to 35.0 ms
    exp_search_params_.last_exposure_ = exp_search_params_.current_exposure_;
    exp_search_params_.current_exposure_ = exposureTime().GetValue();

    return false;
}


template <typename CameraTraitT>
bool PylonCameraImpl<CameraTraitT>::setBrightness(const int& brightness)
{
    if ((brightness == -1 || brightness == -2 || brightness == 0) && !has_auto_exposure_)
    {
        std::cerr << "Error while trying to set auto brightness properties: camera has no auto exposure function!"
                  << std::endl;
        return false;
    }
    else if (!(brightness == -1 || brightness == -2 || brightness == 0) && brightness < 0)
    {
        std::cerr << "Target brightness " << brightness << " not in the allowed range:" << std::endl;
        std::cerr << "-2: AutoExposureOnce, -1: AutoExposureContinuous, 0: AutoExposureOff, else: brightness 0-> black to 255 -> white"
                  << std::endl;
        return false;
    }

    try
    {
        if (brightness == -2)
        {
            cam_->ExposureAuto.SetValue(ExposureAutoEnums::ExposureAuto_Once);
        }
        else if (brightness == -1)
        {
            cam_->ExposureAuto.SetValue(ExposureAutoEnums::ExposureAuto_Continuous);
        }
        else if (brightness == 0)
        {
            cam_->ExposureAuto.SetValue(ExposureAutoEnums::ExposureAuto_Off);
        }
        else
        {
            typename CameraTraitT::AutoTargetBrightnessValueType brightness_value = CameraTraitT::convertBrightness(brightness);
            // Set the target value for luminance control. The value is always expressed
            // as an float value regardless of the current pixel data output format,
            // i.e., 0.0 -> black, 1.0 -> white.
            if (autoTargetBrightness().GetMin() <= brightness_value &&
                autoTargetBrightness().GetMax() >= brightness_value)
            {
                // Use Pylon Auto Function, whenever in possible range
                cam_->ExposureAuto.SetValue(ExposureAutoEnums::ExposureAuto_Once);
                autoTargetBrightness().SetValue(brightness_value, false);
                //cam_->GetNodeMap().InvalidateNodes();
                //cam_->AutoTargetBrightness.InvalidateNode("AutoTargetBrightness");
            }
            else
            {
                // Extended brightness search only available in PylonOpenCVInterface
                setupExtendedBrightnessSearch(brightness);
            }
        }
        last_brightness_val_ = brightness;
    }
    catch (const GenICam::GenericException &e)
    {
        std::cerr << "An exception while setting target brightness to " << brightness << " occurred:"
                  << std::endl;
        std::cerr << e.GetDescription() << std::endl;
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
            return "mono8";
        default:
            std::cerr << "Image encoding " << image_encoding_ << " not yet implemented!"
                      << std::endl;
            return "";
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
            std::cerr << "Image Pixel Depth not yet implemented!" << std::endl;
            return -1;
    }
}


template <typename CameraTraitT>
float PylonCameraImpl<CameraTraitT>::exposureStep()
{
    return (float) exposureTime().GetMin();
}

}

#endif
