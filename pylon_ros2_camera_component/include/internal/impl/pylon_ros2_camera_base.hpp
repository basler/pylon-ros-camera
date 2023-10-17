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

#include <cmath>
#include <string>
#include <vector>

#include <sensor_msgs/image_encodings.hpp>

#include "internal/pylon_ros2_camera_impl.hpp"
#include "encoding_conversions.hpp"

#include <pylon/StringParameter.h>
#include <pylon/BaslerUniversalInstantCamera.h>


namespace pylon_ros2_camera
{

namespace
{
    static const rclcpp::Logger LOGGER_BASE = rclcpp::get_logger("basler.pylon.ros2.pylon_ros2_camera_base");
}

int trigger_timeout;

template <typename CameraTraitT>
PylonROS2CameraImpl<CameraTraitT>::PylonROS2CameraImpl(Pylon::IPylonDevice* device) :
    PylonROS2Camera(),
    cam_(new CBaslerInstantCameraT(device))
{
  // information logging severity mode
  //rcutils_ret_t __attribute__((unused)) res = rcutils_logging_set_logger_level(LOGGER_BASE.get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
  //RCUTILS_LOG_SEVERITY_DEBUG
  //RCUTILS_LOG_SEVERITY_INFO
  //RCUTILS_LOG_SEVERITY_WARN
  //RCUTILS_LOG_SEVERITY_ERROR
  //RCUTILS_LOG_SEVERITY_FATAL
}

template <typename CameraTraitT>
PylonROS2CameraImpl<CameraTraitT>::~PylonROS2CameraImpl()
{
    delete cam_;
    cam_ = nullptr;

    if (binary_exp_search_)
    {
        delete binary_exp_search_;
        binary_exp_search_ = nullptr;
    }
}

template <typename CameraTraitT>
bool PylonROS2CameraImpl<CameraTraitT>::registerCameraConfiguration()
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
        RCLCPP_ERROR_STREAM(LOGGER_BASE, e.GetDescription());
        return false;
    }
}

template <typename CameraTraitT>
bool PylonROS2CameraImpl<CameraTraitT>::openCamera()
{
    try
    {
        cam_->Open();
        return true;
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, e.GetDescription());
        return false;
    }
}

template <typename CameraTraitT>
bool PylonROS2CameraImpl<CameraTraitT>::isCamRemoved()
{
    return cam_->IsCameraDeviceRemoved();
}

template <typename CameraTraitT>
size_t PylonROS2CameraImpl<CameraTraitT>::currentOffsetX()
{
    if ( GenApi::IsAvailable(cam_->OffsetX) )
    {
        return static_cast<size_t>(cam_->OffsetX.GetValue());
    }
    else
    {
        return 0;
    }
}

template <typename CameraTraitT>
size_t PylonROS2CameraImpl<CameraTraitT>::currentOffsetY()
{
    if ( GenApi::IsAvailable(cam_->OffsetY) )
    {
        return static_cast<size_t>(cam_->OffsetY.GetValue());
    }
    else
    {
        return 0;
    }
}

template <typename CameraTraitT>
sensor_msgs::msg::RegionOfInterest PylonROS2CameraImpl<CameraTraitT>::currentROI()
{
    sensor_msgs::msg::RegionOfInterest roi;
    roi.width = cam_->Width.GetValue();
    roi.height = cam_->Height.GetValue();
    roi.x_offset = currentOffsetX();
    roi.y_offset = currentOffsetY();
    return roi;
}

template <typename CameraTraitT>
size_t PylonROS2CameraImpl<CameraTraitT>::currentBinningX()
{
    if ( GenApi::IsAvailable(cam_->BinningHorizontal) )
    {
        return static_cast<size_t>(cam_->BinningHorizontal.GetValue());
    }
    else
    {
        return 1;
    }
}

template <typename CameraTraitT>
size_t PylonROS2CameraImpl<CameraTraitT>::currentBinningY()
{
    if ( GenApi::IsAvailable(cam_->BinningVertical) )
    {
        return static_cast<size_t>(cam_->BinningVertical.GetValue());
    }
    else
    {
        return 1;
    }
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::currentROSEncoding() const
{
    std::string gen_api_encoding(cam_->PixelFormat.ToString().c_str());
    std::string ros_encoding("");
    
    if (!encodingconversions::genAPI2Ros(gen_api_encoding, ros_encoding))
    {
        //std::stringstream ss;
        //ss << "No ROS equivalent to GenApi encoding '" << gen_api_encoding << "' found! This is bad because this case should never occur!";
        //throw std::runtime_error(ss.str());
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "No ROS equivalent to GenApi encoding");
        cam_->StopGrabbing();
        setImageEncoding(gen_api_encoding);
        //cam_->StartGrabbing();
        grabbingStarting();
        //return "NO_ENCODING";
    }

    return ros_encoding;
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::currentBaslerEncoding() const
{

    return (cam_->PixelFormat.ToString().c_str());
}

template <typename CameraTraitT>
float PylonROS2CameraImpl<CameraTraitT>::currentExposure()
{
    return static_cast<float>(exposureTime().GetValue());
}

template <typename CameraTraitT>
float PylonROS2CameraImpl<CameraTraitT>::currentGain()
{
    float curr_gain = (static_cast<float>(gain().GetValue()) - static_cast<float>(gain().GetMin())) /
        (static_cast<float>(gain().GetMax() - static_cast<float>(gain().GetMin())));
        
    return curr_gain;
}

template <typename CameraTraitT>
GenApi::IFloat& PylonROS2CameraImpl<CameraTraitT>::gamma()
{
    if (GenApi::IsAvailable(cam_->Gamma))
    {
        return cam_->Gamma;
    }
    else
    {
        throw std::runtime_error("Error while accessing Gamma in PylonROS2CameraImpl<CameraTraitT>");
    }
}

template <typename CameraTraitT>
float PylonROS2CameraImpl<CameraTraitT>::currentGamma()
{
    return static_cast<float>(gamma().GetValue());
}

template <typename CameraTraitT>
float PylonROS2CameraImpl<CameraTraitT>::currentAutoExposureTimeLowerLimit()
{
    return static_cast<float>(autoExposureTimeLowerLimit().GetValue());
}

template <typename CameraTraitT>
float PylonROS2CameraImpl<CameraTraitT>::currentAutoExposureTimeUpperLimit()
{
    return static_cast<float>(autoExposureTimeUpperLimit().GetValue());
}

template <typename CameraTraitT>
float PylonROS2CameraImpl<CameraTraitT>::currentAutoGainLowerLimit()
{
    return static_cast<float>(autoGainLowerLimit().GetValue());
}

template <typename CameraTraitT>
float PylonROS2CameraImpl<CameraTraitT>::currentAutoGainUpperLimit()
{
    return static_cast<float>(autoGainUpperLimit().GetValue());
}

template <typename CameraTraitT>
bool PylonROS2CameraImpl<CameraTraitT>::isPylonAutoBrightnessFunctionRunning()
{
    return cam_->ExposureAuto.GetValue() != ExposureAutoEnums::ExposureAuto_Off || cam_->GainAuto.GetValue() != GainAutoEnums::GainAuto_Off;
}

template <typename CameraTraitT>
bool PylonROS2CameraImpl<CameraTraitT>::isBrightnessSearchRunning()
{
    return isBinaryExposureSearchRunning() || isPylonAutoBrightnessFunctionRunning();
}

template <typename CameraTraitT>
void PylonROS2CameraImpl<CameraTraitT>::enableContinuousAutoExposure()
{
    if (GenApi::IsAvailable(cam_->ExposureAuto))
    {
        cam_->ExposureAuto.TrySetValue(ExposureAutoEnums::ExposureAuto_Continuous);
    }
    else
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "Trying to enable ExposureAuto_Continuous mode, but "
            << "the camera has no Auto Exposure");
    }
}

template <typename CameraTraitT>
void PylonROS2CameraImpl<CameraTraitT>::enableContinuousAutoGain()
{
    if ( GenApi::IsAvailable(cam_->GainAuto) )
    {
        cam_->GainAuto.TrySetValue(GainAutoEnums::GainAuto_Continuous);
    }
    else
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "Trying to enable GainAuto_Continuous mode, but the camera has no Auto Gain");
    }
}

template <typename CameraTraitT>
void PylonROS2CameraImpl<CameraTraitT>::disableAllRunningAutoBrightessFunctions()
{
    is_binary_exposure_search_running_ = false;
    cam_->ExposureAuto.TrySetValue(ExposureAutoEnums::ExposureAuto_Off);
    cam_->GainAuto.TrySetValue(GainAutoEnums::GainAuto_Off);
    
    if (binary_exp_search_)
    {
        delete binary_exp_search_;
        binary_exp_search_ = nullptr;
    }
}

template <typename CameraTraitT>
bool PylonROS2CameraImpl<CameraTraitT>::setupSequencer(const std::vector<float>& exposure_times)
{
    std::vector<float> exposure_times_set;
    if ( setupSequencer(exposure_times, exposure_times_set) )
    {
        seq_exp_times_ = exposure_times_set;
        std::stringstream ss;
        ss << "Initialized sequencer with the following inverse exposure-times [1/s]: ";
        for ( size_t i = 0; i < seq_exp_times_.size(); ++i )
        {
            ss << seq_exp_times_.at(i);
            if ( i != seq_exp_times_.size() - 1 )
            {
                ss << ", ";
            }
        }
        RCLCPP_INFO(LOGGER_BASE, ss.str().c_str());
        return true;
    }
    else
    {
        return false;
    }
}

template <typename CameraTraitT>
bool PylonROS2CameraImpl<CameraTraitT>::startGrabbing(const PylonROS2CameraParameter& parameters)
{
    try
    {
        if (GenApi::IsAvailable(cam_->ShutterMode))
        {
            setShutterMode(parameters.shutter_mode_);
        }

        available_image_encodings_ = detectAvailableImageEncodings(true); // Basler format

        // Check if the image can be encoded with the parameter defined value
        if (setImageEncoding(parameters.imageEncoding()).find("done") == std::string::npos)
        {
            bool error = true;
            // The desired encoding cannot be used. We will try to use one of the available
            // This avoid the Error while start grabbing program termination
            for (std::string x : available_image_encodings_)
            {
                std::string ros_encoding;
                encodingconversions::genAPI2Ros(x, ros_encoding);
                if ((setImageEncoding(ros_encoding).find("done") != std::string::npos))
                {
                    // Achieved one of the encodings
                    error = false;
                    break;
                }
            }
            
            if (error)
            {
                return false;
            }
        }
        
        grab_strategy = parameters.grab_strategy_;
        //cam_->StartGrabbing();
        grabbingStarting();
        user_output_selector_enums_ = detectAndCountNumUserOutputs();
        device_user_id_ = cam_->DeviceUserID.GetValue();
        img_rows_ = static_cast<size_t>(cam_->Height.GetValue());
        img_cols_ = static_cast<size_t>(cam_->Width.GetValue());
        img_size_byte_ =  img_cols_ * img_rows_ * imagePixelDepth();

        //grab_timeout_ = exposureTime().GetMax() * 1.05;
        grab_timeout_ = parameters.grab_timeout_; // grab timeout = 500 ms
        trigger_timeout = parameters.trigger_timeout_;
        
        // grab one image to be sure, that the communication is successful
        Pylon::CGrabResultPtr grab_result;
        grab(grab_result);
        if ( grab_result.IsValid() )
        {
            is_ready_ = true;
        }
        else
        {
            RCLCPP_ERROR(LOGGER_BASE, "PylonROS2Camera not ready because the result of the initial grab is invalid");
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        return false;
    }
    
    return true;
}

// Grab a picture as std::vector of 8bits objects
template <typename CameraTrait>
bool PylonROS2CameraImpl<CameraTrait>::grab(std::vector<uint8_t>& image)
{ 
    Pylon::CGrabResultPtr ptr_grab_result;
    if ( !grab(ptr_grab_result) )
    {   
        RCLCPP_ERROR(LOGGER_BASE, "Error: Grab was not successful");
        return false;
    }
    const uint8_t *pImageBuffer = reinterpret_cast<uint8_t*>(ptr_grab_result->GetBuffer());

    // ------------------------------------------------------------------------
    // Bit shifting
    // ------------------------------------------------------------------------
    // In case of 12 bits we need to shift the image bits 4 positions to the left
    std::string ros_enc = currentROSEncoding();
    uint16_t * shift_array = new uint16_t[img_size_byte_ / 2]; // Dynamically allocated to avoid heap size error
    std::string gen_api_encoding(cam_->PixelFormat.ToString().c_str());
    if (encodingconversions::is_12_bit_ros_enc(ros_enc) && (gen_api_encoding == "BayerRG12" || gen_api_encoding == "BayerBG12" || gen_api_encoding == "BayerGB12" || gen_api_encoding == "BayerGR12" || gen_api_encoding == "Mono12") ){
        const uint16_t *convert_bits = reinterpret_cast<uint16_t*>(ptr_grab_result->GetBuffer());
        for (size_t i = 0; i < img_size_byte_ / 2; i++){
            shift_array[i] = convert_bits[i] << 4;
        }
        image.assign((uint8_t *) shift_array, (uint8_t *) shift_array + img_size_byte_);
    } else {
        image.assign(pImageBuffer, pImageBuffer + img_size_byte_);
    }

    delete[] shift_array;
    
    if ( !is_ready_ )
        is_ready_ = true;
    return true;
}

// Grab a picture as pointer to 8bit array
template <typename CameraTrait>
bool PylonROS2CameraImpl<CameraTrait>::grab(uint8_t* image)
{   

    // If camera is not grabbing, don't grab
    if (!cam_->IsGrabbing()){
        return false;
    }

    Pylon::CGrabResultPtr ptr_grab_result;
    if ( !grab(ptr_grab_result) )
    {   
        RCLCPP_ERROR(LOGGER_BASE, "Error: Grab was not successful");
        return false;
    }

    // ------------------------------------------------------------------------
    // Bit shifting
    // ------------------------------------------------------------------------
    // In case of 12 bits we need to shift the image bits 4 positions to the left
    std::string ros_enc = currentROSEncoding();
    uint16_t shift_array[img_size_byte_ / 2];

    if (encodingconversions::is_12_bit_ros_enc(ros_enc)){
        const uint16_t *convert_bits = reinterpret_cast<uint16_t*>(ptr_grab_result->GetBuffer());
        for (size_t i = 0; i < img_size_byte_ / 2; i++){
            shift_array[i] = convert_bits[i] << 4;
        }
        memcpy(image, (uint8_t *) shift_array, img_size_byte_);
    } else {
        memcpy(image, ptr_grab_result->GetBuffer(), img_size_byte_);
    }
    
    return true;
}

template <typename CameraTrait>
bool PylonROS2CameraImpl<CameraTrait>::grabBlaze(sensor_msgs::msg::PointCloud2& cloud_msg,
                                                 sensor_msgs::msg::Image& intensity_map_msg, 
                                                 sensor_msgs::msg::Image& depth_map_msg, 
                                                 sensor_msgs::msg::Image& depth_map_color_msg, 
                                                 sensor_msgs::msg::Image& confidence_map_msg)
{
    RCLCPP_WARN(LOGGER_BASE, "The connected camera is not a blaze, nothing is going to be grabbed!");
    return true;
}

// Lowest level grab function called by the other grab functions
template <typename CameraTrait>
bool PylonROS2CameraImpl<CameraTrait>::grab(Pylon::CGrabResultPtr& grab_result)
{
    // If camera is not grabbing, don't grab
    if (!cam_->IsGrabbing())
    {
        return false;
    }

    try
    {
        // WaitForFrameTriggerReady to prevent trigger signal to get lost
        // this could happen, if 2xExecuteSoftwareTrigger() is only followed by 1xgrabResult()
        // -> 2nd trigger might get lost
        if ((cam_->TriggerMode.GetValue() == TriggerModeEnums::TriggerMode_On))
        {
            if (cam_->WaitForFrameTriggerReady(trigger_timeout, Pylon::TimeoutHandling_ThrowException))
            {   
                cam_->ExecuteSoftwareTrigger(); 
            }
            else
            {   
                RCLCPP_ERROR(LOGGER_BASE, "Error WaitForFrameTriggerReady() timed out, impossible to ExecuteSoftwareTrigger()");
                return false;
            }
        }

        cam_->RetrieveResult(grab_timeout_, grab_result, Pylon::TimeoutHandling_ThrowException);
    }
    catch (const GenICam::GenericException &e)
    {   
        if (cam_->IsCameraDeviceRemoved())
        {   
            RCLCPP_ERROR(LOGGER_BASE, "Lost connection to the camera . . .");
        }
        else
        {   
            if ((cam_->TriggerSource.GetValue() != TriggerSourceEnums::TriggerSource_Software) && (cam_->TriggerMode.GetValue() == TriggerModeEnums::TriggerMode_On))
            {
                RCLCPP_ERROR_STREAM(LOGGER_BASE, "Waiting for Trigger signal");
            }
            else 
            {
                RCLCPP_ERROR_STREAM(LOGGER_BASE, "An image grabbing exception in pylon camera occurred: " << e.GetDescription());
            }
        }

        return false;
    }
    catch (...)
    {   
        RCLCPP_ERROR(LOGGER_BASE, "An unspecified image grabbing exception in pylon camera occurred");
        return false;
    }

    if (!grab_result->GrabSucceeded())
    {   
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error: " << grab_result->GetErrorCode() << " " << grab_result->GetErrorDescription());
        return false;
    }

    return true;
}

template <typename CameraTraitT>
bool PylonROS2CameraImpl<CameraTraitT>::isBlaze()
{
    return false;
}

template <typename CameraTraitT>
void PylonROS2CameraImpl<CameraTraitT>::getInitialCameraInfo(sensor_msgs::msg::CameraInfo& cam_info_msg)
{
    // https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/CameraInfo.msg

    // The image dimensions with which the camera was calibrated. Normally
    // this will be the full camera resolution in pixels.
    cam_info_msg.height = this->imageRows();
    cam_info_msg.width = this->imageCols();

    // The distortion model used. Supported models are listed in
    // sensor_msgs/distortion_models.h. For most cameras, "plumb_bob" - a
    // simple model of radial and tangential distortion - is sufficient.
    cam_info_msg.distortion_model = "";

    // The distortion parameters, size depending on the distortion model.
    // For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3) -> float64[] d.
    cam_info_msg.d = std::vector<double>(5, 0.);

    // Intrinsic camera matrix for the raw (distorted) images.
    //     [fx  0 cx]
    // K = [ 0 fy cy]  --> 3x3 row-major matrix
    //     [ 0  0  1]
    // Projects 3D points in the camera coordinate frame to 2D pixel coordinates
    // using the focal lengths (fx, fy) and principal point (cx, cy) -> float64[9] k.
    cam_info_msg.k.fill(0.0);

    // Rectification matrix (stereo cameras only)
    // A rotation matrix aligning the camera coordinate system to the ideal
    // stereo image plane so that epipolar lines in both stereo images are parallel -> float64[9] r, 3x3 row-major matrix.
    cam_info_msg.r.fill(0.0);

    // Projection/camera matrix
    //     [fx'  0  cx' Tx]
    // P = [ 0  fy' cy' Ty]  --> # 3x4 row-major matrix
    //     [ 0   0   1   0]
    // By convention, this matrix specifies the intrinsic (camera) matrix of the
    // processed (rectified) image. That is, the left 3x3 portion is the normal
    // camera intrinsic matrix for the rectified image. It projects 3D points
    // in the camera coordinate frame to 2D pixel coordinates using the focal
    // lengths (fx', fy') and principal point (cx', cy') - these may differ from
    // the values in K. For monocular cameras, Tx = Ty = 0. Normally, monocular
    // cameras will also have R = the identity and P[1:3,1:3] = K.
    // For a stereo pair, the fourth column [Tx Ty 0]' is related to the
    // position of the optical center of the second camera in the first
    // camera's frame. We assume Tz = 0 so both cameras are in the same
    // stereo image plane. The first camera always has Tx = Ty = 0.
    // For the right (second) camera of a horizontal stereo pair,
    // Ty = 0 and Tx = -fx' * B, where B is the baseline between the cameras.
    // Given a 3D point [X Y Z]', the projection (x, y) of the point onto the
    // rectified image is given by:
    // [u v w]' = P * [X Y Z 1]'
    //        x = u / w
    //        y = v / w
    //  This holds for both images of a stereo pair -> float64[12]
    cam_info_msg.p.fill(0.0);

    // Binning refers here to any camera setting which combines rectangular
    // neighborhoods of pixels into larger "super-pixels." It reduces the
    // resolution of the output image to (width / binning_x) x (height / binning_y).
    // The default values binning_x = binning_y = 0 is considered the same as
    // binning_x = binning_y = 1 (no subsampling).
    cam_info_msg.binning_x = this->currentBinningX();
    cam_info_msg.binning_y = this->currentBinningY();

    // Region of interest (subwindow of full camera resolution), given in full
    // resolution (unbinned) image coordinates. A particular ROI always denotes
    // the same window of pixels on the camera sensor, regardless of binning
    // settings. The default setting of roi (all values 0) is considered the same
    // as full resolution (roi.width = width, roi.height = height).
    cam_info_msg.roi.x_offset = cam_info_msg.roi.y_offset = 0;
    cam_info_msg.roi.height = cam_info_msg.roi.width = 0;
}

template <typename CameraTraitT>
std::vector<std::string> PylonROS2CameraImpl<CameraTraitT>::detectAvailableImageEncodings(const bool& show_message)
{
    std::vector<std::string> available_encodings;
    GenApi::INodeMap& node_map = cam_->GetNodeMap();
    GenApi::CEnumerationPtr img_encoding_enumeration_ptr(node_map.GetNode("PixelFormat"));
    GenApi::NodeList_t feature_list;
    img_encoding_enumeration_ptr->GetEntries(feature_list);
    std::stringstream ss;
    ss << "The camera device supports the following [GenAPI|ROS] image encodings: ";
    
    for (GenApi::NodeList_t::iterator it = feature_list.begin(); it != feature_list.end(); ++it)
    {
        if ( GenApi::IsAvailable(*it) )
        {
            GenApi::CEnumEntryPtr enum_entry(*it);
            std::string encoding_gen_api = enum_entry->GetSymbolic().c_str();
            std::string encoding_ros("NO_ROS_EQUIVALENT");
            encodingconversions::genAPI2Ros(encoding_gen_api, encoding_ros);
            ss << "['" << encoding_gen_api << "'|'" << encoding_ros << "'] ";
            available_encodings.push_back(encoding_gen_api);
        }
    }
    
    if (show_message)
    {
        RCLCPP_DEBUG(LOGGER_BASE, ss.str().c_str());
    }
    
    return available_encodings;
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setImageEncoding(const std::string& ros_encoding) const
{
    bool is_16bits_available = false;
    bool is_encoding_available = false;
    std::string gen_api_encoding;
    // An additional check to select the correct basler encoding, as ROS 16-bits encoding will cover both Basler 12-bits and 16-bits encoding
    if (ros_encoding == "bayer_rggb16" || ros_encoding == "bayer_bggr16" || ros_encoding == "bayer_gbrg16" || ros_encoding == "bayer_grbg16") 
    { 
        for ( const std::string& enc : available_image_encodings_ )
            {
                if ( enc == "BayerRG16" || enc == "BayerBG16" || enc == "BayerGB16" || enc == "BayerGR16" || enc == "Mono16")
                {
                    is_16bits_available = true;
                    break;
                }

            }
    }

    bool conversion_found = encodingconversions::ros2GenAPI(ros_encoding, gen_api_encoding, is_16bits_available);
    if (ros_encoding != "")
    {
        for ( const std::string& enc : available_image_encodings_ )
        {
            if ((gen_api_encoding == enc) && conversion_found)
            {
                is_encoding_available = true;
                break;
            }
        }
        if (! is_encoding_available)
            return "Error: unsupported/unknown image format";
    }
    if ( !conversion_found )
    {
        if ( ros_encoding.empty() )
        {
            RCLCPP_WARN_STREAM(LOGGER_BASE, "No image encoding provided -> Will use 'mono8' or 'rgb8' as fallback");
        }
        else
        {
            RCLCPP_ERROR_STREAM(LOGGER_BASE, "Can't convert ROS encoding '" << ros_encoding
                << "' to a corresponding GenAPI encoding! Will use 'mono8' or "
                << "'rgb8' as fallback!");
        }
        bool fallback_found = false;
        for ( const std::string& enc : available_image_encodings_ )
        {
            if ( enc == "Mono8" || enc == "RGB8" )
            {
                fallback_found = true;
                gen_api_encoding = enc;
                break;
            }
        }
        if ( !fallback_found )
        {
            RCLCPP_ERROR_STREAM(LOGGER_BASE, "Couldn't find a fallback solution!");
            return "Error: Couldn't find a fallback solution!";
        }
    }

    bool supports_desired_encoding = false;
    for ( const std::string& enc : available_image_encodings_ )
    {
        supports_desired_encoding = (gen_api_encoding == enc);
        if ( supports_desired_encoding )
        {
            break;
        }
    }
    if ( !supports_desired_encoding )
    {
        RCLCPP_WARN_STREAM(LOGGER_BASE, "Camera does not support the desired image pixel "
            << "encoding '" << ros_encoding << "'!");
        return "Error : Camera does not support the desired image pixel";
    }
    try
    {
        if ( GenApi::IsAvailable(cam_->PixelFormat) )
        {
            GenApi::INodeMap& node_map = cam_->GetNodeMap();
            //cam_->StartGrabbing();
            grabbingStarting();
            cam_->StopGrabbing();
            GenApi::CEnumerationPtr(node_map.GetNode("PixelFormat"))->FromString(gen_api_encoding.c_str());
            return "done";
        }
        else
        {
            RCLCPP_WARN_STREAM(LOGGER_BASE, "Camera does not support variable image pixel "
                << "encoding!");
            return "Error : Camera does not support variable image pixel";
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while setting target image encoding to '"
            << ros_encoding << "' occurred: " << e.GetDescription());
        return e.GetDescription();
    }
}

template <typename CameraTraitT>
int PylonROS2CameraImpl<CameraTraitT>::imagePixelDepth() const
{
    int pixel_depth(0);
    try
    {
        // pylon PixelSize already contains the number of channels
        // the size is given in bit, wheras ROS provides it in byte
        pixel_depth = cam_->PixelSize.GetIntValue() / 8;
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while reading image pixel size occurred: "
                << e.GetDescription());
    }
    return pixel_depth;
}

template <typename CameraTraitT>
bool PylonROS2CameraImpl<CameraTraitT>::setROI(const sensor_msgs::msg::RegionOfInterest target_roi,
                                               sensor_msgs::msg::RegionOfInterest& reached_roi)
{
    int64_t width_to_set = target_roi.width;
    int64_t height_to_set = target_roi.height;
    int64_t offset_x_to_set = target_roi.x_offset;
    int64_t offset_y_to_set = target_roi.y_offset;

    try
    {
        if ( GenApi::IsAvailable(cam_->Width) && GenApi::IsAvailable(cam_->Height) && GenApi::IsAvailable(cam_->OffsetX) && GenApi::IsAvailable(cam_->OffsetY))
        {
            cam_->StopGrabbing();
            int64_t max_image_width = cam_->WidthMax.GetValue();
            int64_t min_image_width = cam_->Width.GetMin();
            int64_t max_image_height = cam_->HeightMax.GetValue();
            int64_t min_image_height = cam_->Height.GetMin();
            int64_t width_inc = cam_->Width.GetInc();
            int64_t height_inc = cam_->Height.GetInc();
            // reset roi to avoid exceptions while setting Width and Height values
            cam_->OffsetX.SetValue(0);
            cam_->OffsetY.SetValue(0);
            cam_->Width.SetValue(max_image_width);
            cam_->Height.SetValue(max_image_height);
            // Force minimum increment of 2 as some cameras wrongly declare that 
            // they have an increment of 1 while it is 2
            if (height_inc == 1)
                height_inc = 2;
            if (width_inc == 1)
                width_inc = 2;

            if (width_to_set > max_image_width)
            {
                RCLCPP_WARN_STREAM(LOGGER_BASE, "Desired width("
                        << width_to_set << ") unreachable! Setting to upper "
                        << "limit: " << max_image_width);
                width_to_set = max_image_width;
            }
            else if (width_to_set < min_image_width)
            {
                RCLCPP_WARN_STREAM(LOGGER_BASE, "Desired width("
                        << width_to_set << ") unreachable! Setting to lower "
                        << "limit: " << min_image_width);
                width_to_set = min_image_width;
            }

            if (height_to_set > max_image_height)
            {
                RCLCPP_WARN_STREAM(LOGGER_BASE, "Desired height("
                        <<height_to_set << ") unreachable! Setting to upper "
                        << "limit: " << max_image_height);
                height_to_set = max_image_height;
            }
            else if (height_to_set < min_image_height)
            {
                RCLCPP_WARN_STREAM(LOGGER_BASE, "Desired height("
                        << height_to_set << ") unreachable! Setting to lower "
                        << "limit: " << min_image_height);
                height_to_set = min_image_height;
            }
            if (offset_x_to_set % width_inc != 0)
            {
                int64_t adapted_offset_x = offset_x_to_set + (width_inc - offset_x_to_set % width_inc);
                RCLCPP_WARN_STREAM(LOGGER_BASE, "Desired x offset ("
                        << offset_x_to_set << ") not an multiple of width increment("
                        << width_inc <<")! Setting to next possible value higher multiple: ("
                        << adapted_offset_x <<")");
                offset_x_to_set = adapted_offset_x;
            }
            if (width_to_set + offset_x_to_set >  max_image_width)
            {
                int64_t adapted_offset_x = max_image_width - width_to_set;
                adapted_offset_x -= adapted_offset_x % width_inc;
                RCLCPP_WARN_STREAM(LOGGER_BASE, "Desired x offset("
                        << offset_x_to_set << ") impossible for desired width("
                        << width_to_set << ")! Setting to next possible value " 
                        << adapted_offset_x);
                offset_x_to_set = adapted_offset_x;
            }
            if (offset_y_to_set % height_inc != 0)
            {
                int64_t adapted_offset_y = offset_y_to_set + (height_inc - offset_y_to_set % height_inc);
                RCLCPP_WARN_STREAM(LOGGER_BASE, "Desired y offset ("
                        << offset_y_to_set << ") not an multiple ofheight increment("
                        << height_inc <<")! Setting to next possible value higher multiple: ("
                        << adapted_offset_y <<")");
                offset_y_to_set = adapted_offset_y;
            }
            if (height_to_set + offset_y_to_set >  max_image_height)
            {
                int64_t adapted_offset_y = max_image_height - height_to_set;
                adapted_offset_y -= adapted_offset_y % height_inc;
                RCLCPP_WARN_STREAM(LOGGER_BASE, "Desired y offset("
                        << offset_y_to_set << ") impossible for desired height("
                        << height_to_set << ")! Setting to next possible value " 
                        << adapted_offset_y);
                offset_y_to_set = adapted_offset_y;
            }

            cam_->Width.SetValue(width_to_set);
            cam_->Height.SetValue(height_to_set);
            cam_->OffsetX.SetValue(offset_x_to_set);
            cam_->OffsetY.SetValue(offset_y_to_set);
            reached_roi = currentROI();
            grabbingStarting();
            //cam_->StartGrabbing();
            
            img_cols_ = static_cast<size_t>(cam_->Width.GetValue());
            img_rows_ = static_cast<size_t>(cam_->Height.GetValue());
            img_size_byte_ =  img_cols_ * img_rows_ * imagePixelDepth();

            // For ACE cameras we need to completely stop grabbing and then the
            // user needs to call start grabbing, if not the driver crashes.
            // in pylon_camera_node.cpp values are updated after this call, so that
            // we cannot start now grabbing until those values are updated

        }
        else
        {
            RCLCPP_WARN_STREAM(LOGGER_BASE, "Camera does not support area of interest. Will keep the "
                    << "current settings");
            reached_roi = currentROI();
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while setting target area of interest "
                << "(width, height, x_offset, y_offset) to:  (" << width_to_set << ", "
                << height_to_set << ", " << offset_x_to_set << ", " << offset_y_to_set <<
                ") occurred: "
                << e.GetDescription());
        return false;
    }

    return true;
}

template <typename CameraTraitT>
bool PylonROS2CameraImpl<CameraTraitT>::setBinningX(const size_t& target_binning_x,
                                                size_t& reached_binning_x)
{
    try
    {
        if ( GenApi::IsAvailable(cam_->BinningHorizontal) )
        {
            cam_->StopGrabbing();
            int64_t binning_x_to_set = target_binning_x;
            if ( binning_x_to_set < cam_->BinningHorizontal.GetMin() )
            {
                RCLCPP_WARN_STREAM(LOGGER_BASE, "Desired horizontal binning_x factor("
                        << binning_x_to_set << ") unreachable! Setting to lower "
                        << "limit: " << cam_->BinningHorizontal.GetMin());
                binning_x_to_set = cam_->BinningHorizontal.GetMin();
            }
            else if ( binning_x_to_set > cam_->BinningHorizontal.GetMax() )
            {
                RCLCPP_WARN_STREAM(LOGGER_BASE, "Desired horizontal binning_x factor("
                        << binning_x_to_set << ") unreachable! Setting to upper "
                        << "limit: " << cam_->BinningHorizontal.GetMax());
                binning_x_to_set = cam_->BinningHorizontal.GetMax();
            }
            cam_->BinningHorizontal.SetValue(binning_x_to_set);
            reached_binning_x = currentBinningX();
            //cam_->StartGrabbing();
            grabbingStarting();
            img_cols_ = static_cast<size_t>(cam_->Width.GetValue());
            img_size_byte_ =  img_cols_ * img_rows_ * imagePixelDepth();
        }
        else
        {
            RCLCPP_WARN_STREAM(LOGGER_BASE, "Camera does not support binning (X). Will keep the current settings.");
            reached_binning_x = currentBinningX();
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while setting target horizontal "
                << "binning_x factor to " << target_binning_x << " occurred: "
                << e.GetDescription());
        return false;
    }
    return true;
}

template <typename CameraTraitT>
bool PylonROS2CameraImpl<CameraTraitT>::setBinningY(const size_t& target_binning_y,
                                                size_t& reached_binning_y)
{
    try
    {
        if ( GenApi::IsAvailable(cam_->BinningVertical) )
        {
            cam_->StopGrabbing();
            int64_t binning_y_to_set = target_binning_y;
            if ( binning_y_to_set < cam_->BinningVertical.GetMin() )
            {
                RCLCPP_WARN_STREAM(LOGGER_BASE, "Desired vertical binning_y factor("
                        << binning_y_to_set << ") unreachable! Setting to lower "
                        << "limit: " << cam_->BinningVertical.GetMin());
                binning_y_to_set = cam_->BinningVertical.GetMin();
            }
            else if ( binning_y_to_set > cam_->BinningVertical.GetMax() )
            {
                RCLCPP_WARN_STREAM(LOGGER_BASE, "Desired vertical binning_y factor("
                        << binning_y_to_set << ") unreachable! Setting to upper "
                        << "limit: " << cam_->BinningVertical.GetMax());
                binning_y_to_set = cam_->BinningVertical.GetMax();
            }
            cam_->BinningVertical.SetValue(binning_y_to_set);
            reached_binning_y = currentBinningY();
            //cam_->StartGrabbing();
            grabbingStarting();
            img_rows_ = static_cast<size_t>(cam_->Height.GetValue());
            img_size_byte_ =  img_cols_ * img_rows_ * imagePixelDepth();
        }
        else
        {
            RCLCPP_WARN_STREAM(LOGGER_BASE, "Camera does not support binning (Y). Will keep the current settings.");
            reached_binning_y = currentBinningY();
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while setting target vertical "
                << "binning_y factor to " << target_binning_y << " occurred: "
                << e.GetDescription());
        return false;
    }
    return true;
}

template <typename CameraTraitT>
bool PylonROS2CameraImpl<CameraTraitT>::setExposure(const float& target_exposure,
                                                    float& reached_exposure)
{
    try
    {
        cam_->ExposureAuto.TrySetValue(ExposureAutoEnums::ExposureAuto_Off);

        float exposure_to_set = target_exposure;
        if ( exposure_to_set < exposureTime().GetMin() )
        {
            RCLCPP_WARN_STREAM(LOGGER_BASE, "Desired exposure (" << exposure_to_set << ") "
                << "time unreachable! Setting to lower limit: "
                << exposureTime().GetMin());
            exposure_to_set = exposureTime().GetMin();
        }
        else if ( exposure_to_set > exposureTime().GetMax() )
        {
            RCLCPP_WARN_STREAM(LOGGER_BASE, "Desired exposure (" << exposure_to_set << ") "
                << "time unreachable! Setting to upper limit: "
                << exposureTime().GetMax());
            exposure_to_set = exposureTime().GetMax();
        }
        exposureTime().SetValue(exposure_to_set);
        reached_exposure = currentExposure();

        if ( std::fabs(reached_exposure - exposure_to_set) > exposureStep() )
        {
            // no success if the delta between target and reached exposure
            // is greater then the exposure step in ms
            return false;
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while setting target exposure to "
                         << target_exposure << " occurred:"
                         << e.GetDescription());
        return false;
    }
    return true;
}

template <typename CameraTraitT>
bool PylonROS2CameraImpl<CameraTraitT>::setAutoflash(
                        const std::map<int, bool> flash_on_lines)
{
    (void)flash_on_lines;
    RCLCPP_ERROR_STREAM(LOGGER_BASE, "Not implemented for this camera type");
    return false;
}

template <typename CameraTraitT>
bool PylonROS2CameraImpl<CameraTraitT>::setGain(const float& target_gain,
                                                float& reached_gain)
{
    try
    {
        cam_->GainAuto.TrySetValue(GainAutoEnums::GainAuto_Off);
        float truncated_gain = target_gain;
        if ( truncated_gain < 0.0 )
        {
            RCLCPP_WARN_STREAM(LOGGER_BASE, "Desired gain (" << target_gain << ") in "
                << "percent out of range [0.0 - 1.0]! Setting to lower "
                << "limit: 0.0");
            truncated_gain = 0.0;
        }
        else if ( truncated_gain > 1.0 )
        {
            RCLCPP_WARN_STREAM(LOGGER_BASE, "Desired gain (" << target_gain << ") in "
                << "percent out of range [0.0 - 1.0]! Setting to upper "
                << "limit: 1.0");
            truncated_gain = 1.0;
        }

        float gain_to_set = gain().GetMin() +
                            truncated_gain * (gain().GetMax() - gain().GetMin());
        gain().SetValue(gain_to_set);
        reached_gain = currentGain();
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while setting target gain to "
               << target_gain << " occurred: " << e.GetDescription());
        return false;
    }
    return true;
}

template <typename CameraTraitT>
bool PylonROS2CameraImpl<CameraTraitT>::setBrightness(const int& target_brightness,
                                                  const float& current_brightness,
                                                  const bool& exposure_auto,
                                                  const bool& gain_auto)
{
    try
    {
        // if the target brightness is greater 255, limit it to 255
        // the brightness_to_set is a float value, regardless of the current
        // pixel data output format, i.e., 0.0 -> black, 1.0 -> white.
        typename CameraTraitT::AutoTargetBrightnessValueType brightness_to_set =
            CameraTraitT::convertBrightness(std::min(255, target_brightness));
/**
#if DEBUG
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
**/
        if ( isPylonAutoBrightnessFunctionRunning() )
        {
            // do nothing while the pylon-auto function is active and if the
            // desired brightness is inside the possible range
            return true;
        }

#if DEBUG
        ROS_INFO("pylon auto finished . . .");
#endif

        float autoTargetBrightnessMin = 0.0;
        float autoTargetBrightnessMax = 0.0;
        if ( GenApi::IsAvailable(cam_->AutoTargetValue) )
        {
            autoTargetBrightnessMin = cam_->AutoTargetValue.GetMin();
            autoTargetBrightnessMax = cam_->AutoTargetValue.GetMax();
        } else if (GenApi::IsAvailable(cam_->AutoTargetBrightness))
        {   
            autoTargetBrightnessMin = cam_->AutoTargetBrightness.GetMin();
            autoTargetBrightnessMax = cam_->AutoTargetBrightness.GetMax();
        }
        if ( autoTargetBrightnessMin <= brightness_to_set &&
             autoTargetBrightnessMax >= brightness_to_set )
        {
            // Use Pylon Auto Function, whenever in possible range
            // -> Own binary exposure search not necessary
            if ( GenApi::IsAvailable(cam_->AutoTargetValue) )
            {
                cam_->AutoTargetValue.SetValue(brightness_to_set, true);
            } else if (GenApi::IsAvailable(cam_->AutoTargetBrightness))
            {   
                cam_->AutoTargetBrightness.SetValue(brightness_to_set);
            }
            if ( exposure_auto )
            {
                cam_->ExposureAuto.TrySetValue(ExposureAutoEnums::ExposureAuto_Once);
            }
            if ( gain_auto )
            {
                cam_->GainAuto.TrySetValue(GainAutoEnums::GainAuto_Once);
            }
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
                // is e.g., 35 and we want to reach e.g., 33, because we first go
                // up to 50 and then start the binary exposure search to go down
                // again.
                // But in fact it's the only solution, because the exact exposure
                // times related to the min & max limit of the pylon auto function
                // are unknown, beacause they depend on the current light scene
                if ( brightness_to_set < autoTargetBrightnessMin )
                {
                    // target < 50 -> pre control to 50
                    if ( GenApi::IsAvailable(cam_->AutoTargetValue) )
                    {
                        cam_->AutoTargetValue.SetValue(autoTargetBrightnessMin, true);
                    } else if (GenApi::IsAvailable(cam_->AutoTargetBrightness))
                    {   
                        cam_->AutoTargetBrightness.SetValue(autoTargetBrightnessMin);
                    }
                    if ( exposure_auto )
                    {
                        cam_->ExposureAuto.TrySetValue(ExposureAutoEnums::ExposureAuto_Once);
                    }
                    if ( gain_auto )
                    {
                        cam_->GainAuto.TrySetValue(GainAutoEnums::GainAuto_Once);
                    }
                }
                else  // target > 205 -> pre control to 205
                {
                    if ( GenApi::IsAvailable(cam_->AutoTargetValue) )
                    {
                        cam_->AutoTargetValue.SetValue(autoTargetBrightnessMax, true);
                    } else if (GenApi::IsAvailable(cam_->AutoTargetBrightness))
                    {   
                        cam_->AutoTargetBrightness.SetValue(autoTargetBrightnessMax);
                    }
                    if ( exposure_auto )
                    {
                        cam_->ExposureAuto.TrySetValue(ExposureAutoEnums::ExposureAuto_Once);
                    }
                    if ( gain_auto )
                    {
                        cam_->GainAuto.TrySetValue(GainAutoEnums::GainAuto_Once);
                    }
                }
                is_binary_exposure_search_running_ = true;
            }
        }
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An generic exception while setting target brightness to "
                << target_brightness << " (= "
                << CameraTraitT::convertBrightness(std::min(255, target_brightness))
                <<  ") occurred: " << e.GetDescription());
        return false;
    }
    return true;
}

template <typename CameraTraitT>
bool PylonROS2CameraImpl<CameraTraitT>::setExtendedBrightness(const int& target_brightness,
                                                              const float& current_brightness)
{
    float autoTargetBrightnessMin = 0.0;
    float autoTargetBrightnessMax = 0.0;
    if ( GenApi::IsAvailable(cam_->AutoTargetValue) )
    {
        autoTargetBrightnessMin = cam_->AutoTargetValue.GetMin();
        autoTargetBrightnessMax = cam_->AutoTargetValue.GetMax();
    } else if (GenApi::IsAvailable(cam_->AutoTargetBrightness))
    {   
        autoTargetBrightnessMin = cam_->AutoTargetBrightness.GetMin();
        autoTargetBrightnessMax = cam_->AutoTargetBrightness.GetMax();
    }
    if (target_brightness > 0 && target_brightness <= 255)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error: Brightness value should be greater than 0 and equal to or smaller than 255");
        return false;
    }

    typename CameraTraitT::AutoTargetBrightnessValueType brightness_to_set =
        CameraTraitT::convertBrightness(target_brightness);

    if ( !binary_exp_search_ )
    {
        if ( brightness_to_set < autoTargetBrightnessMin )  // Range from [0 - 49]
        {
            binary_exp_search_ = new BinaryExposureSearch(target_brightness,
                                                          currentAutoExposureTimeLowerLimit(),
                                                          currentExposure(),
                                                          currentExposure());
        }
        else  // Range from [206-255]
        {
            binary_exp_search_ = new BinaryExposureSearch(target_brightness,
                                                          currentExposure(),
                                                          currentAutoExposureTimeUpperLimit(),
                                                          currentExposure());
        }
    }

    if ( binary_exp_search_->isLimitReached() )
    {
        disableAllRunningAutoBrightessFunctions();
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "BinaryExposureSearach reached the exposure limits that "
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
bool PylonROS2CameraImpl<CameraTraitT>::setShutterMode(const SHUTTER_MODE &shutter_mode)
{
    try
    {
        switch ( shutter_mode )
        {
        case pylon_ros2_camera::SM_ROLLING:
            cam_->ShutterMode.SetValue(ShutterModeEnums::ShutterMode_Rolling);
            break;
        case pylon_ros2_camera::SM_GLOBAL:
            cam_->ShutterMode.SetValue(ShutterModeEnums::ShutterMode_Global);
            break;
        case pylon_ros2_camera::SM_GLOBAL_RESET_RELEASE:
            cam_->ShutterMode.SetValue(ShutterModeEnums::ShutterMode_GlobalResetRelease);
            break;
        default:
            // keep default setting
            break;
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while setting shutter mode to "
               << shutter_mode << " occurred: " << e.GetDescription());
        return false;
    }
    return true;
}

template <typename CameraTraitT>
float PylonROS2CameraImpl<CameraTraitT>::exposureStep()
{
    return static_cast<float>(exposureTime().GetMin());
}

template <typename CameraTraitT>
float PylonROS2CameraImpl<CameraTraitT>::maxPossibleFramerate()
{
    return static_cast<float>(resultingFrameRate().GetValue());
}

template <typename CameraTraitT>
std::vector<int> PylonROS2CameraImpl<CameraTraitT>::detectAndCountNumUserOutputs()
{
    std::vector<int> user_output_vec;
    GenApi::INodeMap& node_map = cam_->GetNodeMap();
    GenApi::CEnumerationPtr output_selector_enumeration_ptr(
                                        node_map.GetNode("UserOutputSelector"));
    GenApi::NodeList_t feature_list;
    output_selector_enumeration_ptr->GetEntries(feature_list);
    for (GenApi::NodeList_t::iterator it = feature_list.begin();
         it != feature_list.end();
         ++it)
    {
        if ( GenApi::IsAvailable(*it) )
        {
            GenApi::CEnumEntryPtr enum_entry(*it);
            GenICam::gcstring symbolic_name = enum_entry->GetSymbolic().c_str();
            int num_value = enum_entry->GetNumericValue();
            if ( 0 != typeName().compare("GigE") )
            {
                // TODO: @marcel: Contact Basler support why this is necessary
                // for all USB cameras
                num_value += 1;
            }
            user_output_vec.push_back(num_value);
        }
    }
    return user_output_vec;
}

template <typename CameraTraitT>
bool PylonROS2CameraImpl<CameraTraitT>::setUserOutput(const int& output_id,
                                                  const bool& value)
{
    RCLCPP_DEBUG_STREAM(LOGGER_BASE, "Setting user_output " << output_id << " to " << value);
    try
    {
        cam_->UserOutputSelector.SetValue(static_cast<UserOutputSelectorEnums>(
                    user_output_selector_enums_.at(output_id)));
        cam_->UserOutputValue.SetValue(value);
    }
    catch (const std::exception& ex)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "Could not set user output "  << output_id << ": "
                << ex.what());
        return false;
    }
    catch (const GenICam_3_1_Basler_pylon::InvalidArgumentException& ex)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "Could not set user output "  << output_id << ": "
                << ex.what());
        return false;
    }

    if ( value != cam_->UserOutputValue.GetValue() )
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "Value " << value << " could not be set to output "
                << output_id);
        return false;
    }
    return true;
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setOffsetXY(const int& offset_value, bool xAxis)
{
    try
    {
        if (xAxis)
        {
            cam_->OffsetX.SetValue(offset_value);
        }
        else
        {
            cam_->OffsetY.SetValue(offset_value);
        }  

    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while trying to set the offset in x-axis/y-axis occurred:" << e.GetDescription());
        return e.GetDescription();
    }
    
    return "done";
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::reverseXY(const bool& data, bool around_x)
{
    try
    {
        if (around_x)
        {
            cam_->ReverseX.SetValue(data);
        }
        else
        {
            cam_->ReverseY.SetValue(data);   
        }  

    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while trying to reverse the image around x-axis/y-axis occurred:" << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

template <typename CameraTraitT>
bool PylonROS2CameraImpl<CameraTraitT>::getReverseXY(const bool& returnX)
{
    try
    {
        if (returnX)
        {
            return static_cast<bool>(cam_->ReverseX.GetValue());
        }
        else
        {
            return static_cast<bool>(cam_->ReverseY.GetValue());
        }  

    }
    catch ( const GenICam::GenericException &e )
    {
        return false;
    }
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setBlackLevel(const int& value)
{
    try
    {
        if ( GenApi::IsAvailable(cam_->BlackLevel) )
        {  
            cam_->BlackLevel.SetValue(value);   
            return "done";
        }
        else 
        {
             RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to change the image black level. The connected Camera not supporting this feature");
             return "The connected Camera not supporting this feature";
        }

    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while trying to change the image black level occurred:" << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

template <typename CameraTraitT>
int PylonROS2CameraImpl<CameraTraitT>::getBlackLevel()
{
    try
    {
        if ( GenApi::IsAvailable(cam_->BlackLevel) )
        {  
            return static_cast<int>(cam_->BlackLevel.GetValue());   
        }
        else 
        {
             return -10000;
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        return -10000;
    }
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setPGIMode(const bool& on)
{
 try
    {
        if ( GenApi::IsAvailable(cam_->PgiMode) )
        { 
            if (on)
            {
                cam_->PgiMode.SetValue(PgiModeEnums::PgiMode_On);
                return "done";
            }
            else
            {
                cam_->PgiMode.SetValue(PgiModeEnums::PgiMode_Off);
                return "done";
            }
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to change the PGI mode. The connected Camera not supporting this feature");
            return "The connected Camera not supporting this feature";
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while trying to set the PGI mode occurred:" << e.GetDescription());
        return e.GetDescription();
    }
}

template <typename CameraTraitT>
int PylonROS2CameraImpl<CameraTraitT>::getPGIMode()
{
 try
    {
        if ( GenApi::IsAvailable(cam_->PgiMode) )
        { 
            if (cam_->PgiMode.GetValue() == PgiModeEnums::PgiMode_On)
            {
                return 1; // On
            }
            else if (cam_->PgiMode.GetValue() == PgiModeEnums::PgiMode_Off)
            {
                return 0; // Off
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

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setDemosaicingMode(const int& mode)
{
 try
    {
        if ( GenApi::IsAvailable(cam_->DemosaicingMode) )
        {
            if (mode == 0 )
            {
                cam_->DemosaicingMode.SetValue(DemosaicingModeEnums::DemosaicingMode_Simple);
                return "done";
            }
            else if (mode == 1 )
            {
                cam_->DemosaicingMode.SetValue(DemosaicingModeEnums::DemosaicingMode_BaslerPGI);
                return "done";
            }
            else
            {
               return "Error: unknown value";
            }
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to change the demosaicing mode. The connected Camera not supporting this feature");
            return "The connected Camera not supporting this feature";
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while trying to set the demosaicing mode occurred:" << e.GetDescription());
        return e.GetDescription();
    }
}

template <typename CameraTraitT>
int PylonROS2CameraImpl<CameraTraitT>::getDemosaicingMode()
{
 try
    {
        if ( GenApi::IsAvailable(cam_->DemosaicingMode) )
        {
            if (cam_->DemosaicingMode.GetValue() == DemosaicingModeEnums::DemosaicingMode_Simple)
            {
                return 0; // Simple
            }
            else if (cam_->DemosaicingMode.GetValue() == DemosaicingModeEnums::DemosaicingMode_BaslerPGI)
            {
                return 1; // BaslerPGI
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

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setNoiseReduction(const float& value)
{
 try
    {
        if ( GenApi::IsAvailable(cam_->PgiMode) )
        {
            if (cam_->PgiMode.GetValue() == PgiModeEnums::PgiMode_On)
            {
                if ( GenApi::IsAvailable(cam_->NoiseReduction) )
                {
                    cam_->NoiseReduction.SetValue(value);
                    return "done";
                }
                else 
                {
                    RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to change the noise reduction value. The camera not having this feature");
                    return "The camera not having this feature";
                }
            }
            else
            {
                return "Error : Noise Reduction feature not available while PGI mode is off";
            }
        }
        else if ( GenApi::IsAvailable(cam_->NoiseReduction) )
            {
                    cam_->NoiseReduction.SetValue(value);
                    return "done";
            }
        else if ( GenApi::IsAvailable(cam_->BslNoiseReduction))
            {
                    cam_->BslNoiseReduction.SetValue(value);
                    return "done";
            }

        else 
            {
                RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to change the noise reduction value. The connected Camera not supporting this feature");
                return "The connected Camera not supporting this feature";
            } 
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while trying to set the noise reduction value occurred:" << e.GetDescription());
        return e.GetDescription();
    }
}

template <typename CameraTraitT>
float PylonROS2CameraImpl<CameraTraitT>::getNoiseReduction()
{
 try
    {
        if ( GenApi::IsAvailable(cam_->NoiseReduction) )
        {
            return static_cast<float>(cam_->NoiseReduction.GetValue());
        } else if ( GenApi::IsAvailable(cam_->BslNoiseReduction) )
        {
            return static_cast<float>(cam_->BslNoiseReduction.GetValue());
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

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setSharpnessEnhancement(const float& value)
{
 try
    {
        if ( GenApi::IsAvailable(cam_->PgiMode) )
        {
            if (cam_->PgiMode.GetValue() == PgiModeEnums::PgiMode_On)
            {
                if ( GenApi::IsAvailable(cam_->SharpnessEnhancement) )
                {
                    cam_->SharpnessEnhancement.SetValue(value);
                    return "done";
                }
                else 
                {
                    RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to change the sharpness enhancement value. The connected Camera not supporting this feature");
                    return "The connected Camera not supporting this feature";
                }
            }
            else
            {
                return "Error : Sharpness Enhancement feature not available while PGI mode is off";
            }
        }
        else if ( GenApi::IsAvailable(cam_->SharpnessEnhancement) )
            {
                    cam_->SharpnessEnhancement.SetValue(value);
                    return "done";
            }
        else if ( GenApi::IsAvailable(cam_->BslSharpnessEnhancement) )
            {
                    cam_->BslSharpnessEnhancement.SetValue(value);
                    return "done";
            }
        else 
            {
                RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to change the sharpness enhancement value, The connected Camera not supporting this feature");
                return "The connected Camera not supporting this feature";
            } 
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while trying to set the sharpness enhancement value occurred:" << e.GetDescription());
        return e.GetDescription();
    }
}

template <typename CameraTraitT>
float PylonROS2CameraImpl<CameraTraitT>::getSharpnessEnhancement()
{
 try
    {
        if ( GenApi::IsAvailable(cam_->SharpnessEnhancement) )
        {
            return static_cast<float>(cam_->SharpnessEnhancement.GetValue());
        }
        else if (GenApi::IsAvailable(cam_->BslSharpnessEnhancement))
        {
            return static_cast<float>(cam_->BslSharpnessEnhancement.GetValue());
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

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setSensorReadoutMode(const int& mode)
{
    try
    {   if ( GenApi::IsAvailable(cam_->SensorReadoutMode) )
        {
            if (mode == 0)
            {
                cam_->SensorReadoutMode.SetValue(SensorReadoutModeEnums::SensorReadoutMode_Normal);

            }
            else if (mode == 1)
            {
                cam_->SensorReadoutMode.SetValue(SensorReadoutModeEnums::SensorReadoutMode_Fast);
            }
            else 
            {
                return "Error: unknown value";
            }  
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to change the sensor readout mode. The connected Camera not supporting this feature");
            return "The connected Camera not supporting this feature";
        }

    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while trying to set the sensor readout mode occurred:" << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

template <typename CameraTraitT>
int PylonROS2CameraImpl<CameraTraitT>::getSensorReadoutMode()
{
    try
    {   if ( GenApi::IsAvailable(cam_->SensorReadoutMode) )
        {
            if (cam_->SensorReadoutMode.GetValue() == SensorReadoutModeEnums::SensorReadoutMode_Normal)
            {
                return 0; // Normal

            }
            else if (cam_->SensorReadoutMode.GetValue() == SensorReadoutModeEnums::SensorReadoutMode_Fast)
            {
                return 1; // Fast
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

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setTriggerSelector(const int& mode)
{
    try
    {
        if ( GenApi::IsAvailable(cam_->TriggerSelector) )
        {  
            if (mode == 0)
            {
                cam_->TriggerSelector.SetValue(TriggerSelectorEnums::TriggerSelector_FrameStart);
                RCLCPP_INFO_STREAM(LOGGER_BASE, "Trigger selector: Frame Start");
                return "done";
            }
            else if (mode == 1)
            {
                cam_->TriggerSelector.SetValue(TriggerSelectorEnums::TriggerSelector_FrameBurstStart);
                RCLCPP_INFO_STREAM(LOGGER_BASE, "Trigger selector: Frame Burst Start");
                return "done";
            }
            else
            {
                return "Error: unknown value";
            }
        }
        else 
        {
             RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to change the Acquisition frame count. The connected Camera not supporting this feature");
             return "The connected Camera not supporting this feature";
        }

    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while trying to change the Acquisition frame count occurred:" << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

template <typename CameraTraitT>
int PylonROS2CameraImpl<CameraTraitT>::getTriggerSelector()
{
    try
    {
        if ( GenApi::IsAvailable(cam_->TriggerSelector) )
        {  
            if (cam_->TriggerSelector.GetValue() == TriggerSelectorEnums::TriggerSelector_FrameStart)
            {
                return 0; // FrameStart
            }
            else if (cam_->TriggerSelector.GetValue() == TriggerSelectorEnums::TriggerSelector_FrameBurstStart)
            {
                return 1; // FrameBurstStart
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
        return -2; // Eror
    }
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setTriggerMode(const bool& value)
{
    try
    {   if ( GenApi::IsAvailable(cam_->TriggerMode) )
        {
            if (value)
            {
                cam_->TriggerMode.SetValue(TriggerModeEnums::TriggerMode_On);
            }
            else
            {
                cam_->TriggerMode.SetValue(TriggerModeEnums::TriggerMode_Off);
            }
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to change the trigger mode. The connected Camera not supporting this feature");
            return "The connected Camera not supporting this feature";
        }

    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while setting the trigger mode occurred:" << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

template <typename CameraTraitT>
int PylonROS2CameraImpl<CameraTraitT>::getTriggerMode()
{
    try
    {   if ( GenApi::IsAvailable(cam_->TriggerMode) )
        {
            if (cam_->TriggerMode.GetValue() == TriggerModeEnums::TriggerMode_On)
            {
                return 1; // On
            }
            else if (cam_->TriggerMode.GetValue() == TriggerModeEnums::TriggerMode_Off)
            {
               return 0; // Off
            }
            else 
            {
               return -3; // Off 
            }
        }
        else 
        {
            return -1; // Not available
        }

    }
    catch ( const GenICam::GenericException &e )
    {
        return -2; // error
    }

}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::executeSoftwareTrigger()
{
    try
    {   if ( GenApi::IsAvailable(cam_->TriggerSoftware) )
        {
            cam_->TriggerSoftware.Execute();
            return "done";
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to change the trigger mode. The connected Camera not supporting this feature");
            return "The connected Camera not supporting this feature";
        }

    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while setting the trigger mode occurred:" << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setTriggerSource(const int& source)
{
    try
    {   if (GenApi::IsAvailable(cam_->TriggerSource))
        {
            switch (source)
            {
                case 0:
                    cam_->TriggerSource.SetValue(TriggerSourceEnums::TriggerSource_Software);
                    RCLCPP_INFO_STREAM(LOGGER_BASE, "Trigger source: Software");
                    break;
                case 1:
                    cam_->TriggerSource.SetValue(TriggerSourceEnums::TriggerSource_Line1);
                    RCLCPP_INFO_STREAM(LOGGER_BASE, "Trigger source: Line 1");
                    break;
                case 2:
                    cam_->TriggerSource.SetValue(TriggerSourceEnums::TriggerSource_Line2);
                    RCLCPP_INFO_STREAM(LOGGER_BASE, "Trigger source: Line 2");
                    break;
                case 3:
                    cam_->TriggerSource.SetValue(TriggerSourceEnums::TriggerSource_Line3);
                    RCLCPP_INFO_STREAM(LOGGER_BASE, "Trigger source: Line 3");
                    break;
                case 4:
                    cam_->TriggerSource.SetValue(TriggerSourceEnums::TriggerSource_Line4);
                    RCLCPP_INFO_STREAM(LOGGER_BASE, "Trigger source: Line 4");
                    break;
                case 5:
                    cam_->TriggerSource.SetValue(TriggerSourceEnums::TriggerSource_Action1);
                    RCLCPP_INFO_STREAM(LOGGER_BASE, "Trigger source: Action 1");
                    break;
                case 6:
                    cam_->TriggerSource.SetValue(TriggerSourceEnums::TriggerSource_PeriodicSignal1);
                    RCLCPP_INFO_STREAM(LOGGER_BASE, "Trigger source: Periodic Signal 1");
                    break;
                default:
                    RCLCPP_ERROR_STREAM(LOGGER_BASE, "Trigger source value is invalid! Please choose between 0 -> Trigger Software / 1 -> Line 1 / 2 -> Line 2 / 3 -> Line 3 / 4 -> Line 4 / 5 -> Action 1 / 6 -> Periodic signal 1");
                    return "Error: unknown value for trigger source";
            }
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to change the trigger source. The connected camera does not support this feature");
            return "The connected camera does not support this feature";
        }

    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while setting the trigger source occurred:" << e.GetDescription());
        RCLCPP_WARN_STREAM(LOGGER_BASE, "If the enum entry is not writable or if the value is invalid, it may be because the connected camera does not support this value");
        return e.GetDescription();
    }
    return "done";
}

template <typename CameraTraitT>
int PylonROS2CameraImpl<CameraTraitT>::getTriggerSource()
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
            else if (cam_->TriggerSource.GetValue() == TriggerSourceEnums::TriggerSource_Line2)
            {
                return 2; // Line2
            }
            else if (cam_->TriggerSource.GetValue() == TriggerSourceEnums::TriggerSource_Line3)
            {
                return 3; // Line3
            }
            else if (cam_->TriggerSource.GetValue() == TriggerSourceEnums::TriggerSource_Line4)
            {
                return 4; // Line4
            }
            else if (cam_->TriggerSource.GetValue() == TriggerSourceEnums::TriggerSource_Action1)
            {
                return 5; // Action1
            }
            else if (cam_->TriggerSource.GetValue() == TriggerSourceEnums::TriggerSource_PeriodicSignal1)
            {
                return 6; // PeriodicSignal1
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
    catch (const GenICam::GenericException &e)
    {
        return -2; // Error
    }
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setTriggerActivation(const int& value)
{
    try
    {   if ( GenApi::IsAvailable(cam_->TriggerActivation) )
        {
            if (value == 0)
            {
                cam_->TriggerActivation.SetValue(TriggerActivationEnums::TriggerActivation_RisingEdge);
            }
            else if (value == 1)
            {
                cam_->TriggerActivation.SetValue(TriggerActivationEnums::TriggerActivation_FallingEdge);
            }
            else 
            {
                return "Error: unknown value";
            }
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to change the trigger activation type. The connected Camera not supporting this feature");
            return "The connected Camera not supporting this feature";
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while setting the trigger activation type occurred:" << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

template <typename CameraTraitT>
int PylonROS2CameraImpl<CameraTraitT>::getTriggerActivation()
{
    try
    {   if ( GenApi::IsAvailable(cam_->TriggerActivation) )
        {
            if (cam_->TriggerActivation.GetValue() == TriggerActivationEnums::TriggerActivation_RisingEdge)
            {
                return 0; // RisingEdge
            }
            else if (cam_->TriggerActivation.GetValue() == TriggerActivationEnums::TriggerActivation_FallingEdge)
            {
                return 1; // FallingEdge
            }
            else 
            {
                return -3; // Unknown
            }
        }
        else 
        {
            return -1; // Not Available
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        return -2; // Error
    }
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setTriggerDelay(const float& delayValue)
{
    try
    {   if ( GenApi::IsAvailable(cam_->TriggerDelay) )
        {

            cam_->TriggerDelay.SetValue(delayValue);
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to change the trigger delay. The connected Camera not supporting this feature");
            return "The connected Camera not supporting this feature";
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while setting the trigger delay occurred:" << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

template <typename CameraTraitT>
float PylonROS2CameraImpl<CameraTraitT>::getTriggerDelay()
{
    try
    {   if ( GenApi::IsAvailable(cam_->TriggerDelay) )
        {

            return static_cast<float>(cam_->TriggerDelay.GetValue());
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

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setLineSelector(const int& value)
{
    try
    {   if ( GenApi::IsAvailable(cam_->LineSelector) )
        {
            if (value == 0)
            {
                cam_->LineSelector.SetValue(LineSelectorEnums::LineSelector_Line1);
            }
            else if (value == 1)
            {
                cam_->LineSelector.SetValue(LineSelectorEnums::LineSelector_Line2);
            }
            else if (value == 2)
            {
                cam_->LineSelector.SetValue(LineSelectorEnums::LineSelector_Line3);
            }
            else if (value == 3)
            {
                cam_->LineSelector.SetValue(LineSelectorEnums::LineSelector_Line4);
            }
            else 
            {
                return "Error: unknown value";
            }
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to set the line selector. The connected Camera not supporting this feature");
            return "The connected Camera not supporting this feature";
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while setting the line selector occurred:" << e.GetDescription());
        return e.GetDescription(); 
    }
    return "done";
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setLineMode(const int& value)
{
    try
    {   if ( (cam_->LineFormat.GetValue() == LineFormatEnums::LineFormat_TTL) || (cam_->LineFormat.GetValue() == LineFormatEnums::LineFormat_LVTTL) )
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
            RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error : the selected line number don't have change line mode feature");
            return "Error : the selected line number dose not have change line mode feature";
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while setting the line mode occurred:" << e.GetDescription());
        return e.GetDescription(); 
    }
    return "done";
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setLineSource(const int& value)
{
    try
    {  
        if (cam_->LineMode.GetValue() == LineModeEnums::LineMode_Output )
        {
            if (value == 0)
            {   
                cam_->LineSource.SetValue(LineSourceEnums::LineSource_ExposureActive);
                return "done";
            }
            else if (value == 1)
            {
                cam_->LineSource.SetValue(LineSourceEnums::LineSource_FrameTriggerWait);
                return "done"; 
            }
            else if (value == 2)
            {
                cam_->LineSource.SetValue(LineSourceEnums::LineSource_UserOutput1);
                return "done"; 
            }
            else if (value == 3)
            {
                cam_->LineSource.SetValue(LineSourceEnums::LineSource_Timer1Active);
                return "done"; 
            }
            else if (value == 4)
            {
                if (cam_->ShutterMode.GetValue() == ShutterModeEnums::ShutterMode_Rolling )
                {
                  cam_->LineSource.SetValue(LineSourceEnums::LineSource_FlashWindow);
                    return "done";  
                }
                else 
                {
                    return "Error: the line source 'FlashWindow' supported by rolling shutter only";
                }
                 
            }
            else 
            {
                return "Error: unknown value";
            }
        }
        else 
        {
            return "Error : can't change the line source, the selected line mode should be output";
        }

    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while setting the line source occurred:" << e.GetDescription());
        return e.GetDescription(); 
    }
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setLineDebouncerTime(const float& value)
{
    try
    {     
        if ( GenApi::IsAvailable(cam_->LineDebouncerTime) ) 
        {
                if ( cam_->LineMode.GetValue() == LineModeEnums::LineMode_Input)
            {
                cam_->LineDebouncerTime.SetValue(value);
            }
            else 
            {
                return "Error: can't set the line debouncer time, the selected line mode should be input";
            }
        } else {
            return "The connected Camera not supporting this feature";
        }
        
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while setting the line debouncer time occurred:" << e.GetDescription());
        return e.GetDescription(); 
    }
    return "done";
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setLineInverter(const bool& value)
{
    try
    {   if ( GenApi::IsAvailable(cam_->LineInverter) )
        {
            cam_->LineInverter.SetValue(value);
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to set the line inverter. The connected Camera or selected line not supporting this feature");
            return "The connected Camera or selected line not supporting this feature";
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while setting the line inverter occurred:" << e.GetDescription());
        return e.GetDescription(); 
    }
    return "done";
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setDeviceLinkThroughputLimitMode(const bool& turnOn)
{
    try
    {
        if ( GenApi::IsAvailable(cam_->DeviceLinkThroughputLimitMode) )
        {  
            if (turnOn)
            {
            cam_->DeviceLinkThroughputLimitMode.SetValue(DeviceLinkThroughputLimitModeEnums::DeviceLinkThroughputLimitMode_On);   
            return "done";
            }
            else 
            {
            cam_->DeviceLinkThroughputLimitMode.SetValue(DeviceLinkThroughputLimitModeEnums::DeviceLinkThroughputLimitMode_Off);    
            return "done";
            }
        }
        else 
        {
             RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to change the device link throughput limit mode. The connected Camera not supporting this feature");
             return "The connected Camera not supporting this feature";
        }

    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while changing the device link throughput limit mode occurred:" << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

template <typename CameraTraitT>
int PylonROS2CameraImpl<CameraTraitT>::getDeviceLinkThroughputLimitMode()
{
    try
    {
        if ( GenApi::IsAvailable(cam_->DeviceLinkThroughputLimitMode) )
        {  
            if (cam_->DeviceLinkThroughputLimitMode.GetValue() == DeviceLinkThroughputLimitModeEnums::DeviceLinkThroughputLimitMode_On)
            {
                return 0; // On
            }
            else if (cam_->DeviceLinkThroughputLimitMode.GetValue() == DeviceLinkThroughputLimitModeEnums::DeviceLinkThroughputLimitMode_Off)
            {
            return 1; // Off
            }
            else 
            {
            return -3; // Unknown
            }
        }
        else 
        {
             return -1; // Not Available
        }

    }
    catch ( const GenICam::GenericException &e )
    {
        return -2; // Error
    }
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setDeviceLinkThroughputLimit(const int& limit)
{
    try
    {
        if ( GenApi::IsAvailable(cam_->DeviceLinkThroughputLimit) )
        {  
            if (cam_->DeviceLinkThroughputLimitMode.GetValue() == DeviceLinkThroughputLimitModeEnums::DeviceLinkThroughputLimitMode_On)
            {
                cam_->DeviceLinkThroughputLimit.SetValue(limit);     
                return "done";
            }
            else 
            {
                return "Error : device link throughput limit mode is off";
            }
        }
        else 
        {
             RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to change the device link throughput limit. The connected Camera not supporting this feature");
             return "The connected Camera not supporting this feature";
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while changing the device link throughput limit occurred:" << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setBalanceWhiteAuto(const int& mode)
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
             RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to change the balance white auto. The connected Camera not supporting this feature");
             return "The connected Camera not supporting this feature";
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while changing the balance white auto occurred:" << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

template <typename CameraTraitT>
int PylonROS2CameraImpl<CameraTraitT>::getBalanceWhiteAuto()
{
    try
    {
        if (GenApi::IsAvailable(cam_->BalanceWhiteAuto))
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
             return -1; // Not Available
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        return -2; // Error
    }
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setLightSourcePreset(const int& mode)
{
    try
    {
        if ( GenApi::IsAvailable(cam_->LightSourcePreset))
        {  
            if (mode == 0)
            {
                cam_->LightSourcePreset.SetValue(LightSourcePresetEnums::LightSourcePreset_Off);
            }  
            else if (mode == 1)
            {
                cam_->LightSourcePreset.SetValue(LightSourcePresetEnums::LightSourcePreset_Daylight5000K);
            } 
            else if (mode == 2)
            {
                cam_->LightSourcePreset.SetValue(LightSourcePresetEnums::LightSourcePreset_Daylight6500K);
            } 
            else if (mode == 3)
            {
                cam_->LightSourcePreset.SetValue(LightSourcePresetEnums::LightSourcePreset_Tungsten2800K);
            } 
            else 
            {
                return "Error: unknown value";
            }
        } else if (GenApi::IsAvailable(cam_->BslLightSourcePreset)) {  
            if (mode == 0)
            {
                cam_->BslLightSourcePreset.SetValue(Basler_UniversalCameraParams::BslLightSourcePresetEnums::BslLightSourcePreset_Off);
            }  
            else if (mode == 1)
            {
                cam_->BslLightSourcePreset.SetValue(Basler_UniversalCameraParams::BslLightSourcePresetEnums::BslLightSourcePreset_Daylight5000K);
            } 
            else if (mode == 2)
            {
                cam_->BslLightSourcePreset.SetValue(Basler_UniversalCameraParams::BslLightSourcePresetEnums::BslLightSourcePreset_Daylight6500K);
            }  
            else 
            {
                return "Error: unknown value";
            }
        }
        else 
        {
             RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to change the light source preset. The connected Camera not supporting this feature");
             return "The connected Camera not supporting this feature";
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while changing the light source preset occurred:" << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

template <typename CameraTraitT>
int PylonROS2CameraImpl<CameraTraitT>::getLightSourcePreset()
{
    try
    {
        if ( GenApi::IsAvailable(cam_->LightSourcePreset))
        {  
            if (cam_->LightSourcePreset.GetValue() == LightSourcePresetEnums::LightSourcePreset_Off)
            {
                return 0; // Off
            }  
            else if (cam_->LightSourcePreset.GetValue() == LightSourcePresetEnums::LightSourcePreset_Daylight5000K)
            {
                return 1; // Daylight5000K
            } 
            else if (cam_->LightSourcePreset.GetValue() == LightSourcePresetEnums::LightSourcePreset_Daylight6500K)
            {
                return 2; // Daylight6500K
            } 
            else if (cam_->LightSourcePreset.GetValue() == LightSourcePresetEnums::LightSourcePreset_Tungsten2800K)
            {
                return 3; // Tungsten2800K
            } 
            else 
            {
                return -3; // Unkonwn
            }
        } else if ( GenApi::IsAvailable(cam_->BslLightSourcePreset))
        {
            if (cam_->BslLightSourcePreset.GetValue() == Basler_UniversalCameraParams::BslLightSourcePresetEnums::BslLightSourcePreset_Off)
            {
                return 0; // Off
            }  
            else if (cam_->BslLightSourcePreset.GetValue() == Basler_UniversalCameraParams::BslLightSourcePresetEnums::BslLightSourcePreset_Daylight5000K)
            {
                return 1; // Daylight5000K
            } 
            else if (cam_->BslLightSourcePreset.GetValue() == Basler_UniversalCameraParams::BslLightSourcePresetEnums::BslLightSourcePreset_Daylight6500K)
            {
                return 2; // Daylight6500K
            } 
            else 
            {
                return -3; // Unkonwn
            }
        }
        else 
        {
             return -1; // Not available
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        return -2; // error
    }
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setUserSetSelector(const int& set)
{
    try
    {
        grabbingStopping();
        if ( GenApi::IsAvailable(cam_->UserSetSelector))
        {  
            if (set == 0)
            {
                cam_->UserSetSelector.SetValue(UserSetSelectorEnums::UserSetSelector_Default);
            }  
            else if (set == 1)
            {
                cam_->UserSetSelector.SetValue(UserSetSelectorEnums::UserSetSelector_UserSet1);
            } 
            else if (set == 2)
            {
                cam_->UserSetSelector.SetValue(UserSetSelectorEnums::UserSetSelector_UserSet2);
            } 
            else if (set == 3)
            {
                cam_->UserSetSelector.SetValue(UserSetSelectorEnums::UserSetSelector_UserSet3);
            } 
            else if (set == 4)
            {
                cam_->UserSetSelector.SetValue(UserSetSelectorEnums::UserSetSelector_HighGain);
            } 
            else if (set == 5)
            {
                cam_->UserSetSelector.SetValue(UserSetSelectorEnums::UserSetSelector_AutoFunctions);
            } 
            else if (set == 6)
            {
                cam_->UserSetSelector.SetValue(UserSetSelectorEnums::UserSetSelector_ColorRaw);
            } 
            else 
            {
                grabbingStarting();
                return "Error: unknown value";
            }
            grabbingStarting();
        }
        else 
        {
             RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to select the user set. The connected Camera not supporting this feature");
             grabbingStarting();
             return "The connected Camera not supporting this feature";
        }
        grabbingStarting();
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while selecting the user set occurred:" << e.GetDescription());
        grabbingStarting();
        return e.GetDescription();
    }
    return "done";
}

template <typename CameraTraitT>
int PylonROS2CameraImpl<CameraTraitT>::getUserSetSelector()
{
    try
    {
        if ( GenApi::IsAvailable(cam_->UserSetSelector))
        {  
            if (cam_->UserSetSelector.GetValue() == UserSetSelectorEnums::UserSetSelector_Default)
            {
                return 0; // Default
            }  
            else if (cam_->UserSetSelector.GetValue() == UserSetSelectorEnums::UserSetSelector_UserSet1)
            {
                return 1; // UserSet1
            } 
            else if (cam_->UserSetSelector.GetValue() == UserSetSelectorEnums::UserSetSelector_UserSet2)
            {
                return 2; // UserSet2
            } 
            else if (cam_->UserSetSelector.GetValue() == UserSetSelectorEnums::UserSetSelector_UserSet3)
            {
                return 3; // UserSet3
            } 
            else if (cam_->UserSetSelector.GetValue() == UserSetSelectorEnums::UserSetSelector_HighGain)
            {
                return 4; // HighGain
            } 
            else if (cam_->UserSetSelector.GetValue() == UserSetSelectorEnums::UserSetSelector_AutoFunctions)
            {
                return 5; // AutoFunctions
            } 
            else if (cam_->UserSetSelector.GetValue() == UserSetSelectorEnums::UserSetSelector_ColorRaw)
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

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::saveUserSet()
{
    try
    {   
        grabbingStopping();
        cam_->UserSetSave.Execute();
        grabbingStarting();
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while saving the user set occurred:" << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::loadUserSet()
{
    try
    {
        grabbingStopping();
        cam_->UserSetLoad.Execute();
        grabbingStarting();
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while loading the user set occurred:" << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

template <typename CameraTraitT>
std::pair<std::string, std::string> PylonROS2CameraImpl<CameraTraitT>::getPfs()
{
    std::pair<std::string, std::string> stringResults;
    try
    {
        grabbingStopping();
        Pylon::String_t cameraPfsString;
        Pylon::CFeaturePersistence::SaveToString(cameraPfsString, &cam_->GetNodeMap());
        stringResults.second = cameraPfsString; 
        grabbingStarting();
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while getting camera configuration as pfs:" << e.GetDescription());
        stringResults.first = e.GetDescription();
        return stringResults;
    }
    stringResults.first = "done";
    return stringResults;
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::savePfs(const std::string& fileName)
{
    try
    {
        grabbingStopping();
        Pylon::String_t fileNameCStr = fileName.c_str();
        RCLCPP_INFO_STREAM(LOGGER_BASE, "Saving the pfs file: " << fileNameCStr );
        Pylon::CFeaturePersistence::Save(fileNameCStr, &cam_->GetNodeMap());
        grabbingStarting();
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while saving the pfs file:" << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::loadPfs(const std::string& fileName)
{
    try
    {
        grabbingStopping();
        Pylon::String_t fileNameCStr = fileName.c_str();
        RCLCPP_INFO_STREAM(LOGGER_BASE, "Loading the pfs file: " << fileNameCStr );
        Pylon::CFeaturePersistence::Load(fileNameCStr, &cam_->GetNodeMap(), true);
        grabbingStarting();
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while loading the pfs file:" << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setUserSetDefaultSelector(const int& set)
{
    try
    {
        if ( GenApi::IsAvailable(cam_->UserSetDefault))
        {  
            if (set == 0)
            {
                grabbingStopping();
                cam_->UserSetDefault.SetValue(UserSetDefaultSelectorEnums::UserSetDefault_Default);
                grabbingStarting();
            }  
            else if (set == 1)
            {
                grabbingStopping();
                cam_->UserSetDefault.SetValue(UserSetDefaultSelectorEnums::UserSetDefault_UserSet1);
                grabbingStarting();
            } 
            else if (set == 2)
            {
                grabbingStopping();
                cam_->UserSetDefault.SetValue(UserSetDefaultSelectorEnums::UserSetDefault_UserSet2);
                grabbingStarting();
            } 
            else if (set == 3)
            {
                grabbingStopping();
                cam_->UserSetDefault.SetValue(UserSetDefaultSelectorEnums::UserSetDefault_UserSet3);
                grabbingStarting();
            } 
            else if (set == 4)
            {
                grabbingStopping();
                cam_->UserSetDefault.SetValue(UserSetDefaultSelectorEnums::UserSetDefault_HighGain);
                grabbingStarting();
            } 
            else if (set == 5)
            {
                grabbingStopping();
                cam_->UserSetDefault.SetValue(UserSetDefaultSelectorEnums::UserSetDefault_AutoFunctions);
                grabbingStarting();
            } 
            else if (set == 6)
            {
                grabbingStopping();
                cam_->UserSetDefault.SetValue(UserSetDefaultSelectorEnums::UserSetDefault_ColorRaw);
                grabbingStarting();
            } 
            else 
            {
                return "Error: unknown value";
            }
        }
        else 
        {
             RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to select the default user set. The connected Camera not supporting this feature");
             return "The connected Camera not supporting this feature";
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while changing user set default selector occurred:" << e.GetDescription());
        return e.GetDescription();
    }
    return "done";
}

template <typename CameraTraitT>
int PylonROS2CameraImpl<CameraTraitT>::getUserSetDefaultSelector()
{
    try
    {
        if ( GenApi::IsAvailable(cam_->UserSetDefault))
        {  
            if (cam_->UserSetDefault.GetValue() == UserSetDefaultSelectorEnums::UserSetDefault_Default)
            {
                return 0; // Default
            }  
            else if (cam_->UserSetDefault.GetValue() == UserSetDefaultSelectorEnums::UserSetDefault_UserSet1)
            {
                return 1; // UserSet1
            } 
            else if (cam_->UserSetDefault.GetValue() == UserSetDefaultSelectorEnums::UserSetDefault_UserSet2)
            {
                return 2; // UserSet2
            } 
            else if (cam_->UserSetDefault.GetValue() == UserSetDefaultSelectorEnums::UserSetDefault_UserSet3)
            {
                return 3; // UserSet3
            } 
            else if (cam_->UserSetDefault.GetValue() == UserSetDefaultSelectorEnums::UserSetDefault_HighGain)
            {
                return 4; // HighGain
            } 
            else if (cam_->UserSetDefault.GetValue() == UserSetDefaultSelectorEnums::UserSetDefault_AutoFunctions)
            {
                return 5; // AutoFunctions
            } 
            else if (cam_->UserSetDefault.GetValue() == UserSetDefaultSelectorEnums::UserSetDefault_ColorRaw)
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

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::triggerDeviceReset()
{
    try
    {
        if ( GenApi::IsAvailable(cam_->DeviceReset))
        {  
            cam_->DeviceReset.Execute();
            return "done";
        }
        else 
        {
             RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to reset the camera. The connected Camera not supporting this feature");
             return "The connected Camera not supporting this feature";
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while reseting the camera occurred:" << e.GetDescription());
        return e.GetDescription();
    }
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::grabbingStarting() const
{
    try
    {
        if(grab_strategy == 0) {
           cam_->StartGrabbing(Pylon::EGrabStrategy::GrabStrategy_OneByOne); 
        } else if (grab_strategy == 1) {
           cam_->StartGrabbing(Pylon::EGrabStrategy::GrabStrategy_LatestImageOnly); 
        } else if (grab_strategy == 2) {
           cam_->StartGrabbing(Pylon::EGrabStrategy::GrabStrategy_LatestImages); 
        } else {
            cam_->StartGrabbing(Pylon::EGrabStrategy::GrabStrategy_OneByOne); 
        }

        return "done";

    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while starting image grabbing occurred:" << e.GetDescription());
        return e.GetDescription();
    }
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::grabbingStopping()
{
    try
    {
        cam_->StopGrabbing();
        return "done";
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while stopping image grabbing occurred:" << e.GetDescription());
        return e.GetDescription();
    }
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setMaxTransferSize(const int& maxTransferSize)
{
    try
    {
        grabbingStopping();
        cam_->GetStreamGrabberParams().MaxTransferSize.SetValue(maxTransferSize);
        grabbingStarting();
        return "done";
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while setting the max transfer size occurred:" << e.GetDescription());
        grabbingStarting();
        return e.GetDescription();
    }
}

template <typename CameraTraitT> 
float PylonROS2CameraImpl<CameraTraitT>::getTemperature(){
    return 0.0;
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setWhiteBalance(const double& redValue, const double& greenValue, const double& blueValue)
{
    try
    {
        if (GenApi::IsAvailable(cam_->BalanceWhiteAuto) && GenApi::IsAvailable(cam_->BalanceRatio))
        {
            cam_->BalanceWhiteAuto.SetValue(BalanceWhiteAutoEnums::BalanceWhiteAuto_Off);
            cam_->BalanceRatioSelector.SetValue(BalanceRatioSelectorEnums::BalanceRatioSelector_Red);
            cam_->BalanceRatio.SetValue(redValue);
            cam_->BalanceRatioSelector.SetValue(BalanceRatioSelectorEnums::BalanceRatioSelector_Green);
            cam_->BalanceRatio.SetValue(greenValue);
            cam_->BalanceRatioSelector.SetValue(BalanceRatioSelectorEnums::BalanceRatioSelector_Blue);
            cam_->BalanceRatio.SetValue(blueValue);

            return "done"; 
        } 
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to set the white balance. The connected Camera not supporting this feature");
            return "The connected Camera not supporting this feature";
        }
        
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while setting the white balance occurred:" << e.GetDescription());
        return e.GetDescription();
    }
}

template <typename CameraTraitT> 
bool PylonROS2CameraImpl<CameraTraitT>::setGrabbingStrategy(const int& strategy) {
    if (strategy >= 0 && strategy <= 2){
        grab_strategy = strategy;
        return true;
    } else {
        return false;
    }

}

template <typename CameraTraitT> 
std::string PylonROS2CameraImpl<CameraTraitT>::setOutputQueueSize(const int& size) {
    if (size >= 0 && size <= cam_->MaxNumBuffer.GetValue()){
        try {
            cam_->OutputQueueSize.SetValue(size);
            return "done";
        } catch ( const GenICam::GenericException &e ){
            RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while setting the output queue size occurred:" << e.GetDescription());
            return e.GetDescription();
        }
    } else {
        return "requested output queue size is out side the limits of : 0-"+std::to_string(cam_->MaxNumBuffer.GetValue());
    }

}

template <typename CameraTraitT> 
std::string PylonROS2CameraImpl<CameraTraitT>::setMaxNumBuffer(const int& size) {
    if (GenApi::IsAvailable(cam_->MaxNumBuffer)){
        try {
            grabbingStopping();
            cam_->MaxNumBuffer.SetValue(size);
            grabbingStarting();
            return "done";
        } catch ( const GenICam::GenericException &e ){
                RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while setting the Maximum number of buffers size occurred:" << e.GetDescription());
                return e.GetDescription();
        }
    } else {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to set the maximum number buffers. The connected Camera not supporting this feature");
        return "The connected Camera not supporting this feature";
    }

}

template <typename CameraTraitT> 
int PylonROS2CameraImpl<CameraTraitT>::getMaxNumBuffer() {
    if (GenApi::IsAvailable(cam_->MaxNumBuffer)){
        try {
            return cam_->MaxNumBuffer.GetValue();
        } catch ( const GenICam::GenericException &e ){
                RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while getting the Maximum number of buffers size occurred:" << e.GetDescription());
                return -2;  // Error
        }
    } else {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to get the maximum number buffers. The connected Camera not supporting this feature");
        return -1;      // No Supported 
    }

}

template <typename CameraTraitT> 
int PylonROS2CameraImpl<CameraTraitT>::getStatisticTotalBufferCount() {
    if (GenApi::IsAvailable(cam_->GetStreamGrabberParams().Statistic_Total_Buffer_Count)){
        try {
            return cam_->GetStreamGrabberParams().Statistic_Total_Buffer_Count.GetValue();
        } catch ( const GenICam::GenericException &e ){
                RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while getting the Statistic Total Buffer Count occurred:" << e.GetDescription());
                return -2;  // Error
        }
    } else {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to get the Statistic Total Buffer Count. The connected Camera not supporting this feature");
        return -1;      // No Supported 
    }

}

template <typename CameraTraitT> 
int PylonROS2CameraImpl<CameraTraitT>::getStatisticFailedBufferCount() {
    if (GenApi::IsAvailable(cam_->GetStreamGrabberParams().Statistic_Failed_Buffer_Count)){
        try {
            return cam_->GetStreamGrabberParams().Statistic_Failed_Buffer_Count.GetValue();
        } catch ( const GenICam::GenericException &e ){
                RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while setting the Statistic Failed Buffer Count occurred:" << e.GetDescription());
                return -2;  // Error
        }
    } else {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to get the Statistic Failed Buffer Count. The connected Camera not supporting this feature");
        return -1;      // No Supported 
    }

}

template <typename CameraTraitT> 
int PylonROS2CameraImpl<CameraTraitT>::getStatisticBufferUnderrunCount() {
    if (GenApi::IsAvailable(cam_->GetStreamGrabberParams().Statistic_Buffer_Underrun_Count)){
        try {
            return cam_->GetStreamGrabberParams().Statistic_Buffer_Underrun_Count.GetValue();
        } catch ( const GenICam::GenericException &e ){
                RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while setting the Statistic Buffer Underrun Count occurred:" << e.GetDescription());
                return -2;  // Error
        }
    } else {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to get the Statistic Buffer Underrun Count. The connected Camera not supporting this feature");
        return -1;      // No Supported 
    }

}

template <typename CameraTraitT> 
int PylonROS2CameraImpl<CameraTraitT>::getStatisticFailedPacketCount() {
    if (GenApi::IsAvailable(cam_->GetStreamGrabberParams().Statistic_Failed_Packet_Count)){
        try {
            return cam_->GetStreamGrabberParams().Statistic_Failed_Packet_Count.GetValue();
        } catch ( const GenICam::GenericException &e ){
                RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while setting the Statistic Field Packet Count occurred:" << e.GetDescription());
                return -2;  // Error
        }
    } else {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to get the Statistic Field Packet Count. The connected Camera not supporting this feature");
        return -1;      // No Supported 
    }

}

template <typename CameraTraitT> 
int PylonROS2CameraImpl<CameraTraitT>::getStatisticResendRequestCount() {
    if (GenApi::IsAvailable(cam_->GetStreamGrabberParams().Statistic_Resend_Request_Count)){
        try {
            return cam_->GetStreamGrabberParams().Statistic_Resend_Request_Count.GetValue();
        } catch ( const GenICam::GenericException &e ){
                RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while setting the Statistic Resend Request Count occurred:" << e.GetDescription());
                return -2;  // Error
        }
    } else {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to get the Statistic Resend Request Count. The connected Camera not supporting this feature");
        return -1;      // No Supported 
    }

}

template <typename CameraTraitT> 
int PylonROS2CameraImpl<CameraTraitT>::getStatisticMissedFrameCount() {
    if (GenApi::IsAvailable(cam_->GetStreamGrabberParams().Statistic_Missed_Frame_Count)){
        try {
            return cam_->GetStreamGrabberParams().Statistic_Missed_Frame_Count.GetValue();
        } catch ( const GenICam::GenericException &e ){
                RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while setting the Statistic Missed Frame Count occurred:" << e.GetDescription());
                return -2;  // Error
        }
    } else {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to get the Statistic Missed Frame Count. The connected Camera not supporting this feature");
        return -1;      // No Supported 
    }

}

template <typename CameraTraitT> 
int PylonROS2CameraImpl<CameraTraitT>::getStatisticResynchronizationCount() {
    if (GenApi::IsAvailable(cam_->GetStreamGrabberParams().Statistic_Resynchronization_Count)){
        try {
            return cam_->GetStreamGrabberParams().Statistic_Resynchronization_Count.GetValue();
        } catch ( const GenICam::GenericException &e ){
                RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while setting the Statistic Resynchronization Count occurred:" << e.GetDescription());
                return -2;  // Error
        }
    } else {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to get the Statistic Resynchronization Count. The connected Camera not supporting this feature");
        return -1;      // No Supported 
    }

}

template <typename CameraTraitT> 
std::string PylonROS2CameraImpl<CameraTraitT>::setChunkModeActive(const bool& enable) {
    if (GenApi::IsAvailable(cam_->ChunkModeActive)){
        try {
            //cam_->StopGrabbing();
            cam_->ChunkModeActive.SetValue(enable);
            //grabbingStarting();
            return "done";
        } catch ( const GenICam::GenericException &e ){
                RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while setting the Chunk Mode Active occurred:" << e.GetDescription());
                return e.GetDescription();
        }
    } else {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to setting the Chunk Mode Active. The connected Camera not supporting this feature");
        return "The connected Camera not supporting this feature";      // No Supported 
    }

}

template <typename CameraTraitT> 
int PylonROS2CameraImpl<CameraTraitT>::getChunkModeActive() {
    if (GenApi::IsAvailable(cam_->ChunkModeActive)){
        try {
            if (cam_->ChunkModeActive.GetValue()){
                return 1;
            } else {
                return 0;
            }
        } catch ( ... ){ //const GenICam::GenericException &e
                //RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while getting the Chunk Mode Active occurred:" << e.GetDescription());
                return -2;
        }
    } else {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to getting the Chunk Mode Active. The connected Camera not supporting this feature");
        return -1;      // No Supported 
    }

}

template <typename CameraTraitT> 
std::string PylonROS2CameraImpl<CameraTraitT>::setChunkSelector(const int& value) {
    if (GenApi::IsAvailable(cam_->ChunkSelector)){
        try {
            switch(value){
                case 1 : 
                    cam_->ChunkSelector.SetValue(Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_AutoBrightnessStatus);
                    return "done";
                    break;
                case 2 :
                    cam_->ChunkSelector.SetValue(Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_BrightPixel);
                    return "done";
                    break;
                case 3 :
                    cam_->ChunkSelector.SetValue(Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_CounterValue);
                    return "done";
                    break;
                case 4 :
                    cam_->ChunkSelector.SetValue(Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_DynamicRangeMax);
                    return "done";
                    break;
                case 5 :
                    cam_->ChunkSelector.SetValue(Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_DynamicRangeMin);
                    return "done";
                    break;
                case 6 :
                    cam_->ChunkSelector.SetValue(Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_ExposureTime);
                    return "done";
                    break;
                case 7 :
                    cam_->ChunkSelector.SetValue(Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_FrameID);
                    return "done";
                    break;
                case 8 :
                    cam_->ChunkSelector.SetValue(Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_FrameTriggerCounter);
                    return "done";
                    break;
                case 9 :
                    cam_->ChunkSelector.SetValue(Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_FrameTriggerIgnoredCounter);
                    return "done";
                    break;
                case 10 :
                    cam_->ChunkSelector.SetValue(Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_Framecounter);
                    return "done";
                    break;
                case 11 :
                    cam_->ChunkSelector.SetValue(Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_FramesPerTriggerCounter);
                    return "done";
                    break;
                case 12 :
                    cam_->ChunkSelector.SetValue(Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_Gain);
                    return "done";
                    break;
                case 13 :
                    cam_->ChunkSelector.SetValue(Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_GainAll);
                    return "done";
                    break;
                case 14 :
                    cam_->ChunkSelector.SetValue(Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_Height);
                    return "done";
                    break;
                case 15 :
                    cam_->ChunkSelector.SetValue(Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_Image);
                    return "done";
                    break;
                case 16 :
                    cam_->ChunkSelector.SetValue(Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_InputStatusAtLineTrigger);
                    return "done";
                    break;
                case 17 :
                    cam_->ChunkSelector.SetValue(Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_LineStatusAll);
                    return "done";
                    break;
                case 18 :
                    cam_->ChunkSelector.SetValue(Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_LineTriggerCounter);
                    return "done";
                    break;
                case 19 :
                    cam_->ChunkSelector.SetValue(Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_LineTriggerEndToEndCounter);
                    return "done";
                    break;
                case 20 :
                    cam_->ChunkSelector.SetValue(Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_LineTriggerIgnoredCounter);
                    return "done";
                    break;
                case 21 :
                    cam_->ChunkSelector.SetValue(Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_OffsetX);
                    return "done";
                    break;
                case 22 :
                    cam_->ChunkSelector.SetValue(Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_OffsetY);
                    return "done";
                    break;
                case 23 :
                    cam_->ChunkSelector.SetValue(Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_PayloadCRC16);
                    return "done";
                    break;
                case 24 :
                    cam_->ChunkSelector.SetValue(Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_PixelFormat);
                    return "done";
                    break;
                case 25 :
                    cam_->ChunkSelector.SetValue(Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_SequenceSetIndex);
                    return "done";
                    break;
                case 26 :
                    cam_->ChunkSelector.SetValue(Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_SequencerSetActive);
                    return "done";
                    break;
                case 27 :
                    cam_->ChunkSelector.SetValue(Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_ShaftEncoderCounter);
                    return "done";
                    break;
                case 28 :
                    cam_->ChunkSelector.SetValue(Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_Stride);
                    return "done";
                    break;
                case 29 :
                    cam_->ChunkSelector.SetValue(Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_Timestamp);
                    return "done";
                    break;
                case 30 :
                    cam_->ChunkSelector.SetValue(Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_Triggerinputcounter);
                    return "done";
                    break;
                case 31 :
                    cam_->ChunkSelector.SetValue(Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_VirtLineStatusAll);
                    return "done";
                    break;
                case 32 :
                    cam_->ChunkSelector.SetValue(Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_Width);
                    return "done";
                    break;
                default :
                    return "Error: Unknown selection number";
                    break;
            }
        } catch ( const GenICam::GenericException &e ){
                RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while setting the Chunk Selector occurred:" << e.GetDescription());
                return e.GetDescription();
        }
    } else {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to setting the Chunk Selector. The connected Camera not supporting this feature");
        return "The connected Camera not supporting this feature";      // No Supported 
    }

}

template <typename CameraTraitT> 
int PylonROS2CameraImpl<CameraTraitT>::getChunkSelector() {
    if (GenApi::IsAvailable(cam_->ChunkSelector)){

        try {
            Basler_UniversalCameraParams::ChunkSelectorEnums value;
            value = cam_->ChunkSelector.GetValue();
            switch(value){
                case Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_AutoBrightnessStatus : 
                    return 1;
                    break;
                case Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_BrightPixel :
                    return 2;
                    break;
                case Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_CounterValue :
                    return 3;
                    break;
                case Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_DynamicRangeMax :
                    return 4;
                    break;
                case Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_DynamicRangeMin :
                    return 5;
                    break;
                case Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_ExposureTime :
                    return 6;
                    break;
                case Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_FrameID :
                    return 7;
                    break;
                case Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_FrameTriggerCounter :
                    return 8;
                    break;
                case Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_FrameTriggerIgnoredCounter :
                    return 9;
                    break;
                case Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_Framecounter :
                    return 10;
                    break;
                case Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_FramesPerTriggerCounter :
                    return 11;
                    break;
                case Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_Gain :
                    return 12;
                    break;
                case Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_GainAll :
                    return 14;
                    break;
                case Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_Height :
                    return 14;
                    break;
                case Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_Image :
                    return 15;
                    break;
                case Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_InputStatusAtLineTrigger :
                    return 16;
                    break;
                case Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_LineStatusAll :
                    return 17;
                    break;
                case Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_LineTriggerCounter :
                    return 18;
                    break;
                case Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_LineTriggerEndToEndCounter :
                    return 19;
                    break;
                case Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_LineTriggerIgnoredCounter :
                    return 20;
                    break;
                case Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_OffsetX :
                    return 21;
                    break;
                case Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_OffsetY :
                    return 22;
                    break;
                case Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_PayloadCRC16 :
                    return 23;
                    break;
                case Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_PixelFormat :
                    return 24;
                    break;
                case Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_SequenceSetIndex :
                    return 25;
                    break;
                case Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_SequencerSetActive :
                    return 26;
                    break;
                case Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_ShaftEncoderCounter :
                    return 27;
                    break;
                case Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_Stride :
                    return 28;
                    break;
                case Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_Timestamp :
                    return 29;
                    break;
                case Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_Triggerinputcounter :
                    return 30;
                    break;
                case Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_VirtLineStatusAll :
                    return 31;
                    break;
                case Basler_UniversalCameraParams::ChunkSelectorEnums::ChunkSelector_Width :
                    return 32;
                    break;
                default :
                    return -3;
                    break;
            }
        } catch ( const GenICam::GenericException &e ){
                RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while getting the Chunk Selector occurred:" << e.GetDescription());
                return -2;
        }
    } else {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to getting the Chunk Selector. The connected Camera not supporting this feature");
        return -1;      // No Supported 
    }

}

template <typename CameraTraitT> 
std::string PylonROS2CameraImpl<CameraTraitT>::setChunkEnable(const bool& enable) {
    if (GenApi::IsAvailable(cam_->ChunkEnable)){
        try {
            cam_->ChunkEnable.SetValue(enable);
            return "done";
        } catch ( const GenICam::GenericException &e ){
                RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while setting the Chunk Enable occurred:" << e.GetDescription());
                return e.GetDescription();
        }
    } else {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to setting the Chunk Enable. The connected Camera not supporting this feature");
        return "The connected Camera not supporting this feature";      // No Supported 
    }

}

template <typename CameraTraitT> 
int PylonROS2CameraImpl<CameraTraitT>::getChunkEnable() {
    if (GenApi::IsAvailable(cam_->ChunkEnable)){
        try {
            if (cam_->ChunkEnable.GetValue()){
                return 1;
            } else {
                return 0;
            }
        } catch ( const GenICam::GenericException &e ){
                RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while getting the Chunk Enable occurred:" << e.GetDescription());
                return -2;
        }
    } else {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to getting the Chunk Enable. The connected Camera not supporting this feature");
        return -1;      // No Supported 
    }
}

template <typename CameraTraitT> 
int PylonROS2CameraImpl<CameraTraitT>::getChunkTimestamp() {
    if (GenApi::IsAvailable(cam_->ChunkTimestamp)){
        try {
            return cam_->ChunkTimestamp.GetValue();
        } catch ( const GenICam::GenericException &e ){
                RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while getting the Chunk Timestamp occurred:" << e.GetDescription());
                return -2;
        }
    } else {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to getting the Chunk Timestamp. The connected Camera not supporting this feature");
        return -1;      // No Supported 
    }
}

template <typename CameraTraitT> 
float PylonROS2CameraImpl<CameraTraitT>::getChunkExposureTime() {
    if (GenApi::IsAvailable(cam_->ChunkExposureTime)){
        try {
            return cam_->ChunkExposureTime.GetValue();
        } catch ( const GenICam::GenericException &e ){
                RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while getting the Chunk Exposure Time occurred:" << e.GetDescription());
                return -2.0;
        }
    } else {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to getting the Chunk Exposure Time. The connected Camera not supporting this feature");
        return -1.0;      // No Supported 
    }
}

template <typename CameraTraitT> 
std::string PylonROS2CameraImpl<CameraTraitT>::setChunkExposureTime(const float& value) {
    if (GenApi::IsAvailable(cam_->ChunkExposureTime)){
        try {
            cam_->ChunkExposureTime.SetValue(value);
            return "done";
        } catch ( const GenICam::GenericException &e ){
                RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while setting the Chunk Exposure Time occurred:" << e.GetDescription());
                return e.GetDescription();
        }
    } else {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to setting the Chunk Exposure Time. The connected Camera not supporting this feature");
        return "The connected Camera not supporting this feature";      // No Supported 
    }
}

template <typename CameraTraitT> 
int PylonROS2CameraImpl<CameraTraitT>::getChunkLineStatusAll() {
    if (GenApi::IsAvailable(cam_->ChunkLineStatusAll)){
        try {
            return cam_->ChunkLineStatusAll.GetValue();
        } catch ( const GenICam::GenericException &e ){
                RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while getting the Chunk Line Status All occurred:" << e.GetDescription());
                return -2;
        }
    } else {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to getting the Chunk Line Status All. The connected Camera not supporting this feature");
        return -1;      // No Supported 
    }
}

template <typename CameraTraitT> 
int PylonROS2CameraImpl<CameraTraitT>::getChunkFramecounter() {
    if (GenApi::IsAvailable(cam_->ChunkFramecounter)){
        try {
            return cam_->ChunkFramecounter.GetValue();
        } catch ( const GenICam::GenericException &e ){
                RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while getting the Chunk Frame Counter occurred:" << e.GetDescription());
                return -2;
        }
    } else {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to getting the Chunk Frame Counter. The connected Camera not supporting this feature");
        return -1;      // No Supported 
    }
}

template <typename CameraTraitT> 
int PylonROS2CameraImpl<CameraTraitT>::getChunkCounterValue() {
    if (GenApi::IsAvailable(cam_->ChunkCounterValue)){
        try {
            return cam_->ChunkCounterValue.GetValue();
        } catch ( const GenICam::GenericException &e ){
                RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while getting the Chunk Counter Value occurred:" << e.GetDescription());
                return -2;
        }
    } else {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to getting the Chunk Counter Value. The connected Camera not supporting this feature");
        return -1;      // No Supported 
    }
}

template <typename CameraTraitT> 
std::string PylonROS2CameraImpl<CameraTraitT>::setTimerSelector(const int& selector)
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
                    RCLCPP_ERROR_STREAM(LOGGER_BASE, "Timer selector value is invalid! Please choose between 1 -> Timer 1 / 2 -> Timer 2 / 3 -> Timer 3 / 4 -> Timer 4");
                    return "Error: unknown value for timer selector";
            }
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to change the timer selector. The connected camera does not support this feature");
            return "The connected camera does not support this feature";
        }
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while setting the timer selector occurred:" << e.GetDescription());
        return e.GetDescription(); 
    }

    return "done";
}

template <typename CameraTraitT> 
std::string PylonROS2CameraImpl<CameraTraitT>::setTimerTriggerSource(const int& source)
{
    try
    {   if (GenApi::IsAvailable(cam_->TimerTriggerSource))
        {
            switch (source)
            {
                case 1:
                    cam_->TimerTriggerSource.SetValue(TimerTriggerSourceEnums::TimerTriggerSource_AcquisitionActive);
                    break;
                case 2:
                    cam_->TimerTriggerSource.SetValue(TimerTriggerSourceEnums::TimerTriggerSource_Action1);
                    break;
                case 3:
                    cam_->TimerTriggerSource.SetValue(TimerTriggerSourceEnums::TimerTriggerSource_Action2);
                    break;
                case 4:
                    cam_->TimerTriggerSource.SetValue(TimerTriggerSourceEnums::TimerTriggerSource_Counter1Active);
                    break;
                case 5:
                    cam_->TimerTriggerSource.SetValue(TimerTriggerSourceEnums::TimerTriggerSource_Counter1End);
                    break;
                case 6:
                    cam_->TimerTriggerSource.SetValue(TimerTriggerSourceEnums::TimerTriggerSource_Counter1Start);
                    break;
                case 7:
                    cam_->TimerTriggerSource.SetValue(TimerTriggerSourceEnums::TimerTriggerSource_Counter2Active);
                    break;
                case 8:
                    cam_->TimerTriggerSource.SetValue(TimerTriggerSourceEnums::TimerTriggerSource_Counter2End);
                    break;
                case 9:
                    cam_->TimerTriggerSource.SetValue(TimerTriggerSourceEnums::TimerTriggerSource_Counter2Start);
                    break;
                case 10:
                    cam_->TimerTriggerSource.SetValue(TimerTriggerSourceEnums::TimerTriggerSource_CxpTrigger0);
                    break;
                case 11:
                    cam_->TimerTriggerSource.SetValue(TimerTriggerSourceEnums::TimerTriggerSource_CxpTrigger1);
                    break;
                case 12:
                    cam_->TimerTriggerSource.SetValue(TimerTriggerSourceEnums::TimerTriggerSource_ExposureActive);
                    break;
                case 13:
                    cam_->TimerTriggerSource.SetValue(TimerTriggerSourceEnums::TimerTriggerSource_ExposureStart);
                    break;
                case 14:
                    cam_->TimerTriggerSource.SetValue(TimerTriggerSourceEnums::TimerTriggerSource_ExposureTriggerWait);
                    break;
                case 15:
                    cam_->TimerTriggerSource.SetValue(TimerTriggerSourceEnums::TimerTriggerSource_FlashWindowStart);
                    break;
                case 16:
                    cam_->TimerTriggerSource.SetValue(TimerTriggerSourceEnums::TimerTriggerSource_FrameBurstActive);
                    break;
                case 17:
                    cam_->TimerTriggerSource.SetValue(TimerTriggerSourceEnums::TimerTriggerSource_FrameBurstTriggerWait);
                    break;
                case 18:
                    cam_->TimerTriggerSource.SetValue(TimerTriggerSourceEnums::TimerTriggerSource_FrameTriggerWait);
                    break;
                case 19:
                    cam_->TimerTriggerSource.SetValue(TimerTriggerSourceEnums::TimerTriggerSource_Line1);
                    break;
                case 20:
                    cam_->TimerTriggerSource.SetValue(TimerTriggerSourceEnums::TimerTriggerSource_Line2);
                    break;
                case 21:
                    cam_->TimerTriggerSource.SetValue(TimerTriggerSourceEnums::TimerTriggerSource_Line3);
                    break;
                case 22:
                    cam_->TimerTriggerSource.SetValue(TimerTriggerSourceEnums::TimerTriggerSource_Line4);
                    break;
                case 23:
                    cam_->TimerTriggerSource.SetValue(TimerTriggerSourceEnums::TimerTriggerSource_Off);
                    break;
                case 24:
                    cam_->TimerTriggerSource.SetValue(TimerTriggerSourceEnums::TimerTriggerSource_PeriodicSignal1);
                    break;
                case 25:
                    cam_->TimerTriggerSource.SetValue(TimerTriggerSourceEnums::TimerTriggerSource_SoftwareSignal1);
                    break;
                case 26:
                    cam_->TimerTriggerSource.SetValue(TimerTriggerSourceEnums::TimerTriggerSource_SoftwareSignal2);
                    break;
                case 27:
                    cam_->TimerTriggerSource.SetValue(TimerTriggerSourceEnums::TimerTriggerSource_SoftwareSignal3);
                    break;
                case 28:
                    cam_->TimerTriggerSource.SetValue(TimerTriggerSourceEnums::TimerTriggerSource_Timer1Active);
                    break;
                case 29:
                    cam_->TimerTriggerSource.SetValue(TimerTriggerSourceEnums::TimerTriggerSource_Timer1End);
                    break;
                case 30:
                    cam_->TimerTriggerSource.SetValue(TimerTriggerSourceEnums::TimerTriggerSource_Timer2Active);
                    break;
                case 31:
                    cam_->TimerTriggerSource.SetValue(TimerTriggerSourceEnums::TimerTriggerSource_Timer2End);
                    break;
                default:
                    RCLCPP_ERROR_STREAM(LOGGER_BASE, "Timer trigger source index is invalid! Please refer to documentation: file:///opt/pylon/share/pylon/doc/C++/___basler_universal_camera_params_8h.html#a9d203212dbc78053b2a1f989dac3f095");
                    RCLCPP_INFO_STREAM(LOGGER_BASE, "For periodic signal implementation, TimerTriggerSource_PeriodicSignal1 -> source = 24 - Applies to: ace 2 GigE");
                    return "Error: unknown value for timer selector";
            }
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to change the timer trigger source. The connected camera does not support this feature");
            RCLCPP_INFO_STREAM(LOGGER_BASE, "For periodic signal implementation, TimerTriggerSource_PeriodicSignal1 -> source = 24 - Applies to: ace 2 GigE");
            return "The connected camera does not support this feature";
        }
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while setting the timer trigger source occurred:" << e.GetDescription());
        RCLCPP_INFO_STREAM(LOGGER_BASE, "For periodic signal implementation, TimerTriggerSource_PeriodicSignal1 -> source = 24 - Applies to: ace 2 GigE");
        return e.GetDescription(); 
    }

    return "done";
}

template <typename CameraTraitT> 
std::string PylonROS2CameraImpl<CameraTraitT>::setTimerDuration(const float& duration)
{
    try
    {   if (GenApi::IsAvailable(cam_->TimerDuration))
        {
            cam_->TimerDuration.SetValue(duration);
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to change the timer duration. The connected camera does not support this feature");
            return "The connected camera does not support this feature";
        }
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while setting the timer duration occurred:" << e.GetDescription());
        return e.GetDescription(); 
    }

    return "done";
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setPTPPriority(const int& value)
{
    try
    {
        if (GenApi::IsAvailable(cam_->BslPtpPriority1))
        {
            cam_->BslPtpPriority1.SetValue(value);   
            return "done";
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to set PTP priority. The connected camera does not support this feature.");
            return "The connected camera does not support this feature";
        }

    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while setting PTP priority:" << e.GetDescription());
        return e.GetDescription();
    }
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setPTPProfile(const int& value)
{
    try
    {
        if (GenApi::IsAvailable(cam_->BslPtpProfile))
        {
            switch(value)
            {
                case 1:
                    cam_->BslPtpProfile.SetValue(Basler_UniversalCameraParams::BslPtpProfileEnums::BslPtpProfile_DelayRequestResponseDefaultProfile);
                    return "done";
                    break;
                case 2:
                    cam_->BslPtpProfile.SetValue(Basler_UniversalCameraParams::BslPtpProfileEnums::BslPtpProfile_PeerToPeerDefaultProfile);
                    return "done";
                    break;
                default:
                    return "Error: Unknown ptp profile index";
                    break;
            }
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to set PTP profile. The connected camera does not support this feature.");
            return "The connected camera does not support this feature";
        }

    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while setting PTP profile:" << e.GetDescription());
        return e.GetDescription();
    }
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setPTPNetworkMode(const int& value)
{
    try
    {
        if (GenApi::IsAvailable(cam_->BslPtpNetworkMode))
        {
            switch(value)
            {
                case 1:
                    cam_->BslPtpNetworkMode.SetValue(Basler_UniversalCameraParams::BslPtpNetworkModeEnums::BslPtpNetworkMode_Hybrid);
                    return "done";
                    break;
                case 2:
                    cam_->BslPtpNetworkMode.SetValue(Basler_UniversalCameraParams::BslPtpNetworkModeEnums::BslPtpNetworkMode_Multicast);
                    return "done";
                    break;
                case 3:
                    cam_->BslPtpNetworkMode.SetValue(Basler_UniversalCameraParams::BslPtpNetworkModeEnums::BslPtpNetworkMode_Unicast);
                    return "done";
                    break;        
                default:
                    return "Error: Unknown ptp network mode";
                    break;
            }
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to set PTP network mode. The connected camera does not support this feature.");
            return "The connected camera does not support this feature";
        }

    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while setting PTP network mode:" << e.GetDescription());
        return e.GetDescription();
    }
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setPTPUCPortAddressIndex(const int& value)
{
    try
    {
        if (GenApi::IsAvailable(cam_->BslPtpUcPortAddrIndex))
        {
            cam_->BslPtpUcPortAddrIndex.SetValue(value);   
            return "done";
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to set PTP UC address index. The connected camera does not support this feature.");
            return "The connected camera does not support this feature";
        }

    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while setting PTP UC address index:" << e.GetDescription());
        return e.GetDescription();
    }
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setPTPUCPortAddress(const int& value)
{
    try
    {
        if (GenApi::IsAvailable(cam_->BslPtpUcPortAddr))
        {
            cam_->BslPtpUcPortAddr.SetValue(value);   
            return "done";
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to set PTP UC address. The connected camera does not support this feature.");
            return "The connected camera does not support this feature";
        }

    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while setting PTP UC address:" << e.GetDescription());
        return e.GetDescription();
    }
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setPeriodicSignalPeriod(const float& value)
{
    try
    {
        if (GenApi::IsAvailable(cam_->BslPeriodicSignalPeriod))
        {
            cam_->BslPeriodicSignalPeriod.SetValue(value);
            return "done";
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to set periodic signal period. The connected camera does not support this feature.");
            return "The connected camera does not support this feature";
        }

    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while setting periodic signal period:" << e.GetDescription());
        return e.GetDescription();
    }
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setPeriodicSignalDelay(const float& value)
{
    try
    {
        if (GenApi::IsAvailable(cam_->BslPeriodicSignalDelay))
        {
            cam_->BslPeriodicSignalDelay.SetValue(value);
            return "done";
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to set periodic signal delay. The connected camera does not support this feature.");
            return "The connected camera does not support this feature";
        }

    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while setting periodic signal delay:" << e.GetDescription());
        return e.GetDescription();
    }
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setSyncFreeRunTimerStartTimeLow(const int& value)
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
            RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to set sync. free run timer start time low. The connected camera does not support this feature.");
            return "The connected camera does not support this feature";
        }

    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while setting sync. free run timer start time low:" << e.GetDescription());
        return e.GetDescription();
    }
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setSyncFreeRunTimerStartTimeHigh(const int& value)
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
            RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to set sync. free run timer start time high. The connected camera does not support this feature.");
            return "The connected camera does not support this feature";
        }

    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while setting sync. free run timer start time high:" << e.GetDescription());
        return e.GetDescription();
    }
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setSyncFreeRunTimerTriggerRateAbs(const float& value)
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
            RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to set sync. free run timer trigger rate abs. The connected camera does not support this feature.");
            return "The connected camera does not support this feature";
        }

    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while setting sync. free run timer trigger rate abs:" << e.GetDescription());
        return e.GetDescription();
    }
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::enablePTPManagementProtocol(const bool& value)
{
    try
    {
        if (GenApi::IsAvailable(cam_->BslPtpManagementEnable))
        {
            cam_->BslPtpManagementEnable.SetValue(value);   
            return "done";
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to enable/disable PTP management protocol. The connected camera does not support this feature.");
            return "The connected camera does not support this feature";
        }

    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while enabling/disabling PTP management protocol:" << e.GetDescription());
        return e.GetDescription();
    }
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::enablePTPTwoStepOperation(const bool& value)
{
    try
    {
        if (GenApi::IsAvailable(cam_->BslPtpTwoStep))
        {
            cam_->BslPtpTwoStep.SetValue(value);   
            return "done";
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to enable/disable PTP two step operation. The connected camera does not support this feature.");
            return "The connected camera does not support this feature";
        }

    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while enabling/disabling PTP two step operation:" << e.GetDescription());
        return e.GetDescription();
    }
}

template <typename CameraTraitT> 
std::string PylonROS2CameraImpl<CameraTraitT>::enablePTP(const bool& value)
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
            if (GenApi::IsAvailable(cam_->PtpEnable))
            {
                cam_->PtpEnable.SetValue(value);
                return "done";
            }
            else
            {
                RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to enable/disable PTP. The connected camera does not support this feature.");
                return "The connected camera does not support this feature";
            }
        }
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while enabling/disabling PTP occurred:" << e.GetDescription());
        return e.GetDescription();
    }
}

template <typename CameraTraitT> 
std::string PylonROS2CameraImpl<CameraTraitT>::enableSyncFreeRunTimer(const bool& value)
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
            RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to enable/disable sync. free run timer. The connected camera does not support this feature.");
            return "The connected camera does not support this feature";
        }
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while enabling/disabling sync. free run timer:" << e.GetDescription());
        return e.GetDescription();
    }
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::updateSyncFreeRunTimer()
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
            RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to update sync. free run timer. The connected camera does not support this feature.");
            return "The connected camera does not support this feature";
        }
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BASE, "An exception while updating sync. free run timer:" << e.GetDescription());
        return e.GetDescription();
    }
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setActionTriggerConfiguration(const int& action_device_key, const int& action_group_key, const unsigned int& action_group_mask,
                                                                             const int& registration_mode, const int& cleanup)
{
    (void)action_device_key, (void)action_group_key, (void)action_group_mask, (void)registration_mode, (void)cleanup;
    RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to set action trigger configuration. The connected camera does not support this feature.");
    return "The connected camera does not support this feature";
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::issueActionCommand(const int& device_key, const int& group_key, const unsigned int& group_mask, const std::string& broadcast_address)
{
    (void)device_key, (void)group_key, (void)group_mask, (void)broadcast_address;
    RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to issue action command. The connected camera does not support this feature.");
    return "The connected camera does not support this feature";
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::issueScheduledActionCommand(const int& device_key, const int& group_key, const unsigned int& group_mask, const int64_t& action_time_ns_from_current_timestamp, const std::string& broadcast_address)
{
    (void)device_key, (void)group_key, (void)group_mask, (void)action_time_ns_from_current_timestamp, (void)broadcast_address;
    RCLCPP_ERROR_STREAM(LOGGER_BASE, "Error while trying to issue scheduled action command. The connected camera does not support this feature.");
    return "The connected camera does not support this feature";
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setDepthMin(const int& depth_min)
{
    RCLCPP_DEBUG(LOGGER_BASE, "Feature not available except for blaze");
    return "Feature not available except for blaze";
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setDepthMax(const int& depth_max)
{
    RCLCPP_DEBUG(LOGGER_BASE, "Feature not available except for blaze");
    return "Feature not available except for blaze";
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setTemporalFilterStrength(const int& strength)
{
    RCLCPP_DEBUG(LOGGER_BASE, "Feature not available except for blaze");
    return "Feature not available except for blaze";
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setOutlierRemovalThreshold(const int& threshold)
{
    RCLCPP_DEBUG(LOGGER_BASE, "Feature not available except for blaze");
    return "Feature not available except for blaze";
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setOutlierRemovalTolerance(const int& tolerance)
{
    RCLCPP_DEBUG(LOGGER_BASE, "Feature not available except for blaze");
    return "Feature not available except for blaze";
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setAmbiguityFilterThreshold(const int& threshold)
{
    RCLCPP_DEBUG(LOGGER_BASE, "Feature not available except for blaze");
    return "Feature not available except for blaze";
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setConfidenceThreshold(const int& threshold)
{
    RCLCPP_DEBUG(LOGGER_BASE, "Feature not available except for blaze");
    return "Feature not available except for blaze";
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setIntensityCalculation(const int& calculation)
{
    RCLCPP_DEBUG(LOGGER_BASE, "Feature not available except for blaze");
    return "Feature not available except for blaze";
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setExposureTimeSelector(const int& selector)
{
    RCLCPP_DEBUG(LOGGER_BASE, "Feature not available except for blaze");
    return "Feature not available except for blaze";
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setOperatingMode(const int& mode)
{
    RCLCPP_DEBUG(LOGGER_BASE, "Feature not available except for blaze");
    return "Feature not available except for blaze";
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setMultiCameraChannel(const int& channel)
{
    RCLCPP_DEBUG(LOGGER_BASE, "Feature not available except for blaze");
    return "Feature not available except for blaze";
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setAcquisitionFrameRate(const float& framerate)
{
    RCLCPP_DEBUG(LOGGER_BASE, "Feature not available except for blaze");
    return "Feature not available except for blaze";
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::setScan3dCalibrationOffset(const float& offset)
{
    RCLCPP_DEBUG(LOGGER_BASE, "Feature not available except for blaze");
    return "Feature not available except for blaze";
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::enableSpatialFilter(const bool& enable)
{
    RCLCPP_DEBUG(LOGGER_BASE, "Feature not available except for blaze");
    return "Feature not available except for blaze";
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::enableTemporalFilter(const bool& enable)
{
    RCLCPP_DEBUG(LOGGER_BASE, "Feature not available except for blaze");
    return "Feature not available except for blaze";
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::enableOutlierRemoval(const bool& enable)
{
    RCLCPP_DEBUG(LOGGER_BASE, "Feature not available except for blaze");
    return "Feature not available except for blaze";
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::enableAmbiguityFilter(const bool& enable)
{
    RCLCPP_DEBUG(LOGGER_BASE, "Feature not available except for blaze");
    return "Feature not available except for blaze";
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::enableThermalDriftCorrection(const bool& enable)
{
    RCLCPP_DEBUG(LOGGER_BASE, "Feature not available except for blaze");
    return "Feature not available except for blaze";
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::enableDistortionCorrection(const bool& enable)
{
    RCLCPP_DEBUG(LOGGER_BASE, "Feature not available except for blaze");
    return "Feature not available except for blaze";
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::enableAcquisitionFrameRate(const bool& enable)
{
    RCLCPP_DEBUG(LOGGER_BASE, "Feature not available except for blaze");
    return "Feature not available except for blaze";
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::enableHDRMode(const bool& enable)
{
    RCLCPP_DEBUG(LOGGER_BASE, "Feature not available except for blaze");
    return "Feature not available except for blaze";
}

template <typename CameraTraitT>
std::string PylonROS2CameraImpl<CameraTraitT>::enableFastMode(const bool& enable)
{
    RCLCPP_DEBUG(LOGGER_BASE, "Feature not available except for blaze");
    return "Feature not available except for blaze";
}

}  // namespace pylon_ros2_camera
