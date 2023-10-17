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

#include "internal/pylon_ros2_camera_impl.hpp"

#include <string>
#include <vector>

namespace pylon_ros2_camera
{

namespace
{
    static const rclcpp::Logger LOGGER = rclcpp::get_logger("basler.pylon.ros2.pylon_ros2_camera");
}

enum PYLON_CAM_TYPE
{
    GIGE = 1,
    USB = 2,
    DART = 3,
    GIGE2 = 4,
    BLAZE = 5,
    UNKNOWN = -1,
};

PylonROS2Camera::PylonROS2Camera()
    : binary_exp_search_(nullptr)
    , device_user_id_("")
    , img_rows_(0)
    , img_cols_(0)
    , img_size_byte_(0)
    , grab_timeout_(-1.0)
    , is_ready_(false)
    , is_binary_exposure_search_running_(false)
    , max_brightness_tolerance_(2.5)
{}

PYLON_CAM_TYPE detectPylonCamType(const Pylon::CDeviceInfo& device_info)
{
    Pylon::String_t device_class;

    if (device_info.IsDeviceClassAvailable())
    {
        device_class = device_info.GetDeviceClass();
        
        if (device_class == "BaslerGigE")
        {
            if (device_info.IsModelNameAvailable())
            {
                std::string model_name(device_info.GetModelName());

                if (model_name.compare(0, 3, "acA") == 0)
                {
                    return GIGE;
                }
                else if (model_name.compare(0, 3, "a2A") == 0)
                {
                    return GIGE2;
                } 
                else 
                {
                    RCLCPP_ERROR_STREAM(LOGGER, "Found 'BaslerGigE' camera device type, "
                        << "but it is neither a ace, nor a ace2 camera device. "
                        << "Other camera types are not supported by this driver for now!");

                    return UNKNOWN;
                }
            }
        }
        else if (device_class == "BaslerUsb")
        {
            if (device_info.IsModelNameAvailable())
            {
                std::string model_name(device_info.GetModelName());

                if ( model_name.compare(0, 3, "acA") == 0 )
                {
                    return USB;
                }
                else if (model_name.compare(0, 3, "a2A") == 0)
                {
                    return USB;
                }
                else if (model_name.compare(0, 3, "daA") == 0)
                {
                    return DART;
                }
                else
                {
                    RCLCPP_ERROR_STREAM(LOGGER, "Found 'BaslerUsb' camera device type, "
                        << "but it is neither a Dart, nor a USB camera device. "
                        << "Other camera types are not supported by this driver for now!");
                    
                    return UNKNOWN;
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(LOGGER, "Error while detecting the pylon camera type from its Model Name. "
                    << "The connected camera has no Model Name available!");
                
                return UNKNOWN;
            }
        }
        else if (device_class == "BaslerGTC/Basler/GenTL_Producer_for_Basler_blaze_101_cameras")
        {
            if (device_info.IsModelNameAvailable())
            {
                std::string model_name(device_info.GetModelName());
                //RCLCPP_INFO_STREAM(LOGGER, "blaze model name: " << model_name);
            }

            return BLAZE;
        }
        else
        {
            RCLCPP_ERROR_STREAM(LOGGER, "The detected camera type is: " << device_class << ". "
                << "Only 'BaslerUsb' and 'BaslerGigE' types are supported by this driver for now!");

            return UNKNOWN;
        }
    }
    else
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Error while detecting the pylon camera type from "
                << "its DeviceClass: Camera has no DeviceClass available!");

        return UNKNOWN;
    }

    return UNKNOWN;
}

PylonROS2Camera* createFromDevice(PYLON_CAM_TYPE cam_type, Pylon::IPylonDevice* device)
{
    switch (cam_type)
    {
        case GIGE:
            return new PylonROS2GigECamera(device);
        case GIGE2 :
            return new PylonROS2GigEAce2Camera(device);
        case USB:
            return new PylonROS2USBCamera(device);
        case DART:
            return new PylonROS2DARTCamera(device);
        case BLAZE:
            return new PylonROS2BlazeCamera(device);
        case UNKNOWN:
        default:
            return nullptr;
    }
}

PylonROS2Camera* PylonROS2Camera::create(const std::string& device_user_id_to_open)
{
    try
    {
        // Before using any pylon methods, the pylon runtime must be initialized.
        Pylon::PylonInitialize();
        Pylon::CTlFactory& tl_factory = Pylon::CTlFactory::GetInstance();

        Pylon::DeviceInfoList_t device_list;
        
        // EnumerateDevices() returns the number of devices found
        if (0 == tl_factory.EnumerateDevices(device_list))
        {
            Pylon::PylonTerminate();
            RCLCPP_ERROR_ONCE(LOGGER, "No available camera device");
            return nullptr;
        }
        else
        {
            Pylon::DeviceInfoList_t::const_iterator it;
            if (device_user_id_to_open.empty())
            {
                for (it = device_list.begin(); it != device_list.end(); ++it)
                {
                    RCLCPP_INFO_STREAM(LOGGER, "Found camera device!"
                                            << " Device Model: " << it->GetModelName()
                                            << " with Device User Id: " << it->GetUserDefinedName());
                    
                    PYLON_CAM_TYPE cam_type = detectPylonCamType(*it);
                    if (cam_type != UNKNOWN)
                    {
                        //RCLCPP_ERROR_STREAM(LOGGER, "CAM TYPE: " << cam_type);
                        PylonROS2Camera* new_cam_ptr = createFromDevice(cam_type, tl_factory.CreateDevice(*it));
                        new_cam_ptr->device_user_id_ = it->GetUserDefinedName();
                        
                        return new_cam_ptr;
                    }
                }

                Pylon::PylonTerminate();
                RCLCPP_ERROR_ONCE(LOGGER, "No available compatible camera device");
                
                return nullptr;
            }

            bool found_desired_device = false;
            for ( it = device_list.begin(); it != device_list.end(); ++it )
            {
                std::string device_user_id_found(it->GetUserDefinedName());
                if ( (0 == device_user_id_to_open.compare(device_user_id_found)) ||
                     (device_user_id_to_open.length() < device_user_id_found.length() &&
                     (0 == device_user_id_found.compare(device_user_id_found.length() -
                                                         device_user_id_to_open.length(),
                                                         device_user_id_to_open.length(),
                                                         device_user_id_to_open) )
                     )
                   )
                {
                    found_desired_device = true;
                    break;
                }
            }

            if (found_desired_device)
            {
                RCLCPP_INFO_STREAM(LOGGER, "Found camera device!"
                                            << " Device Model: " << it->GetModelName()
                                            << " with Device User Id: " << device_user_id_to_open);

                PYLON_CAM_TYPE cam_type = detectPylonCamType(*it);
                return createFromDevice(cam_type, tl_factory.CreateDevice(*it));
            }
            else
            {
                RCLCPP_ERROR_STREAM(LOGGER, "Couldn't find the camera that matches the "
                    << "specified Device User ID: " << device_user_id_to_open << "! "
                    << "Either the ID is wrong or the camera device is not connected (yet)");
                
                return nullptr;
            }
        }
    }
    catch (GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER, "An exception occurred while opening the specified camera device "
            << "with Device User ID: " << device_user_id_to_open << ": \r\n"
            << e.GetDescription());

        return nullptr;
    }
}

const std::string& PylonROS2Camera::deviceUserID() const
{
    return device_user_id_;
}

const size_t& PylonROS2Camera::imageRows() const
{
    return img_rows_;
}

const size_t& PylonROS2Camera::imageCols() const
{
    return img_cols_;
}

const size_t& PylonROS2Camera::imageSize() const
{
    return img_size_byte_;
}

const float& PylonROS2Camera::maxBrightnessTolerance() const
{
    return max_brightness_tolerance_;
}

const bool& PylonROS2Camera::isReady() const
{
    return is_ready_;
}

std::size_t PylonROS2Camera::numUserOutputs() const
{
    return user_output_selector_enums_.size();
}

const std::vector<float>& PylonROS2Camera::sequencerExposureTimes() const
{
    return seq_exp_times_;
}

const bool& PylonROS2Camera::isBinaryExposureSearchRunning() const
{
    return is_binary_exposure_search_running_;
}

PylonROS2Camera::~PylonROS2Camera()
{
    // Releases all Pylon resources.
    Pylon::PylonTerminate();
    if (binary_exp_search_)
    {
        delete binary_exp_search_;
        binary_exp_search_ = nullptr;
    }
}

}  // namespace pylon_ros2_camera
