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

#include "pylon_ros2_camera_parameter.hpp"
#include <sensor_msgs/image_encodings.hpp>


namespace pylon_ros2_camera
{

namespace
{
    static const rclcpp::Logger LOGGER = rclcpp::get_logger("basler.pylon.ros2.pylon_ros2_camera_parameter");
}

PylonROS2CameraParameter::PylonROS2CameraParameter() :
    binning_x_(1),
    binning_y_(1),
    binning_x_given_(false),
    binning_y_given_(false),
    downsampling_factor_exp_search_(1),
    exposure_(10000.0),
    exposure_given_(false),
    gain_(0.5),
    gain_given_(false),
    gamma_(1.0),
    gamma_given_(false),
    brightness_(100),
    brightness_given_(false),
    brightness_continuous_(false),
    exposure_auto_(true),
    gain_auto_(true),
    exposure_search_timeout_(5.),
    auto_exp_upper_lim_(0.0),
    mtu_size_(3000),
    enable_status_publisher_(false),
    enable_current_params_publisher_(false),
    startup_user_set_(""),
    inter_pkg_delay_(1000),
    shutter_mode_(SM_DEFAULT),
    auto_flash_(false), 
    grab_timeout_(500),
    trigger_timeout_(5000),
    white_balance_auto_(0),
    white_balance_ratio_red_(1.0),
    white_balance_ratio_green_(1.0),
    white_balance_ratio_blue_(1.0),
    grab_strategy_(0),
    camera_frame_("pylon_camera"),
    device_user_id_(""),
    frame_rate_(5.0),
    camera_info_url_(""),
    image_encoding_("")
{}

PylonROS2CameraParameter::~PylonROS2CameraParameter()
{}

void PylonROS2CameraParameter::readFromRosParameterServer(rclcpp::Node& nh)
{
    // if the parameter does not exist, the variable stays unchanged
    nh.get_parameter("camera_frame", this->camera_frame_);
    nh.get_parameter("device_user_id", this->device_user_id_);
    nh.get_parameter("frame_rate", this->frame_rate_);
    nh.get_parameter("camera_info_url", this->camera_info_url_);

    this->binning_x_given_ = nh.has_parameter("binning_x");
    if (this->binning_x_given_)
    {
        int binning_x;
        nh.get_parameter("binning_x", binning_x);
        RCLCPP_DEBUG_STREAM(LOGGER, "binning x is given and has value " << binning_x);
        if (binning_x > 32 || binning_x < 0)
        {
            RCLCPP_WARN_STREAM(LOGGER, "Desired horizontal binning_x factor not in valid "
                << "range! Binning x = " << binning_x << ". Will reset it to "
                << "default value (1)");
            this->binning_x_given_ = false;
        }
        else
        {
            this->binning_x_ = static_cast<size_t>(binning_x);
        }
    }

    this->binning_y_given_ = nh.has_parameter("binning_y");
    if (this->binning_y_given_)
    {
        int binning_y;
        nh.get_parameter("binning_y", binning_y);
        RCLCPP_DEBUG_STREAM(LOGGER, "binning y is given and has value " << binning_y);
        if (binning_y > 32 || binning_y < 0)
        {
            RCLCPP_WARN_STREAM(LOGGER, "Desired vertical binning_y factor not in valid "
                << "range! Binning y = " << binning_y << ". Will reset it to "
                << "default value (1)");
            this->binning_y_given_ = false;
        }
        else
        {
            this->binning_y_ = static_cast<size_t>(binning_y);
        }
    }

    try
    {
        nh.declare_parameter<int>("downsampling_factor_exposure_search", 20);
    }
    catch(const rclcpp::exceptions::ParameterAlreadyDeclaredException& e)
    {
        RCLCPP_DEBUG_STREAM(LOGGER, "PylonROS2CameraParameter::readFromRosParameterServer: " << e.what());
    }
    nh.get_parameter("downsampling_factor_exposure_search", this->downsampling_factor_exp_search_);

    if (nh.has_parameter("image_encoding"))
    {
        std::string encoding;
        nh.get_parameter("image_encoding", encoding);
        if (!encoding.empty() &&
            !sensor_msgs::image_encodings::isMono(encoding) &&
            !sensor_msgs::image_encodings::isColor(encoding) &&
            !sensor_msgs::image_encodings::isBayer(encoding) &&
            encoding != sensor_msgs::image_encodings::YUV422)
        {
            RCLCPP_WARN_STREAM(LOGGER, "Desired image encoding parameter: '" << encoding
                << "' is not part of the 'sensor_msgs/image_encodings.h' list!"
                << " Will not set encoding");
            encoding = std::string("");
        }
        this->image_encoding_ = encoding;
    }

    // ##########################
    //  image intensity settings
    // ##########################

    // > 0: Exposure time in microseconds
    this->exposure_given_ = nh.has_parameter("exposure");
    if (this->exposure_given_)
    {
        nh.get_parameter("exposure", this->exposure_);
        RCLCPP_DEBUG_STREAM(LOGGER, "exposure is given and has value " << this->exposure_);
    }

    this->gain_given_ = nh.has_parameter("gain");
    if (this->gain_given_)
    {
        nh.get_parameter("gain", this->gain_);
        RCLCPP_DEBUG_STREAM(LOGGER, "gain is given and has value " << this->gain_);
    }

    this->gamma_given_ = nh.has_parameter("gamma");
    if (this->gamma_given_)
    {
        nh.get_parameter("gamma", this->gamma_);
        RCLCPP_DEBUG_STREAM(LOGGER, "gamma is given and has value " << this->gamma_);
    }

    this->brightness_given_ = nh.has_parameter("brightness");
    if (this->brightness_given_)
    {
        nh.get_parameter("brightness", this->brightness_);
        RCLCPP_DEBUG_STREAM(LOGGER, "brightness is given and has value " << this->brightness_);
        if (this->gain_given_ && this->exposure_given_)
        {
            RCLCPP_WARN_STREAM(LOGGER, "Gain ('gain') and Exposure Time ('exposure') "
                << "are given as startup ros-parameter and hence assumed to be "
                << "fix! The desired brightness (" << this->brightness_ << ") can't "
                << "be reached! Will ignore the brightness by only "
                << "setting gain and exposure . . .");
            this->brightness_given_ = false;
        }
        else
        {
            if (nh.has_parameter("brightness_continuous"))
            {
                nh.get_parameter("brightness_continuous", this->brightness_continuous_);
                RCLCPP_DEBUG_STREAM(LOGGER, "brightness is continuous");
            }
            if (nh.has_parameter("exposure_auto"))
            {
                nh.get_parameter("exposure_auto", this->exposure_auto_);
                RCLCPP_DEBUG_STREAM(LOGGER, "exposure is set to auto");
            }
            if (nh.has_parameter("gain_auto") )
            {
                nh.get_parameter("gain_auto", this->gain_auto_);
                RCLCPP_DEBUG_STREAM(LOGGER, "gain is set to auto");
            }
        }
    }

    // ##########################

    try
    {
        nh.declare_parameter<double>("exposure_search_timeout", 5.);
    }
    catch(const rclcpp::exceptions::ParameterAlreadyDeclaredException& e)
    {
        RCLCPP_DEBUG_STREAM(LOGGER, "PylonROS2CameraParameter::readFromRosParameterServer: " << e.what());
    }
    nh.get_parameter("exposure_search_timeout", this->exposure_search_timeout_);

    try
    {
        nh.declare_parameter<double>("auto_exposure_upper_limit", 10000000.);
    }
    catch(const rclcpp::exceptions::ParameterAlreadyDeclaredException& e)
    {
        RCLCPP_DEBUG_STREAM(LOGGER, "PylonROS2CameraParameter::readFromRosParameterServer: " << e.what());
    }
    nh.get_parameter("auto_exposure_upper_limit", this->auto_exp_upper_lim_);

    // if the parameter does not exist, the variable stays unchanged
    nh.get_parameter("gige/mtu_size", this->mtu_size_);

    // if the parameter does not exist, the variable stays unchanged
    nh.get_parameter("enable_status_publisher", this->enable_status_publisher_);

    // if the parameter does not exist, the variable stays unchanged
    nh.get_parameter("enable_current_params_publisher", this->enable_current_params_publisher_);

    // if the parameter does not exist, the variable stays unchanged
    nh.get_parameter("gige/inter_pkg_delay", this->inter_pkg_delay_);

    try
    {
        nh.declare_parameter<std::string>("shutter_mode", "");
    }
    catch(const rclcpp::exceptions::ParameterAlreadyDeclaredException& e)
    {
        RCLCPP_DEBUG_STREAM(LOGGER, "PylonROS2CameraParameter::readFromRosParameterServer: " << e.what());
    }
    std::string shutter_param_string;
    nh.get_parameter("shutter_mode", shutter_param_string);
    if (shutter_param_string == "rolling")
    {
        this->shutter_mode_ = SM_ROLLING;
    }
    else if (shutter_param_string == "global")
    {
        this->shutter_mode_ = SM_GLOBAL;
    }
    else if (shutter_param_string == "global_reset")
    {
        this->shutter_mode_ = SM_GLOBAL_RESET_RELEASE;
    }
    else
    {
        this->shutter_mode_ = SM_DEFAULT;
    }

    // if the parameter does not exist, the variable stays unchanged
    nh.get_parameter("startup_user_set", this->startup_user_set_);

    // if the parameter does not exist, the variable stays unchanged
    nh.get_parameter("grab_timeout", this->grab_timeout_);

    // if the parameter does not exist, the variable stays unchanged
    nh.get_parameter("trigger_timeout", this->trigger_timeout_);

    // if the parameter does not exist, the variable stays unchanged
    nh.get_parameter("white_balance_auto", this->white_balance_auto_);

    // if the parameter does not exist, the variable stays unchanged
    nh.get_parameter("white_balance_ratio_red", this->white_balance_ratio_red_);

    // if the parameter does not exist, the variable stays unchanged
    nh.get_parameter("white_balance_ratio_green", this->white_balance_ratio_green_);

    // if the parameter does not exist, the variable stays unchanged
    nh.get_parameter("white_balance_ratio_blue", this->white_balance_ratio_blue_);

    // if the parameter does not exist, the variable stays unchanged
    nh.get_parameter("grab_strategy", this->grab_strategy_);

    try
    {
        nh.declare_parameter<bool>("auto_flash", false);
    }
    catch(const rclcpp::exceptions::ParameterAlreadyDeclaredException& e)
    {
        RCLCPP_DEBUG_STREAM(LOGGER, "PylonROS2CameraParameter::readFromRosParameterServer: " << e.what());
    }
    nh.get_parameter("auto_flash", this->auto_flash_);

    try
    {
        nh.declare_parameter<bool>("auto_flash_line_2", true);
    }
    catch(const rclcpp::exceptions::ParameterAlreadyDeclaredException& e)
    {
        RCLCPP_DEBUG_STREAM(LOGGER, "PylonROS2CameraParameter::readFromRosParameterServer: " << e.what());
    }
    nh.get_parameter("auto_flash_line_2", this->auto_flash_line_2_);

    try
    {
        nh.declare_parameter<bool>("auto_flash_line_3", true);
    }
    catch(const rclcpp::exceptions::ParameterAlreadyDeclaredException& e)
    {
        RCLCPP_DEBUG_STREAM(LOGGER, "PylonROS2CameraParameter::readFromRosParameterServer: " << e.what());
    }
    nh.get_parameter("auto_flash_line_3", this->auto_flash_line_3_);

    RCLCPP_WARN(LOGGER, "Autoflash: %i, line2: %i , line3: %i ", this->auto_flash_, this->auto_flash_line_2_, this->auto_flash_line_3_);

    this->validateParameterSet(nh);
}

void PylonROS2CameraParameter::setDeviceUserId(rclcpp::Node& nh, const std::string& device_user_id)
{
    if (!nh.has_parameter("device_user_id"))
    {
        // parameter does not exist, it needs to be declared first
        try
        {
            nh.declare_parameter<std::string>("device_user_id", "");
        }
        catch(const std::exception& e)
        {
            RCLCPP_WARN_STREAM(LOGGER, "PylonROS2CameraParameter::setDeviceUserId: " << e.what());
        }
    }

    try
    {
        this->device_user_id_ = device_user_id;
        nh.set_parameter(rclcpp::Parameter("device_user_id", this->device_user_id_));
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Error saving the new device user id as ROS Param: " << e.what());
    }
}

void PylonROS2CameraParameter::validateParameterSet(rclcpp::Node& nh)
{
    if (!this->device_user_id_.empty())
    {
        RCLCPP_INFO_STREAM(LOGGER, "Trying to open the following camera: " << this->device_user_id_.c_str());
    }
    else
    {
        RCLCPP_INFO_STREAM(LOGGER, "No Device User ID set -> Will open the camera device " << "found first");
    }

    if (this->frame_rate_ < 0 && this->frame_rate_ != -1)
    {
        RCLCPP_WARN_STREAM(LOGGER, "Unexpected frame rate (" << this->frame_rate_ << "). Will "
                << "reset it to default value which is 5 Hz");
        this->setFrameRate(nh, 5.0);
    }

    if (this->exposure_given_ && ( this->exposure_ <= 0.0 || this->exposure_ > 1e7 ))
    {
        RCLCPP_WARN_STREAM(LOGGER, "Desired exposure measured in microseconds not in "
                << "valid range! Exposure time = " << this->exposure_ << ". Will "
                << "reset it to default value!");
        this->exposure_given_ = false;
    }

    if (this->gain_given_ && ( this->gain_ < 0.0 || this->gain_ > 1.0 ))
    {
        RCLCPP_WARN_STREAM(LOGGER, "Desired gain (in percent) not in allowed range! "
                << "Gain = " << this->gain_ << ". Will reset it to default value!");
        this->gain_given_ = false;
    }

    if (this->brightness_given_ && ( this->brightness_ < 0.0 || this->brightness_ > 255 ))
    {
        RCLCPP_WARN_STREAM(LOGGER, "Desired brightness not in allowed range [0 - 255]! "
               << "Brightness = " << this->brightness_ << ". Will reset it to "
               << "default value!");
        this->brightness_given_ = false;
    }

    if (this->exposure_search_timeout_ < 5.)
    {
        RCLCPP_WARN_STREAM(LOGGER, "Low timeout for exposure search detected! Exposure " << "search may fail.");
    }
}

const std::string& PylonROS2CameraParameter::deviceUserID() const
{
    return this->device_user_id_;
}

std::string PylonROS2CameraParameter::shutterModeString() const
{
    if ( this->shutter_mode_ == SM_ROLLING )
    {
        return "rolling";
    }
    else if ( this->shutter_mode_ == SM_GLOBAL )
    {
        return "global";
    }
    else if ( this->shutter_mode_ == SM_GLOBAL_RESET_RELEASE )
    {
        return "global_reset";
    }
    else
    {
        return "default_shutter_mode";
    }
}

const std::string& PylonROS2CameraParameter::imageEncoding() const
{
    return this->image_encoding_;
}

bool PylonROS2CameraParameter::setimageEncodingParam(rclcpp::Node& nh, const std::string& format) 
{
    if (!nh.has_parameter("image_encoding"))
    {
        // parameter does not exist, it needs to be declared first
        try
        {
            nh.declare_parameter<std::string>("image_encoding", "");
        }
        catch(const std::exception& e)
        {
            RCLCPP_WARN_STREAM(LOGGER, "PylonROS2CameraParameter::setimageEncodingParam: " << e.what());
        }
    }

    try
    {
        this->image_encoding_ = format;
        nh.set_parameter(rclcpp::Parameter("image_encoding", this->image_encoding_));
        return true;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Error saving the new image encoding as ROS Param: " << e.what());
        return false;
    }
}

const std::string& PylonROS2CameraParameter::cameraFrame() const
{
    return this->camera_frame_;
}

const double& PylonROS2CameraParameter::frameRate() const
{
    return this->frame_rate_;
}

void PylonROS2CameraParameter::setFrameRate(rclcpp::Node& nh, const double& frame_rate)
{
    if (!nh.has_parameter("frame_rate"))
    {
        // parameter does not exist, it needs to be declared first
        try
        {
            nh.declare_parameter<double>("frame_rate", 5.0);
        }
        catch(const std::exception& e)
        {
            RCLCPP_WARN_STREAM(LOGGER, "PylonROS2CameraParameter::setFrameRate: " << e.what());
        }
    }

    try
    {
        this->frame_rate_ = frame_rate;
        nh.set_parameter(rclcpp::Parameter("frame_rate", this->frame_rate_));
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Error saving the new frame rate as ROS Param: " << e.what());
    }
}

const std::string& PylonROS2CameraParameter::cameraInfoURL() const
{
    return this->camera_info_url_;
}

void PylonROS2CameraParameter::setCameraInfoURL(rclcpp::Node& nh, const std::string& camera_info_url)
{
    if (!nh.has_parameter("camera_info_url"))
    {
        // parameter does not exist, it needs to be declared first
        try
        {
            nh.declare_parameter<std::string>("camera_info_url", "");
        }
        catch(const std::exception& e)
        {
            RCLCPP_WARN_STREAM(LOGGER, "PylonROS2CameraParameter::setCameraInfoURL: " << e.what());
        }
    }

    try
    {
        this->camera_info_url_ = camera_info_url;
        nh.set_parameter(rclcpp::Parameter("camera_info_url", this->camera_info_url_));
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Error saving the new camera info url as ROS Param: " << e.what());
    }
}

}  // namespace pylon_ros2_camera
