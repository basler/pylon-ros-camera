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
    downsampling_factor_exposure_search_(1),
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
    auto_exposure_upper_limit_(0.0),
    mtu_size_(3000),
    enable_status_publisher_(false),
    enable_current_params_publisher_(false),
    startup_user_set_(""),
    inter_pkg_delay_(1000),
    frame_transmission_delay_(0),
    shutter_mode_(SM_DEFAULT),
    auto_flash_(false),
    auto_flash_line_2_(true),
    auto_flash_line_3_(true),
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
{
    // information logging severity mode
    //rcutils_ret_t __attribute__((unused)) res = rcutils_logging_set_logger_level(LOGGER.get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
    //RCUTILS_LOG_SEVERITY_DEBUG
    //RCUTILS_LOG_SEVERITY_INFO
    //RCUTILS_LOG_SEVERITY_WARN
    //RCUTILS_LOG_SEVERITY_ERROR
    //RCUTILS_LOG_SEVERITY_FATAL
}

PylonROS2CameraParameter::~PylonROS2CameraParameter()
{}

void PylonROS2CameraParameter::readFromRosParameterServer(rclcpp::Node& nh)
{
    RCLCPP_DEBUG(LOGGER, "-> Reading parameters from ROS2 server");

    // camera frame
    RCLCPP_DEBUG(LOGGER, "---> camera_frame");
    
    if (!nh.has_parameter("camera_frame"))
    {
        nh.declare_parameter<std::string>("camera_frame", "pylon_camera");
    }

    nh.get_parameter("camera_frame", this->camera_frame_);

    // device user id
    RCLCPP_DEBUG(LOGGER, "---> device_user_id");
    
    if (!nh.has_parameter("device_user_id"))
    {
        nh.declare_parameter<std::string>("device_user_id", "");
    }
    
    nh.get_parameter("device_user_id", this->device_user_id_);

    // frame rate
    RCLCPP_DEBUG(LOGGER, "---> frame_rate");
    
    if (!nh.has_parameter("frame_rate"))
    {
        nh.declare_parameter<double>("frame_rate", 5.0);
    }

    nh.get_parameter("frame_rate", this->frame_rate_);
    
    // camera info url
    RCLCPP_DEBUG(LOGGER, "---> camera_info_url");
    
    if (!nh.has_parameter("camera_info_url"))
    {
        nh.declare_parameter<std::string>("camera_info_url", "");
    }
    
    nh.get_parameter("camera_info_url", this->camera_info_url_);

    // binning x
    RCLCPP_DEBUG(LOGGER, "---> binning_x");

    int binning_x;
    if (nh.has_parameter("binning_x"))
    {
        nh.get_parameter("binning_x", binning_x);
        //RCLCPP_DEBUG_STREAM(LOGGER, "binning_x is already specified and has value " << binning_x);
    }
    else
    {
        nh.declare_parameter<int>("binning_x", 1);
        binning_x = 1;
    }

    this->binning_x_given_ = true;
    if (binning_x > 32 || binning_x < 0)
    {
        RCLCPP_WARN_STREAM(LOGGER, "Specified binning_x factor not in valid "
            << "range! Binning x = " << binning_x << ". Will reset it to "
            << "default value (1)");
        binning_x_ = 1;
        this->binning_x_given_ = false;
    }

    this->binning_x_ = static_cast<size_t>(binning_x);

    // binning y
    RCLCPP_DEBUG(LOGGER, "---> binning_y");

    int binning_y;
    if (nh.has_parameter("binning_y"))
    {
        nh.get_parameter("binning_y", binning_y);
        //RCLCPP_DEBUG_STREAM(LOGGER, "binning_y is already specified and has value " << binning_y);
    }
    else
    {
        nh.declare_parameter<int>("binning_y", 1);
        binning_y = 1;
    }

    this->binning_y_given_ = true;
    if (binning_y > 32 || binning_y < 0)
    {
        RCLCPP_WARN_STREAM(LOGGER, "Specified binning_y factor not in valid "
            << "range! Binning x = " << binning_y << ". Will reset it to "
            << "default value (1)");
        binning_y_ = 1;
        this->binning_y_given_ = false;
    }

    this->binning_y_ = static_cast<size_t>(binning_y);

    // downsampling_factor_exposure_search
    RCLCPP_DEBUG(LOGGER, "---> downsampling_factor_exposure_search");
    
    if (!nh.has_parameter("downsampling_factor_exposure_search"))
    {
        nh.declare_parameter<int>("downsampling_factor_exposure_search", 20);
    }
    
    nh.get_parameter("downsampling_factor_exposure_search", this->downsampling_factor_exposure_search_);

    // image encoding
    RCLCPP_DEBUG(LOGGER, "---> image_encoding");
    
    if (!nh.has_parameter("image_encoding"))
    {
        nh.declare_parameter<std::string>("image_encoding", "");
    }

    std::string encoding;
    nh.get_parameter("image_encoding", encoding);

    if (!encoding.empty() &&
        !sensor_msgs::image_encodings::isMono(encoding) &&
        !sensor_msgs::image_encodings::isColor(encoding) &&
        !sensor_msgs::image_encodings::isBayer(encoding) &&
        encoding != sensor_msgs::image_encodings::YUV422)
    {
        RCLCPP_WARN_STREAM(LOGGER, "Specified image encoding parameter: '" << encoding
            << "' is not part of the 'sensor_msgs/image_encodings.h' list!"
            << " Will not set encoding");
        encoding = std::string("");
    }

    this->image_encoding_ = encoding;

    // ##########################
    //  image intensity settings
    // ##########################

    // exposure
    // > 0: Exposure time in microseconds
    RCLCPP_DEBUG(LOGGER, "---> exposure");
    
    this->exposure_given_ = nh.has_parameter("exposure");
    if (!this->exposure_given_)
    {
        nh.declare_parameter<double>("exposure", 10000.0);
    }
    
    nh.get_parameter("exposure", this->exposure_);

    // gain
    RCLCPP_DEBUG(LOGGER, "---> gain");
    
    this->gain_given_ = nh.has_parameter("gain");
    if (!this->gain_given_)
    {
        nh.declare_parameter<double>("gain", 0.5);
    }

    nh.get_parameter("gain", this->gain_);

    // gamma
    RCLCPP_DEBUG(LOGGER, "---> gamma");
    
    this->gamma_given_ = nh.has_parameter("gamma");
    if (!this->gamma_given_)
    {
        nh.declare_parameter<double>("gamma", 1.0);
    }

    nh.get_parameter("gamma", this->gamma_);

    // brightness_continuous
    RCLCPP_DEBUG(LOGGER, "---> brightness_continuous");

    if (!nh.has_parameter("brightness_continuous"))
    {
        nh.declare_parameter<bool>("brightness_continuous", false);
    }
    
    nh.get_parameter("brightness_continuous", this->brightness_continuous_);

    // exposure_auto
    RCLCPP_DEBUG(LOGGER, "---> exposure_auto");

    if (!nh.has_parameter("exposure_auto"))
    {
        nh.declare_parameter<bool>("exposure_auto", true);
    }
    
    nh.get_parameter("exposure_auto", this->exposure_auto_);

    // gain_auto
    RCLCPP_DEBUG(LOGGER, "---> gain_auto");

    if (!nh.has_parameter("gain_auto"))
    {
        nh.declare_parameter<bool>("gain_auto", true);
    }
    
    nh.get_parameter("gain_auto", this->gain_auto_);

    // brightness
    RCLCPP_DEBUG(LOGGER, "---> brightness");

    this->brightness_given_ = nh.has_parameter("brightness");

    if (!this->brightness_given_)
    {
        nh.declare_parameter<int>("brightness", 100);
    }
    
    nh.get_parameter("brightness", this->brightness_);

    if (this->gain_given_ && this->exposure_given_)
    {
        RCLCPP_WARN_STREAM(LOGGER, "Gain and exposure are specified as startup parameters and hence assumed to be fixed! "
            << "The specified brightness (" << this->brightness_ << ") can't be reached! "
            << "Brightness is going to ignored by only setting gain and exposure.");
        this->brightness_given_ = false;
    }
    else
    {
        this->brightness_continuous_ = true;
        nh.set_parameter(rclcpp::Parameter("brightness_continuous", this->brightness_continuous_));

        this->exposure_auto_ = true;
        nh.set_parameter(rclcpp::Parameter("exposure_auto", this->exposure_auto_));

        this->gain_auto_ = true;
        nh.set_parameter(rclcpp::Parameter("gain_auto", this->gain_auto_));
    }

    // ##########################

    // exposure_search_timeout
    RCLCPP_DEBUG(LOGGER, "---> exposure_search_timeout");
    
    if (!nh.has_parameter("exposure_search_timeout"))
    {
        nh.declare_parameter<double>("exposure_search_timeout", 5.);
    }

    nh.get_parameter("exposure_search_timeout", this->exposure_search_timeout_);

    // auto_exposure_upper_limit
    RCLCPP_DEBUG(LOGGER, "---> auto_exposure_upper_limit");
    
    if (!nh.has_parameter("auto_exposure_upper_limit"))
    {
        nh.declare_parameter<double>("auto_exposure_upper_limit", 10000000.);
    }
    
    nh.get_parameter("auto_exposure_upper_limit", this->auto_exposure_upper_limit_);

    // mtu_size
    RCLCPP_DEBUG(LOGGER, "---> gige/mtu_size");
    
    if (!nh.has_parameter("gige/mtu_size"))
    {
        nh.declare_parameter<int>("gige/mtu_size", 3000);
    }
    
    nh.get_parameter("gige/mtu_size", this->mtu_size_);

    // enable_status_publisher
    RCLCPP_DEBUG(LOGGER, "---> enable_status_publisher");
    
    if (!nh.has_parameter("enable_status_publisher"))
    {
        nh.declare_parameter<bool>("enable_status_publisher", false);
    }
    
    nh.get_parameter("enable_status_publisher", this->enable_status_publisher_);

    // enable_current_params_publisher
    RCLCPP_DEBUG(LOGGER, "---> enable_current_params_publisher");
    
    if (!nh.has_parameter("enable_current_params_publisher"))
    {
        nh.declare_parameter<bool>("enable_current_params_publisher", false);
    }
    
    nh.get_parameter("enable_current_params_publisher", this->enable_current_params_publisher_);

    // startup_user_set
    RCLCPP_DEBUG(LOGGER, "---> startup_user_set");
    
    if (!nh.has_parameter("startup_user_set"))
    {
        nh.declare_parameter<std::string>("startup_user_set", "");
    }
    
    nh.get_parameter("startup_user_set", this->startup_user_set_);

    // inter_pkg_delay
    RCLCPP_DEBUG(LOGGER, "---> gige/inter_pkg_delay");
    
    if (!nh.has_parameter("gige/inter_pkg_delay"))
    {
        nh.declare_parameter<int>("gige/inter_pkg_delay", 1000);
    }
    
    nh.get_parameter("gige/inter_pkg_delay", this->inter_pkg_delay_);

    // frame_transmission_delay
    RCLCPP_DEBUG(LOGGER, "---> gige/frame_transmission_delay");
    
    if (!nh.has_parameter("gige/frame_transmission_delay"))
    {
        nh.declare_parameter<int>("gige/frame_transmission_delay", 0);
    }
    
    nh.get_parameter("gige/frame_transmission_delay", this->frame_transmission_delay_);

    // shutter mode
    RCLCPP_DEBUG(LOGGER, "---> shutter_mode");
    
    if (!nh.has_parameter("shutter_mode"))
    {
        nh.declare_parameter<std::string>("shutter_mode", "");
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

    // auto_flash
    RCLCPP_DEBUG(LOGGER, "---> auto_flash");

    if (!nh.has_parameter("auto_flash"))
    {
        nh.declare_parameter<bool>("auto_flash", false);
    }
    
    nh.get_parameter("auto_flash", this->auto_flash_);

    RCLCPP_DEBUG(LOGGER, "---> auto_flash_line_2");
    
    if (!nh.has_parameter("auto_flash_line_2"))
    {
        nh.declare_parameter<bool>("auto_flash_line_2", true);
    }
    
    nh.get_parameter("auto_flash_line_2", this->auto_flash_line_2_);

    RCLCPP_DEBUG(LOGGER, "---> auto_flash_line_3");
    
    if (!nh.has_parameter("auto_flash_line_3"))
    {
        nh.declare_parameter<bool>("auto_flash_line_3", true);
    }
    
    nh.get_parameter("auto_flash_line_3", this->auto_flash_line_3_);

    RCLCPP_INFO(LOGGER, "Autoflash: %i, line2: %i, line3: %i", this->auto_flash_, this->auto_flash_line_2_, this->auto_flash_line_3_);

    // grab_timeout
    RCLCPP_DEBUG(LOGGER, "---> grab_timeout");

    if (!nh.has_parameter("grab_timeout"))
    {
        nh.declare_parameter<int>("grab_timeout", 500);
    }
    
    nh.get_parameter("grab_timeout", this->grab_timeout_);

    // trigger_timeout
    RCLCPP_DEBUG(LOGGER, "---> trigger_timeout");
    
    if (!nh.has_parameter("trigger_timeout"))
    {
        nh.declare_parameter<int>("trigger_timeout", 5000);
    }
    
    nh.get_parameter("trigger_timeout", this->trigger_timeout_);

    // white_balance_auto
    RCLCPP_DEBUG(LOGGER, "---> white_balance_auto");
    
    if (!nh.has_parameter("white_balance_auto"))
    {
        nh.declare_parameter<int>("white_balance_auto", 0);
    }
    
    nh.get_parameter("white_balance_auto", this->white_balance_auto_);

    // white_balance_ratio_red
    RCLCPP_DEBUG(LOGGER, "---> white_balance_ratio_red");
    
    if (!nh.has_parameter("white_balance_ratio_red"))
    {
        nh.declare_parameter<float>("white_balance_ratio_red", 1.0);
    }
    
    nh.get_parameter("white_balance_ratio_red", this->white_balance_ratio_red_);

    // white_balance_ratio_green
    RCLCPP_DEBUG(LOGGER, "---> white_balance_ratio_green");
    
    if (!nh.has_parameter("white_balance_ratio_green"))
    {
        nh.declare_parameter<float>("white_balance_ratio_green", 1.0);
    }
    
    nh.get_parameter("white_balance_ratio_green", this->white_balance_ratio_green_);

    // white_balance_ratio_blue
    RCLCPP_DEBUG(LOGGER, "---> white_balance_ratio_blue");
    
    if (!nh.has_parameter("white_balance_ratio_blue"))
    {
        nh.declare_parameter<float>("white_balance_ratio_blue", 1.0);
    }
    
    nh.get_parameter("white_balance_ratio_blue", this->white_balance_ratio_blue_);

    // grab_strategy
    RCLCPP_DEBUG(LOGGER, "---> grab_strategy");
    
    if (!nh.has_parameter("grab_strategy"))
    {
        nh.declare_parameter<int>("grab_strategy", 0);
    }
    
    nh.get_parameter("grab_strategy", this->grab_strategy_);

    // validating parameters
    this->validateParameterSet(nh);
}

void PylonROS2CameraParameter::setDeviceUserId(rclcpp::Node& nh, const std::string& device_user_id)
{
    if (!nh.has_parameter("device_user_id"))
    {
        nh.declare_parameter<std::string>("device_user_id", "");
    }

    this->device_user_id_ = device_user_id;

    nh.set_parameter(rclcpp::Parameter("device_user_id", this->device_user_id_));
}

void PylonROS2CameraParameter::validateParameterSet(rclcpp::Node& nh)
{
    if (!this->device_user_id_.empty())
    {
        RCLCPP_INFO_STREAM(LOGGER, "Trying to connect the camera device with the following device user id: " << this->device_user_id_.c_str());
    }
    else
    {
        RCLCPP_INFO_STREAM(LOGGER, "No Device User ID set -> Will connect the first available camera device");
    }

    if (this->frame_rate_ < 0 && this->frame_rate_ != -1)
    {
        RCLCPP_WARN_STREAM(LOGGER, "The specified frame rate value - " << this->frame_rate_ << " Hz - is not valid!"
                                << "-> Will reset it to default value (5 Hz).");
        this->setFrameRate(nh, 5.0);
    }

    if (this->exposure_given_ && (this->exposure_ <= 0.0 || this->exposure_ > 1e7))
    {
        RCLCPP_WARN_STREAM(LOGGER, "The specified exposure value - " << this->exposure_ << " ms - is out of valid range!"
                                << "-> Will reset it to default value.");
        this->exposure_given_ = false;
    }

    if (this->gain_given_ && ( this->gain_ < 0.0 || this->gain_ > 1.0 ))
    {
        RCLCPP_WARN_STREAM(LOGGER, "The specified gain value - " << this->gain_ << " % - is out of valid range!"
                                << "-> Will reset it to default value.");
        this->gain_given_ = false;
    }

    if (this->brightness_given_ && ( this->brightness_ < 0.0 || this->brightness_ > 255 ))
    {
        RCLCPP_WARN_STREAM(LOGGER, "The specified brightness value - " << this->brightness_ << " - is out of valid range (0 to 255)!"
                                << "-> Will reset it to default value.");
        this->brightness_given_ = false;
    }

    if (this->exposure_search_timeout_ < 5.)
    {
        RCLCPP_WARN_STREAM(LOGGER, "The specified exposure search timeout value - " << this->exposure_search_timeout_ << " - is too low!"
                                << "-> Exposure search may fail.");
    }
}

const std::string& PylonROS2CameraParameter::deviceUserID() const
{
    return this->device_user_id_;
}

std::string PylonROS2CameraParameter::shutterModeString() const
{
    if (this->shutter_mode_ == SM_ROLLING)
    {
        return "rolling";
    }
    else if (this->shutter_mode_ == SM_GLOBAL)
    {
        return "global";
    }
    else if (this->shutter_mode_ == SM_GLOBAL_RESET_RELEASE)
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

void PylonROS2CameraParameter::setimageEncodingParam(rclcpp::Node& nh, const std::string& format) 
{
    if (!nh.has_parameter("image_encoding"))
    {
        nh.declare_parameter<std::string>("image_encoding", "");
    }

    this->image_encoding_ = format;

    nh.set_parameter(rclcpp::Parameter("image_encoding", this->image_encoding_));
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
        nh.declare_parameter<double>("frame_rate", 5.0);
    }

    this->frame_rate_ = frame_rate;
    
    nh.set_parameter(rclcpp::Parameter("frame_rate", this->frame_rate_));
}

const std::string& PylonROS2CameraParameter::cameraInfoURL() const
{
    return this->camera_info_url_;
}

void PylonROS2CameraParameter::setCameraInfoURL(rclcpp::Node& nh, const std::string& camera_info_url)
{
    if (!nh.has_parameter("camera_info_url"))
    {
        nh.declare_parameter<std::string>("camera_info_url", "");
    }

    this->camera_info_url_ = camera_info_url;
    
    nh.set_parameter(rclcpp::Parameter("camera_info_url", this->camera_info_url_));
}

}  // namespace pylon_ros2_camera
