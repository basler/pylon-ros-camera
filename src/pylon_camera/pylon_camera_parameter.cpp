// Copyright 2015 <Magazino GmbH>

#include <pylon_camera/pylon_camera_parameter.h>
#include <string>

namespace pylon_camera
{

PylonCameraParameter::PylonCameraParameter() :
        binning_(1),
        camera_frame_("pylon_camera"),
        device_user_id_(""),
        // ##########################
        //  image intensity settings
        // ##########################
        exposure_(10000.0),
        exposure_fixed_(false),
        exposure_given_(false),
        brightness_(100),
        brightness_continuous_(false),
        brightness_given_(false),
        gain_(0.5),
        gain_fixed_(true),
        gain_given_(false),
        gamma_(1.0),
        gamma_given_(false),
        // #########################
        frame_rate_(-1.),
        mtu_size_(3000),
        shutter_mode_(SM_DEFAULT)
{}

PylonCameraParameter::~PylonCameraParameter()
{}

bool PylonCameraParameter::readFromRosParameterServer(const ros::NodeHandle& nh)
{
    nh.param<int>("binning", binning_, 1);
    nh.param<std::string>("camera_frame", camera_frame_, "pylon_camera");
    nh.param<std::string>("device_user_id", device_user_id_, "");

    std::cout << "device_user_id is: " << device_user_id_ << std::endl;
    // > 0: Exposure time in microseconds
    exposure_given_ = nh.hasParam("exposure");
    if ( exposure_given_ )
    {
        nh.getParam("exposure", exposure_);
        std::cout << "exposure is given and has value " << exposure_ << std::endl;
    }
    if ( nh.hasParam("exposure_fixed") )
    {
        nh.getParam("exposure_fixed", exposure_fixed_);
        std::cout << "exposure is fixed" << std::endl;
    }

    brightness_given_ = nh.hasParam("brightness");
    if ( brightness_given_ )
    {
        nh.getParam("brightness", brightness_);
        std::cout << "brightness is given and has value " << brightness_ << std::endl;
    }
    if ( nh.hasParam("brightness_continuous") )
    {
        nh.getParam("brightness_continuous", brightness_continuous_);
        std::cout << "brightness is continuous" << std::endl;
    }

    gain_given_ = nh.hasParam("gain");
    if ( gain_given_ )
    {
        nh.getParam("gain", gain_);
        std::cout << "gain is given and has value " << gain_ << std::endl;
    }
    if ( nh.hasParam("gain_fixed") )
    {
        nh.getParam("gain_fixed", gain_fixed_);
        std::cout << "gain is fixed" << std::endl;
    }

    gamma_given_ = nh.hasParam("gamma");
    if ( gamma_given_ )
    {
        nh.getParam("gamma", gamma_);
        std::cout << "gamma is given and has value " << gamma_ << std::endl;
    }

    if ( nh.hasParam("frame_rate") )
    {
        nh.getParam("frame_rate", frame_rate_);
    }
    if ( nh.hasParam("gige/mtu_size") )
    {
        nh.getParam("gige/mtu_size", mtu_size_);
    }

    if ( !device_user_id_.empty() )
    {
        ROS_INFO("Trying to open the following camera: %s", device_user_id_.c_str());
    }
    else
    {
        ROS_INFO("No Device User ID set -> Will open the camera device found first");
    }

    std::string shutter_param_string;
    nh.param<std::string>("shutter_mode", shutter_param_string, "");
    if (shutter_param_string == "rolling")
    {
        shutter_mode_ = SM_ROLLING;
    }
    else if (shutter_param_string == "global")
    {
        shutter_mode_ = SM_GLOBAL;
    }
    else if (shutter_param_string == "global_reset")
    {
        shutter_mode_ = SM_GLOBAL_RESET_RELEASE;
    }
    else
    {
        shutter_mode_ = SM_DEFAULT;
    }

    return validateParameterSet(nh);
}

bool PylonCameraParameter::validateParameterSet(const ros::NodeHandle& nh)
{
    if ( binning_ > 4 || binning_ < 1 )
    {
        ROS_ERROR_STREAM("Unsupported binning settings! Binning is " << binning_
                << ", but valid are only values in this range: [1, 2, 3, 4]");
        return false;
    }

    if ( frame_rate_ < 0 && frame_rate_ != -1 )
    {
        frame_rate_ = -1.0;
        nh.setParam("frame_rate", frame_rate_);
        ROS_ERROR("Unexpected frame rate (%f). Setting to -1 (max possible)",
                  frame_rate_);
    }

    if ( brightness_ < 0.0 || brightness_ > 255 )
    {
        ROS_ERROR_STREAM("Desired brightness not in allowed range [0 - 255]! "
               << "Brightness = " << brightness_);
        return false;
    }
    if ( gain_ < 0.0 || gain_ > 1.0 )
    {
        ROS_ERROR_STREAM("Desired gain (in percent) not in allowed range! Gain = "
                << gain_);
        return false;
    }

    return true;
}

const std::string& PylonCameraParameter::deviceUserID() const
{
    return device_user_id_;
}

std::string PylonCameraParameter::shutterModeString() const
{
    if ( shutter_mode_ == SM_ROLLING )
    {
        return "rolling";
    }
    else if ( shutter_mode_ == SM_GLOBAL )
    {
        return "global";
    }
    else if ( shutter_mode_ == SM_GLOBAL_RESET_RELEASE )
    {
        return "global_reset";
    }
    else
    {
        return "default_shutter_mode";
    }
}

const std::string& PylonCameraParameter::cameraFrame() const
{
    return camera_frame_;
}

void PylonCameraParameter::setFrameRate(const ros::NodeHandle& nh,
                                        const double& frame_rate)
{
    frame_rate_ = frame_rate;
    nh.setParam("frame_rate", frame_rate_);
}

const double& PylonCameraParameter::frameRate() const
{
    return frame_rate_;
}

}  // namespace pylon_camera
