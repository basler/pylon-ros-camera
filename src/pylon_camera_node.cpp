/*
 * pylon_camera_node.cpp
 *
 *  Created on: Jun 10, 2015
 *      Author: md
 */

#include <pylon_camera/pylon_camera_node.h>

namespace pylon_camera
{

PylonCameraNode::PylonCameraNode() :
                    nh_("~"),
                    it_(NULL),
                    pylon_interface_(NULL),
                    params_(),
                    set_exposure_service_(),
                    set_brightness_service_(),
                    img_raw_msg_(),
                    cam_info_msg_(),
                    img_raw_pub_()

{
    it_ = new image_transport::ImageTransport(nh_);
    img_raw_pub_ = it_->advertiseCamera("image_raw", 10);
    set_exposure_service_ = nh_.advertiseService("set_exposure_srv",
                                                 &PylonCameraNode::setExposureCallback,
                                                 this);
    set_brightness_service_ = nh_.advertiseService("set_brightness_srv",
                                                   &PylonCameraNode::setBrightnessCallback,
                                                   this);
}

// Set parameter to open the desired camera
void PylonCameraNode::getInitialCameraParameter()
{
    // Write Magazino cam id to the camera using write_magazino_id_2_camera
    nh_.param<std::string>("magazino_cam_id", params_.magazino_cam_id_, "x");
    if (params_.magazino_cam_id_ != "x")
    {
        ROS_INFO("Using Camera: %s", params_.magazino_cam_id_.c_str());
    } else
    {
        ROS_INFO("No Magazino Cam ID set -> Will use the camera device found fist");
    }

    nh_.param<double>("desired_framerate", params_.desired_frame_rate_, 10.0);
    if (params_.desired_frame_rate_ < 0 && params_.desired_frame_rate_ != -1)
    {
        params_.desired_frame_rate_ = -1.0;
        nh_.setParam("desired_framerate", params_.desired_frame_rate_);
        ROS_ERROR("Unexpected framerate (%f). Setting to -1 (max possible)",
                  params_.desired_frame_rate_);
    }
    nh_.param<std::string>("camera_frame", params_.camera_frame_, "pylon_camera");
}
void PylonCameraNode::getRuntimeCameraParameter()
{
    nh_.param<bool>("use_trigger_service", params_.use_trigger_service_, false);
    nh_.param<int>("parameter_update_frequency", params_.param_update_frequency_, 100);

    nh_.param<double>("exposure", params_.exposure_, 35000.0); 	// -2: AutoExposureOnce
    // -1: AutoExposureContinuous
    //  0: AutoExposureOff
    // > 0: Exposure in micro-seconds
    nh_.param<bool>("use_brightness", params_.use_brightness_, false); // Using exposure or brightness
    nh_.param<int>("brightness", params_.brightness_, 128); 	// -2: AutoExposureOnce
    // -1: AutoExposureContinuous
    //  0: AutoExposureOff
    // > 0: Intensity Value (0-255)

//	if ((params_.exposure_ == -1.0 || params_.exposure_ == -2.0 || params_.exposure_ == 0.0) && !pylon_interface_->has_auto_exposure()) {
//		ROS_WARN("Illegal operation: Camera has NO auto-exposure. Set parameter back to 'false'");
//		nh_.setParam("auto_exposure", false);
//	}
}
uint32_t PylonCameraNode::getNumSubscribers()
{
    return img_raw_pub_.getNumSubscribers();
}

// Using OpenCV -> creates PylonOpenCVNode (with sequencer and rectification), else: only image_raw
void PylonCameraNode::createPylonInterface()
{
    pylon_interface_ = new PylonInterface();
    ROS_INFO("Created PylonInterface");
}
void PylonCameraNode::updateROSBirghtnessParameter()
{
    params_.brightness_ = pylon_interface_->last_brightness_val();
    nh_.setParam("brightness", params_.brightness_);
}
bool PylonCameraNode::init(){
    if (!initAndRegister())
    {
        return false;
    }

    if (!startGrabbing())
    {
        return false;
    }
    return true;
}
bool PylonCameraNode::initAndRegister()
{
    if (pylon_interface_->initialize(params_) != 0)
    {
        ROS_ERROR("Error while initializing the Pylon Interface");
        return false;
    }

    if (!pylon_interface_->registerCameraConfiguration(params_))
    {
        ROS_ERROR("Error while registering the camera configuration");
        return false;
    }
    return true;
}

bool PylonCameraNode::startGrabbing(){
    if (!pylon_interface_->startGrabbing(params_))
    {
        ROS_ERROR("Error while start grabbing");
        return false;
    }

    // Framrate Settings
    if (pylon_interface_->max_possible_framerate() < params_.desired_frame_rate_)
    {
        ROS_INFO("Desired framerate %.2f is higher than max possible. Will limit framerate to: %.2f Hz",
                 params_.desired_frame_rate_,
                 pylon_interface_->max_possible_framerate());
        params_.desired_frame_rate_ = pylon_interface_->max_possible_framerate();
        nh_.setParam("desired_framerate", pylon_interface_->max_possible_framerate());
    }
    else if (params_.desired_frame_rate_ == -1)
    {
        params_.desired_frame_rate_ = pylon_interface_->max_possible_framerate();
        ROS_INFO("Max possible framerate is %.2f Hz", pylon_interface_->max_possible_framerate());
    }

    std_msgs::Header header;
    header.frame_id = params_.camera_frame_;
    //header.seq =
    header.stamp = ros::Time::now();

    cam_info_msg_.header = header;
    cam_info_msg_.height = pylon_interface_->img_rows();
    cam_info_msg_.width = pylon_interface_->img_cols();
    cam_info_msg_.distortion_model = "plumb_bob";

    img_raw_msg_.header = header;
    // Encoding of pixels -- channel meaning, ordering, size
    // taken from the list of strings in include/sensor_msgs/image_encodings.h
    img_raw_msg_.encoding = pylon_interface_->img_encoding();
    img_raw_msg_.height = pylon_interface_->img_rows();
    img_raw_msg_.width = pylon_interface_->img_cols();
    // step = full row length in bytes
    img_raw_msg_.step = img_raw_msg_.width * pylon_interface_->img_pixel_depth();
    // img_raw_msg_.data // actual matrix data, size is (step * rows)
    pylon_interface_->set_image_size(img_raw_msg_.step * img_raw_msg_.height);

    return true;
}
void PylonCameraNode::updateAquisitionSettings()
{
    if (params_.use_sequencer_)
    {
        // Up-To-Now: Impossible to change runtime parameter, when in sequencer mode
        return;
    }
    else
    {
        ROS_INFO("Updating runtime parameter (update frequency is %d cycles)",
                 params_.param_update_frequency_);

        getRuntimeCameraParameter();

        if (params_.use_brightness_)
        {
            if (pylon_interface_->last_brightness_val() != params_.brightness_)
            {
                if (pylon_interface_->setBrightness(params_.brightness_))
                {
                    ROS_ERROR("Error while updating brightness!");
                }

            }
        }
        else
        {
            if (pylon_interface_->last_exposure_val() != params_.exposure_)
            {
                if (pylon_interface_->setExposure(params_.exposure_))
                {
                    ROS_ERROR("Error while updating exposure!");
                }
                params_.exposure_ = pylon_interface_
                                                    ->last_exposure_val();
                nh_.setParam("exposure", params_.exposure_);
            }
        }
    }
}
bool PylonCameraNode::grabImage()
{
    if (!pylon_interface_->grab(params_, img_raw_msg_.data ))
    {
        if (pylon_interface_->is_cam_removed())
        {
            ROS_ERROR("Pylon Camera has been removed!");
            ros::shutdown();
        }
        else
        {
            ROS_ERROR("Pylon Interface returned invalid image!");
        }
        return false;
    }
    img_raw_msg_.header.stamp = ros::Time::now();
    cam_info_msg_.header.stamp = img_raw_msg_.header.stamp;
    return true;
}
bool PylonCameraNode::setExposureCallback(pylon_camera_msgs::SetExposureSrv::Request &req,
    pylon_camera_msgs::SetExposureSrv::Response &res)
{
    if (pylon_interface_->setExposure(req.target_exposure))
    {
        res.success = false;
    }
    else
    {
        res.success = true;
        params_.exposure_ = pylon_interface_->last_exposure_val();
        nh_.setParam("exposure", params_.exposure_);
    }
    return res.success;
}
bool PylonCameraNode::setBrightnessCallback(pylon_camera_msgs::SetBrightnessSrv::Request &req,
    pylon_camera_msgs::SetBrightnessSrv::Response &res)
{
    if (pylon_interface_->setBrightness(req.target_brightness))
    {
        res.success = false;
    }
    else
    {
        res.success = true;
//        params_.brightness_ = pylon_interface_->last_brightness_val();
//        nh_.setParam("brightness", params_.brightness_);
    }
    return res.success;
}
PylonCameraNode::~PylonCameraNode()
{
    delete it_;
    it_ = NULL;
    delete pylon_interface_;
    pylon_interface_ = NULL;
}
} /* namespace pylon_camera */
