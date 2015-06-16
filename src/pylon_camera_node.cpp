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
        pylon_interface_(NULL),
        it_(NULL),
        img_size_byte_(-1),
        img_raw_pub_(NULL),
        cam_info_msg_(),
        img_raw_msg_()

{
  it_ = new image_transport::ImageTransport(nh_);
  img_raw_pub_ = new image_transport::CameraPublisher(it_->advertiseCamera("image_raw", 10));
//	img_grabbing_server_ = nh_.advertiseService("img_grab_srv", &PylonCameraNode::grabbingCallback, this);
  set_exposure_service_ = nh_.advertiseService("set_exposure_srv", &PylonCameraNode::setExposureCallback, this);
  set_brightness_service_ = nh_.advertiseService("set_brightness_srv", &PylonCameraNode::setBrightnessCallback, this);
}
PylonCameraNode::~PylonCameraNode()
{
  // TODO Auto-generated destructor stub
}
void PylonCameraNode::setInitialCameraParameter()
{
  // Write Magazino cam id using the
  nh_.param<std::string>("magazino_cam_id", params_.magazino_cam_id_, "x");

  if (params_.magazino_cam_id_ != "x")
  {
    ROS_INFO("Using Camera: %s", params_.magazino_cam_id_.c_str());
  }
  else
  {
    ROS_INFO("No Magazino Cam ID set -> Will use the camera device found fist");
  }

  nh_.param<double>("desired_framerate", params_.desired_frame_rate_, 10.0);
  if (!(params_.desired_frame_rate_ > 0 || params_.desired_frame_rate_ == -1))
  {
    ROS_ERROR("Unexpected framerate (%f). Setting to -1 (max possible)", params_.desired_frame_rate_);
  }
  nh_.param<std::string>("camera_frame", params_.camera_frame_, "pylon_camera");
}
void PylonCameraNode::setRuntimeCameraParameter()
{
  nh_.param<bool>("use_trigger_service", params_.use_trigger_service_, false);
  nh_.param<int>("parameter_update_frequency", params_.param_update_frequency_, 100);

  nh_.param<double>("exposure", params_.exposure_, 2000.0); 	// -2: AutoExposureOnce
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
  return img_raw_pub_->getNumSubscribers();
}
void PylonCameraNode::createPylonInterface()
{
  pylon_interface_ = new PylonInterface();
  ROS_INFO("Created PylonInterface");
}
int PylonCameraNode::init()
{
  if (pylon_interface_->initialize(params_) != 0)
  {
    ROS_ERROR("Error while initializing the Pylon Interface");
    ros::shutdown();
    return 1;
  }
  if (pylon_interface_->setupCameraConfiguration(params_) != 0)
  {
    ROS_ERROR("Error while setup the image aquisation config");
    ros::shutdown();
    return 2;
  }
  // Framrate Settings
  if (pylon_interface_->max_possible_framerate() < params_.desired_frame_rate_)
  {
    ROS_INFO("Desired framerate %.2f is higher than max possible. Will limit framerate to: %.2f Hz",
             params_.desired_frame_rate_, pylon_interface_->max_possible_framerate());
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
  img_size_byte_ = img_raw_msg_.step * img_raw_msg_.height;

  return 0;
}
const uint8_t* PylonCameraNode::grabbingCallback()
{
  const uint8_t* img(pylon_interface_->grab(params_));
  if (img == NULL)
  {
    if (pylon_interface_->is_cam_removed())
    {
      ROS_ERROR("Pylon Camera has been removed!");
      ros::shutdown();
    }
    else
    {
      ROS_ERROR("Pylon Interface returned NULL-Pointer!");
    }
    return NULL;
  }
  img_raw_msg_.data = std::vector<uint8_t>(img, img + img_size_byte_);
  img_raw_msg_.header.stamp = ros::Time::now();
  cam_info_msg_.header.stamp = img_raw_msg_.header.stamp;
  return img;
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
    params_.brightness_ = pylon_interface_->last_brightness_val();
    nh_.setParam("brightness", params_.brightness_);
  }
  return res.success;
}
} /* namespace pylon_camera */
