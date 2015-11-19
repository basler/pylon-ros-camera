#include <pylon_camera/pylon_camera_node.h>
#include <GenApi/GenApi.h>
#include <cmath>

namespace pylon_camera
{

PylonCameraNode::PylonCameraNode() :
        nh_("~"),
        pylon_camera_(NULL),
        params_(),
        it_(new image_transport::ImageTransport(nh_)),
		img_raw_pub_(it_->advertiseCamera("image_raw", 10)),
        grab_images_raw_action_server_(nh_, "grab_images_raw", boost::bind(&PylonCameraNode::grabImagesRawActionExecuteCB, this, _1), false),
		set_sleeping_service_(nh_.advertiseService("set_sleeping_srv", &PylonCameraNode::setSleepingCallback, this)),
        target_brightness_(-42),
        brightness_service_running_(false),
        is_sleeping_(false)
{
	// Set parameter to open the desired camera
	params_.readFromRosParameterServer(nh_);
}

const double& PylonCameraNode::desiredFrameRate() const
{
	return params_.desired_frame_rate_;
}

uint32_t PylonCameraNode::getNumSubscribers() const
{
    return img_raw_pub_.getNumSubscribers();
}
void PylonCameraNode::checkForPylonAutoFunctionRunning()
{
    brightness_service_running_ = pylon_camera_->isAutoBrightnessFunctionRunning();
}

bool PylonCameraNode::init()
{
    if (!initAndRegister())
    {
        ros::shutdown();
        return false;
    }

    if (!startGrabbing())
    {
        ros::shutdown();
        return false;
    }
    return true;
}

bool PylonCameraNode::initAndRegister()
{
	set_exposure_service_ = nh_.advertiseService("set_exposure_srv",
												 &PylonCameraNode::setExposureCallback,
												 this);
    set_brightness_service_ = nh_.advertiseService("set_brightness_srv",
    											   &PylonCameraNode::setBrightnessCallback,
    											   this);

    pylon_camera_ = PylonCamera::create(params_.magazino_cam_id_);

    if (pylon_camera_ == NULL)
    {
        ROS_ERROR("Error while initializing the Pylon Interface");
        return false;
    }

    if (!pylon_camera_->registerCameraConfiguration(params_))
    {
        ROS_ERROR("Error while registering the camera configuration");
        return false;
    }

    grab_images_raw_action_server_.start();
    return true;
}

bool PylonCameraNode::startGrabbing()
{
    if (!pylon_camera_->startGrabbing(params_))
    {
        ROS_ERROR("Error while start grabbing");
        return false;
    }

    // Framrate Settings
    if (pylon_camera_->maxPossibleFramerate() < params_.desired_frame_rate_)
    {
        ROS_INFO("Desired framerate %.2f is higher than max possible. Will limit framerate to: %.2f Hz",
                 params_.desired_frame_rate_,
                 pylon_camera_->maxPossibleFramerate());
        params_.desired_frame_rate_ = pylon_camera_->maxPossibleFramerate();
        nh_.setParam("desired_framerate", pylon_camera_->maxPossibleFramerate());
    }
    else if (params_.desired_frame_rate_ == -1)
    {
        params_.desired_frame_rate_ = pylon_camera_->maxPossibleFramerate();
        ROS_INFO("Max possible framerate is %.2f Hz", pylon_camera_->maxPossibleFramerate());
    }

    std_msgs::Header header;
    header.frame_id = params_.camera_frame_;
    header.stamp = ros::Time::now();

    cam_info_msg_.header = header;
    cam_info_msg_.height = pylon_camera_->imageRows();
    cam_info_msg_.width = pylon_camera_->imageCols();
    cam_info_msg_.distortion_model = "plumb_bob";

    img_raw_msg_.header = header;
    // Encoding of pixels -- channel meaning, ordering, size
    // taken from the list of strings in include/sensor_msgs/image_encodings.h
    img_raw_msg_.encoding = pylon_camera_->imageEncoding();
    img_raw_msg_.height = pylon_camera_->imageRows();
    img_raw_msg_.width = pylon_camera_->imageCols();
    // step = full row length in bytes
    img_raw_msg_.step = img_raw_msg_.width * pylon_camera_->imagePixelDepth();
    // img_raw_msg_.data // actual matrix data, size is (step * rows)
    pylon_camera_->setImageSize(img_raw_msg_.step * img_raw_msg_.height);

    return true;
}

bool PylonCameraNode::grabImage()
{
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    if (!pylon_camera_->grab(img_raw_msg_.data))
    {
        if (pylon_camera_->isCamRemoved())
        {
            ROS_ERROR("Pylon Camera has been removed!");
            ros::shutdown();
        }
        else
        {
            ROS_WARN("Pylon Interface returned invalid image! Skipping");
        }
        return false;
    }
    img_raw_msg_.header.stamp = ros::Time::now();
    cam_info_msg_.header.stamp = img_raw_msg_.header.stamp;
    return true;
}

bool PylonCameraNode::grabSequence()
{
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    bool success = true;
    std::size_t mid = params_.desired_seq_exp_times_.size() / 2;
    std::vector<uint8_t> tmp_image;
    for (std::size_t i = 0; i < params_.desired_seq_exp_times_.size(); ++i)
    {
        if (!(pylon_camera_->grab(tmp_image)))
        {
            if (pylon_camera_->isCamRemoved())
            {
                ROS_ERROR("Pylon Camera has been removed!");
                ros::shutdown();
            }
            else
            {
                ROS_INFO("Pylon Interface returned NULL-Pointer!");
            }
            success = false;
        }
        if (i == mid && success)
        {
        	img_raw_msg_.data = tmp_image;
            img_raw_msg_.header.stamp = ros::Time::now();
        }
    }
    if (!success)
        return false;

    cam_info_msg_.header.stamp = img_raw_msg_.header.stamp;
    return true;
}

void PylonCameraNode::spin()
{
    if (getNumSubscribers() > 0 && ! is_sleeping())
    {
        try
        {
            checkForPylonAutoFunctionRunning();
        }
        catch (GenICam::AccessException &e)
        {
        }

        if (grabImage())
        {
            img_raw_pub_.publish(img_raw_msg_, cam_info_msg_);
        }
    }
}

void PylonCameraNode::grabImagesRawActionExecuteCB(const camera_control_msgs::GrabImagesGoal::ConstPtr& goal)
{
    camera_control_msgs::GrabImagesResult result;
    camera_control_msgs::GrabImagesFeedback feedback;

    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);

    result.images.resize(goal->target_values.size());
    result.reached_values.resize(goal->target_values.size());
    result.success = true;
    for (std::size_t i = 0; i < goal->target_values.size(); ++i)
    {
    	// setGain(goal->gain);
    	// setGamma(goal->gamma);
    	if(goal->target_type == goal->EXPOSURE) {
    		setExposure(goal->target_values[i], result.reached_values[i]);
    	}
    	if(goal->target_type == goal->BRIGHTNESS) {
    		int reached_val;
    		setBrightness(goal->target_values[i], reached_val);
    		result.reached_values[i] = (float)reached_val;
    	}

        sensor_msgs::Image& img = result.images[i];
        img.encoding = pylon_camera_->imageEncoding();
        img.height = pylon_camera_->imageRows();
        img.width = pylon_camera_->imageCols();
        // step = full row length in bytes
        img.step = img.width * pylon_camera_->imagePixelDepth();

        if (!(pylon_camera_->grab(img.data)))
        {
            result.success = false;
        }
        img.header.stamp = ros::Time::now();
        feedback.curr_nr_images_taken = i+1;
        grab_images_raw_action_server_.publishFeedback(feedback);
    }

    if (!result.success)
    {
        result.images.clear();
    }

    grab_images_raw_action_server_.setSucceeded(result);
}

bool PylonCameraNode::setExposure(const float& target_exposure, float& reached_exposure)
{
	if (!pylon_camera_->isReady())
	{
		ROS_WARN("Error in setExposure(): pylon_camera_ is not ready!");
		return false;
	}

	reached_exposure = getCurrentExposure();

	if (reached_exposure != target_exposure)
	{
		pylon_camera_->setExposure(target_exposure);
	}

	// wait for max 5s till the cam has updated the exposure
	ros::Rate r(10.0);
	ros::Time start = ros::Time::now();
	while (ros::ok())
	{
		reached_exposure = getCurrentExposure();

		bool success = fabs(reached_exposure - target_exposure) < pylon_camera_->exposureStep();

		if(success)
		{
			return true;
		}

		if (ros::Time::now() - start > ros::Duration(5.0))
		{
			ROS_ERROR("Error in setExposure(): Did not reach the desired brightness in time");
			return false;
	    }
		r.sleep();
	}
	return true;
}

bool PylonCameraNode::setExposureCallback(camera_control_msgs::SetExposureSrv::Request &req,
										  camera_control_msgs::SetExposureSrv::Response &res)
{
	res.success = setExposure(req.target_exposure, res.reached_exposure);
	return true;
}

bool PylonCameraNode::setBrightness(const int& target_brightness, int& reached_brightness)
{
	// Brightness Service can only work, if an image has already been grabbed (calc mean on current img)
	if (!pylon_camera_->isReady())
	{
		ros::Rate r(2.0);
		ros::Time start = ros::Time::now();
		while (ros::ok() && !pylon_camera_->isReady())
		{
			if (ros::Time::now() - start > ros::Duration(3.0))
			{
				ROS_ERROR("Pylon Interface has not yet grabbed an image, although waiting for 3 seconds!");
				return false;
	        }
			ros::spinOnce();
			r.sleep();
		}
	}

	// Get actual image
	ros::spinOnce();

	int current_brightness = calcCurrentBrightness();
	ROS_INFO("New brightness request for brightness %i, current brightness = %i",
			target_brightness,
			current_brightness);

	target_brightness_ = target_brightness;
	brightness_service_running_ = true;

	if (current_brightness != target_brightness_)
	{
		pylon_camera_->setBrightness(target_brightness_);
	}
	else
	{
		return true;
	}

	ros::Duration duration;
	if (target_brightness_ > 205)
	{
		// Need more time for great exposure values
		duration = ros::Duration(15.0);
	}
	else
	{
		duration = ros::Duration(5.0);
	}
	ros::Rate r(5.0);
	ros::Time start = ros::Time::now();
	while (ros::ok() && brightness_service_running_)
	{
		if (ros::Time::now() - start > duration)
		{
			ROS_ERROR("Did not reach the required brightness in time");
			brightness_service_running_ = false;
			return false;
		}
		ros::spinOnce();
		r.sleep();
	}
	reached_brightness = calcCurrentBrightness();
	return brightnessValidation(target_brightness);
}

bool PylonCameraNode::setBrightnessCallback(camera_control_msgs::SetBrightnessSrv::Request &req,
    camera_control_msgs::SetBrightnessSrv::Response &res)
{
	res.success = setBrightness(req.target_brightness, res.reached_brightness);
	return true;
}

bool PylonCameraNode::brightnessValidation(int target)
{
    int mean = calcCurrentBrightness();
    if (abs(target - mean) > 2)
    {
        return false;
    }
    return true;
}

int PylonCameraNode::calcCurrentBrightness()
{
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    int sum = std::accumulate(img_raw_msg_.data.begin(), img_raw_msg_.data.end(), 0);
    assert(img_raw_msg_.data.size() > 0);
    float mean = sum / img_raw_msg_.data.size();
    return (int)mean;
}

float PylonCameraNode::getCurrentExposure()
{
    return pylon_camera_->currentExposure();
}

/// Warum Service, wenn sofort immer true zurueckgegeben wird?
bool PylonCameraNode::setSleepingCallback(camera_control_msgs::SetSleepingSrv::Request &req,
    camera_control_msgs::SetSleepingSrv::Response &res)
{
    is_sleeping_ = req.set_sleeping;

    if (is_sleeping_)
    {
        ROS_INFO("Seting Pylon Camera Node to sleep...");
    } else
    {
        ROS_INFO("Pylon Camera Node continues grabbing");
    }

    res.success = true;
    return true;
}

bool PylonCameraNode::is_sleeping()
{
    return is_sleeping_;
}

PylonCameraNode::~PylonCameraNode()
{
    delete pylon_camera_;
    pylon_camera_ = NULL;
    delete it_;
    it_ = NULL;
}

}
