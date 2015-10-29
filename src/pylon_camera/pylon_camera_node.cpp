#include <pylon_camera/pylon_camera_node.h>
#include <GenApi/GenApi.h>
#include <cmath>

namespace pylon_camera
{

PylonCameraNode::PylonCameraNode() :
        nh_("~"),
        pylon_camera_(NULL),
        it_(new image_transport::ImageTransport(nh_)),
		img_raw_pub_(it_->advertiseCamera("image_raw", 10)),
        exp_times_pub_(nh_.advertise<pylon_camera_msgs::SequenceExposureTimes>("seq_exp_times", 10)),
        sequence_raw_as_(nh_, "grab_sequence_raw", boost::bind(&PylonCameraNode::sequenceRawActionExecuteCB, this, _1), false),
		set_sleeping_service_(nh_.advertiseService("set_sleeping_srv", &PylonCameraNode::setSleepingCallback, this)),
        target_brightness_(-42),
        brightness_service_running_(false),
        is_sleeping_(false)
{
}


const double& PylonCameraNode::desiredFrameRate() const
{
	return params_.desired_frame_rate_;
}

// Set parameter to open the desired camera
void PylonCameraNode::getInitialCameraParameter()
{
    // Write Magazino cam id to the camera using write_magazino_id_2_camera
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
    if (params_.desired_frame_rate_ < 0 && params_.desired_frame_rate_ != -1)
    {
        params_.desired_frame_rate_ = -1.0;
        nh_.setParam("desired_framerate", params_.desired_frame_rate_);
        ROS_ERROR("Unexpected framerate (%f). Setting to -1 (max possible)",
                  params_.desired_frame_rate_);
    }
    nh_.param<std::string>("camera_frame", params_.camera_frame_, "pylon_camera");
    nh_.param<int>("mtu_size", params_.mtu_size_, 3000);

// -2: AutoExposureOnce
// -1: AutoExposureContinuous
//  0: AutoExposureOff
// > 0: Exposure in micro-seconds
    nh_.param<double>("start_exposure", params_.start_exposure_, 35000.0);

    nh_.param<bool>("use_brightness", params_.use_brightness_, false); // Using exposure or brightness
    // -2: AutoExposureOnce
    // -1: AutoExposureContinuous
    //  0: AutoExposureOff
    // > 0: Intensity Value (0-255)
    nh_.param<int>("start_brightness", params_.start_brightness_, 128);

    nh_.param<bool>("use_sequencer", params_.use_sequencer_, false);
    if (params_.use_sequencer_)
    {
        nh_.getParam("desired_seq_exp_times", params_.desired_seq_exp_times_);
    }
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
    if (!params_.use_sequencer_)
    {
        set_exposure_service_ = nh_.advertiseService("set_exposure_srv",
                                                     &PylonCameraNode::setExposureCallback,
                                                     this);
        set_brightness_service_ = nh_.advertiseService("set_brightness_srv",
                                                       &PylonCameraNode::setBrightnessCallback,
                                                       this);
    }

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

    if (params_.use_sequencer_)
    {
        sequence_raw_as_.start();
    }
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
    exp_times_.header = img_raw_msg_.header;

    if (params_.use_sequencer_)
    {
    	exp_times_.exp_times.data = params_.desired_seq_exp_times_;
    }

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
    exp_times_.header.stamp = img_raw_msg_.header.stamp;
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


void PylonCameraNode::sequenceRawActionExecuteCB(const pylon_camera_msgs::GrabSequenceGoal::ConstPtr& goal)
{
    pylon_camera_msgs::GrabSequenceResult result;
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    result.images.resize(params_.desired_seq_exp_times_.size());
    result.exposureTimes = params_.desired_seq_exp_times_;
    result.success = true;
    for (std::size_t i = 0; i < params_.desired_seq_exp_times_.size(); ++i)
    {
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
    }

    if (!result.success)
    {
        result.images.clear();
    }

    sequence_raw_as_.setSucceeded(result);
}


bool PylonCameraNode::setExposureCallback(pylon_camera_msgs::SetExposureSrv::Request &req,
										  pylon_camera_msgs::SetExposureSrv::Response &res)
{
    if (!pylon_camera_->isReady())
    {
        res.success = false;
        return true;
    }

    float current_exposure = getCurrenCurrentExposure();
//    ROS_INFO("New exposure request for exposure %.f, current exposure = %.f", req.target_exposure, current_exposure);

    if (current_exposure != req.target_exposure)
    {
        pylon_camera_->setExposure(req.target_exposure);
    }

    // wait for 2 cycles till the cam has updated the exposure
    ros::Rate r(5.0);
    ros::Time start = ros::Time::now();
    int ctr = 0;
    while (ros::ok() && ctr < 2)
    {
        if (ros::Time::now() - start > ros::Duration(5.0))
        {
            ROS_ERROR("Did not reach the required brightness in time");
            res.success = false;
            return true;
        }
        ros::spinOnce();
        r.sleep();
        ctr++;
    }

    current_exposure = getCurrenCurrentExposure();

    if (fabs(current_exposure - req.target_exposure) < pylon_camera_->exposureStep())
    {
        res.success = true;
    } else
    {
        res.success = false;
    }
    return true;
}

bool PylonCameraNode::setBrightnessCallback(pylon_camera_msgs::SetBrightnessSrv::Request &req,
    pylon_camera_msgs::SetBrightnessSrv::Response &res)
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
                res.success = false;
                return true;
            }
            ros::spinOnce();
            r.sleep();
        }
    }

    // Get actual image
    ros::spinOnce();

    int current_brightness = calcCurrentBrightness();
    ROS_INFO("New brightness request for brightness %i, current brightness = %i",
             req.target_brightness,
             current_brightness);

    target_brightness_ = req.target_brightness;
    brightness_service_running_ = true;

    if (current_brightness != target_brightness_)
    {
        pylon_camera_->setBrightness(target_brightness_);
    }
    else
    {
        res.success = true;
        return true;
    }

    ros::Duration duration;
    if (target_brightness_ > 205)
    {
        // Need more time for great exposure values
        duration = ros::Duration(15.0);
    } else
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
            res.success = false;
            return true;
        }
        ros::spinOnce();
        r.sleep();
    }

    if (!brightnessValidation(req.target_brightness))
        res.success = false;
    else
        res.success = true;
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

float PylonCameraNode::getCurrenCurrentExposure()
{
    return pylon_camera_->currentExposure();
}

/// Warum Service, wenn sofort immer true zurueckgegeben wird?
bool PylonCameraNode::setSleepingCallback(pylon_camera_msgs::SetSleepingSrv::Request &req,
    pylon_camera_msgs::SetSleepingSrv::Response &res)
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

bool PylonCameraNode::have_intrinsic_data()
{
    return params_.have_intrinsic_data_;
}

PylonCameraNode::~PylonCameraNode()
{
    delete pylon_camera_;
    pylon_camera_ = NULL;
    delete it_;
    it_ = NULL;
}

}
