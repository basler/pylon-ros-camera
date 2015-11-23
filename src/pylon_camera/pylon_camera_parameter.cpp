#include <pylon_camera/pylon_camera_parameter.h>

namespace pylon_camera
{

PylonCameraParameter::PylonCameraParameter() :
                    magazino_cam_id_("x"),
                    camera_frame_(""),
                    desired_frame_rate_(-1.0),
                    target_exposure_(3000),
                    param_update_frequency_(50),
                    start_exposure_(2000.0),
                    use_brightness_(false),
                    start_brightness_(128),
                    desired_seq_exp_times_(),
                    mtu_size_(3000)
{}

PylonCameraParameter::~PylonCameraParameter()
{}

bool PylonCameraParameter::writeToYamlFile(std::string& yaml_string)
{
	ROS_ERROR("PylonCameraParameter::writeToYamlFile() not yet implemented!");
	return true;
}
bool PylonCameraParameter::readFromYamlFile(const std::string& yaml_string)
{
	ROS_ERROR("PylonCameraParameter::readFromYamlFile() not yet implemented!");
	return true;
}

bool PylonCameraParameter::readFromRosParameterServer(ros::NodeHandle& nh)
{
	std::stringstream ss;
	ss << "Reading PylonCameraParameter from ROS-Parameter-Server";

	// Write Magazino cam id to the camera using write_magazino_id_2_camera
	nh.param<std::string>("magazino_cam_id", magazino_cam_id_, "x");

	nh.param<double>("desired_framerate", desired_frame_rate_, 10.0);

	nh.param<std::string>("camera_frame", camera_frame_, "pylon_camera");

	nh.param<int>("mtu_size", mtu_size_, 3000);

	// -1: AutoExposureContinuous
	//  0: AutoExposureOff
	// > 0: Exposure in micro-seconds
	nh.param<double>("start_exposure", start_exposure_, 35000.0);

	nh.param<bool>("use_brightness", use_brightness_, false); // Using exposure or brightness

	// -1: AutoExposureContinuous
	//  0: AutoExposureOff
	// > 0: Intensity Value (0-255)
	nh.param<int>("start_brightness", start_brightness_, 128);

//	nh.param<bool>("use_sequencer", use_sequencer_, false);
//	if (use_sequencer_)
//	{
		nh.getParam("desired_seq_exp_times", desired_seq_exp_times_);
//	}

	bool is_valid = validateParameterSet(nh);
	if(is_valid)
	{
		ss << " . . . done!";
		ROS_INFO(ss.str().c_str());
	}

	return is_valid;
}

bool PylonCameraParameter::validateParameterSet(ros::NodeHandle& nh)
{
	if (magazino_cam_id_ != "x")
	{
		ROS_INFO("Using Camera: %s", magazino_cam_id_.c_str());
	}
	else
	{
		ROS_INFO("No Magazino Cam ID set -> Will use the camera device found fist");
	}


	if (desired_frame_rate_ < 0 && desired_frame_rate_ != -1)
	{
		desired_frame_rate_ = -1.0;
		nh.setParam("desired_framerate", desired_frame_rate_);
		ROS_ERROR("Unexpected framerate (%f). Setting to -1 (max possible)",
				  desired_frame_rate_);
	}

	return true;
}
}
