#include <iostream>
#include <memory>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/array.hpp>
#include <image_transport/image_transport.h>
//#include <camera_info_manager/camera_info_manager.h>

#include <pylon_camera/pylon_camera_parameter.h>
#include <pylon_camera/pylon_single_exposure_interface.h>

#include <pylon_camera_msgs/SetExposureSrv.h>
#include <pylon_camera_msgs/ImgGrabbingTrigger.h>

#ifdef WITH_OPENCV
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pylon_camera/image_rectifier.h>
#include <pylon_camera/pylon_hdr_interface.h>
#endif

using std::cout;
using std::endl;
using namespace cv;
using namespace pylon_camera;

class PylonCameraNode {
public:
	PylonCameraNode();
	ros::NodeHandle nh_;

	PylonInterface* pylon_interface_;
	PylonCameraParameter params_;

#ifdef WITH_OPENCV
	cv_bridge::CvImage cv_img_raw_;
//	ImageRectifier* img_rectifier_;
#else
	sensor_msgs::Image img_raw_msg_;
#endif

	void setInitialCameraParameter();
	void setRuntimeCameraParameter();
	void init();
	void grabbingCallback();
	bool setExposureCallback(pylon_camera_msgs::SetExposureSrv::Request &req, pylon_camera_msgs::SetExposureSrv::Response &res);

	image_transport::ImageTransport* it_;
	image_transport::CameraPublisher img_raw_pub_;

//	camera_info_manager::CameraInfoManager* cam_info_manager_;
	sensor_msgs::CameraInfo cam_info_msg_;

	ros::ServiceServer img_grabbing_server_;
	ros::ServiceServer set_exposure_service_;

	int img_size_byte_;
private:
	bool imgGrabbingTriggerCallback();
};
PylonCameraNode::PylonCameraNode() :
		nh_("~"),
		pylon_interface_(NULL),
		it_(NULL),
		img_size_byte_(-1) {
}
void PylonCameraNode::setInitialCameraParameter() {
	// Write Magazino cam id using the
	nh_.param<std::string>("magazino_cam_id", params_.magazino_cam_id_, "x");

	if (params_.magazino_cam_id_ != "x") {
		ROS_INFO("Using Camera: %s", params_.magazino_cam_id_.c_str());
	} else {
		ROS_INFO("No Magazino Cam ID set -> Will use the camera device found fist");
	}

	nh_.param<double>("desired_framerate", params_.desired_frame_rate_, 10.0);
	if (!(params_.desired_frame_rate_ > 0 || params_.desired_frame_rate_ == -1)) {
		ROS_ERROR("Unexpected framerate (%f). Setting to -1 (max possible)", params_.desired_frame_rate_);
	}
	nh_.param<std::string>("camera_frame", params_.camera_frame_, "pylon_camera");
	nh_.param<bool>("use_hdr", params_.use_hdr_, false);
}
void PylonCameraNode::setRuntimeCameraParameter() {
	nh_.param<bool>("use_trigger_service", params_.use_trigger_service_, false);
	nh_.param<int>("parameter_update_frequency", params_.param_update_frequency_, 100);

	nh_.param<double>("exposure", params_.exposure_, 2000.0); 	// -2: AutoExposureOnce
																// -1: AutoExposureContinuous
																//  0: AutoExposureOff
																// > 0: Exposure in micro-seconds
	nh_.param<bool>("use_brightness",params_.use_brightness_,false); // Using exposure or brightness
	nh_.param<int>("brightness", params_.brightness_, 128); 	// -2: AutoExposureOnce
																// -1: AutoExposureContinuous
																// > 0: Intensity Value (0-255)

	if((params_.exposure_ == -1 || params_.exposure_ == -2) && !pylon_interface_->has_auto_exposure()){
		ROS_WARN("Illegal operation: Camera has NO auto-exposure. Set parameter back to 'false'");
		nh_.setParam("auto_exposure", false);
	}
}
void PylonCameraNode::init() {
	it_ = new image_transport::ImageTransport(nh_);
	img_raw_pub_ = it_->advertiseCamera("image_raw", 10);
	//img_grabbing_server_ = nh_.advertiseService("img_grabbing_trigger", )
	set_exposure_service_ = nh_.advertiseService("set_exposure_srv", &PylonCameraNode::setExposureCallback, this);

//	cam_info_manager_ = new camera_info_manager::CameraInfoManager(nh_);
//
//	if (!cam_info_manager_->setCameraName(params_.camera_frame_))
//	{
//		// GUID is 16 hex digits, which should be valid.
//		// If not, use it for log messages anyway.
//		ROS_ERROR("[ %s ] name not valid for camera_info_manger", params_.camera_frame_.c_str());
//	}

#ifdef WITH_OPENCV
	if(params_.use_hdr_)
	{
		pylon_interface_ = new PylonHDRInterface();
		ROS_INFO("Created PylonHDRInterface");
	} else {
		pylon_interface_ = new PylonSingleExposureInterface();
		ROS_INFO("Created PylonSingleExposureInterface");
	}
#else
	pylon_interface_ = new PylonSingleExposureInterface();
	ROS_INFO("Created PylonSingleExposureInterface");
#endif

	if (pylon_interface_->initialize(params_) != 0) {
		ROS_ERROR("Error while initializing the Pylon Interface");
		ros::shutdown();
		return;
	}
	if (pylon_interface_->setupCameraConfiguration(params_) != 0) {
		ROS_ERROR("Error while setup the image aquisation config");
		ros::shutdown();
		return;
	}

	// Framrate Settings
	if(pylon_interface_->max_possible_framerate() < params_.desired_frame_rate_){
		ROS_INFO("Desired framerate %.2f is higher than max possible. Will limit framerate to: %.2f Hz",params_.desired_frame_rate_,pylon_interface_->max_possible_framerate());
		params_.desired_frame_rate_ = pylon_interface_->max_possible_framerate();
		nh_.setParam("desired_framerate", pylon_interface_->max_possible_framerate());
	}else if(params_.desired_frame_rate_ == -1){
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

//	cam_info_msg_.D = {0.0,0.0,0.0,0.0,0.0};
//	std::vector<double>(5, 0.0);
//	boost::array<double, 9> K;
//	std::fill(K.data(), K.data() + K.size(), 0.0);
//	K.at<double>()
//	cam_info_msg_.K = 0.0;
//	boost::array<double, 12> P;
//	std::fill(P.data(), P.data() + P.size(), 0.0);
//	cam_info_msg_.P = 0.0;
//	cam_info_msg_.R = 0.0;
//	cam_info_msg_.binning_x =
//	cam_info_msg_.binning_y =
//	cam_info_msg_.roi =


#if WITH_OPENCV
	cv_img_raw_.header = header;
	// Encoding of pixels -- channel meaning, ordering, size
	// taken from the list of strings in include/sensor_msgs/image_encodings.h
	cv_img_raw_.encoding = pylon_interface_->img_encoding();
	cv_img_raw_.height = pylon_interface_->img_rows();
	cv_img_raw_.width = pylon_interface_->img_cols();
	// step = full row length in bytes
	cv_img_raw_.step = cv_img_raw_.width * pylon_interface_->img_pixel_depth();
	// img_raw_msg_.data // actual matrix data, size is (step * rows)
	img_size_byte_ = cv_img_raw_.step * cv_img_raw_.height;
#else
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
#endif

}
void PylonCameraNode::grabbingCallback() {

	const uint8_t* img(pylon_interface_->grab(params_));
	if (img == NULL) {
		ROS_ERROR("Pylon Interface returned NULL-Pointer!");
		return;
	}

#if WITH_OPENCV
// Not implemented
#else
	img_raw_msg_.data = std::vector<uint8_t>(img, img + img_size_byte_);
#endif

	img_raw_msg_.header.stamp = ros::Time::now();
	cam_info_msg_.header.stamp = img_raw_msg_.header.stamp;

	// Publish via image_transport
	img_raw_pub_.publish(img_raw_msg_, cam_info_msg_);
}
bool PylonCameraNode::setExposureCallback(pylon_camera_msgs::SetExposureSrv::Request &req, pylon_camera_msgs::SetExposureSrv::Response &res) {
	if(pylon_interface_->setExposure(req.target_exposure)){
		res.success = false;
	} else {
		res.success = true;
		params_.exposure_ = req.target_exposure;
		nh_.setParam("exposure", params_.exposure_);
	}
	return res.success;
}
int main(int argc, char **argv) {

	ros::init(argc, argv, "pylon_camera_node");

#if WITH_OPENCV
	ROS_INFO("PylonCameraNode running WITH OpenCV");
#else
	ROS_INFO("PylonCameraNode running WITHOUT OpenCV");
#endif

	PylonCameraNode pylon_camera_node;
	pylon_camera_node.nh_ = ros::NodeHandle("~");

	pylon_camera_node.setInitialCameraParameter();

	//ROS_INFO("Got all desired camera parameter");

	pylon_camera_node.init();

	pylon_camera_node.setRuntimeCameraParameter();

	ros::Rate r(pylon_camera_node.params_.desired_frame_rate_);


	pylon_camera_msgs::SetExposureSrv set_exp_srv;

	if(pylon_camera_node.params_.use_trigger_service_){
		ROS_INFO("Start image grabbing in service trigger mode with framerate: %.2f Hz", pylon_camera_node.params_.desired_frame_rate_);
	}else{
		ROS_INFO("Start image grabbing if node connects to topic with framerate: %.2f Hz", pylon_camera_node.params_.desired_frame_rate_);
	}

	int params_update_counter = 0;
	while (ros::ok()) {
		if(pylon_camera_node.params_.use_trigger_service_){

		} else if(pylon_camera_node.img_raw_pub_.getNumSubscribers() > 0){

			// Update all possible runtime parameter (exposure, brightness, etc) every param_update_frequency_ cycles
			params_update_counter++;
			if(params_update_counter%pylon_camera_node.params_.param_update_frequency_ == 0){
				ROS_INFO("Updating runtime parameter (update frequency is %d cycles)", pylon_camera_node.params_.param_update_frequency_);
				pylon_camera_node.setRuntimeCameraParameter();
				params_update_counter = 0;
				if(pylon_camera_node.pylon_interface_->setExposure(pylon_camera_node.params_.exposure_)){
					ROS_ERROR("Error while updating the runtime parameter!");
				}
			}

			// main grabbing function:
			pylon_camera_node.grabbingCallback();
		}
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
