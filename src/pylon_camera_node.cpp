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

	void setCameraParameter();
	void init();

	image_transport::ImageTransport* it_;
	image_transport::CameraPublisher img_raw_pub_;

//	camera_info_manager::CameraInfoManager* cam_info_manager_;
	sensor_msgs::CameraInfo cam_info_msg_;

	int img_size_byte_;
private:
};
PylonCameraNode::PylonCameraNode() :
		nh_("~"), pylon_interface_(NULL), it_(NULL), img_size_byte_(-1) {
}
void PylonCameraNode::setCameraParameter() {
	// Write Magazino cam id using the
	nh_.param<std::string>("magazino_cam_id", params_.magazino_cam_id_, "x");

	if (params_.magazino_cam_id_ != "x") {
		ROS_INFO("Using Camera: %s", params_.magazino_cam_id_.c_str());
	} else {
		ROS_INFO("No Magazino Cam ID set -> Will use the camera device found fist");
	}

	nh_.param<double>("desired_framerate", params_.desired_frame_rate_, 0.5);

	if (!(params_.desired_frame_rate_ > 0 || params_.desired_frame_rate_ == -1)) {
		ROS_ERROR("Unexpected framerate (%f). Setting to -1 (max possible)", params_.desired_frame_rate_);
	}

	nh_.param<std::string>("camera_frame", params_.camera_frame_, "base_link");
	nh_.param<bool>("use_hdr", params_.use_hdr_, false);

}
void PylonCameraNode::init() {
	it_ = new image_transport::ImageTransport(nh_);
	img_raw_pub_ = it_->advertiseCamera("image_raw", 10);

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
	}
	if (pylon_interface_->setupCameraConfiguration(params_) != 0) {
		ROS_ERROR("Error while setup the image aquisation config");
		ros::shutdown();
	}

	if(pylon_interface_->max_possible_framerate() < params_.desired_frame_rate_){
		ROS_INFO("Desired framerate %.1f is higher than max possible. Will limit framerate to: %.1f",params_.desired_frame_rate_,pylon_interface_->max_possible_framerate());
		params_.desired_frame_rate_ = pylon_interface_->max_possible_framerate();
		nh_.setParam("desired_framerate", pylon_interface_->max_possible_framerate());
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
int main(int argc, char **argv) {

	ros::init(argc, argv, "pylon_camera_node");

#if WITH_OPENCV
	ROS_INFO("PylonCameraNode running WITH OpenCV");
#else
	ROS_INFO("PylonCameraNode running WITHOUT OpenCV");
#endif

	PylonCameraNode pylon_camera_node;
	pylon_camera_node.nh_ = ros::NodeHandle("~");

	pylon_camera_node.setCameraParameter();
	ROS_INFO("Got all desired camera parameter");

	pylon_camera_node.init();

	ros::Rate r(pylon_camera_node.params_.desired_frame_rate_);

	ROS_INFO("Start image grabbing with framerate: %.2f", pylon_camera_node.params_.desired_frame_rate_);
	while (ros::ok()) {
		if(pylon_camera_node.img_raw_pub_.getNumSubscribers() > 0){

			const uint8_t* img(pylon_camera_node.pylon_interface_->grab(pylon_camera_node.params_));
			if (img == NULL) {
				ROS_ERROR("Pylon Interface returned NULL-Pointer!");
				ros::spinOnce();
				r.sleep();
				continue;
			}

#if WITH_OPENCV
		// Not implemented
#else
			pylon_camera_node.img_raw_msg_.data = std::vector<uint8_t>(img, img + pylon_camera_node.img_size_byte_);
#endif

			pylon_camera_node.img_raw_msg_.header.stamp = ros::Time::now();
			pylon_camera_node.cam_info_msg_.header.stamp = pylon_camera_node.img_raw_msg_.header.stamp;

			// Publish via image_transport
			pylon_camera_node.img_raw_pub_.publish(pylon_camera_node.img_raw_msg_, pylon_camera_node.cam_info_msg_);
		}
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
