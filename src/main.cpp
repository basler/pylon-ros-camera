/*
 * main.cpp
 *
 * main function of the pylon_node
 *
 *  Created on: May 10, 2015
 *      Author: md
 */

#include <ros/ros.h>
#include <pylon_camera/pylon_camera_node.h>
#include <pylon_camera/pylon_camera_parameter.h>

#ifdef WITH_OPENCV
	#include <pylon_camera/pylon_camera_opencv_node.h>
#endif

using namespace pylon_camera;

int main(int argc, char **argv) {

	ros::init(argc, argv, "pylon_camera_node");

#ifdef WITH_OPENCV
	ROS_INFO("PylonCameraNode running WITH OpenCV");
	PylonCameraOpenCVNode pylon_camera_node;
#else
	ROS_INFO("PylonCameraNode running WITHOUT OpenCV");
	PylonCameraNode pylon_camera_node;
#endif

	pylon_camera_node.nh_ = ros::NodeHandle("~");

	pylon_camera_node.setInitialCameraParameter();
	pylon_camera_node.createPylonInterface();
	if(pylon_camera_node.init()){
		ROS_ERROR("Error while initializing the pylon node!");
	}
	pylon_camera_node.setRuntimeCameraParameter();

	ros::Rate r(pylon_camera_node.params_.desired_frame_rate_);

	if (pylon_camera_node.params_.use_trigger_service_) {
		ROS_INFO("Start image grabbing in service trigger mode with framerate: %.2f Hz", pylon_camera_node.params_.desired_frame_rate_);
	} else {
		ROS_INFO("Start image grabbing if node connects to topic with framerate: %.2f Hz", pylon_camera_node.params_.desired_frame_rate_);
	}

	int params_update_counter = 0;

	while (ros::ok()) {
		if(pylon_camera_node.getNumSubscribers() > 0){
			// Update all possible runtime parameter (exposure, brightness, etc) every param_update_frequency_ cycles
			params_update_counter++;
			if (params_update_counter % pylon_camera_node.params_.param_update_frequency_ == 0) {
				ROS_INFO("Updating runtime parameter (update frequency is %d cycles)", pylon_camera_node.params_.param_update_frequency_);
				pylon_camera_node.setRuntimeCameraParameter();
				params_update_counter = 0;
				if (pylon_camera_node.params_.use_brightness_) {
					if (pylon_camera_node.pylon_interface_->last_brightness_val() != pylon_camera_node.params_.brightness_) {
						if (pylon_camera_node.pylon_interface_->setBrightness(pylon_camera_node.params_.brightness_)) {
							ROS_ERROR("Error while updating brightness!");
						}
						pylon_camera_node.params_.brightness_ = pylon_camera_node.pylon_interface_->last_brightness_val();
						pylon_camera_node.nh_.setParam("brightness", pylon_camera_node.params_.brightness_);
					}
				} else {
					if (pylon_camera_node.pylon_interface_->last_exposure_val() != pylon_camera_node.params_.exposure_) {
						if (pylon_camera_node.pylon_interface_->setExposure(pylon_camera_node.params_.exposure_)) {
							ROS_ERROR("Error while updating exposure!");
						}
						pylon_camera_node.params_.exposure_ = pylon_camera_node.pylon_interface_->last_exposure_val();
						pylon_camera_node.nh_.setParam("exposure", pylon_camera_node.params_.exposure_);
					}
				}
			}
			pylon_camera_node.grabbingCallback();

#ifdef WITH_OPENCV
			if(pylon_camera_node.getNumSubscribersRaw() > 0) {
				// Publish via image_transport
				pylon_camera_node.img_raw_pub_->publish(pylon_camera_node.img_raw_msg_, pylon_camera_node.cam_info_msg_);
			}
			if(pylon_camera_node.getNumSubscribersRect() > 0) {
				// Publish via normal publisher
				pylon_camera_node.img_rect_pub_->publish(pylon_camera_node.cv_img_rect_);
			}
			if(pylon_camera_node.params_.use_sequencer_ && pylon_camera_node.getNumSubscribersSeq() > 0){
				pylon_camera_node.img_seq_pub_->publish(pylon_camera_node.cv_img_seq_);
			}
#else
			pylon_camera_node.img_raw_pub_.publish(pylon_camera_node.img_raw_msg_, pylon_camera_node.cam_info_msg_);
#endif

		}
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}
