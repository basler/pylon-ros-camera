/*
 * pylon_camera_opencv_node.h
 *
 *  Created on: Jun 10, 2015
 *      Author: md
 */

#ifndef PYLON_CAMERA_OPENCV_NODE_H_
#define PYLON_CAMERA_OPENCV_NODE_H_

#include <opencv2/opencv.hpp>
#include <pylon_camera/pylon_camera_node.h>
#include <pylon_camera/image_rectifier.h>
#include <pylon_camera/pylon_sequencer_interface.h>
#include <pylon_camera/intrinsic_calib_loader.h>

namespace pylon_camera {

class PylonCameraOpenCVNode: public PylonCameraNode {
public:
	PylonCameraOpenCVNode();
	virtual ~PylonCameraOpenCVNode();

	void setInitialCameraParameter();
	void createPylonInterface();
	int init();

	void setupCameraInfoMsg();
	uint32_t getNumSubscribers();
	uint32_t getNumSubscribersRaw();
	uint32_t getNumSubscribersRect();
	uint32_t getNumSubscribersSeq();

	cv_bridge::CvImage cv_img_rect_;
	ros::Publisher* img_rect_pub_;

	cv_bridge::CvImage cv_img_seq_;
	ros::Publisher* img_seq_pub_;

	ImageRectifier img_rectifier_;
	IntrinsicCalibLoader* calib_loader_;
	const uint8_t* grabbingCallback();

};
} /* namespace pylon_camera */
#endif /* PYLON_CAMERA_OPENCV_NODE_H_ */

