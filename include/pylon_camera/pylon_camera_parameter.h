/*
 * pylon_camera_parameter.h
 *
 *  Created on: May 21, 2015
 *      Author: md
 */

#ifndef PYLON_CAMERA_PARAMETER_H_
#define PYLON_CAMERA_PARAMETER_H_

#include <string>
#include <opencv2/opencv.hpp>

namespace pylon_camera {

class PylonCameraParameter {
public:
	PylonCameraParameter();
	virtual ~PylonCameraParameter();

	std::string magazino_cam_id_;
    std::string camera_frame_;
    double desired_frame_rate_;
    double target_exposure_;
    bool use_hdr_;
    int param_update_frequency_;
	double exposure_;
	bool use_brightness_;
	int brightness_;
	bool use_trigger_service_;
};

} /* namespace pylon_camera */

#endif /* PYLON_CAMERA_PARAMETER_H_ */
