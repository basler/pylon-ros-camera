/*
 * pylon_camera_opencv_node.h
 *
 *  Created on: Jun 10, 2015
 *      Author: md
 */
#ifndef PYLON_CAMERA_NODE_H_
#define PYLON_CAMERA_NODE_H_

#include <iostream>
#include <memory>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/array.hpp>
#include <image_transport/image_transport.h>
//#include <camera_info_manager/camera_info_manager.h>

#include <pylon_camera/pylon_camera_parameter.h>
#include <pylon_camera/pylon_interface.h>

#include <pylon_camera_msgs/SetExposureSrv.h>
#include <pylon_camera_msgs/SetBrightnessSrv.h>
#include <pylon_camera_msgs/ImgGrabbingTrigger.h>

#ifdef WITH_OPENCV
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pylon_camera/image_rectifier.h>
#include <pylon_camera/pylon_sequencer_interface.h>
#endif

using std::cout;
using std::endl;
using namespace cv;

namespace pylon_camera
{

class PylonCameraNode
{
public:
  PylonCameraNode();
  virtual ~PylonCameraNode();
  ros::NodeHandle nh_;

  PylonInterface* pylon_interface_;
  PylonCameraParameter params_;

  sensor_msgs::Image img_raw_msg_;

  void setInitialCameraParameter();
  void setRuntimeCameraParameter();
  int init();
  uint32_t getNumSubscribers();
  void createPylonInterface();
  const uint8_t* grabbingCallback();

  ros::ServiceServer set_exposure_service_;
  ros::ServiceServer set_brightness_service_;
  bool setExposureCallback(pylon_camera_msgs::SetExposureSrv::Request &req,
                           pylon_camera_msgs::SetExposureSrv::Response &res);
  bool setBrightnessCallback(pylon_camera_msgs::SetBrightnessSrv::Request &req,
                             pylon_camera_msgs::SetBrightnessSrv::Response &res);

  image_transport::ImageTransport* it_;
  image_transport::CameraPublisher* img_raw_pub_;

//	camera_info_manager::CameraInfoManager* cam_info_manager_;
  sensor_msgs::CameraInfo cam_info_msg_;

  //ros::ServiceServer img_grabbing_server_;

  int img_size_byte_;
private:
//	bool imgGrabbingTriggerCallback();
};
} /* namespace pylon_camera */
#endif /* PYLON_CAMERA_NODE_H_ */
