/*
 * pylon_camera_opencv_node.h
 *
 *  Created on: Jun 10, 2015
 *      Author: md
 */

#ifndef PYLON_CAMERA_OPENCV_NODE_H_
#define PYLON_CAMERA_OPENCV_NODE_H_

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pylon_camera_msgs/SequenceExposureTimes.h>

#include <pylon_camera/pylon_camera_node.h>
#include <pylon_camera/image_rectifier.h>
#include <pylon_camera/pylon_opencv_interface.h>
#include <pylon_camera/intrinsic_calib_loader.h>

#if CV_MAJOR_VERSION > 2   // If you are using OpenCV 3
    #include <pylon_camera/hdr_generator.h>
#endif

namespace pylon_camera
{

class PylonCameraOpenCVNode : public PylonCameraNode
{
public:
    PylonCameraOpenCVNode();
    virtual ~PylonCameraOpenCVNode();

    cv_bridge::CvImage cv_img_rect_;
    cv_bridge::CvImage cv_img_seq_;
    cv_bridge::CvImage cv_img_hdr_;
    pylon_camera_msgs::SequenceExposureTimes exp_times_;

    ros::Publisher img_rect_pub_;
    ros::Publisher img_seq_pub_;
    ros::Publisher img_hdr_pub_;
    ros::Publisher exp_times_pub_;

    PylonOpenCVInterface pylon_opencv_interface_;

    bool init();
    void createPylonInterface();
    void getInitialCameraParameter();
    void setupCameraInfoMsg();
    uint32_t getNumSubscribers();
    uint32_t getNumSubscribersRaw();
    uint32_t getNumSubscribersRect();
    uint32_t getNumSubscribersSeq();
    uint32_t getNumSubscribersHdr();
    bool grabImage();
    bool grabSequence();

private:
    ImageRectifier img_rectifier_;
    IntrinsicCalibLoader calib_loader_;
};
} /* namespace pylon_camera */
#endif /* PYLON_CAMERA_OPENCV_NODE_H_ */

