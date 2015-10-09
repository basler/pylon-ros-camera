#ifndef PYLON_CAMERA_OPENCV_NODE_H_
#define PYLON_CAMERA_OPENCV_NODE_H_

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pylon_camera_msgs/SequenceExposureTimes.h>

#include <pylon_camera/pylon_camera_node.h>
#include <pylon_camera/image_rectifier.h>
#include <pylon_camera/pylon_opencv_interface.h>
#include <pylon_camera/intrinsic_calib_loader.h>
#include <pylon_camera/hdr_generator.h>

namespace pylon_camera
{

class PylonCameraOpenCVNode : public PylonCameraNode
{
public:
    PylonCameraOpenCVNode();
    virtual ~PylonCameraOpenCVNode();

    bool init();
    void createPylonInterface();
    void getInitialCameraParameter();
    void setupCameraInfoMsg();
    uint32_t getNumSubscribers() const;
    uint32_t getNumSubscribersRaw() const;
    uint32_t getNumSubscribersRect() const;
    uint32_t getNumSubscribersSeq() const;
    uint32_t getNumSubscribersHdr() const;
    bool grabImage();
    bool grabSequence();

    PylonOpenCVInterface pylon_opencv_interface_;

    cv_bridge::CvImage cv_img_rect_;
    cv_bridge::CvImage cv_img_seq_;
    cv_bridge::CvImage cv_img_hdr_;
    pylon_camera_msgs::SequenceExposureTimes exp_times_;

    ros::Publisher img_rect_pub_;
    ros::Publisher img_seq_pub_;
    ros::Publisher img_hdr_pub_;
    ros::Publisher exp_times_pub_;

private:

    ImageRectifier img_rectifier_;
    IntrinsicCalibLoader calib_loader_;

    cv::Mat img_raw_;

    std::vector<cv::Mat> image_sequence_;

    HDRGenerator hdr_generator_;
    cv::Mat hdr_img_;

};

}

#endif
