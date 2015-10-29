#ifndef PYLON_CAMERA_NODE_H_
#define PYLON_CAMERA_NODE_H_

#include <boost/thread.hpp>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <pylon_camera/pylon_camera_parameter.h>
#include <pylon_camera/pylon_camera.h>
#include <pylon_camera_msgs/SetExposureSrv.h>
#include <pylon_camera_msgs/SetBrightnessSrv.h>
#include <pylon_camera_msgs/SetSleepingSrv.h>
#include <pylon_camera_msgs/GrabSequenceAction.h>
#include <pylon_camera_msgs/SequenceExposureTimes.h>

namespace pylon_camera
{

class PylonCameraNode
{
public:
    PylonCameraNode();
    virtual ~PylonCameraNode();

    bool init();
    void getInitialCameraParameter();
    virtual void spin();
    const double& desiredFrameRate() const;

protected:

    bool initAndRegister();
    bool startGrabbing();
    void updateAquisitionSettings();
    uint32_t getNumSubscribers() const;
    virtual bool grabImage();
    virtual bool grabSequence();

    bool setExposureCallback(pylon_camera_msgs::SetExposureSrv::Request &req,
                             pylon_camera_msgs::SetExposureSrv::Response &res);
    bool setBrightnessCallback(pylon_camera_msgs::SetBrightnessSrv::Request &req,
                               pylon_camera_msgs::SetBrightnessSrv::Response &res);
    bool setSleepingCallback(pylon_camera_msgs::SetSleepingSrv::Request &req,
                             pylon_camera_msgs::SetSleepingSrv::Response &res);
    bool have_intrinsic_data();
    bool is_sleeping();
    void checkForPylonAutoFunctionRunning();

    virtual bool brightnessValidation(int target);
    virtual int calcCurrentBrightness();
    virtual float getCurrenCurrentExposure();

    void sequenceRawActionExecuteCB(const pylon_camera_msgs::GrabSequenceGoal::ConstPtr& goal);

    ros::NodeHandle nh_;

    PylonCamera* pylon_camera_;
    PylonCameraParameter params_;

    image_transport::ImageTransport* it_;
    image_transport::CameraPublisher img_raw_pub_;
    ros::Publisher exp_times_pub_;

    actionlib::SimpleActionServer<pylon_camera_msgs::GrabSequenceAction> sequence_raw_as_;

    ros::ServiceServer set_exposure_service_;
    ros::ServiceServer set_brightness_service_;
    ros::ServiceServer set_sleeping_service_;

    sensor_msgs::Image img_raw_msg_;
    sensor_msgs::CameraInfo cam_info_msg_;
    pylon_camera_msgs::SequenceExposureTimes exp_times_;

    bool brightness_service_running_;
    int target_brightness_;
    bool is_sleeping_;
    boost::recursive_mutex grab_mutex_;

};

}

#endif
