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
#include <camera_control_msgs/SetExposureSrv.h>
#include <camera_control_msgs/SetBrightnessSrv.h>
#include <camera_control_msgs/SetSleepingSrv.h>
#include <camera_control_msgs/GrabImagesAction.h>

namespace pylon_camera
{

typedef actionlib::SimpleActionServer<camera_control_msgs::GrabImagesAction> GrabImagesAction;

class PylonCameraNode
{
public:
    PylonCameraNode();
    virtual ~PylonCameraNode();

    bool init();
    virtual void spin();
    const double& desiredFrameRate() const;

protected:

    bool initAndRegister();
    bool startGrabbing();
    void updateAquisitionSettings();
    uint32_t getNumSubscribers() const;
    virtual bool grabImage();
    virtual bool grabSequence();

    bool setExposure(const float& target_exposure, float& reached_exposure);
    bool setExposureCallback(camera_control_msgs::SetExposureSrv::Request &req,
                             camera_control_msgs::SetExposureSrv::Response &res);
    bool setBrightness(const int& target_brightness, int& reached_brightness);
    bool setBrightnessCallback(camera_control_msgs::SetBrightnessSrv::Request &req,
                               camera_control_msgs::SetBrightnessSrv::Response &res);
    bool setSleepingCallback(camera_control_msgs::SetSleepingSrv::Request &req,
                             camera_control_msgs::SetSleepingSrv::Response &res);
    bool is_sleeping();
    void checkForPylonAutoFunctionRunning();

    virtual bool brightnessValidation(int target);
    virtual int calcCurrentBrightness();
    virtual float getCurrentExposure();

    void grabImagesRawActionExecuteCB(const camera_control_msgs::GrabImagesGoal::ConstPtr& goal);

    ros::NodeHandle nh_;

    PylonCamera* pylon_camera_;
    PylonCameraParameter pylon_camera_parameter_set_;

    image_transport::ImageTransport* it_;
    image_transport::CameraPublisher img_raw_pub_;

    GrabImagesAction grab_images_raw_action_server_;

    ros::ServiceServer set_exposure_service_;
    ros::ServiceServer set_brightness_service_;
    ros::ServiceServer set_sleeping_service_;

    sensor_msgs::Image img_raw_msg_;
    sensor_msgs::CameraInfo cam_info_msg_;

    bool brightness_service_running_;
    int target_brightness_;
    bool is_sleeping_;
    boost::recursive_mutex grab_mutex_;

};

}

#endif
