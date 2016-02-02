// Copyright 2015 <Magazino GmbH>

#ifndef PYLON_CAMERA_PYLON_CAMERA_NODE_H
#define PYLON_CAMERA_PYLON_CAMERA_NODE_H

#include <boost/thread.hpp>
#include <string>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <camera_control_msgs/SetBool.h>

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

    /**
     * initialize the camera and the ros node.
     * @return false if an error occurred.
     */
    bool init();

    /**
     * spin the node
     */
    virtual void spin();

    /**
     * Getter for the desired frame rate.
     * @return the desired frame rate.
     */
    const double& desiredFrameRate() const;

    /**
     * Getter for the tf frame.
     * @return the camera frame.
     */
    const std::string& cameraFrame() const;

protected:
    /**
     * Creates the camera instance and starts the services and action servers.
     * @return false if an error occurred
     */
    bool initAndRegister();

    /**
     * Start the camera and initialize the messages
     * @return
     */
    bool startGrabbing();

    /**
     * Returns the total number of subscribers on any advertised image topic.
     */
    uint32_t getNumSubscribers() const;

    /**
     * Grabs an image and stores the image in img_raw_msg_
     * @return false if an error occurred.
     */
    virtual bool grabImage();

    /**
     * Fills the ros CameraInfo-Object with the image dimensions
     */
    virtual void setupCameraInfo(sensor_msgs::CameraInfo& cam_info_msg);

    /**
     * Update the exposure value on the camera
     * @param target_exposure the targeted exposure
     * @param reached_exposure the exposure that could be reached
     * @return true if the targeted exposure could be reached
     */
    bool setExposure(const float& target_exposure, float& reached_exposure);

    /**
     * Service callback for setting the exposure
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setExposureCallback(camera_control_msgs::SetExposureSrv::Request &req,
                             camera_control_msgs::SetExposureSrv::Response &res);

    /**
     * Set the target brightness value of the published images.
     * This is the intensity-mean over all pixels.
     * The autofunction of the Pylon-API supports values from [50 - 205].
     * Using a binary search, this range will be extended up to [1 - 254].
     * @param target_brightness
     * @param reached_brightness
     * @return true on success
     */
    bool setBrightness(const int& target_brightness, int& reached_brightness);

    /**
     * Service callback for setting the brightness
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setBrightnessCallback(camera_control_msgs::SetBrightnessSrv::Request &req,
                               camera_control_msgs::SetBrightnessSrv::Response &res);

    /**
     * Callback that puts the camera to sleep
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setSleepingCallback(camera_control_msgs::SetSleepingSrv::Request &req,
                             camera_control_msgs::SetSleepingSrv::Response &res);

    /**
     * Returns true if the camera was put into sleep mode
     * @return true if in sleep mode
     */
    bool isSleeping();

    /**
     * Checks if the auto brightness function is running
     */
    void checkForPylonAutoFunctionRunning();

    /**
     * Calculates the mean brightness of the image
     * @return the mean brightness of the image
     */
    virtual float calcCurrentBrightness();

    /**
     * Getter for the current exposure
     * @return the current exposure
     */
    virtual float getCurrentExposure();

    /**
     * Callback for the grab images action
     * @param goal the goal
     */
    void grabImagesRawActionExecuteCB(const camera_control_msgs::GrabImagesGoal::ConstPtr& goal);

    /**
     * Callback that sets the digital output
     * @param output_id the ID of the output to set
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setDigitalOutputCB(const int& output_id,
                            camera_control_msgs::SetBool::Request &req,
                            camera_control_msgs::SetBool::Response &res);

    /**
     * Waits till the pylon_camera_ isReady() observing a given timeout
     * @return true when the camera's state toggles to 'isReady()'
     */
    bool waitForCamera(const ros::Duration& timeout) const;

    ros::NodeHandle nh_;

    PylonCamera* pylon_camera_;
    PylonCameraParameter pylon_camera_parameter_set_;

    image_transport::ImageTransport* it_;
    image_transport::CameraPublisher img_raw_pub_;

    GrabImagesAction grab_images_raw_action_server_;

    ros::ServiceServer set_exposure_service_;
    ros::ServiceServer set_brightness_service_;
    ros::ServiceServer set_sleeping_service_;
    ros::ServiceServer set_digital_output_1_service_;

    sensor_msgs::Image img_raw_msg_;
    sensor_msgs::CameraInfo cam_info_msg_;

    bool brightness_service_running_;
    int target_brightness_;
    bool is_sleeping_;
    boost::recursive_mutex grab_mutex_;
};

}  // namespace pylon_camera

#endif  // PYLON_CAMERA_PYLON_CAMERA_NODE_H
