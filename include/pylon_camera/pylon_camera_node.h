/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (C) 2016, Magazino GmbH. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Magazino GmbH nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#ifndef PYLON_CAMERA_PYLON_CAMERA_NODE_H
#define PYLON_CAMERA_PYLON_CAMERA_NODE_H

#include <boost/thread.hpp>
#include <string>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <pylon_camera/pylon_camera_parameter.h>
#include <pylon_camera/pylon_camera.h>

#include <camera_control_msgs/SetBool.h>
#include <camera_control_msgs/SetBinning.h>
#include <camera_control_msgs/SetBrightness.h>
#include <camera_control_msgs/SetExposure.h>
#include <camera_control_msgs/SetGain.h>
#include <camera_control_msgs/SetGamma.h>
#include <camera_control_msgs/SetSleeping.h>
#include <camera_control_msgs/GrabImagesAction.h>
// ###################################### Deprecated !
#include <camera_control_msgs/SetBrightnessSrv.h>
#include <camera_control_msgs/SetExposureSrv.h>
#include <camera_control_msgs/SetSleepingSrv.h>
// ###################################### Deprecated !

namespace pylon_camera
{

typedef actionlib::SimpleActionServer<camera_control_msgs::GrabImagesAction> GrabImagesAS;

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
     * Getter for the frame rate set by the launch script or from the ros parameter
     * server
     * @return the desired frame rate.
     */
    const double& frameRate() const;

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
     * Update the horizontal binning_x factor to get downsampled images
     * @param target_binning_x the target horizontal binning_x factor
     * @param reached_binning_x the horizontal binning_x factor that could be
     *        reached
     * @return true if the targeted binning could be reached
     */
    bool setBinningX(const size_t& target_binning_x,
                     size_t& reached_binning_x);

    /**
     * Update the vertical binning_y factor to get downsampled images
     * @param target_binning_y the target vertical binning_y factor
     * @param reached_binning_y the vertical binning_y factor that could be
     *        reached
     * @return true if the targeted binning could be reached
     */
    bool setBinningY(const size_t& target_binning_y,
                     size_t& reached_binning_y);

    /**
     * Service callback for updating the cameras binning setting
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setBinningCallback(camera_control_msgs::SetBinning::Request &req,
                            camera_control_msgs::SetBinning::Response &res);
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
    bool setExposureCallback(camera_control_msgs::SetExposure::Request &req,
                             camera_control_msgs::SetExposure::Response &res);

    /** DEPRECATED handling
     * Service callback for setting the exposure
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setExposureCallbackDeprecated(camera_control_msgs::SetExposureSrv::Request &req,
                                       camera_control_msgs::SetExposureSrv::Response &res);
    /**
     * Sets the target brightness which is the intensity-mean over all pixels.
     * If the target exposure time is not in the range of Pylon's auto target
     * brightness range the extended brightness search is started.
     * The Auto function of the Pylon-API supports values from [50 - 205].
     * Using a binary search, this range will be extended up to [1 - 255].
     * @param target_brightness is the desired brightness. Range is [1...255].
     * @param current_brightness is the current brightness with the given settings.
     * @param exposure_auto flag which indicates if the target_brightness
     *                      should be reached adapting the exposure time
     * @param gain_auto flag which indicates if the target_brightness should be
     *                      reached adapting the gain.
     * @return true if the brightness could be reached or false otherwise.
     */
    bool setBrightness(const int& target_brightness,
                       int& reached_brightness,
                       const bool& exposure_auto,
                       const bool& gain_auto);

    /**
     * Service callback for setting the brightness
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setBrightnessCallback(camera_control_msgs::SetBrightness::Request &req,
                               camera_control_msgs::SetBrightness::Response &res);

    /**
     * Service callback for setting the brightness
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setBrightnessCallbackDeprecated(camera_control_msgs::SetBrightnessSrv::Request &req,
                                         camera_control_msgs::SetBrightnessSrv::Response &res);

    /**
     * Update the gain from the camera to a target gain in percent
     * @param target_gain the targeted gain in percent
     * @param reached_gain the gain that could be reached
     * @return true if the targeted gain could be reached
     */
    bool setGain(const float& target_gain, float& reached_gain);

    /**
     * Service callback for setting the desired gain in percent
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setGainCallback(camera_control_msgs::SetGain::Request &req,
                         camera_control_msgs::SetGain::Response &res);

    /**
     * Update the gamma from the camera to a target gamma correction value
     * @param target_gamma the targeted gamma
     * @param reached_gamma the gamma that could be reached
     * @return true if the targeted gamma could be reached
     */
    bool setGamma(const float& target_gamma, float& reached_gamma);

    /**
     * Service callback for setting the desired gamma correction value
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setGammaCallback(camera_control_msgs::SetGamma::Request &req,
                         camera_control_msgs::SetGamma::Response &res);

    /**
     * Callback that puts the camera to sleep
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setSleepingCallback(camera_control_msgs::SetSleeping::Request &req,
                             camera_control_msgs::SetSleeping::Response &res);

    /**
     * Callback that puts the camera to sleep
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setSleepingCallbackDeprecated(camera_control_msgs::SetSleepingSrv::Request &req,
                                       camera_control_msgs::SetSleepingSrv::Response &res);

    /**
     * Returns true if the camera was put into sleep mode
     * @return true if in sleep mode
     */
    bool isSleeping();

    /**
     * Calculates the mean brightness of the image
     * @return the mean brightness of the image
     */
    float calcCurrentBrightness();

    /**
     * Callback for the grab images action
     * @param goal the goal
     */
    void grabImagesRawActionExecuteCB(const camera_control_msgs::GrabImagesGoal::ConstPtr& goal);

    /**
     * This function can also be called from the derived PylonCameraOpenCV-Class
     */
    camera_control_msgs::GrabImagesResult grabImagesRaw(
                    const camera_control_msgs::GrabImagesGoal::ConstPtr& goal,
                    GrabImagesAS* action_server);

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

    GrabImagesAS grab_imgs_raw_as_;

    ros::ServiceServer set_binning_service_;
    ros::ServiceServer set_exposure_service_;
    ros::ServiceServer set_brightness_service_;
    ros::ServiceServer set_gain_service_;
    ros::ServiceServer set_gamma_service_;
    ros::ServiceServer set_sleeping_service_;
    ros::ServiceServer set_digital_output_1_service_;
    // ################### DEPRECATED !
    ros::ServiceServer set_exposure_service_deprecated_;
    ros::ServiceServer set_brightness_service_deprecated_;
    ros::ServiceServer set_sleeping_service_deprecated_;
    // ################### DEPRECATED !

    sensor_msgs::Image img_raw_msg_;
    sensor_msgs::CameraInfo cam_info_msg_;

    camera_info_manager::CameraInfoManager* camera_info_manager_;

    bool is_sleeping_;
    boost::recursive_mutex grab_mutex_;
};

}  // namespace pylon_camera

#endif  // PYLON_CAMERA_PYLON_CAMERA_NODE_H
