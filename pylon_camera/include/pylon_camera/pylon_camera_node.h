/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (C) 2016, Magazino GmbH. All rights reserved.
 *
 * Improved by drag and bot GmbH (www.dragandbot.com), 2019
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
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <pylon_camera/pylon_camera_parameter.h>
#include <pylon_camera/pylon_camera.h>

#include <camera_control_msgs/SetBinning.h>
#include <camera_control_msgs/SetBrightness.h>
#include <camera_control_msgs/SetExposure.h>
#include <camera_control_msgs/SetGain.h>
#include <camera_control_msgs/SetGamma.h>
#include <camera_control_msgs/SetROI.h>
#include <camera_control_msgs/SetSleeping.h>
#include <camera_control_msgs/SetIntegerValue.h>
#include <camera_control_msgs/SetFloatValue.h>
#include <camera_control_msgs/SetStringValue.h>
#include <camera_control_msgs/currentParams.h>
#include <camera_control_msgs/SetWhiteBalance.h>

#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include <camera_control_msgs/GrabImagesAction.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <dnb_msgs/ComponentStatus.h>


namespace pylon_camera
{

typedef actionlib::SimpleActionServer<camera_control_msgs::GrabImagesAction> GrabImagesAS;

/**
 * The ROS-node of the pylon_camera interface
 */
class PylonCameraNode
{
public:

    PylonCameraNode();
    virtual ~PylonCameraNode();

    /**
     * initialize the camera and the ros node.
     * calls ros::shutdown if an error occurs.
     */
    void init();

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
     * Initializing of img_rect_pub_, grab_img_rect_as_ and the pinhole_model_,
     * in case that a valid camera info has been set
     * @return
     */
    void setupRectification();

    /**
     * Returns the total number of subscribers on any advertised image topic.
     */
    uint32_t getNumSubscribers() const;

    /**
     * Returns the number of subscribers for the raw image topic
     */
    uint32_t getNumSubscribersRaw() const;

    /**
     * Returns the number of subscribers for the rect image topic
     */
    uint32_t getNumSubscribersRect() const;

    /**
     * Grabs an image and stores the image in img_raw_msg_
     * @return false if an error occurred.
     */
    virtual bool grabImage();

    /**
     * Fills the ros CameraInfo-Object with the image dimensions
     */
    virtual void setupInitialCameraInfo(sensor_msgs::CameraInfo& cam_info_msg);

    /**
     * Update area of interest in the camera image
     * @param target_roi the target roi
     * @param reached_roi the roi that could be set
     * @return true if the targeted roi could be reached
     */
    bool setROI(const sensor_msgs::RegionOfInterest target_roi,
                sensor_msgs::RegionOfInterest& reached_roi);
    
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
     * Service callback for updating the cameras roi setting
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setROICallback(camera_control_msgs::SetROI::Request &req,
			camera_control_msgs::SetROI::Response &res);
    
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
     * Returns true if the camera was put into sleep mode
     * @return true if in sleep mode
     */
    bool isSleeping();

    /**
     * Generates the subset of points on which the brightness search will be
     * executed in order to speed it up. The subset are the indices of the
     * one-dimensional image_raw data vector. The base generation is done in a
     * recursive manner, by calling genSamplingIndicesRec
     * @return indices describing the subset of points
     */
    void setupSamplingIndices(std::vector<std::size_t>& indices,
                              std::size_t rows,
                              std::size_t cols,
                              int downsampling_factor);

    /**
     * This function will recursively be called from above setupSamplingIndices()
     * to generate the indices of pixels given the actual ROI.
     * @return indices describing the subset of points
     */
    void genSamplingIndicesRec(std::vector<std::size_t>& indices,
                               const std::size_t& min_window_height,
                               const cv::Point2i& start,
                               const cv::Point2i& end);

    /**
     * Calculates the mean brightness of the image based on the subset indices
     * @return the mean brightness of the image
     */
    float calcCurrentBrightness();

    /**
     * Callback for the grab images action
     * @param goal the goal
     */
    void grabImagesRawActionExecuteCB(
                    const camera_control_msgs::GrabImagesGoal::ConstPtr& goal);

    /**
     * Callback for the grab rectified images action
     * @param goal the goal
     */
    void grabImagesRectActionExecuteCB(
                    const camera_control_msgs::GrabImagesGoal::ConstPtr& goal);
    /**
     * This function can also be called from the derived PylonCameraOpenCV-Class
     */
    camera_control_msgs::GrabImagesResult grabImagesRaw(
                    const camera_control_msgs::GrabImagesGoal::ConstPtr& goal,
                    GrabImagesAS* action_server);

    void initCalibrationMatrices(sensor_msgs::CameraInfo& info,
                                 const cv::Mat& D,
                                 const cv::Mat& K);

    /**
     * Callback that sets the digital user output
     * @param output_id the ID of the user output to set
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setUserOutputCB(int output_id,
                         std_srvs::SetBool::Request &req,
                         std_srvs::SetBool::Response &res);

    /**
     * Callback that activates the digital user output to
      be used as autoflash
     * @param output_id the ID of the user output to set
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setAutoflash(const int output_id,
                      std_srvs::SetBool::Request &req,
                      std_srvs::SetBool::Response &res);


    /**
     * Waits till the pylon_camera_ isReady() observing a given timeout
     * @return true when the camera's state toggles to 'isReady()'
     */
    bool waitForCamera(const ros::Duration& timeout) const;

    /**
     * Service callback for setting camera x-axis offset 
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setOffsetXCallback(camera_control_msgs::SetIntegerValue::Request &req, camera_control_msgs::SetIntegerValue::Response &res);

    /**
     * Service callback for setting camera y-axis offset 
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setOffsetYCallback(camera_control_msgs::SetIntegerValue::Request &req, camera_control_msgs::SetIntegerValue::Response &res);

    /**
     * Method to set the offset in the camera x-axis or y-axis.  
     * @param offsetValue the targeted offset value.
     * @param xAxis when true set the oddset in the x-axis, otherwise in the y-axis.
     * @return error message if an error occurred or done message otherwise.
     */
    std::string setOffsetXY(const int& offsetValue, bool xAxis);

    /**
     * Service callback for reversing X
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setReverseXCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

    /**
     * Service callback for reversing Y
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setReverseYCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

    /**
     * reverse X, Y on the camera
     * @param reverse_x reverse the image around x-axis
     * @param reverse_y reverse the image around y-axis
     * @return error message if an error occurred or done message otherwise.
     */
    std::string reverseXY(const bool& data, bool around_x);

    /**
     * Service callback for increasing/decreasing the image black level 
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setBlackLevelCallback(camera_control_msgs::SetIntegerValue::Request &req, camera_control_msgs::SetIntegerValue::Response &res);

    /**
     * method for increasing/decreasing the image black level 
     * @param value the new black level value
     * @return error message if an error occurred or done message otherwise.
     */
    std::string setBlackLevel(const int& value);

    /**
     * Service callback for setting the PGI mode 
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setPGIModeCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

    /**
     * method for setting the PGI mode. 
     * @param on : true = on, false = off.
     * @return error message if an error occurred or done message otherwise.
     */
    std::string setPGIMode(const bool& on);

    /**
     * Service callback for setting the demosaicing mode 
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setDemosaicingModeCallback(camera_control_msgs::SetIntegerValue::Request &req, camera_control_msgs::SetIntegerValue::Response &res);

    /**
     * method for setting the demosaicing mode. 
     * @param mode : 0 = simple, 1 = Basler PGI.
     * @return error message if an error occurred or done message otherwise.
     */
    std::string setDemosaicingMode(const int& mode);

    /**
     * Service callback for setting the noise reduction value 
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setNoiseReductionCallback(camera_control_msgs::SetFloatValue::Request &req, camera_control_msgs::SetFloatValue::Response &res);

    /**
     * method for setting the noise reduction value 
     * @param value : targeted noise reduction value (range : 0.0 to 0.2).
     * @return error message if an error occurred or done message otherwise.
     */
    std::string setNoiseReduction(const float& value);

    /**
     * Service callback for setting the sharpness enhancement value. 
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setSharpnessEnhancementCallback(camera_control_msgs::SetFloatValue::Request &req, camera_control_msgs::SetFloatValue::Response &res);

    /**
     * method for setting the sharpness enhancement value. 
     * @param value : targeted sharpness enhancement value (range : 1.0 to 3.98438).
     * @return error message if an error occurred or done message otherwise.
     */
    std::string setSharpnessEnhancement(const float& value);

    /**
     * Service callback for setting the camera light source preset 
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setLightSourcePresetCallback(camera_control_msgs::SetIntegerValue::Request &req, camera_control_msgs::SetIntegerValue::Response &res);

    /**
     * Method to set the camera light source preset  
     * @param mode : 0 = off, 1 = Daylight5000K, 2 = Daylight6500K, 3 = Tungsten2800K
     * @return error message if an error occurred or done message otherwise.
     */
    std::string setLightSourcePreset(const int& mode);

    /**
     * Service callback for setting the camera balance white auto 
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setBalanceWhiteAutoCallback(camera_control_msgs::SetIntegerValue::Request &req, camera_control_msgs::SetIntegerValue::Response &res);

    /**
     * Method to set the camera balance white auto 
     * @param mode : 0 = off , 1 = once, 2 = continuous
     * @return error message if an error occurred or done message otherwise.
     */
    std::string setBalanceWhiteAuto(const int& mode);

    /**
     * Service callback for setting the sensor readout mode (Normal or Fast)
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setSensorReadoutModeCallback(camera_control_msgs::SetIntegerValue::Request &req, camera_control_msgs::SetIntegerValue::Response &res);

    /**
     * Method to set the sensor readout mode (Normal or Fast)
     * @param mode : 0 = normal , 1 = fast.
     * @return error message if an error occurred or done message otherwise.
     */
    std::string setSensorReadoutMode(const int& mode);

    /**
     * Service callback for setting the acquisition frame count
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setAcquisitionFrameCountCallback(camera_control_msgs::SetIntegerValue::Request &req, camera_control_msgs::SetIntegerValue::Response &res);

    /**
     * Method to set the camera acquisition frame count
     * @param frameCount trageted frame count.
     * @return error message if an error occurred or done message otherwise.
     */
    std::string setAcquisitionFrameCount(const int& frameCount);

    /**
     * Service callback for setting the trigger selector
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setTriggerSelectorCallback(camera_control_msgs::SetIntegerValue::Request &req, camera_control_msgs::SetIntegerValue::Response &res);

     /**
     * Method to set the trigger selector   
     * @param mode : 0 = Frame Start, 1 = Frame Burst Start (ace USB) / Acquisition Start (ace GigE)
     * @return error message if an error occurred or done message otherwise.
     */
    std::string setTriggerSelector(const int& mode);

    /**
     * Service callback for setting the trigger mode
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setTriggerModeCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

     /**
     * Method to set the trigger mode   
     * @param value : false = off, true = on
     * @return error message if an error occurred or done message otherwise.
     */
    std::string setTriggerMode(const bool& value);

    /**
     * Service callback for executing a software trigger
     * @param req request
     * @param res response
     * @return true on success
     */
    bool executeSoftwareTriggerCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

     /**
     * Method to execute a software trigger   
     * @return error message if an error occurred or done message otherwise.
     */
    std::string executeSoftwareTrigger();

    /**
     * Service callback for setting the camera trigger source
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setTriggerSourceCallback(camera_control_msgs::SetIntegerValue::Request &req, camera_control_msgs::SetIntegerValue::Response &res);

     /**
     * Method to set the camera trigger source.
     * @param source : 0 = software, 1 = Line1, 2 = Line3, 3 = Line4, 4 = Action1(only selected GigE Camera)  
     * @return error message if an error occurred or done message otherwise.
     */
    std::string setTriggerSource(const int& source);

    /**
     * Service callback for setting the camera trigger activation type
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setTriggerActivationCallback(camera_control_msgs::SetIntegerValue::Request &req, camera_control_msgs::SetIntegerValue::Response &res);

    /**
     * Method to set camera trigger activation type  
     * @param value : 0 = RigingEdge, 1 = FallingEdge
     * @return error message if an error occurred or done message otherwise.
     */
    std::string setTriggerActivation(const int& value);

    /**
     * Service callback for setting the camera trigger delay value
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setTriggerDelayCallback(camera_control_msgs::SetFloatValue::Request &req, camera_control_msgs::SetFloatValue::Response &res);

    /**
     * Method to set camera trigger delay value  
     * @param delayValue required dely value in µs 
     * @return error message if an error occurred or done message otherwise.
     */
    std::string setTriggerDelay(const float& value);

    /**
     * Service callback for setting the camera line selector
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setLineSelectorCallback(camera_control_msgs::SetIntegerValue::Request &req, camera_control_msgs::SetIntegerValue::Response &res);

    /**
     * Method to set camera line selector
     * @param value : 0 = line1, 1 = line2, 2 = line3, 3 = line4  
     * @return error message if an error occurred or done message otherwise.
     */
    std::string setLineSelector(const int& value);

    /**
     * Service callback for setting the camera line mode
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setLineModeCallback(camera_control_msgs::SetIntegerValue::Request &req, camera_control_msgs::SetIntegerValue::Response &res);

    /**
     * Method to set camera line Mode
     * @param value : 0 = input, 1 = output   
     * @return error message if an error occurred or done message otherwise.
     */
    std::string setLineMode(const int& value);

    /**
     * Service callback for setting the camera line source
     * @param req request
     * @param res response
     * @return true on success
     */
   bool setLineSourceCallback(camera_control_msgs::SetIntegerValue::Request &req, camera_control_msgs::SetIntegerValue::Response &res);

    /**
     * Method to set camera line source
     * @param value : 0 = exposure active  
     * @return error message if an error occurred or done message otherwise.
     */
    std::string setLineSource(const int& value);

    /**
     * Service callback for setting the camera line inverter
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setLineInverterCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

    /**
     * Method to set camera line inverter
     * @param value : ture = invert line
     * @return error message if an error occurred or done message otherwise.
     */
    std::string setLineInverter(const bool& value);

    /**
     * Service callback for setting the camera line debouncer time
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setLineDebouncerTimeCallback(camera_control_msgs::SetFloatValue::Request &req, camera_control_msgs::SetFloatValue::Response &res);

    /**
     * Method to set camera line debouncer time
     * @param value delay time in microseconds
     * @return error message if an error occurred or done message otherwise.
     */
    std::string setLineDebouncerTime(const float& value);

    /**
     * Service callback for setting the camera user set selector
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setUserSetSelectorCallback(camera_control_msgs::SetIntegerValue::Request &req, camera_control_msgs::SetIntegerValue::Response &res);

    /**
     * Method to set camera user set selector
     * @param set : 0 = Default, 1 = UserSet1, 2 = UserSet2, 3 = UserSet3, 4 = HighGain, 5 = AutoFunctions, 6 = ColorRaw
     * @return error message if an error occurred or done message otherwise.
     */
    std::string setUserSetSelector(const int& set);



    /**
     * Service callback for saving the user set
     * @param req request
     * @param res response
     * @return true on success
     */
    bool saveUserSetCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    /**
     * Method to save user set
     * @return error message if an error occurred or done message otherwise.
     */
    std::string saveUserSet();

    /**
     * Service callback for loading the user set
     * @param req request
     * @param res response
     * @return true on success
     */
    bool loadUserSetCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    /**
     * Method to load user set
     * @return error message if an error occurred or done message otherwise.
     */
    std::string loadUserSet();

    /**
     * Service callback for setting the camera user set default selector
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setUserSetDefaultSelectorCallback(camera_control_msgs::SetIntegerValue::Request &req, camera_control_msgs::SetIntegerValue::Response &res);

    /**
     * Method to set camera user set default selector
     * @param set : 0 = Default, 1 = UserSet1, 2 = UserSet2, 3 = UserSet3, 4 = HighGain, 5 = AutoFunctions, 6 = ColorRaw
     * @return error message if an error occurred or done message otherwise.
     */
    std::string setUserSetDefaultSelector(const int& set);


    /**
     * Service callback for setting the device link throughput limit mode 
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setDeviceLinkThroughputLimitModeCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

    /**
     * Method to set device link throughput limit mode 
     * @param turnOn : true = on , false = Off
     * @return error message if an error occurred or done message otherwise.
     */
    std::string setDeviceLinkThroughputLimitMode(const bool& turnOn);

    /**
     * Service callback for setting the device link throughput limit  
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setDeviceLinkThroughputLimitCallback(camera_control_msgs::SetIntegerValue::Request &req, camera_control_msgs::SetIntegerValue::Response &res);

    /**
     * Method to set device link throughput limit  
     * @param limit : device link throughput limit  Bytes/second
     * @return error message if an error occurred or done message otherwise.
     */
    std::string setDeviceLinkThroughputLimit(const int& limit);

    /**
     * Service callback for reseting the camera device
     * @param req request
     * @param res response
     * @return true on success
     */
    bool triggerDeviceResetCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    /**
     * Method to reset the camera device
     * @return error message if an error occurred or done message otherwise.
     */
    std::string triggerDeviceReset();

    /**
     * Service callback for starting camera aqcuisition
     * @param req request
     * @param res response
     * @return true on success
     */
    bool StartGrabbingCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    /**
     * Method to starting camera aqcuisition
     * @return error message if an error occurred or done message otherwise.
     */
    std::string grabbingStarting();

    /**
     * Service callback for stopping camera aqcuisition
     * @param req request
     * @param res response
     * @return true on success
     */
    bool StopGrabbingCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    /**
     * Method to stopping camera aqcuisition
     * @return error message if an error occurred or done message otherwise.
     */
    std::string grabbingStopping();

    /**
     * Method to collect and publish the current camera parameters
     */
    void currentParamPub();

    /**
     * Service callback for setting the camera image encoding
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setImageEncodingCallback(camera_control_msgs::SetStringValue::Request &req, camera_control_msgs::SetStringValue::Response &res);

    /**
     * Method to set the camera image encoding
     * @param target_ros_endcoding: string describing the encoding (mono8, mono16, bgr8, rgb8, bayer_bggr8, bayer_gbrg8, bayer_rggb8, bayer_grbg8, bayer_rggb16, bayer_bggr16, bayer_gbrg16, Bayer_grbg16).
     * @return false if a communication error occurred or true otherwise.
     */
    std::string setImageEncoding(const std::string& target_ros_encoding);

    /**
     * Service callback for setting the camera Maximum USB data transfer size in bytes
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setMaxTransferSizeCallback(camera_control_msgs::SetIntegerValue::Request &req, camera_control_msgs::SetIntegerValue::Response &res);

    /**
     * Method to set the camera Maximum USB data transfer size in bytes
     * @param maxTransferSize targeted new camera Maximum USB data transfer size in bytes
     * @return error message in case of error occurred or done otherwise.
     */
    std::string setMaxTransferSize(const int& maxTransferSize);

    /**
     * Service callback for setting the camera gamma selector (GigE Camera only)
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setGammaSelectorCallback(camera_control_msgs::SetIntegerValue::Request &req, camera_control_msgs::SetIntegerValue::Response &res);

    /**
     * set the camera Gamma selector (GigE Camera only)
     * @param gammaSelector : 0 = User, 1 = sRGB
     * @return error message if an error occurred or done message otherwise.
     */
    std::string setGammaSelector(const int& gammaSelector);

    /**
     * Service callback for enable/disable the camera gamma
     * @param req request
     * @param res response
     * @return true on success
     */
    bool gammaEnableCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

    /**
     * enable/disable the camera Gamma (GigE Camera only)
     * @param enable
     * @return error message if an error occurred or done message otherwise.
     */
    std::string gammaEnable(const int& enable);

    /**
     * Service callback for setting the camera grab timeout in ms
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setGrabTimeoutCallback(camera_control_msgs::SetIntegerValue::Request &req, camera_control_msgs::SetIntegerValue::Response &res);

    /**
     * Service callback for setting the camera trigger timeout in ms
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setTriggerTimeoutCallback(camera_control_msgs::SetIntegerValue::Request &req, camera_control_msgs::SetIntegerValue::Response &res);

    /**
     * Service callback for setting the white balance of the image channels
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setWhiteBalanceCallback(camera_control_msgs::SetWhiteBalance::Request &req, camera_control_msgs::SetWhiteBalance::Response &res);


    /**
     * Service callback for setting the camera grabbing strategy 
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setGrabbingStrategyCallback(camera_control_msgs::SetIntegerValue::Request &req, camera_control_msgs::SetIntegerValue::Response &res);

    /**
     * Service callback for setting the size of the grab result buffer output queue.
     * @param req request
     * @param res response
     * @return true on success
     */
    bool setOutputQueueSizeCallback(camera_control_msgs::SetIntegerValue::Request &req, camera_control_msgs::SetIntegerValue::Response &res);


    ros::NodeHandle nh_;
    PylonCameraParameter pylon_camera_parameter_set_;
    ros::ServiceServer set_binning_srv_;
    ros::ServiceServer set_roi_srv_;
    ros::ServiceServer set_exposure_srv_;
    ros::ServiceServer set_gain_srv_;
    ros::ServiceServer set_gamma_srv_;
    ros::ServiceServer set_brightness_srv_;
    ros::ServiceServer set_sleeping_srv_;
    ros::ServiceServer set_offset_x_srv_;
    ros::ServiceServer set_offset_y_srv_;    
    ros::ServiceServer reverse_x_srv_;
    ros::ServiceServer reverse_y_srv_;
    ros::ServiceServer set_black_level_srv_;
    ros::ServiceServer set_PGI_mode_srv_;
    ros::ServiceServer set_demosaicing_mode_srv_;
    ros::ServiceServer set_noise_reduction_srv_;
    ros::ServiceServer set_sharpness_enhancement_srv_;
    ros::ServiceServer set_light_source_preset_srv_;
    ros::ServiceServer set_balance_white_auto_srv_;
    ros::ServiceServer set_sensor_readout_mode_srv_;
    ros::ServiceServer set_acquisition_frame_count_srv_;
    ros::ServiceServer set_trigger_selector_srv_;
    ros::ServiceServer set_trigger_mode_srv_;
    ros::ServiceServer execute_software_trigger_srv_;
    ros::ServiceServer set_trigger_source_srv_;
    ros::ServiceServer set_trigger_activation_srv_;
    ros::ServiceServer set_trigger_delay_srv_;
    ros::ServiceServer set_line_selector_srv_;
    ros::ServiceServer set_line_mode_srv_;
    ros::ServiceServer set_line_source_srv_;
    ros::ServiceServer set_line_inverter_srv_;
    ros::ServiceServer set_line_debouncer_time_srv_;
    ros::ServiceServer set_user_set_selector_srv_;
    ros::ServiceServer save_user_set_srv_;
    ros::ServiceServer load_user_set_srv_;
    ros::ServiceServer set_user_set_default_selector_srv_;
    ros::ServiceServer set_device_link_throughput_limit_mode_srv_;
    ros::ServiceServer set_device_link_throughput_limit_srv_;
    ros::ServiceServer set_image_encoding_srv_;
    ros::ServiceServer reset_device_srv_;    
    ros::ServiceServer start_grabbing_srv_;  
    ros::ServiceServer stop_grabbing_srv_;  
    ros::ServiceServer set_max_transfer_size_srv_; 
    ros::ServiceServer set_gamma_selector_srv; 
    ros::ServiceServer gamma_enable_srv;
    ros::ServiceServer set_grab_timeout_srv;
    ros::ServiceServer set_trigger_timeout_srv;
    ros::ServiceServer set_white_balance_srv;
    ros::ServiceServer set_grabbing_strategy_srv;
    ros::ServiceServer set_output_queue_size_srv;

    std::vector<ros::ServiceServer> set_user_output_srvs_;

    // DNB component status publisher
    ros::Publisher componentStatusPublisher;
    dnb_msgs::ComponentStatus cm_status;
    
    // current params publisher
    ros::Publisher currentParamsPublisher;
    camera_control_msgs::currentParams params;




    PylonCamera* pylon_camera_;

    image_transport::ImageTransport* it_;
    image_transport::CameraPublisher img_raw_pub_;

    image_transport::Publisher* img_rect_pub_;
    image_geometry::PinholeCameraModel* pinhole_model_;

    GrabImagesAS grab_imgs_raw_as_;
    GrabImagesAS* grab_imgs_rect_as_;

    sensor_msgs::Image img_raw_msg_;
    cv_bridge::CvImage* cv_bridge_img_rect_;

    camera_info_manager::CameraInfoManager* camera_info_manager_;

    std::vector<std::size_t> sampling_indices_;
    std::array<float, 256> brightness_exp_lut_;

    bool is_sleeping_;
    boost::recursive_mutex grab_mutex_;

    /// diagnostics:
    diagnostic_updater::Updater diagnostics_updater_;
    void diagnostics_timer_callback_(const ros::TimerEvent&);
    ros::Timer diagnostics_trigger_;
    void create_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void create_camera_info_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
};

}  // namespace pylon_camera

#endif  // PYLON_CAMERA_PYLON_CAMERA_NODE_H
