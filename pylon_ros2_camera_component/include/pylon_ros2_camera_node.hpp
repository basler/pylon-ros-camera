/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (C) 2022, Basler AG. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * No contributors' name may be used to endorse or promote products derived from
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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rcutils/logging_macros.h>

#include "visibility_control.hpp"

// topics
#include "pylon_ros2_camera_interfaces/msg/current_params.hpp"
#include "pylon_ros2_camera_interfaces/msg/component_status.hpp"

#include <sensor_msgs/msg/image.hpp>

// services
#include "pylon_ros2_camera_interfaces/srv/get_integer_value.hpp"
#include "pylon_ros2_camera_interfaces/srv/get_float_value.hpp"
#include "pylon_ros2_camera_interfaces/srv/set_binning.hpp"
#include "pylon_ros2_camera_interfaces/srv/set_brightness.hpp"
#include "pylon_ros2_camera_interfaces/srv/set_exposure.hpp"
#include "pylon_ros2_camera_interfaces/srv/set_gain.hpp"
#include "pylon_ros2_camera_interfaces/srv/set_gamma.hpp"
#include "pylon_ros2_camera_interfaces/srv/set_integer_value.hpp"
#include "pylon_ros2_camera_interfaces/srv/set_roi.hpp"
#include "pylon_ros2_camera_interfaces/srv/set_sleeping.hpp"
#include "pylon_ros2_camera_interfaces/srv/set_white_balance.hpp"
#include "pylon_ros2_camera_interfaces/srv/set_integer_value.hpp"
#include "pylon_ros2_camera_interfaces/srv/set_float_value.hpp"
#include "pylon_ros2_camera_interfaces/srv/set_string_value.hpp"
#include "pylon_ros2_camera_interfaces/srv/set_action_trigger_configuration.hpp"
#include "pylon_ros2_camera_interfaces/srv/issue_action_command.hpp"
#include "pylon_ros2_camera_interfaces/srv/issue_scheduled_action_command.hpp"

#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

// actions
#include "pylon_ros2_camera_interfaces/action/grab_images.hpp"

// camera
#include "pylon_ros2_camera.hpp"
#include "pylon_ros2_camera_parameter.hpp"

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_geometry/pinhole_camera_model.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.hpp>
#include <image_transport/camera_publisher.hpp>
#include <image_transport/publisher.hpp>

#include <diagnostic_updater/diagnostic_updater.hpp>


namespace pylon_ros2_camera
{

using GetIntegerSrv                 = pylon_ros2_camera_interfaces::srv::GetIntegerValue;
using GetFloatSrv                   = pylon_ros2_camera_interfaces::srv::GetFloatValue;

using SetBinningSrv                 = pylon_ros2_camera_interfaces::srv::SetBinning;
using SetBrightnessSrv              = pylon_ros2_camera_interfaces::srv::SetBrightness;
using SetExposureSrv                = pylon_ros2_camera_interfaces::srv::SetExposure;
using SetGainSrv                    = pylon_ros2_camera_interfaces::srv::SetGain;
using SetGammaSrv                   = pylon_ros2_camera_interfaces::srv::SetGamma;
using SetROISrv                     = pylon_ros2_camera_interfaces::srv::SetROI;
using SetSleepingSrv                = pylon_ros2_camera_interfaces::srv::SetSleeping;
using SetWhiteBalanceSrv            = pylon_ros2_camera_interfaces::srv::SetWhiteBalance;
using SetIntegerSrv                 = pylon_ros2_camera_interfaces::srv::SetIntegerValue;
using SetFloatSrv                   = pylon_ros2_camera_interfaces::srv::SetFloatValue;
using SetStringSrv                  = pylon_ros2_camera_interfaces::srv::SetStringValue;
using SetActionTriggerConfiguration = pylon_ros2_camera_interfaces::srv::SetActionTriggerConfiguration;
using IssueActionCommand            = pylon_ros2_camera_interfaces::srv::IssueActionCommand;
using IssueScheduledActionCommand   = pylon_ros2_camera_interfaces::srv::IssueScheduledActionCommand;

using SetBoolSrv                    = std_srvs::srv::SetBool;
using TriggerSrv                    = std_srvs::srv::Trigger;

using GrabImagesAction              = pylon_ros2_camera_interfaces::action::GrabImages;
using GrabImagesGoalHandle          = rclcpp_action::ServerGoalHandle<GrabImagesAction>;


class PylonROS2CameraNode : public rclcpp::Node
{
  
public:
  PYLON_ROS2_CAMERA_PUBLIC
  explicit PylonROS2CameraNode(const rclcpp::NodeOptions& options);
  virtual ~PylonROS2CameraNode();

  /**
   * @brief Getter for the frame rate set by the launch script or from the ros parameter server
   * @return the desired frame rate.
   */
  const double& frameRate() const;

  /**
   * @brief Getter for the tf frame.
   * @return the camera frame.
   */
  const std::string& cameraFrame() const;

protected:
  
  /**
   * @brief initialize the camera and the node. Calls ros::shutdown if an error occurs.
   */
  bool init();

  /**
   * @brief initialize the node interfaces (topics, services, actions, diagnostics)
   */
  void initInterfaces();

  /**
   * @brief initialize the node publishers
   */
  void initPublishers();
  
  /**
   * @brief initialize the node services
   */
  void initServices();

  /**
   * @brief initialize the node actions
   */
  void initActions();

  /**
   * @brief initialize the node diagnostics
   */
  void initDiagnostics();

  /**
   * @brief Creates the camera instance and starts the services and action servers.
   * @return false if an error occurred
   */
  bool initAndRegister();

  /**
   * @brief Start the camera and initialize the messages
   * @return false if an error occured
   */
  bool startGrabbing();

  /**
   * @brief Spinning grabbing thread
   */
  virtual void spin();

  /**
   * @brief Grabs an image and stores the image in img_raw_msg_
   * @return false if an error occurred.
   */
  virtual bool grabImage();

  /**
   * @brief Update the exposure value on the camera
   * @param target_exposure the targeted exposure
   * @param reached_exposure the exposure that could be reached
   * @return true if the targeted exposure could be reached
   */
  bool setExposure(const float& target_exposure, float& reached_exposure);

  /**
   * @brief Sets the target brightness which is the intensity-mean over all pixels.
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
   * @brief Update the gain from the camera to a target gain in percent
   * @param target_gain the targeted gain in percent
   * @param reached_gain the gain that could be reached
   * @return true if the targeted gain could be reached
   */
  bool setGain(const float& target_gain, float& reached_gain);

  /**
   * @brief Update the gamma from the camera to a target gamma correction value
   * @param target_gamma the targeted gamma
   * @param reached_gamma the gamma that could be reached
   * @return true if the targeted gamma could be reached
   */
  bool setGamma(const float& target_gamma, float& reached_gamma);

  /**
   * @brief Update area of interest in the camera image
   * @param target_roi the target roi
   * @param reached_roi the roi that could be set
   * @return true if the targeted roi could be reached
   */
  bool setROI(const sensor_msgs::msg::RegionOfInterest target_roi,
              sensor_msgs::msg::RegionOfInterest& reached_roi);
  
  /**
   * @brief Update the horizontal binning_x factor to get downsampled images
   * @param target_binning_x the target horizontal binning_x factor
   * @param reached_binning_x the horizontal binning_x factor that could be
   *        reached
   * @return true if the targeted binning could be reached
   */
  bool setBinningX(const size_t& target_binning_x,
                   size_t& reached_binning_x);

  /**
   * @brief Update the vertical binning_y factor to get downsampled images
   * @param target_binning_y the target vertical binning_y factor
   * @param reached_binning_y the vertical binning_y factor that could be
   *        reached
   * @return true if the targeted binning could be reached
   */
  bool setBinningY(const size_t& target_binning_y,
                   size_t& reached_binning_y);

  /**
   * @brief Method to set the offset in the camera x-axis or y-axis.  
   * @param offsetValue the targeted offset value.
   * @param xAxis when true set the oddset in the x-axis, otherwise in the y-axis.
   * @return error message if an error occurred or done message otherwise.
   */
  std::string setOffsetXY(const int& offsetValue, bool xAxis);

  /**
   * @brief Reverse X, Y on the camera
   * @param reverse_x reverse the image around x-axis
   * @param reverse_y reverse the image around y-axis
   * @return error message if an error occurred or done message otherwise.
   */
  std::string reverseXY(const bool& data, bool around_x);

  /**
   * @brief Method for increasing/decreasing the image black level 
   * @param value the new black level value
   * @return error message if an error occurred or done message otherwise.
   */
  std::string setBlackLevel(const int& value);

  /**
   * @brief Method for setting the PGI mode. 
   * @param on : true = on, false = off.
   * @return error message if an error occurred or done message otherwise.
   */
  std::string setPGIMode(const bool& on);

  /**
   * @brief Method for setting the demosaicing mode. 
   * @param mode : 0 = simple, 1 = Basler PGI.
   * @return error message if an error occurred or done message otherwise.
   */
  std::string setDemosaicingMode(const int& mode);

  /**
   * @brief Method for setting the noise reduction value 
   * @param value : targeted noise reduction value (range : 0.0 to 0.2).
   * @return error message if an error occurred or done message otherwise.
   */
  std::string setNoiseReduction(const float& value);

  /**
   * @brief Method for setting the sharpness enhancement value. 
   * @param value : targeted sharpness enhancement value (range : 1.0 to 3.98438).
   * @return error message if an error occurred or done message otherwise.
   */
  std::string setSharpnessEnhancement(const float& value);

  /**
   * @brief Method to set the camera light source preset  
   * @param mode : 0 = off, 1 = Daylight5000K, 2 = Daylight6500K, 3 = Tungsten2800K
   * @return error message if an error occurred or done message otherwise.
   */
  std::string setLightSourcePreset(const int& mode);

  /**
   * @brief Method to set the camera white balance auto 
   * @param mode : 0 = off , 1 = once, 2 = continuous
   * @return error message if an error occurred or done message otherwise.
   */
  std::string setWhiteBalanceAuto(const int& mode);

  /**
   * @brief Method to set the sensor readout mode (Normal or Fast)
   * @param mode : 0 = normal , 1 = fast.
   * @return error message if an error occurred or done message otherwise.
   */
  std::string setSensorReadoutMode(const int& mode);

  /**
   * @brief Method to set the camera acquisition frame count
   * @param frameCount trageted frame count.
   * @return error message if an error occurred or done message otherwise.
   */
  std::string setAcquisitionFrameCount(const int& frameCount);

  /**
   * @brief Method to set the trigger selector   
   * @param mode : 0 = Frame Start, 1 = Frame Burst Start (ace USB) / Acquisition Start (ace GigE)
   * @return error message if an error occurred or done message otherwise.
   */
  std::string setTriggerSelector(const int& mode);

  /**
   * @brief Method to set the trigger mode   
   * @param value : false = off, true = on
   * @return error message if an error occurred or done message otherwise.
   */
  std::string setTriggerMode(const bool& value);

  /**
   * @brief Method to execute a software trigger   
   * @return error message if an error occurred or done message otherwise.
   */
  std::string executeSoftwareTrigger();

  /**
   * @brief Method to set the camera trigger source.
   * @param source : 0 = software, 1 = Line1, 2 = Line3, 3 = Line4, 4 = Action1(only selected GigE Camera)  
   * @return error message if an error occurred or done message otherwise.
   */
  std::string setTriggerSource(const int& source);

  /**
   * @brief Method to set camera trigger activation type  
   * @param value : 0 = RigingEdge, 1 = FallingEdge
   * @return error message if an error occurred or done message otherwise.
   */
  std::string setTriggerActivation(const int& value);

  /**
   * @brief Method to set camera trigger delay value  
   * @param delayValue required dely value in µs 
   * @return error message if an error occurred or done message otherwise.
   */
  std::string setTriggerDelay(const float& value);

  /**
   * @brief Method to set camera line selector
   * @param value : 0 = line1, 1 = line2, 2 = line3, 3 = line4  
   * @return error message if an error occurred or done message otherwise.
   */
  std::string setLineSelector(const int& value);

  /**
   * @brief Method to set camera line Mode
   * @param value : 0 = input, 1 = output   
   * @return error message if an error occurred or done message otherwise.
   */
  std::string setLineMode(const int& value);

  /**
   * @brief Method to set camera line source
   * @param value : 0 = exposure active  
   * @return error message if an error occurred or done message otherwise.
   */
  std::string setLineSource(const int& value);

  /**
   * @brief Method to set camera line inverter
   * @param value : ture = invert line
   * @return error message if an error occurred or done message otherwise.
   */
  std::string setLineInverter(const bool& value);
  
  /**
   * @brief Method to set camera line debouncer time
   * @param value delay time in microseconds
   * @return error message if an error occurred or done message otherwise.
   */
  std::string setLineDebouncerTime(const float& value);

  /**
   * @brief Method to set camera user set selector
   * @param set : 0 = Default, 1 = UserSet1, 2 = UserSet2, 3 = UserSet3, 4 = HighGain, 5 = AutoFunctions, 6 = ColorRaw
   * @return error message if an error occurred or done message otherwise.
   */
  std::string setUserSetSelector(const int& set);

  /**
   * @brief Method to save user set
   * @return error message if an error occurred or done message otherwise.
   */
  std::string saveUserSet();

  /**
   * @brief Method to load user set
   * @return error message if an error occurred or done message otherwise.
   */
  std::string loadUserSet();

  /**
   * @brief Method to load a pfs file
   * @return error message if an error occurred or done message otherwise.
   */
  std::string loadPfs(const std::string& fileName);

  /**
   * @brief Method to set camera user set default selector
   * @param set : 0 = Default, 1 = UserSet1, 2 = UserSet2, 3 = UserSet3, 4 = HighGain, 5 = AutoFunctions, 6 = ColorRaw
   * @return error message if an error occurred or done message otherwise.
   */
  std::string setUserSetDefaultSelector(const int& set);

  /**
   * @brief Method to set device link throughput limit mode 
   * @param turnOn : true = on , false = Off
   * @return error message if an error occurred or done message otherwise.
   */
  std::string setDeviceLinkThroughputLimitMode(const bool& turnOn);

  /**
   * @brief Method to set device link throughput limit  
   * @param limit : device link throughput limit  Bytes/second
   * @return error message if an error occurred or done message otherwise.
   */
  std::string setDeviceLinkThroughputLimit(const int& limit);

  /**
   * @brief Method to reset the camera device
   * @return error message if an error occurred or done message otherwise.
   */
  std::string triggerDeviceReset();

  /**
   * @brief Method to set the camera image encoding
   * @param target_ros_endcoding: string describing the encoding (mono8, mono16, bgr8, rgb8, bayer_bggr8, bayer_gbrg8, bayer_rggb8, bayer_grbg8, bayer_rggb16, bayer_bggr16, bayer_gbrg16, Bayer_grbg16).
   * @return false if a communication error occurred or true otherwise.
   */
  std::string setImageEncoding(const std::string& target_ros_encoding);

  /**
   * @brief Method to set the camera Maximum USB data transfer size in bytes
   * @param maxTransferSize targeted new camera Maximum USB data transfer size in bytes
   * @return error message in case of error occurred or done otherwise.
   */
  std::string setMaxTransferSize(const int& maxTransferSize);

  /**
   * @brief Set the camera Gamma selector (GigE Camera only)
   * @param gammaSelector : 0 = User, 1 = sRGB
   * @return error message if an error occurred or done message otherwise.
   */
  std::string setGammaSelector(const int& gammaSelector);

  /**
   * @brief Enable/disable the camera Gamma (GigE Camera only)
   * @param enable
   * @return error message if an error occurred or done message otherwise.
   */
  std::string gammaEnable(const int& enable);

  /**
   * @brief Return the number of subscribers for the rect image topic
   * @return The number of subscribers for the rect image topic
   */
  uint32_t getNumSubscribersRectImagePub() const;

  /**
   * @brief Service callback for getting the maximum number of buffers that can be used simultaneously for grabbing images - Applies to: BCON, GigE, USB and blaze.
   * @param req request
   * @param res response
   */
  void getMaxNumBufferCallback(const std::shared_ptr<GetIntegerSrv::Request> request,
                               std::shared_ptr<GetIntegerSrv::Response> response);

  /**
   * @brief Service callback for getting the GigE cameras: Number of frames received Other cameras: Number of buffers processed - Applies to: BCON, GigE, USB and blaze.
   * @param req request
   * @param res response
   */
  void getStatisticTotalBufferCountCallback(const std::shared_ptr<GetIntegerSrv::Request> request,
                                            std::shared_ptr<GetIntegerSrv::Response> response);

  /**
   * @brief Service callback for getting the GigE cameras: Number of buffers with at least one failed packet. A packet is considered failed if its status is not 'success'. Other cameras: Number of buffers that returned an error.
   * @param req request
   * @param res response
   */
  void getStatisticFailedBufferCountCallback(const std::shared_ptr<GetIntegerSrv::Request> request,
                                             std::shared_ptr<GetIntegerSrv::Response> response);

  /**
   * @brief Service callback for getting the Number of frames lost because there were no buffers in the queue - Applies to: GigE and blaze.
   * @param req request
   * @param res response
   */
  void getStatisticBufferUnderrunCountCallback(const std::shared_ptr<GetIntegerSrv::Request> request,
                                               std::shared_ptr<GetIntegerSrv::Response> response);

  /**
   * @brief Service callback for getting the Number of failed packets, i.e., the number of packets whose status is not 'success'.
   * @param req request
   * @param res response
   */
  void getStatisticFailedPacketCountCallback(const std::shared_ptr<GetIntegerSrv::Request> request,
                                             std::shared_ptr<GetIntegerSrv::Response> response);

  /**
   * @brief Service callback for getting the Number of emitted packet resend commands sent - Applies to: GigE and blaze.
   * @param req request
   * @param res response
   */
  void getStatisticResendRequestCountCallback(const std::shared_ptr<GetIntegerSrv::Request> request,
                                              std::shared_ptr<GetIntegerSrv::Response> response);

  /**
   * @brief Service callback for getting the Number of corrupt or lost frames between successfully grabbed images - Applies to: BCON and USB.
   * @param req request
   * @param res response
   */
  void getStatisticMissedFrameCountCallback(const std::shared_ptr<GetIntegerSrv::Request> request,
                                            std::shared_ptr<GetIntegerSrv::Response> response);

  /**
   * @brief Service callback for getting the Number of stream resynchronizations - Applies to: USB.
   * @param req request
   * @param res response
   */
  void getStatisticResynchronizationCountCallback(const std::shared_ptr<GetIntegerSrv::Request> request,
                                                  std::shared_ptr<GetIntegerSrv::Response> response);

  /**
   * @brief Service callback for getting  the chunk mode - Applies to: GigE, ace 2 GigE, ace 2 USB and ace USB.
   * @param req request
   * @param res response
   */
  void getChunkModeActiveCallback(const std::shared_ptr<GetIntegerSrv::Request> request,
                                  std::shared_ptr<GetIntegerSrv::Response> response);

  /**
   * @brief Service callback for getting the sets for the chunk - Applies to: GigE, ace 2 GigE, ace 2 USB and ace USB.
   * @param req request
   * @param res response
   */
  void getChunkSelectorCallback(const std::shared_ptr<GetIntegerSrv::Request> request,
                                std::shared_ptr<GetIntegerSrv::Response> response);

  /**
   * @brief Service callback for getting  the currently selected chunk in the payload data - Applies to: GigE, ace 2 GigE, ace 2 USB and ace USB.
   * @param req request
   * @param res response
   */
  void getChunkEnableCallback(const std::shared_ptr<GetIntegerSrv::Request> request,
                              std::shared_ptr<GetIntegerSrv::Response> response);

  /**
   * @brief Service callback for getting the Value of the timestamp when the image was acquired - Applies to: GigE and ace USB..
   * @param req request
   * @param res response
   */
  void getChunkTimestampCallback(const std::shared_ptr<GetIntegerSrv::Request> request,
                                 std::shared_ptr<GetIntegerSrv::Response> response);

  /**
   * @brief Service callback for getting Bit field that indicates the status of all of the camera's input and output lines when the image was acquired - Applies to: GigE, ace 2 GigE, ace 2 USB and ace USB.
   * @param req request
   * @param res response
   */
  void getChunkLineStatusAllCallback(const std::shared_ptr<GetIntegerSrv::Request> request,
                                      std::shared_ptr<GetIntegerSrv::Response> response);

  /**
   * @brief Service callback for getting Value of the Frame counter when the image was acquired - Applies to: GigE.
   * @param req request
   * @param res response
   */
  void getChunkFramecounterCallback(const std::shared_ptr<GetIntegerSrv::Request> request,
                                    std::shared_ptr<GetIntegerSrv::Response> response);

  /**
   * @brief Service callback for getting Value of the selected chunk counter - Applies to: ace 2 GigE, ace 2 USB and ace USB.
   * @param req request
   * @param res response
   */
  void getChunkCounterValueCallback(const std::shared_ptr<GetIntegerSrv::Request> request,
                                    std::shared_ptr<GetIntegerSrv::Response> response);

  /**
   * @brief Service callback for getting the Exposure time used to acquire the image - Applies to: GigE, ace 2 GigE, ace 2 USB and ace USB 
   * @param req request
   * @param res response
   */
  void getChunkExposureTimeCallback(const std::shared_ptr<GetFloatSrv::Request> request,
                                    std::shared_ptr<GetFloatSrv::Response> response);

  /**
   * @brief Service callback for updating the cameras binning setting
   * @param req request
   * @param res response
   */
  void setBinningCallback(const std::shared_ptr<SetBinningSrv::Request> request,
                          std::shared_ptr<SetBinningSrv::Response> response);

  /**
   * @brief Service callback for setting the brightness
   * @param req request
   * @param res response
   */
  void setBrightnessCallback(const std::shared_ptr<SetBrightnessSrv::Request> request,
                             std::shared_ptr<SetBrightnessSrv::Response> response);

  /**
   * @brief Service callback for setting the exposure
   * @param req request
   * @param res response
   */
  void setExposureCallback(const std::shared_ptr<SetExposureSrv::Request> request,
                           std::shared_ptr<SetExposureSrv::Response> response);

  /**
   * @brief Service callback for setting the desired gain in percent
   * @param req request
   * @param res response
   */
  void setGainCallback(const std::shared_ptr<SetGainSrv::Request> request,
                       std::shared_ptr<SetGainSrv::Response> response);

  /**
   * @brief Service callback for setting the desired gamma correction value
   * @param req request
   * @param res response
   */
  void setGammaCallback(const std::shared_ptr<SetGammaSrv::Request> request,
                        std::shared_ptr<SetGammaSrv::Response> response);

  /**
   * @brief Service callback for updating the cameras roi setting
   * @param req request
   * @param res response
   */
  void setROICallback(const std::shared_ptr<SetROISrv::Request> request,
                      std::shared_ptr<SetROISrv::Response> response);

  /**
   * @brief Callback that puts the camera to sleep
   * @param req request
   * @param res response
   */
  void setSleepingCallback(const std::shared_ptr<SetSleepingSrv::Request> request,
                           std::shared_ptr<SetSleepingSrv::Response> response);
  
  /**
   * @brief Service callback for setting the white balance of the image channels
   * @param req request
   * @param res response
   */
  void setWhiteBalanceCallback(const std::shared_ptr<SetWhiteBalanceSrv::Request> request,
                               std::shared_ptr<SetWhiteBalanceSrv::Response> response);

  /**
   * @brief Service callback for setting the action trigger configuration
   * @param req request
   * @param res response
   */
  void setActionTriggerConfigurationCallback(const std::shared_ptr<SetActionTriggerConfiguration::Request> request,
                                             std::shared_ptr<SetActionTriggerConfiguration::Response> response);

  /**
   * @brief Service callback for issuing an action command
   * @param req request
   * @param res response
   */
  void issueActionCommandCallback(const std::shared_ptr<IssueActionCommand::Request> request,
                                  std::shared_ptr<IssueActionCommand::Response> response);

  /**
   * @brief Service callback for issuing a scheduled action command
   * @param req request
   * @param res response
   */
  void issueScheduledActionCommandCallback(const std::shared_ptr<IssueScheduledActionCommand::Request> request,
                                           std::shared_ptr<IssueScheduledActionCommand::Response> response);

  /**
   * @brief Service callback for setting camera x-axis offset 
   * @param req request
   * @param res response
   */
  void setOffsetXCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                          std::shared_ptr<SetIntegerSrv::Response> response);

  /**
   * @brief Service callback for setting camera y-axis offset 
   * @param req request
   * @param res response
   */
  void setOffsetYCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                          std::shared_ptr<SetIntegerSrv::Response> response);

  /**
   * @brief Service callback for increasing/decreasing the image black level 
   * @param req request
   * @param res response
   */
  void setBlackLevelCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                             std::shared_ptr<SetIntegerSrv::Response> response);

  /**
   * @brief Service callback for setting the demosaicing mode 
   * @param req request
   * @param res response
   */
  void setDemosaicingModeCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                  std::shared_ptr<SetIntegerSrv::Response> response);

  /**
   * @brief Service callback for setting the camera light source preset 
   * @param req request
   * @param res response
   */
  void setLightSourcePresetCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                    std::shared_ptr<SetIntegerSrv::Response> response);

  /**
   * @brief Service callback for setting the camera balance white auto 
   * @param req request
   * @param res response
   */
  void setWhiteBalanceAutoCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                   std::shared_ptr<SetIntegerSrv::Response> response);

  /**
   * @brief Service callback for setting the sensor readout mode (Normal or Fast)
   * @param req request
   * @param res response
   */
  void setSensorReadoutModeCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                    std::shared_ptr<SetIntegerSrv::Response> response);
  
  /**
   * @brief Service callback for setting the acquisition frame count
   * @param req request
   * @param res response
   */
  void setAcquisitionFrameCountCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                        std::shared_ptr<SetIntegerSrv::Response> response);
  
  /**
   * @brief Service callback for setting the trigger selector
   * @param req request
   * @param res response
   */
  void setTriggerSelectorCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                  std::shared_ptr<SetIntegerSrv::Response> response);

  /**
   * @brief Service callback for setting the camera trigger source
   * @param req request
   * @param res response
   */
  void setTriggerSourceCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                std::shared_ptr<SetIntegerSrv::Response> response);

  /**
   * @brief Service callback for setting the camera trigger activation type
   * @param req request
   * @param res response
   */
  void setTriggerActivationCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                    std::shared_ptr<SetIntegerSrv::Response> response);

  /**
   * @brief Service callback for setting the camera line selector
   * @param req request
   * @param res response
   */
  void setLineSelectorCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                               std::shared_ptr<SetIntegerSrv::Response> response);

  /**
   * @brief Service callback for setting the camera line mode
   * @param req request
   * @param res response
   */
  void setLineModeCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                           std::shared_ptr<SetIntegerSrv::Response> response);

  /**
   * @brief Service callback for setting the camera line source
   * @param req request
   * @param res response
   */
  void setLineSourceCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                             std::shared_ptr<SetIntegerSrv::Response> response);

  /**
   * @brief Service callback for setting the camera user set selector
   * @param req request
   * @param res response
   */
  void setUserSetSelectorCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                  std::shared_ptr<SetIntegerSrv::Response> response);

  /**
   * @brief Service callback for setting the camera user set default selector
   * @param req request
   * @param res response
   */
  void setUserSetDefaultSelectorCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                         std::shared_ptr<SetIntegerSrv::Response> response);

  /**
   * @brief Service callback for setting the device link throughput limit  
   * @param req request
   * @param res response
   */
  void setDeviceLinkThroughputLimitCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                            std::shared_ptr<SetIntegerSrv::Response> response);

  /**
   * @brief Service callback for setting the camera Maximum USB data transfer size in bytes
   * @param req request
   * @param res response
   */
  void setMaxTransferSizeCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                  std::shared_ptr<SetIntegerSrv::Response> response);

  /**
   * @brief Service callback for setting the camera gamma selector (GigE Camera only)
   * @param req request
   * @param res response
   */
  void setGammaSelectorCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                std::shared_ptr<SetIntegerSrv::Response> response);

  /**
   * @brief Service callback for setting the camera grab timeout in ms
   * @param req request
   * @param res response
   */
  void setGrabTimeoutCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                              std::shared_ptr<SetIntegerSrv::Response> response);

  /**
   * @brief Service callback for setting the camera trigger timeout in ms
   * @param req request
   * @param res response
   */
  void setTriggerTimeoutCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                 std::shared_ptr<SetIntegerSrv::Response> response);

  /**
   * @brief Service callback for setting the camera grabbing strategy 
   * @param req request
   * @param res response
   */
  void setGrabbingStrategyCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                   std::shared_ptr<SetIntegerSrv::Response> response);

  /**
   * @brief Service callback for setting the size of the grab result buffer output queue.
   * @param req request
   * @param res response
   */
  void setOutputQueueSizeCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                  std::shared_ptr<SetIntegerSrv::Response> response);

  /**
   * @brief Service callback for setting the maximum number of buffers that can be used simultaneously for grabbing images - Applies to: BCON, GigE, USB and blaze.
   * @param req request
   * @param res response
   */
  void setMaxNumBufferCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                               std::shared_ptr<SetIntegerSrv::Response> response);

  /**
   * @brief Service callback for selecting the sets for the chunk - Applies to: GigE, ace 2 GigE, ace 2 USB and ace USB.
   * @param req request
   * @param res response
   */
  void setChunkSelectorCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                std::shared_ptr<SetIntegerSrv::Response> response);

  /**
   * @brief Service callback for setting timer selector - Applies to: GigE, ace 2 GigE, ace 2 USB and ace USB.
   * @param req request
   * @param res response
   */
  void setTimerSelectorCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                std::shared_ptr<SetIntegerSrv::Response> response);

  /**
   * @brief Service callback for setting the internal camera signal used to trigger the selected timer - Applies to: GigE, ace 2 GigE, ace 2 USB, ace USB and dart 2 USB.
   * @param req request
   * @param res response
   */
  void setTimerTriggerSourceCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                     std::shared_ptr<SetIntegerSrv::Response> response);

  /**
   * @brief Service callback for setting the ptp priority - Applies to: GigE, ace 2 GigE, ace 2 USB, ace USB and dart 2 USB.
   * @param req request
   * @param res response
   */
  void setPTPPriorityCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                              std::shared_ptr<SetIntegerSrv::Response> response);
  
  /**
   * @brief Service callback for setting the ptp profile - Applies to: ace 2 GigE.
   * @param req request
   * @param res response
   */
  void setPTPProfileCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                             std::shared_ptr<SetIntegerSrv::Response> response);

  /**
   * @brief Service callback for setting the ptp network mode - Applies to: ace 2 GigE.
   * @param req request
   * @param res response
   */
  void setPTPNetworkModeCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                 std::shared_ptr<SetIntegerSrv::Response> response);

  /**
   * @brief Service callback for setting the ptp uc port address index - Applies to: ace 2 GigE.
   * @param req request
   * @param res response
   */
  void setPTPUCPortAddressIndexCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                        std::shared_ptr<SetIntegerSrv::Response> response);

  /**
   * @brief Service callback for setting the ptp uc port address - Applies to: ace 2 GigE.
   * @param req request
   * @param res response
   */
  void setPTPUCPortAddressCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                   std::shared_ptr<SetIntegerSrv::Response> response);

  /**
   * @brief Service callback for setting the sync free run timer start time low - Applies to: GigE.
   * @param req request
   * @param res response
   */
  void setSyncFreeRunTimerStartTimeLowCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                               std::shared_ptr<SetIntegerSrv::Response> response);

  /**
   * @brief Service callback for setting the sync free run timer start time high - Applies to: GigE.
   * @param req request
   * @param res response
   */
  void setSyncFreeRunTimerStartTimeHighCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                                std::shared_ptr<SetIntegerSrv::Response> response);

  /**
   * @brief Service callback for setting the noise reduction value 
   * @param req request
   * @param res response
   */
  void setNoiseReductionCallback(const std::shared_ptr<SetFloatSrv::Request> request,
                                 std::shared_ptr<SetFloatSrv::Response> response);

  /**
   * @brief Service callback for setting the sharpness enhancement value. 
   * @param req request
   * @param res response
   */
  void setSharpnessEnhancementCallback(const std::shared_ptr<SetFloatSrv::Request> request,
                                       std::shared_ptr<SetFloatSrv::Response> response);

  /**
   * @brief Service callback for setting the camera trigger delay value
   * @param req request
   * @param res response
   */
  void setTriggerDelayCallback(const std::shared_ptr<SetFloatSrv::Request> request,
                               std::shared_ptr<SetFloatSrv::Response> response);

  /**
   * @brief Service callback for setting the camera line debouncer time
   * @param req request
   * @param res response
   */
  void setLineDebouncerTimeCallback(const std::shared_ptr<SetFloatSrv::Request> request,
                                    std::shared_ptr<SetFloatSrv::Response> response);

  /**
   * @brief Service callback for setting the Exposure time used to acquire the image - Applies to: GigE, ace 2 GigE, ace 2 USB and ace USB 
   * @param req request
   * @param res response
   */
  void setChunkExposureTimeCallback(const std::shared_ptr<SetFloatSrv::Request> request,
                                    std::shared_ptr<SetFloatSrv::Response> response);

  /**
   * @brief Service callback for setting the timer duration - Applies to: ace 2 GigE, ace 2 USB, ace USB and dart 2 USB.
   * @param req request
   * @param res response
   */
  void setTimerDurationCallback(const std::shared_ptr<SetFloatSrv::Request> request,
                                std::shared_ptr<SetFloatSrv::Response> response);

  /**
   * @brief Service callback for setting the length of the periodic signal in microseconds - Applies to: ace 2 GigE.
   * @param req request
   * @param res response
   */
  void setPeriodicSignalPeriodCallback(const std::shared_ptr<SetFloatSrv::Request> request,
                                       std::shared_ptr<SetFloatSrv::Response> response);

  /**
   * @brief Service callback for setting the delay to be applied to the periodic signal in microseconds - Applies to: ace 2 GigE.
   * @param req request
   * @param res response
   */
  void setPeriodicSignalDelayCallback(const std::shared_ptr<SetFloatSrv::Request> request,
                                      std::shared_ptr<SetFloatSrv::Response> response);

  /**
   * @brief Service callback for setting the synchronous free run trigger rate - Applies to: GigE.
   * @param req request
   * @param res response
   */
  void setSyncFreeRunTimerTriggerRateAbsCallback(const std::shared_ptr<SetFloatSrv::Request> request,
                                                 std::shared_ptr<SetFloatSrv::Response> response);

  /**
   * @brief Service callback for setting the camera image encoding
   * @param req request
   * @param res response
   */
  void setImageEncodingCallback(const std::shared_ptr<SetStringSrv::Request> request,
                                std::shared_ptr<SetStringSrv::Response> response);

  /**
   * @brief Service callback for reversing X
   * @param req request
   * @param res response
   */
  void setReverseXCallback(const std::shared_ptr<SetBoolSrv::Request> request,
                           std::shared_ptr<SetBoolSrv::Response> response);

  /**
   * @brief Service callback for reversing Y
   * @param req request
   * @param res response
   */
  void setReverseYCallback(const std::shared_ptr<SetBoolSrv::Request> request,
                           std::shared_ptr<SetBoolSrv::Response> response);

  /**
   * @brief Service callback for setting the PGI mode 
   * @param req request
   * @param res response
   */
  void setPGIModeCallback(const std::shared_ptr<SetBoolSrv::Request> request,
                          std::shared_ptr<SetBoolSrv::Response> response);

  /**
   * @brief Service callback for setting the trigger mode
   * @param req request
   * @param res response
   */
  void setTriggerModeCallback(const std::shared_ptr<SetBoolSrv::Request> request,
                              std::shared_ptr<SetBoolSrv::Response> response);
  
  /**
   * @brief Service callback for setting the camera line inverter
   * @param req request
   * @param res response
   */
  void setLineInverterCallback(const std::shared_ptr<SetBoolSrv::Request> request,
                               std::shared_ptr<SetBoolSrv::Response> response);

  /**
   * @brief Service callback for setting the device link throughput limit mode 
   * @param req request
   * @param res response
   */
  void setDeviceLinkThroughputLimitModeCallback(const std::shared_ptr<SetBoolSrv::Request> request,
                                                std::shared_ptr<SetBoolSrv::Response> response);

  /**
   * @brief Service callback for enable/disable the camera gamma
   * @param req request
   * @param res response
   */
  void setGammaEnableCallback(const std::shared_ptr<SetBoolSrv::Request> request,
                              std::shared_ptr<SetBoolSrv::Response> response);

  /**
   * @brief Service callback for enabling/disabling  the chunk mode - Applies to: GigE, ace 2 GigE, ace 2 USB and ace USB.
   * @param req request
   * @param res response
   */
  void setChunkModeActiveCallback(const std::shared_ptr<SetBoolSrv::Request> request,
                                  std::shared_ptr<SetBoolSrv::Response> response);

  /**
   * @brief Service callback for enabling/disabling  the Includes the currently selected chunk in the payload data - Applies to: GigE, ace 2 GigE, ace 2 USB and ace USB.
   * @param req request
   * @param res response
   */
  void setChunkEnableCallback(const std::shared_ptr<SetBoolSrv::Request> request,
                              std::shared_ptr<SetBoolSrv::Response> response);

  /**
   * @brief Callback that sets the digital user output
   * @param output_id the ID of the user output to set
   * @param req request
   * @param res response
   */
  void setUserOutputCallback(const int output_id,
                             const std::shared_ptr<SetBoolSrv::Request> request,
                             std::shared_ptr<SetBoolSrv::Response> response);

  /**
   * @brief Callback that activates the digital user output to be used as autoflash
   * @param output_id the ID of the user output to set
   * @param req request
   * @param res response
   */
  void setAutoflashCallback(const int output_id,
                            const std::shared_ptr<SetBoolSrv::Request> request,
                            std::shared_ptr<SetBoolSrv::Response> response);

  /**
   * @brief Service callback for enabling/disabling PTP management protocol - Applies to: ace 2 GigE.
   * @param req request
   * @param res response
   */
  void enablePTPManagementProtocolCallback(const std::shared_ptr<SetBoolSrv::Request> request,
                                           std::shared_ptr<SetBoolSrv::Response> response);
  
  /**
   * @brief Service callback for enabling/disabling PTP two step operation - Applies to: ace 2 GigE.
   * @param req request
   * @param res response
   */
  void enablePTPTwoStepOperationCallback(const std::shared_ptr<SetBoolSrv::Request> request,
                                         std::shared_ptr<SetBoolSrv::Response> response);
  
  /**
   * @brief Service callback for enabling/disabling PTP - Applies to: GigE, ace 2 GigE.
   * @param req request
   * @param res response
   */
  void enablePTPCallback(const std::shared_ptr<SetBoolSrv::Request> request,
                         std::shared_ptr<SetBoolSrv::Response> response);

  /**
   * @brief Service callback for enabling/disabling the synchronous free run mode - Applies to: GigE.
   * @param req request
   * @param res response
   */
  void enableSyncFreeRunTimerCallback(const std::shared_ptr<SetBoolSrv::Request> request,
                                      std::shared_ptr<SetBoolSrv::Response> response);

  /**
   * @brief Service callback for executing a software trigger
   * @param req request
   * @param res response
   */
  void executeSoftwareTriggerCallback(const std::shared_ptr<TriggerSrv::Request> request,
                                      std::shared_ptr<TriggerSrv::Response> response);

  /**
   * @brief Service callback for saving the user set
   * @param req request
   * @param res response
   */
  void saveUserSetCallback(const std::shared_ptr<TriggerSrv::Request> request,
                           std::shared_ptr<TriggerSrv::Response> response);

  /**
   * @brief Service callback for loading the user set
   * @param req request
   * @param res response
   */
  void loadUserSetCallback(const std::shared_ptr<TriggerSrv::Request> request,
                           std::shared_ptr<TriggerSrv::Response> response);

  /**
   * @brief Service callback for loading a pfs file
   * @param req request
   * @param res response
   */
  void loadPfsCallback(const std::shared_ptr<SetStringSrv::Request> request,
                           std::shared_ptr<SetStringSrv::Response> response);

  /**
   * @brief Service callback for reseting the camera device
   * @param req request
   * @param res response
   */
  void triggerDeviceResetCallback(const std::shared_ptr<TriggerSrv::Request> request,
                                  std::shared_ptr<TriggerSrv::Response> response);

  /**
   * @brief Service callback for starting camera aqcuisition
   * @param req request
   * @param res response
   */
  void startGrabbingCallback(const std::shared_ptr<TriggerSrv::Request> request,
                             std::shared_ptr<TriggerSrv::Response> response);

  /**
   * @brief Service callback for stopping camera aqcuisition
   * @param req request
   * @param res response
   */
  void stopGrabbingCallback(const std::shared_ptr<TriggerSrv::Request> request,
                            std::shared_ptr<TriggerSrv::Response> response);

  /**
   * @brief Service callback for updating synchronous free run settings - Applies to: GigE.
   * @param req request
   * @param res response
   */
  void updateSyncFreeRunTimerCallback(const std::shared_ptr<TriggerSrv::Request> request,
                                      std::shared_ptr<TriggerSrv::Response> response);

  /**
   * @brief Handle action goal relatively raw image grabbing
   * @return goal response
   */
  rclcpp_action::GoalResponse handleGrabRawImagesActionGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const GrabImagesAction::Goal> goal);

  /**
   * @brief Handle action cancellation relatively to raw image grabbing
   * @param goal_handle Goal handle
   * @return Response of the action is cancelled 
   */
  rclcpp_action::CancelResponse handleGrabRawImagesActionGoalCancel(const std::shared_ptr<GrabImagesGoalHandle> goal_handle);

  /**
   * @brief Handle action if goal is accepted relatively to raw image grabbing
   * @param goal_handle handle 
   */
  void handleGrabRawImagesActionGoalAccepted(const std::shared_ptr<GrabImagesGoalHandle> goal_handle);

  /**
   * @brief Grab raw images through action
   * @param goal_handle 
   */
  void executeGrabRawImagesAction(const std::shared_ptr<GrabImagesGoalHandle> goal_handle);

  /**
   * @brief Handle action goal relatively rectified image grabbing
   * @return goal response
   */
  rclcpp_action::GoalResponse handleGrabRectImagesActionGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const GrabImagesAction::Goal> goal);

  /**
   * @brief Handle action cancellation relatively to rectified image grabbing
   * @param goal_handle Goal handle
   * @return Response of the action is cancelled 
   */
  rclcpp_action::CancelResponse handleGrabRectImagesActionGoalCancel(const std::shared_ptr<GrabImagesGoalHandle> goal_handle);

  /**
   * @brief Handle action if goal is accepted relatively to rectified image grabbing
   * @param goal_handle handle 
   */
  void handleGrabRectImagesActionGoalAccepted(const std::shared_ptr<GrabImagesGoalHandle> goal_handle);

  /**
   * @brief Grab rectified images through action
   * @param goal_handle 
   */
  void executeGrabRectImagesAction(const std::shared_ptr<GrabImagesGoalHandle> goal_handle);

  /**
   * @brief Create diagnostics
   * @param stat Diagnostic status wrapper
   */
  void createDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

  /**
   * @brief Create camera info diagnostics
   * @param stat Diagnostic status wrapper
   */
  void createCameraInfoDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

  /**
   * @brief Callback to diagnostics
   */
  void diagnosticsTimerCallback();

  /**
   * @brief Check if service exists
   * @param service_name Service name
   * @return true if service exists otherwise false
   */
  bool serviceExists(const std::string& service_name);

  /**
   * @brief Generates the subset of points on which the brightness search will be
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
   * @brief This function will recursively be called from above setupSamplingIndices()
   * to generate the indices of pixels given the actual ROI.
   * @return indices describing the subset of points
   */
  void genSamplingIndicesRec(std::vector<std::size_t>& indices,
                             const std::size_t& min_window_height,
                             const cv::Point2i& start,
                             const cv::Point2i& end);

  /**
   * @brief Calculates the mean brightness of the image based on the subset indices
   * @return the mean brightness of the image
   */
  float calcCurrentBrightness();

  /**
   * @brief Fills the ros CameraInfo-Object with the image dimensions
   */
  virtual void setupInitialCameraInfo(sensor_msgs::msg::CameraInfo& cam_info_msg);

  /**
   * @brief Initializing of img_rect_pub_, grab_img_rect_as_ and the pinhole_model_,
   * in case that a valid camera info has been set
   * @return
   */
  void setupRectification();

  /**
   * @brief Grab raw images triggered by action
   * @param goal_handle Goal handle
   * @return Action result
   */
  std::shared_ptr<GrabImagesAction::Result> grabRawImages(const std::shared_ptr<GrabImagesGoalHandle> goal_handle);

  /**
   * @brief Method to starting camera aqcuisition
   * @return error message if an error occurred or done message otherwise.
   */
  std::string grabbingStarting();

  /**
   * @brief Method to stopping camera aqcuisition
   * @return error message if an error occurred or done message otherwise.
   */
  std::string grabbingStopping();

  /**
   * @brief Waits till the pylon_camera_ isReady() observing a given timeout
   * @return true when the camera's state toggles to 'isReady()'
   */
  bool waitForCamera(const std::chrono::duration<double>& timeout) const;

  /**
   * @brief Method to collect and publish the current camera parameters
   */
  void currentParamPub();

  /**
   * @brief Returns true if the camera was put into sleep mode
   * @return true if in sleep mode
   */
  bool isSleeping();

protected:

  // camera
  PylonROS2Camera* pylon_camera_;
  image_geometry::PinholeCameraModel* pinhole_model_;

  PylonROS2CameraParameter pylon_camera_parameter_set_;
  camera_info_manager::CameraInfoManager* camera_info_manager_;

  sensor_msgs::msg::Image img_raw_msg_;

  cv_bridge::CvImage* cv_bridge_img_rect_;

  // topics
  rclcpp::Publisher<pylon_ros2_camera_interfaces::msg::CurrentParams>::SharedPtr current_params_pub_;
  pylon_ros2_camera_interfaces::msg::CurrentParams current_params_;
  rclcpp::Publisher<pylon_ros2_camera_interfaces::msg::ComponentStatus>::SharedPtr component_status_pub_;
  pylon_ros2_camera_interfaces::msg::ComponentStatus cm_status_;
  // image transport publishers
  image_transport::CameraPublisher img_raw_pub_;
  image_transport::Publisher* img_rect_pub_;

  // services
  rclcpp::Service<GetIntegerSrv>::SharedPtr get_max_num_buffer_srv_;
  rclcpp::Service<GetIntegerSrv>::SharedPtr get_statistic_total_buffer_count_srv_;
  rclcpp::Service<GetIntegerSrv>::SharedPtr get_statistic_failed_buffer_count_srv_;
  rclcpp::Service<GetIntegerSrv>::SharedPtr get_statistic_buffer_underrun_count_srv_;
  rclcpp::Service<GetIntegerSrv>::SharedPtr get_statistic_failed_packet_count_srv_;
  rclcpp::Service<GetIntegerSrv>::SharedPtr get_statistic_resend_request_count_srv_;
  rclcpp::Service<GetIntegerSrv>::SharedPtr get_statistic_missed_frame_count_srv_;
  rclcpp::Service<GetIntegerSrv>::SharedPtr get_statistic_resynchronization_count_srv_;
  rclcpp::Service<GetIntegerSrv>::SharedPtr get_chunk_mode_active_srv_;
  rclcpp::Service<GetIntegerSrv>::SharedPtr get_chunk_selector_srv_;
  rclcpp::Service<GetIntegerSrv>::SharedPtr get_chunk_enable_srv_;
  rclcpp::Service<GetIntegerSrv>::SharedPtr get_chunk_timestamp_srv_;
  rclcpp::Service<GetIntegerSrv>::SharedPtr get_chunk_line_status_all_srv_;
  rclcpp::Service<GetIntegerSrv>::SharedPtr get_chunk_frame_counter_srv_;
  rclcpp::Service<GetIntegerSrv>::SharedPtr get_chunk_counter_value_srv_;
  
  rclcpp::Service<GetFloatSrv>::SharedPtr get_chunk_exposure_time_srv_;
  
  rclcpp::Service<SetBinningSrv>::SharedPtr set_binning_srv_;
  rclcpp::Service<SetBrightnessSrv>::SharedPtr set_brightness_srv_;
  rclcpp::Service<SetExposureSrv>::SharedPtr set_exposure_srv_;
  rclcpp::Service<SetGainSrv>::SharedPtr set_gain_srv_;
  rclcpp::Service<SetGammaSrv>::SharedPtr set_gamma_srv_;
  rclcpp::Service<SetROISrv>::SharedPtr set_roi_srv_;
  rclcpp::Service<SetSleepingSrv>::SharedPtr set_sleeping_srv_;
  rclcpp::Service<SetWhiteBalanceSrv>::SharedPtr set_white_balance_srv_;
  rclcpp::Service<SetActionTriggerConfiguration>::SharedPtr set_ac_trigger_config_srv_;
  rclcpp::Service<IssueActionCommand>::SharedPtr issue_action_command_srv_;
  rclcpp::Service<IssueScheduledActionCommand>::SharedPtr issue_scheduled_action_command_srv_;

  rclcpp::Service<SetIntegerSrv>::SharedPtr set_offset_x_srv_;
  rclcpp::Service<SetIntegerSrv>::SharedPtr set_offset_y_srv_;
  rclcpp::Service<SetIntegerSrv>::SharedPtr set_black_level_srv_;
  rclcpp::Service<SetIntegerSrv>::SharedPtr set_demosaicing_mode_srv_;
  rclcpp::Service<SetIntegerSrv>::SharedPtr set_light_source_preset_srv_;
  rclcpp::Service<SetIntegerSrv>::SharedPtr set_white_balance_auto_srv_;
  rclcpp::Service<SetIntegerSrv>::SharedPtr set_sensor_readout_mode_srv_;
  rclcpp::Service<SetIntegerSrv>::SharedPtr set_acquisition_frame_count_srv_;
  rclcpp::Service<SetIntegerSrv>::SharedPtr set_trigger_selector_srv_;
  rclcpp::Service<SetIntegerSrv>::SharedPtr set_trigger_source_srv_;
  rclcpp::Service<SetIntegerSrv>::SharedPtr set_trigger_activation_srv_;
  rclcpp::Service<SetIntegerSrv>::SharedPtr set_line_selector_srv_;
  rclcpp::Service<SetIntegerSrv>::SharedPtr set_line_mode_srv_;
  rclcpp::Service<SetIntegerSrv>::SharedPtr set_line_source_srv_;
  rclcpp::Service<SetIntegerSrv>::SharedPtr set_user_set_selector_srv_;
  rclcpp::Service<SetIntegerSrv>::SharedPtr set_user_set_default_selector_srv_;
  rclcpp::Service<SetIntegerSrv>::SharedPtr set_device_link_throughput_limit_srv_;
  rclcpp::Service<SetIntegerSrv>::SharedPtr set_max_transfer_size_srv_;
  rclcpp::Service<SetIntegerSrv>::SharedPtr set_gamma_selector_srv_; 
  rclcpp::Service<SetIntegerSrv>::SharedPtr set_grab_timeout_srv_;
  rclcpp::Service<SetIntegerSrv>::SharedPtr set_trigger_timeout_srv_;
  rclcpp::Service<SetIntegerSrv>::SharedPtr set_grabbing_strategy_srv_;
  rclcpp::Service<SetIntegerSrv>::SharedPtr set_output_queue_size_srv_;
  rclcpp::Service<SetIntegerSrv>::SharedPtr set_max_num_buffer_srv_;
  rclcpp::Service<SetIntegerSrv>::SharedPtr set_chunk_selector_srv_;
  rclcpp::Service<SetIntegerSrv>::SharedPtr set_timer_selector_srv_;
  rclcpp::Service<SetIntegerSrv>::SharedPtr set_timer_trigger_source_srv_;
  rclcpp::Service<SetIntegerSrv>::SharedPtr set_ptp_priority_srv_;
  rclcpp::Service<SetIntegerSrv>::SharedPtr set_ptp_profile_srv_;
  rclcpp::Service<SetIntegerSrv>::SharedPtr set_ptp_network_mode_srv_;
  rclcpp::Service<SetIntegerSrv>::SharedPtr set_ptp_uc_port_address_index_srv_;
  rclcpp::Service<SetIntegerSrv>::SharedPtr set_ptp_uc_port_address_srv_;
  rclcpp::Service<SetIntegerSrv>::SharedPtr set_sync_free_run_timer_start_time_low_srv_;
  rclcpp::Service<SetIntegerSrv>::SharedPtr set_sync_free_run_timer_start_time_high_srv_;

  rclcpp::Service<SetFloatSrv>::SharedPtr set_noise_reduction_srv_;
  rclcpp::Service<SetFloatSrv>::SharedPtr set_sharpness_enhancement_srv_;
  rclcpp::Service<SetFloatSrv>::SharedPtr set_trigger_delay_srv_;
  rclcpp::Service<SetFloatSrv>::SharedPtr set_line_debouncer_time_srv_;
  rclcpp::Service<SetFloatSrv>::SharedPtr set_chunk_exposure_time_srv_;
  rclcpp::Service<SetFloatSrv>::SharedPtr set_timer_duration_srv_;
  rclcpp::Service<SetFloatSrv>::SharedPtr set_periodic_signal_period_srv_;
  rclcpp::Service<SetFloatSrv>::SharedPtr set_periodic_signal_delay_srv_;
  rclcpp::Service<SetFloatSrv>::SharedPtr set_sync_free_run_timer_trigger_rate_abs_srv_;

  rclcpp::Service<SetStringSrv>::SharedPtr set_image_encoding_srv_;
  
  rclcpp::Service<SetBoolSrv>::SharedPtr set_reverse_x_srv_;
  rclcpp::Service<SetBoolSrv>::SharedPtr set_reverse_y_srv_;
  rclcpp::Service<SetBoolSrv>::SharedPtr set_PGI_mode_srv_;
  rclcpp::Service<SetBoolSrv>::SharedPtr set_trigger_mode_srv_;
  rclcpp::Service<SetBoolSrv>::SharedPtr set_line_inverter_srv_;
  rclcpp::Service<SetBoolSrv>::SharedPtr set_device_link_throughput_limit_mode_srv_;
  rclcpp::Service<SetBoolSrv>::SharedPtr set_gamma_activation_srv_;
  rclcpp::Service<SetBoolSrv>::SharedPtr set_chunk_mode_active_srv_;
  rclcpp::Service<SetBoolSrv>::SharedPtr set_chunk_enable_srv_;
  rclcpp::Service<SetBoolSrv>::SharedPtr enable_ptp_management_protocol_srv_;
  rclcpp::Service<SetBoolSrv>::SharedPtr enable_two_step_operation_srv_;
  rclcpp::Service<SetBoolSrv>::SharedPtr enable_ptp_srv_;
  rclcpp::Service<SetBoolSrv>::SharedPtr enable_sync_free_run_timer_srv_;
  
  rclcpp::Service<TriggerSrv>::SharedPtr execute_software_trigger_srv_;
  rclcpp::Service<TriggerSrv>::SharedPtr save_user_set_srv_;
  rclcpp::Service<TriggerSrv>::SharedPtr load_user_set_srv_;
  rclcpp::Service<TriggerSrv>::SharedPtr reset_device_srv_;
  rclcpp::Service<TriggerSrv>::SharedPtr start_grabbing_srv_;
  rclcpp::Service<TriggerSrv>::SharedPtr stop_grabbing_srv_;
  rclcpp::Service<TriggerSrv>::SharedPtr update_sync_free_run_timer_srv_;

  std::vector<rclcpp::Service<SetBoolSrv>::SharedPtr> set_user_output_srvs_;

  // actions
  rclcpp_action::Server<GrabImagesAction>::SharedPtr grab_imgs_raw_as_;
  rclcpp_action::Server<GrabImagesAction>::SharedPtr grab_imgs_rect_as_;

  // spinning thread
  rclcpp::TimerBase::SharedPtr timer_;
  // mutex
  std::recursive_mutex grab_mutex_;

  // intern
  std::vector<std::size_t> sampling_indices_;
  std::array<float, 256> brightness_exp_lut_;

  bool is_sleeping_;

  // diagnostics
  diagnostic_updater::Updater diagnostics_updater_;
};

} // namespace pylon_camera