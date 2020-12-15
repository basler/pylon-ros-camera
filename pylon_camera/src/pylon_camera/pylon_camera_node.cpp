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

#include <pylon_camera/pylon_camera_node.h>
#include <GenApi/GenApi.h>
#include <algorithm>
#include <cmath>
#include <vector>
#include "boost/multi_array.hpp"
//#include <dnb_msgs/ComponentStatus.h>

using diagnostic_msgs::DiagnosticStatus;

namespace pylon_camera
{

using sensor_msgs::CameraInfo;
using sensor_msgs::CameraInfoPtr;

PylonCameraNode::PylonCameraNode()
    : nh_("~"),
      pylon_camera_parameter_set_(),
      set_binning_srv_(nh_.advertiseService("set_binning",
                                            &PylonCameraNode::setBinningCallback,
                                            this)),
      set_roi_srv_(nh_.advertiseService("set_roi",
                                        &PylonCameraNode::setROICallback,
                                        this)),
      set_exposure_srv_(nh_.advertiseService("set_exposure",
                                             &PylonCameraNode::setExposureCallback,
                                             this)),
      set_gain_srv_(nh_.advertiseService("set_gain",
                                         &PylonCameraNode::setGainCallback,
                                         this)),
      set_gamma_srv_(nh_.advertiseService("set_gamma",
                                          &PylonCameraNode::setGammaCallback,
                                          this)),
      set_brightness_srv_(nh_.advertiseService("set_brightness",
                                               &PylonCameraNode::setBrightnessCallback,
                                               this)),
      set_sleeping_srv_(nh_.advertiseService("set_sleeping",
                                             &PylonCameraNode::setSleepingCallback,
                                             this)),
      set_offset_x_srv_(nh_.advertiseService("set_offset_x",
                                             &PylonCameraNode::setOffsetXCallback,
                                             this)),
      set_offset_y_srv_(nh_.advertiseService("set_offset_y",
                                             &PylonCameraNode::setOffsetYCallback,
                                             this)),
      reverse_x_srv_(nh_.advertiseService("set_reverse_x",
                                             &PylonCameraNode::setReverseXCallback,
                                             this)),
      reverse_y_srv_(nh_.advertiseService("set_reverse_y",
                                             &PylonCameraNode::setReverseYCallback,
                                             this)),
      set_black_level_srv_(nh_.advertiseService("set_black_level",
                                             &PylonCameraNode::setBlackLevelCallback,
                                             this)),
      set_PGI_mode_srv_(nh_.advertiseService("set_pgi_mode",
                                             &PylonCameraNode::setPGIModeCallback,
                                             this)),
      set_demosaicing_mode_srv_(nh_.advertiseService("set_demosaicing_mode",
                                             &PylonCameraNode::setDemosaicingModeCallback,
                                             this)),
      set_noise_reduction_srv_(nh_.advertiseService("set_noise_reduction",
                                             &PylonCameraNode::setNoiseReductionCallback,
                                             this)),
      set_sharpness_enhancement_srv_(nh_.advertiseService("set_sharpness_enhancement",
                                             &PylonCameraNode::setSharpnessEnhancementCallback,
                                             this)),
      set_light_source_preset_srv_(nh_.advertiseService("set_light_source_preset",
                                             &PylonCameraNode::setLightSourcePresetCallback,
                                             this)),
      set_balance_white_auto_srv_(nh_.advertiseService("set_balance_white_auto",
                                             &PylonCameraNode::setBalanceWhiteAutoCallback,
                                             this)),
      set_sensor_readout_mode_srv_(nh_.advertiseService("set_sensor_readout_mode",
                                             &PylonCameraNode::setSensorReadoutModeCallback,
                                             this)),
      set_acquisition_frame_count_srv_(nh_.advertiseService("set_acquisition_frame_count",
                                             &PylonCameraNode::setAcquisitionFrameCountCallback,
                                             this)),
      set_trigger_selector_srv_(nh_.advertiseService("set_trigger_selector",
                                             &PylonCameraNode::setTriggerSelectorCallback,
                                             this)),
      set_trigger_mode_srv_(nh_.advertiseService("set_trigger_mode",
                                             &PylonCameraNode::setTriggerModeCallback,
                                             this)),
      execute_software_trigger_srv_(nh_.advertiseService("execute_software_trigger",
                                             &PylonCameraNode::executeSoftwareTriggerCallback,
                                             this)),
      set_trigger_source_srv_(nh_.advertiseService("set_trigger_source",
                                             &PylonCameraNode::setTriggerSourceCallback,
                                             this)),
      set_trigger_activation_srv_(nh_.advertiseService("set_trigger_activation",
                                             &PylonCameraNode::setTriggerActivationCallback,
                                             this)),
      set_trigger_delay_srv_(nh_.advertiseService("set_trigger_delay",
                                             &PylonCameraNode::setTriggerDelayCallback,
                                             this)),
      set_line_selector_srv_(nh_.advertiseService("set_line_selector",
                                             &PylonCameraNode::setLineSelectorCallback,
                                             this)),
      set_line_mode_srv_(nh_.advertiseService("set_line_mode",
                                             &PylonCameraNode::setLineModeCallback,
                                             this)),
      set_line_source_srv_(nh_.advertiseService("set_line_source",
                                             &PylonCameraNode::setLineSourceCallback,
                                             this)),
      set_line_inverter_srv_(nh_.advertiseService("set_line_inverter",
                                             &PylonCameraNode::setLineInverterCallback,
                                             this)),
      set_line_debouncer_time_srv_(nh_.advertiseService("set_line_debouncer_time",
                                             &PylonCameraNode::setLineDebouncerTimeCallback,
                                             this)),
      set_user_set_selector_srv_(nh_.advertiseService("select_user_set",
                                             &PylonCameraNode::setUserSetSelectorCallback,
                                             this)),
      save_user_set_srv_(nh_.advertiseService("save_user_set",
                                             &PylonCameraNode::saveUserSetCallback,
                                             this)),
      load_user_set_srv_(nh_.advertiseService("load_user_set",
                                             &PylonCameraNode::loadUserSetCallback,
                                             this)),
      set_user_set_default_selector_srv_(nh_.advertiseService("select_default_user_set",
                                             &PylonCameraNode::setUserSetDefaultSelectorCallback,
                                             this)),
      set_device_link_throughput_limit_mode_srv_(nh_.advertiseService("set_device_link_throughput_limit_mode",
                                             &PylonCameraNode::setDeviceLinkThroughputLimitModeCallback,
                                             this)),
      set_device_link_throughput_limit_srv_(nh_.advertiseService("set_device_link_throughput_limit",
                                             &PylonCameraNode::setDeviceLinkThroughputLimitCallback,
                                             this)),
      reset_device_srv_(nh_.advertiseService("reset_device",
                                             &PylonCameraNode::triggerDeviceResetCallback,
                                             this)),
      start_grabbing_srv_(nh_.advertiseService("start_grabbing",
                                             &PylonCameraNode::StartGrabbingCallback,
                                             this)),
      stop_grabbing_srv_(nh_.advertiseService("stop_grabbing",
                                             &PylonCameraNode::StopGrabbingCallback,
                                             this)),
      set_image_encoding_srv_(nh_.advertiseService("set_image_encoding",
                                             &PylonCameraNode::setImageEncodingCallback,
                                             this)),
      set_max_transfer_size_srv_(nh_.advertiseService("set_max_transfer_size",
                                             &PylonCameraNode::setMaxTransferSizeCallback,
                                             this)),
      set_gamma_selector_srv(nh_.advertiseService("set_gamma_selector",
                                             &PylonCameraNode::setGammaSelectorCallback,
                                             this)),
      gamma_enable_srv(nh_.advertiseService("gamma_enable",
                                             &PylonCameraNode::gammaEnableCallback,
                                             this)),
      set_grab_timeout_srv(nh_.advertiseService("set_grab_timeout",
                                             &PylonCameraNode::setGrabTimeoutCallback,
                                             this)),
      set_trigger_timeout_srv(nh_.advertiseService("set_trigger_timeout",
                                             &PylonCameraNode::setTriggerTimeoutCallback,
                                             this)),
      set_grabbing_strategy_srv(nh_.advertiseService("set_grabbing_strategy",
                                             &PylonCameraNode::setGrabbingStrategyCallback,
                                             this)),
      set_output_queue_size_srv(nh_.advertiseService("set_output_queue_size",
                                             &PylonCameraNode::setOutputQueueSizeCallback,
                                             this)),
      set_white_balance_srv(nh_.advertiseService("set_white_balance",
                                             &PylonCameraNode::setWhiteBalanceCallback,
                                             this)),
      set_user_output_srvs_(),
      pylon_camera_(nullptr),
      it_(new image_transport::ImageTransport(nh_)),
      img_raw_pub_(it_->advertiseCamera("image_raw", 1)),
      img_rect_pub_(nullptr),
      grab_imgs_raw_as_(
              nh_,
              "grab_images_raw",
              boost::bind(&PylonCameraNode::grabImagesRawActionExecuteCB,
                          this,
                          _1),
              false),
      grab_imgs_rect_as_(nullptr),
      pinhole_model_(nullptr),
      cv_bridge_img_rect_(nullptr),
      camera_info_manager_(new camera_info_manager::CameraInfoManager(nh_)),
      sampling_indices_(),
      brightness_exp_lut_(),
      is_sleeping_(false)
{
    diagnostics_updater_.setHardwareID("none");
    diagnostics_updater_.add("camera_availability", this, &PylonCameraNode::create_diagnostics);
    diagnostics_updater_.add("intrinsic_calibration", this, &PylonCameraNode::create_camera_info_diagnostics);
    diagnostics_trigger_ = nh_.createTimer(ros::Duration(2), &PylonCameraNode::diagnostics_timer_callback_, this);
    componentStatusPublisher = nh_.advertise<dnb_msgs::ComponentStatus>("/pylon_camera_node/status", 5, true); // DNB component status publisher
    currentParamsPublisher = nh_.advertise<camera_control_msgs::currentParams>("/pylon_camera_node/currentParams", 5, true); // current camera params publisher

    init();
}

void PylonCameraNode::create_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    if (pylon_camera_parameter_set_.deviceUserID().empty())
    {
        return;
    }

    if (pylon_camera_)
    {
        diagnostics_updater_.setHardwareID( pylon_camera_->deviceUserID());
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Device is connected");
    }
    else
    {
        diagnostics_updater_.setHardwareID(pylon_camera_parameter_set_.deviceUserID());
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "No camera connected");
        cm_status.status_id = dnb_msgs::ComponentStatus::ERROR;
        cm_status.status_msg = "No camera connected";
        if (pylon_camera_parameter_set_.enable_status_publisher_)
        {
          componentStatusPublisher.publish(cm_status);
        }
    }
}

void PylonCameraNode::create_camera_info_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    if (camera_info_manager_->isCalibrated())
    {
        stat.summaryf(DiagnosticStatus::OK, "Intrinsic calibration found");
    }else{
        stat.summaryf(DiagnosticStatus::ERROR, "No intrinsic calibration found");
    }
}

void PylonCameraNode::diagnostics_timer_callback_(const ros::TimerEvent&)
{
    diagnostics_updater_.update();
}

void PylonCameraNode::init()
{
    // reading all necessary parameter to open the desired camera from the
    // ros-parameter-server. In case that invalid parameter values can be
    // detected, the interface will reset them to the default values.
    // These parameters furthermore contain the intrinsic calibration matrices,
    // in case they are provided
    pylon_camera_parameter_set_.readFromRosParameterServer(nh_);

    // creating the target PylonCamera-Object with the specified
    // device_user_id, registering the Software-Trigger-Mode, starting the
    // communication with the device and enabling the desired startup-settings
    if ( !initAndRegister() )
    {
        ros::shutdown();
        return;
    }

    // starting the grabbing procedure with the desired image-settings
    if ( !startGrabbing() )
    {
        ros::shutdown();
        return;
    }
}

bool PylonCameraNode::initAndRegister()
{
    pylon_camera_ = PylonCamera::create(
                                    pylon_camera_parameter_set_.deviceUserID());

    if ( pylon_camera_ == nullptr )
    {
        cm_status.status_id = dnb_msgs::ComponentStatus::ERROR;
        cm_status.status_msg = "No camera present";
        if (pylon_camera_parameter_set_.enable_status_publisher_)
          {
            componentStatusPublisher.publish(cm_status);
          }
        // wait and retry until a camera is present
        ros::Time end = ros::Time::now() + ros::Duration(15.0);
        ros::Rate r(0.5);
        while ( ros::ok() && pylon_camera_ == nullptr )
        {
            pylon_camera_ = PylonCamera::create(
                                    pylon_camera_parameter_set_.deviceUserID());
            if ( ros::Time::now() > end )
            {
                ROS_WARN_STREAM("No camera present. Keep waiting ...");
                cm_status.status_id = dnb_msgs::ComponentStatus::ERROR;
                cm_status.status_msg = "No camera present";
                if (pylon_camera_parameter_set_.enable_status_publisher_)
                {
                  componentStatusPublisher.publish(cm_status);
                }
                end = ros::Time::now() + ros::Duration(15.0);
            }
            r.sleep();
            ros::spinOnce();
        }
    }

    if (pylon_camera_ != nullptr && pylon_camera_parameter_set_.deviceUserID().empty())
    {
        pylon_camera_parameter_set_.adaptDeviceUserId(nh_, pylon_camera_->deviceUserID());
        cm_status.status_id = dnb_msgs::ComponentStatus::RUNNING;
        cm_status.status_msg = "running";
        if (pylon_camera_parameter_set_.enable_status_publisher_)
        {
          componentStatusPublisher.publish(cm_status);
        }
    }

    if ( !ros::ok() )
    {
        return false;
    }

    if ( !pylon_camera_->registerCameraConfiguration() )
    {
        ROS_ERROR_STREAM("Error while registering the camera configuration to "
            << "software-trigger mode!");
        cm_status.status_id = dnb_msgs::ComponentStatus::ERROR;
        cm_status.status_msg = "Error while registering the camera configuration";
        if (pylon_camera_parameter_set_.enable_status_publisher_)
        {
          componentStatusPublisher.publish(cm_status);
        }
        return false;
    }

    if ( !pylon_camera_->openCamera() )
    {
        ROS_ERROR("Error while trying to open the desired camera!");
        cm_status.status_id = dnb_msgs::ComponentStatus::ERROR;
        cm_status.status_msg = "Error while trying to open the desired camera!";
        if (pylon_camera_parameter_set_.enable_status_publisher_)
        {
          componentStatusPublisher.publish(cm_status);
        }
        return false;
    }

    if ( !pylon_camera_->applyCamSpecificStartupSettings(pylon_camera_parameter_set_) )
    {
        ROS_ERROR_STREAM("Error while applying the cam specific startup settings "
                << "(e.g. mtu size for GigE, ...) to the camera!");
        cm_status.status_id = dnb_msgs::ComponentStatus::ERROR;
        cm_status.status_msg = "Error while applying the cam specific startup settings";
        if (pylon_camera_parameter_set_.enable_status_publisher_)
        {
          componentStatusPublisher.publish(cm_status);
        }
        return false;
    }

    return true;
}

bool PylonCameraNode::startGrabbing()
{
    if ( !pylon_camera_->startGrabbing(pylon_camera_parameter_set_) )
    {
        ROS_ERROR("Error while start grabbing");
        return false;
    }

    size_t num_user_outputs = pylon_camera_->numUserOutputs();
    set_user_output_srvs_.resize(2*num_user_outputs);
    for ( int i = 0; i < num_user_outputs; ++i )
    {
        std::string srv_name = "set_user_output_" + std::to_string(i);
        std::string srv_name_af = "activate_autoflash_output_" + std::to_string(i);
        if (! ros::service::exists("/pylon_camera_node/"+srv_name, false))
        {
        set_user_output_srvs_.at(i) =
            nh_.advertiseService< std_srvs::SetBool::Request,
                                  std_srvs::SetBool::Response >(
                                    srv_name,
                                    boost::bind(&PylonCameraNode::setUserOutputCB,
                                                this, i ,_1 ,_2));
        }
        if (! ros::service::exists("/pylon_camera_node/"+srv_name_af, false))
        {
        set_user_output_srvs_.at(num_user_outputs+i) =
            nh_.advertiseService< std_srvs::SetBool::Request,
                                  std_srvs::SetBool::Response >(
                                    srv_name_af,
                                    boost::bind(&PylonCameraNode::setAutoflash,
                                                this, i+2, _1, _2)); // ! using lines 2 and 3
        }
    }
    img_raw_msg_.header.frame_id = pylon_camera_parameter_set_.cameraFrame();
    // Encoding of pixels -- channel meaning, ordering, size
    // taken from the list of strings in include/sensor_msgs/image_encodings.h
    img_raw_msg_.encoding = pylon_camera_->currentROSEncoding();
    img_raw_msg_.height = pylon_camera_->imageRows();
    img_raw_msg_.width = pylon_camera_->imageCols();
    // step = full row length in bytes, img_size = (step * rows), imagePixelDepth
    // already contains the number of channels
    img_raw_msg_.step = img_raw_msg_.width * pylon_camera_->imagePixelDepth();
    if ( !camera_info_manager_->setCameraName(pylon_camera_->deviceUserID()) )
    { 
        // valid name contains only alphanumeric signs and '_'
        ROS_WARN_STREAM("[" << pylon_camera_->deviceUserID()
                << "] name not valid for camera_info_manager");
    }
    setupSamplingIndices(sampling_indices_,
                         pylon_camera_->imageRows(),
                         pylon_camera_->imageCols(),
                         pylon_camera_parameter_set_.downsampling_factor_exp_search_);
    grab_imgs_raw_as_.start();

    // Initial setting of the CameraInfo-msg, assuming no calibration given
    CameraInfo initial_cam_info;
    setupInitialCameraInfo(initial_cam_info);
    camera_info_manager_->setCameraInfo(initial_cam_info);
    if ( pylon_camera_parameter_set_.cameraInfoURL().empty() ||
         !camera_info_manager_->validateURL(pylon_camera_parameter_set_.cameraInfoURL()) )
    { 
        ROS_INFO_STREAM("CameraInfoURL needed for rectification! ROS-Param: "
            << "'" << nh_.getNamespace() << "/camera_info_url' = '"
            << pylon_camera_parameter_set_.cameraInfoURL() << "' is invalid!");
        ROS_DEBUG_STREAM("CameraInfoURL should have following style: "
            << "'file:///full/path/to/local/file.yaml' or "
            << "'file://${ROS_HOME}/camera_info/${NAME}.yaml'");
        ROS_WARN("Will only provide distorted /image_raw images!");
    }
    else
    { 
        // override initial camera info if the url is valid
        if ( camera_info_manager_->loadCameraInfo(
                                pylon_camera_parameter_set_.cameraInfoURL()) )
        {
            setupRectification();
            // set the correct tf frame_id
            CameraInfoPtr cam_info(new CameraInfo(
                                        camera_info_manager_->getCameraInfo()));
            cam_info->header.frame_id = img_raw_msg_.header.frame_id;
            camera_info_manager_->setCameraInfo(*cam_info);
        }
        else
        { 
            ROS_WARN("Will only provide distorted /image_raw images!");
        }
    }
    if ( pylon_camera_parameter_set_.binning_x_given_ )
    {   
        size_t reached_binning_x;
        setBinningX(pylon_camera_parameter_set_.binning_x_, reached_binning_x);
        ROS_INFO_STREAM("Setting horizontal binning_x to "
                << pylon_camera_parameter_set_.binning_x_);
        ROS_WARN_STREAM("The image width of the camera_info-msg will "
            << "be adapted, so that the binning_x value in this msg remains 1");
    }
    if ( pylon_camera_parameter_set_.binning_y_given_ )
    {   
        size_t reached_binning_y;
        setBinningY(pylon_camera_parameter_set_.binning_y_, reached_binning_y);
        ROS_INFO_STREAM("Setting vertical binning_y to "
                << pylon_camera_parameter_set_.binning_y_);
        ROS_WARN_STREAM("The image height of the camera_info-msg will "
            << "be adapted, so that the binning_y value in this msg remains 1");
    }
    if ( pylon_camera_parameter_set_.exposure_given_ )
    {   
        float reached_exposure;
        setExposure(pylon_camera_parameter_set_.exposure_, reached_exposure);
        ROS_INFO_STREAM("Setting exposure to "
                << pylon_camera_parameter_set_.exposure_ << ", reached: "
                << reached_exposure);
    }
    if ( pylon_camera_parameter_set_.gain_given_ )
    {   
        float reached_gain;
        setGain(pylon_camera_parameter_set_.gain_, reached_gain);
        ROS_INFO_STREAM("Setting gain to: "
                << pylon_camera_parameter_set_.gain_ << ", reached: "
                << reached_gain);
    }
    if ( pylon_camera_parameter_set_.gamma_given_ )
    {   
        float reached_gamma;
        setGamma(pylon_camera_parameter_set_.gamma_, reached_gamma);
        ROS_INFO_STREAM("Setting gamma to " << pylon_camera_parameter_set_.gamma_
                << ", reached: " << reached_gamma);
    }
    if ( pylon_camera_parameter_set_.brightness_given_ )
    {   
        int reached_brightness;
        setBrightness(pylon_camera_parameter_set_.brightness_,
                      reached_brightness,
                      pylon_camera_parameter_set_.exposure_auto_,
                      pylon_camera_parameter_set_.gain_auto_);
        ROS_INFO_STREAM("Setting brightness to: "
                << pylon_camera_parameter_set_.brightness_ << ", reached: "
                << reached_brightness);
        if ( pylon_camera_parameter_set_.brightness_continuous_ )
        {   
            if ( pylon_camera_parameter_set_.exposure_auto_ )
            {
                pylon_camera_->enableContinuousAutoExposure();
            }
            if ( pylon_camera_parameter_set_.gain_auto_ )
            {
                pylon_camera_->enableContinuousAutoGain();
            }
        }
        else
        { 
            pylon_camera_->disableAllRunningAutoBrightessFunctions();
        }
    }

    ROS_INFO_STREAM("Startup settings: "
            << "encoding = '" << pylon_camera_->currentROSEncoding() << "', "
            << "binning = [" << pylon_camera_->currentBinningX() << ", "
            << pylon_camera_->currentBinningY() << "], "
            << "exposure = " << pylon_camera_->currentExposure() << ", "
            << "gain = " << pylon_camera_->currentGain() << ", "
            << "gamma = " <<  pylon_camera_->currentGamma() << ", "
            << "shutter mode = "
            << pylon_camera_parameter_set_.shutterModeString());
    // Framerate Settings
    if ( pylon_camera_->maxPossibleFramerate() < pylon_camera_parameter_set_.frameRate() )
    {   
        ROS_INFO("Desired framerate %.2f is higher than max possible. Will limit framerate to: %.2f Hz",
                 pylon_camera_parameter_set_.frameRate(),
                 pylon_camera_->maxPossibleFramerate());
        pylon_camera_parameter_set_.setFrameRate(
                nh_,
                pylon_camera_->maxPossibleFramerate());
    }
    else if ( pylon_camera_parameter_set_.frameRate() == -1 )
    {     
        pylon_camera_parameter_set_.setFrameRate(nh_,
                                                 pylon_camera_->maxPossibleFramerate());
        ROS_INFO("Max possible framerate is %.2f Hz",
                 pylon_camera_->maxPossibleFramerate());
    }
    return true;
}

void PylonCameraNode::setupRectification()
{
    if ( !img_rect_pub_ )
    {
        img_rect_pub_ = new image_transport::Publisher(it_->advertise("image_rect", 1));
    }

    if ( !grab_imgs_rect_as_ )
    {
        grab_imgs_rect_as_ =
            new GrabImagesAS(nh_,
                             "grab_images_rect",
                             boost::bind(
                                &PylonCameraNode::grabImagesRectActionExecuteCB,
                                this,
                                _1),
                             false);
        grab_imgs_rect_as_->start();
    }

    if ( !pinhole_model_ )
    {
        pinhole_model_ = new image_geometry::PinholeCameraModel();
    }

    pinhole_model_->fromCameraInfo(camera_info_manager_->getCameraInfo());
    if ( !cv_bridge_img_rect_ )
    {
        cv_bridge_img_rect_ = new cv_bridge::CvImage();
    }
    cv_bridge_img_rect_->header = img_raw_msg_.header;
    cv_bridge_img_rect_->encoding = img_raw_msg_.encoding;
}


struct CameraPublisherImpl
{
  image_transport::Publisher image_pub_;
  ros::Publisher info_pub_;
  bool unadvertised_;
  //double constructed_;
};

class CameraPublisherLocal
{
public:
  struct Impl;
  typedef boost::shared_ptr<Impl> ImplPtr;
  typedef boost::weak_ptr<Impl> ImplWPtr;
  
  CameraPublisherImpl* impl_;
};

uint32_t  PylonCameraNode::getNumSubscribersRaw() const
{
    return ((CameraPublisherLocal*)(&img_raw_pub_))->impl_->image_pub_.getNumSubscribers();
}

void PylonCameraNode::spin()
{   
    if ( camera_info_manager_->isCalibrated() )
    {
        ROS_INFO_ONCE("Camera is calibrated");
    }
    else
    {
        ROS_INFO_ONCE("Camera not calibrated");
    }

    if ( pylon_camera_->isCamRemoved() )
    {
        ROS_ERROR("Pylon camera has been removed, trying to reset");
        cm_status.status_id = dnb_msgs::ComponentStatus::ERROR;
        cm_status.status_msg = "Pylon camera has been removed, trying to reset";
        if (pylon_camera_parameter_set_.enable_status_publisher_)
        {
          componentStatusPublisher.publish(cm_status);
        }
        delete pylon_camera_;
        pylon_camera_ = nullptr;
        for ( ros::ServiceServer& user_output_srv : set_user_output_srvs_ )
        {
            user_output_srv.shutdown();
        }
        set_user_output_srvs_.clear();
        ros::Duration(0.5).sleep();  // sleep for half a second
        init();
        return;
    }
    // images were published if subscribers are available or if someone calls
    // the GrabImages Action
    if ( !isSleeping() && (img_raw_pub_.getNumSubscribers() || getNumSubscribersRect() ) )
    { 
        if ( getNumSubscribersRaw() || getNumSubscribersRect())
        { 
            if (!grabImage() )
            { 
                return;
            }
        }
        if ( img_raw_pub_.getNumSubscribers() > 0 )
        { 
            // get actual cam_info-object in every frame, because it might have
            // changed due to a 'set_camera_info'-service call
            sensor_msgs::CameraInfoPtr cam_info(
                        new sensor_msgs::CameraInfo(
                                        camera_info_manager_->getCameraInfo()));
            cam_info->header.stamp = img_raw_msg_.header.stamp;

            // Publish via image_transport
            img_raw_pub_.publish(img_raw_msg_, *cam_info);
        }
        if ( getNumSubscribersRect() > 0 && camera_info_manager_->isCalibrated() )
        { 
            cv_bridge_img_rect_->header.stamp = img_raw_msg_.header.stamp;
            assert(pinhole_model_->initialized());
            cv_bridge::CvImagePtr cv_img_raw = cv_bridge::toCvCopy(
                    img_raw_msg_,
                    img_raw_msg_.encoding);
            pinhole_model_->fromCameraInfo(camera_info_manager_->getCameraInfo());
            pinhole_model_->rectifyImage(cv_img_raw->image, cv_bridge_img_rect_->image);
            img_rect_pub_->publish(cv_bridge_img_rect_->toImageMsg());
        }
    }
    // Check if the image encoding changed , then save the new image encoding and restart the image grabbing to fix the ros sensor message type issue.
    if (pylon_camera_parameter_set_.imageEncoding() != pylon_camera_->currentROSEncoding()) 
      {
        pylon_camera_parameter_set_.setimageEncodingParam(nh_,pylon_camera_->currentROSEncoding());
        grabbingStopping();
        grabbingStarting();
      }
    if (pylon_camera_parameter_set_.enable_status_publisher_)
    { 
      componentStatusPublisher.publish(cm_status);
    } 
    if (pylon_camera_parameter_set_.enable_current_params_publisher_)
    { 
      currentParamPub();
    } 
}

bool PylonCameraNode::grabImage()
{
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_); 
    if ( !pylon_camera_->grab(img_raw_msg_.data) )
    {
        return false;
    }
    img_raw_msg_.header.stamp = ros::Time::now(); 
    return true;
}

void PylonCameraNode::grabImagesRawActionExecuteCB(
                    const camera_control_msgs::GrabImagesGoal::ConstPtr& goal)
{
    camera_control_msgs::GrabImagesResult result;
    result = grabImagesRaw(goal, &grab_imgs_raw_as_);
    grab_imgs_raw_as_.setSucceeded(result);
}

void PylonCameraNode::grabImagesRectActionExecuteCB(
                    const camera_control_msgs::GrabImagesGoal::ConstPtr& goal)
{
    camera_control_msgs::GrabImagesResult result;
    if ( !camera_info_manager_->isCalibrated() )
    {
        result.success = false;
        grab_imgs_rect_as_->setSucceeded(result);
        return;
    }
    else
    {
        result = grabImagesRaw(goal, std::ref(grab_imgs_rect_as_));
        if ( !result.success )
        {
            grab_imgs_rect_as_->setSucceeded(result);
            return;
        }

        for ( std::size_t i = 0; i < result.images.size(); ++i)
        {
            cv_bridge::CvImagePtr cv_img_raw = cv_bridge::toCvCopy(
                                                        result.images[i],
                                                        result.images[i].encoding);
            pinhole_model_->fromCameraInfo(camera_info_manager_->getCameraInfo());
            cv_bridge::CvImage cv_bridge_img_rect;
            cv_bridge_img_rect.header = result.images[i].header;
            cv_bridge_img_rect.encoding = result.images[i].encoding;
            pinhole_model_->rectifyImage(cv_img_raw->image, cv_bridge_img_rect.image);
            cv_bridge_img_rect.toImageMsg(result.images[i]);
        }
        grab_imgs_rect_as_->setSucceeded(result);
    }
}

camera_control_msgs::GrabImagesResult PylonCameraNode::grabImagesRaw(
        const camera_control_msgs::GrabImagesGoal::ConstPtr& goal,
        GrabImagesAS* action_server)
{
    camera_control_msgs::GrabImagesResult result;
    camera_control_msgs::GrabImagesFeedback feedback;

#if DEBUG
    std::cout << *goal << std::endl;
#endif

    if ( goal->exposure_given && goal->exposure_times.empty() )
    {
        ROS_ERROR_STREAM("GrabImagesRaw action server received request and "
            << "'exposure_given' is true, but the 'exposure_times' vector is "
            << "empty! Not enough information to execute acquisition!");
        result.success = false;
        return result;
    }

    if ( goal->gain_given && goal->gain_values.empty() )
    {
        ROS_ERROR_STREAM("GrabImagesRaw action server received request and "
            << "'gain_given' is true, but the 'gain_values' vector is "
            << "empty! Not enough information to execute acquisition!");
        result.success = false;
        return result;
    }

    if ( goal->brightness_given && goal->brightness_values.empty() )
    {
        ROS_ERROR_STREAM("GrabImagesRaw action server received request and "
            << "'brightness_given' is true, but the 'brightness_values' vector"
            << " is empty! Not enough information to execute acquisition!");
        result.success = false;
        return result;
    }

    if ( goal->gamma_given && goal->gamma_values.empty() )
    {
        ROS_ERROR_STREAM("GrabImagesRaw action server received request and "
            << "'gamma_given' is true, but the 'gamma_values' vector is "
            << "empty! Not enough information to execute acquisition!");
        result.success = false;
        return result;
    }

    std::vector<size_t> candidates;
    candidates.resize(4);  // gain, exposure, gamma, brightness
    candidates.at(0) = goal->gain_given ? goal->gain_values.size() : 0;
    candidates.at(1) = goal->exposure_given ? goal->exposure_times.size() : 0;
    candidates.at(2) = goal->brightness_given ? goal->brightness_values.size() : 0;
    candidates.at(3) = goal->gamma_given ? goal->gamma_values.size() : 0;

    size_t n_images = *std::max_element(candidates.begin(), candidates.end());

    if ( goal->exposure_given && goal->exposure_times.size() != n_images )
    {
        ROS_ERROR_STREAM("Size of requested exposure times does not match to "
            << "the size of the requested vaules of brightness, gain or "
            << "gamma! Can't grab!");
        result.success = false;
        return result;
    }

    if ( goal->gain_given && goal->gain_values.size() != n_images )
    {
        ROS_ERROR_STREAM("Size of requested gain values does not match to "
            << "the size of the requested exposure times or the vaules of "
            << "brightness or gamma! Can't grab!");
        result.success = false;
        return result;
    }

    if ( goal->gamma_given && goal->gamma_values.size() != n_images )
    {
        ROS_ERROR_STREAM("Size of requested gamma values does not match to "
            << "the size of the requested exposure times or the vaules of "
            << "brightness or gain! Can't grab!");
        result.success = false;
        return result;
    }

    if ( goal->brightness_given && goal->brightness_values.size() != n_images )
    {
        ROS_ERROR_STREAM("Size of requested brightness values does not match to "
            << "the size of the requested exposure times or the vaules of gain or "
            << "gamma! Can't grab!");
        result.success = false;
        return result;
    }

    if ( goal->brightness_given && !( goal->exposure_auto || goal->gain_auto ) )
    {
        ROS_ERROR_STREAM("Error while executing the GrabImagesRawAction: A "
            << "target brightness is provided but Exposure time AND gain are "
            << "declared as fix, so its impossible to reach the brightness");
        result.success = false;
        return result;
    }

    result.images.resize(n_images);
    result.reached_exposure_times.resize(n_images);
    result.reached_gain_values.resize(n_images);
    result.reached_gamma_values.resize(n_images);
    result.reached_brightness_values.resize(n_images);

    result.success = true;

    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);

    float previous_exp, previous_gain, previous_gamma;
    if ( goal->exposure_given )
    {
        previous_exp = pylon_camera_->currentExposure();
    }
    if ( goal->gain_given )
    {
        previous_gain = pylon_camera_->currentGain();
    }
    if ( goal->gamma_given )
    {
        previous_gamma = pylon_camera_->currentGamma();
    }
    if ( goal->brightness_given )
    {
        previous_gain = pylon_camera_->currentGain();
        previous_exp = pylon_camera_->currentExposure();
    }

    for ( std::size_t i = 0; i < n_images; ++i )
    {
        if ( goal->exposure_given )
        {
            result.success = setExposure(goal->exposure_times[i],
                                         result.reached_exposure_times[i]);
        }
        if ( goal->gain_given )
        {
            result.success = setGain(goal->gain_values[i],
                                     result.reached_gain_values[i]);
        }
        if ( goal->gamma_given )
        {
            result.success = setGamma(goal->gamma_values[i],
                                      result.reached_gamma_values[i]);
        }
        if ( goal->brightness_given )
        {
            int reached_brightness;
            result.success = setBrightness(goal->brightness_values[i],
                                           reached_brightness,
                                           goal->exposure_auto,
                                           goal->gain_auto);
            result.reached_brightness_values[i] = static_cast<float>(
                                                            reached_brightness);
            result.reached_exposure_times[i] = pylon_camera_->currentExposure();
            result.reached_gain_values[i] = pylon_camera_->currentGain();
        }
        if ( !result.success )
        {
            ROS_ERROR_STREAM("Error while setting one of the desired image "
                << "properties in the GrabImagesRawActionCB. Aborting!");
            break;
        }

        sensor_msgs::Image& img = result.images[i];
        img.encoding = pylon_camera_->currentROSEncoding();
        img.height = pylon_camera_->imageRows();
        img.width = pylon_camera_->imageCols();
        // step = full row length in bytes, img_size = (step * rows), imagePixelDepth
        // already contains the number of channels
        img.step = img.width * pylon_camera_->imagePixelDepth();

        if ( !pylon_camera_->grab(img.data) )
        {
            result.success = false;
            break;
        }

        img.header.stamp = ros::Time::now();
        img.header.frame_id = cameraFrame();
        feedback.curr_nr_images_taken = i+1;

        if ( action_server != nullptr )
        {
            action_server->publishFeedback(feedback);
        }
    }
    if ( camera_info_manager_ )
    {
        sensor_msgs::CameraInfoPtr cam_info(
                                new sensor_msgs::CameraInfo(
                                    camera_info_manager_->getCameraInfo()));
        result.cam_info = *cam_info;
    }

    // restore previous settings:
    float reached_val;
    if ( goal->exposure_given )
    {
        setExposure(previous_exp, reached_val);
    }
    if ( goal->gain_given )
    {
        setGain(previous_gain, reached_val);
    }
    if ( goal->gamma_given )
    {
        setGamma(previous_gamma, reached_val);
    }
    if ( goal->brightness_given )
    {
        setGain(previous_gain, reached_val);
        setExposure(previous_exp, reached_val);
    }
    return result;
}

bool PylonCameraNode::setUserOutputCB(const int output_id,
                                      std_srvs::SetBool::Request &req,
                                      std_srvs::SetBool::Response &res)
{
    res.success = pylon_camera_->setUserOutput(output_id, req.data);
    return true;
}

bool PylonCameraNode::setAutoflash(const int output_id,
                                   std_srvs::SetBool::Request &req,
                                   std_srvs::SetBool::Response &res)
{
    ROS_INFO("AUtoFlashCB: %i -> %i", output_id, req.data);
    std::map<int, bool> auto_flashs;
    auto_flashs[output_id] = req.data;
    pylon_camera_->setAutoflash(auto_flashs);
    res.success = true;
    return true;
}

const double& PylonCameraNode::frameRate() const
{
    return pylon_camera_parameter_set_.frameRate();
}

const std::string& PylonCameraNode::cameraFrame() const
{
    return pylon_camera_parameter_set_.cameraFrame();
}

uint32_t PylonCameraNode::getNumSubscribersRect() const
{
    return camera_info_manager_->isCalibrated() ? img_rect_pub_->getNumSubscribers() : 0;
}

uint32_t PylonCameraNode::getNumSubscribers() const
{
    return img_raw_pub_.getNumSubscribers() + img_rect_pub_->getNumSubscribers();
}

void PylonCameraNode::setupInitialCameraInfo(sensor_msgs::CameraInfo& cam_info_msg)
{
    std_msgs::Header header;
    header.frame_id = pylon_camera_parameter_set_.cameraFrame();
    header.stamp = ros::Time::now();

    // http://www.ros.org/reps/rep-0104.html
    // If the camera is uncalibrated, the matrices D, K, R, P should be left
    // zeroed out. In particular, clients may assume that K[0] == 0.0
    // indicates an uncalibrated camera.
    cam_info_msg.header = header;

    // The image dimensions with which the camera was calibrated. Normally
    // this will be the full camera resolution in pixels. They remain fix, even
    // if binning is applied
    cam_info_msg.height = pylon_camera_->imageRows();
    cam_info_msg.width = pylon_camera_->imageCols();

    // The distortion model used. Supported models are listed in
    // sensor_msgs/distortion_models.h. For most cameras, "plumb_bob" - a
    // simple model of radial and tangential distortion - is sufficient.
    // Empty D and distortion_model indicate that the CameraInfo cannot be used
    // to rectify points or images, either because the camera is not calibrated
    // or because the rectified image was produced using an unsupported
    // distortion model, e.g. the proprietary one used by Bumblebee cameras
    // [http://www.ros.org/reps/rep-0104.html].
    cam_info_msg.distortion_model = "";

    // The distortion parameters, size depending on the distortion model.
    // For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3) -> float64[] D.
    cam_info_msg.D = std::vector<double>(5, 0.);

    // Intrinsic camera matrix for the raw (distorted) images.
    //     [fx  0 cx]
    // K = [ 0 fy cy]  --> 3x3 row-major matrix
    //     [ 0  0  1]
    // Projects 3D points in the camera coordinate frame to 2D pixel coordinates
    // using the focal lengths (fx, fy) and principal point (cx, cy).
    cam_info_msg.K.assign(0.0);

    // Rectification matrix (stereo cameras only)
    // A rotation matrix aligning the camera coordinate system to the ideal
    // stereo image plane so that epipolar lines in both stereo images are parallel.
    cam_info_msg.R.assign(0.0);

    // Projection/camera matrix
    //     [fx'  0  cx' Tx]
    // P = [ 0  fy' cy' Ty]  --> # 3x4 row-major matrix
    //     [ 0   0   1   0]
    // By convention, this matrix specifies the intrinsic (camera) matrix of the
    // processed (rectified) image. That is, the left 3x3 portion is the normal
    // camera intrinsic matrix for the rectified image. It projects 3D points
    // in the camera coordinate frame to 2D pixel coordinates using the focal
    // lengths (fx', fy') and principal point (cx', cy') - these may differ from
    // the values in K. For monocular cameras, Tx = Ty = 0. Normally, monocular
    // cameras will also have R = the identity and P[1:3,1:3] = K.
    // For a stereo pair, the fourth column [Tx Ty 0]' is related to the
    // position of the optical center of the second camera in the first
    // camera's frame. We assume Tz = 0 so both cameras are in the same
    // stereo image plane. The first camera always has Tx = Ty = 0.
    // For the right (second) camera of a horizontal stereo pair,
    // Ty = 0 and Tx = -fx' * B, where B is the baseline between the cameras.
    // Given a 3D point [X Y Z]', the projection (x, y) of the point onto the
    // rectified image is given by:
    // [u v w]' = P * [X Y Z 1]'
    //        x = u / w
    //        y = v / w
    //  This holds for both images of a stereo pair.
    cam_info_msg.P.assign(0.0);

    // Binning refers here to any camera setting which combines rectangular
    // neighborhoods of pixels into larger "super-pixels." It reduces the
    // resolution of the output image to (width / binning_x) x (height / binning_y).
    // The default values binning_x = binning_y = 0 is considered the same as
    // binning_x = binning_y = 1 (no subsampling).
    cam_info_msg.binning_x = pylon_camera_->currentBinningX();
    cam_info_msg.binning_y = pylon_camera_->currentBinningY();

    // Region of interest (subwindow of full camera resolution), given in full
    // resolution (unbinned) image coordinates. A particular ROI always denotes
    // the same window of pixels on the camera sensor, regardless of binning
    // settings. The default setting of roi (all values 0) is considered the same
    // as full resolution (roi.width = width, roi.height = height).
    cam_info_msg.roi.x_offset = cam_info_msg.roi.y_offset = 0;
    cam_info_msg.roi.height = cam_info_msg.roi.width = 0;
}

/**
 * Waits till the pylon_camera_ isReady() observing a given timeout
 * @return true when the camera's state toggles to 'isReady()'
 */
bool PylonCameraNode::waitForCamera(const ros::Duration& timeout) const
{
    bool result = false;
    ros::Time start_time = ros::Time::now();

    while ( ros::ok() )
    {
        if ( pylon_camera_->isReady() )
        {
            result = true;
            break;
        }
        else
        {
            if ( timeout >= ros::Duration(0) )
            {
                if ( ros::Time::now() - start_time >= timeout )
                {
                    ROS_ERROR_STREAM("Setting brightness failed, because the "
                        << "interface is not ready. This happens although "
                        << "waiting for " << timeout.sec << " seconds!");
                    return false;
                }
            }
            ros::Duration(0.02).sleep();
        }
    }
    return result;
}


bool PylonCameraNode::setROI(const sensor_msgs::RegionOfInterest target_roi,
			     sensor_msgs::RegionOfInterest& reached_roi)
{
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    if ( !pylon_camera_->setROI(target_roi, reached_roi) )
    {
        // retry till timeout
        ros::Rate r(10.0);
        ros::Time timeout(ros::Time::now() + ros::Duration(2.0));
        while ( ros::ok() )
        {
            if ( pylon_camera_->setROI(target_roi, reached_roi) )
            {
                break;
            }
            if ( ros::Time::now() > timeout )
            {
                ROS_ERROR_STREAM("Error in setROI(): Unable to set target "
                << "roi before timeout");
                CameraInfoPtr cam_info(new CameraInfo(camera_info_manager_->getCameraInfo()));
                cam_info->roi = pylon_camera_->currentROI();
                camera_info_manager_->setCameraInfo(*cam_info);
                img_raw_msg_.width = pylon_camera_->imageCols();
		img_raw_msg_.height = pylon_camera_->imageRows();
                // step = full row length in bytes, img_size = (step * rows), imagePixelDepth
                // already contains the number of channels
                img_raw_msg_.step = img_raw_msg_.width * pylon_camera_->imagePixelDepth();
                return false;
            }
            r.sleep();
        }
    }
    CameraInfoPtr cam_info(new CameraInfo(camera_info_manager_->getCameraInfo()));
    cam_info->roi = pylon_camera_->currentROI();
    camera_info_manager_->setCameraInfo(*cam_info);
    img_raw_msg_.height = pylon_camera_->imageRows();
    img_raw_msg_.width = pylon_camera_->imageCols();
    // step = full row length in bytes, img_size = (step * rows), imagePixelDepth
    // already contains the number of channels
    img_raw_msg_.step = img_raw_msg_.width * pylon_camera_->imagePixelDepth();
    return true;
}


bool PylonCameraNode::setBinningX(const size_t& target_binning_x,
                                  size_t& reached_binning_x)
{
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    if ( !pylon_camera_->setBinningX(target_binning_x, reached_binning_x) )
    {
        // retry till timeout
        ros::Rate r(10.0);
        ros::Time timeout(ros::Time::now() + ros::Duration(2.0));
        while ( ros::ok() )
        {
            if ( pylon_camera_->setBinningX(target_binning_x, reached_binning_x) )
            {
                break;
            }
            if ( ros::Time::now() > timeout )
            {
                ROS_ERROR_STREAM("Error in setBinningX(): Unable to set target "
                << "binning_x factor before timeout");
                CameraInfoPtr cam_info(new CameraInfo(camera_info_manager_->getCameraInfo()));
                cam_info->binning_x = pylon_camera_->currentBinningX();
                camera_info_manager_->setCameraInfo(*cam_info);
                img_raw_msg_.width = pylon_camera_->imageCols();
                // step = full row length in bytes, img_size = (step * rows), imagePixelDepth
                // already contains the number of channels
                img_raw_msg_.step = img_raw_msg_.width * pylon_camera_->imagePixelDepth();
                return false;
            }
            r.sleep();
        }
    }
    CameraInfoPtr cam_info(new CameraInfo(camera_info_manager_->getCameraInfo()));
    cam_info->binning_x = pylon_camera_->currentBinningX();
    camera_info_manager_->setCameraInfo(*cam_info);
    img_raw_msg_.width = pylon_camera_->imageCols();
    // step = full row length in bytes, img_size = (step * rows), imagePixelDepth
    // already contains the number of channels
    img_raw_msg_.step = img_raw_msg_.width * pylon_camera_->imagePixelDepth();
    setupSamplingIndices(sampling_indices_,
                         pylon_camera_->imageRows(),
                         pylon_camera_->imageCols(),
                         pylon_camera_parameter_set_.downsampling_factor_exp_search_);
    return true;
}

bool PylonCameraNode::setBinningY(const size_t& target_binning_y,
                                  size_t& reached_binning_y)
{
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    if ( !pylon_camera_->setBinningY(target_binning_y, reached_binning_y) )
    {
        // retry till timeout
        ros::Rate r(10.0);
        ros::Time timeout(ros::Time::now() + ros::Duration(2.0));
        while ( ros::ok() )
        {
            if ( pylon_camera_->setBinningY(target_binning_y, reached_binning_y) )
            {
                break;
            }
            if ( ros::Time::now() > timeout )
            {
                ROS_ERROR_STREAM("Error in setBinningY(): Unable to set target "
                    << "binning_y factor before timeout");
                CameraInfoPtr cam_info(new CameraInfo(camera_info_manager_->getCameraInfo()));
                cam_info->binning_y = pylon_camera_->currentBinningY();
                camera_info_manager_->setCameraInfo(*cam_info);
                img_raw_msg_.height = pylon_camera_->imageRows();
                // step = full row length in bytes, img_size = (step * rows), imagePixelDepth
                // already contains the number of channels
                img_raw_msg_.step = img_raw_msg_.width * pylon_camera_->imagePixelDepth();
                return false;
            }
            r.sleep();
        }
    }
    CameraInfoPtr cam_info(new CameraInfo(camera_info_manager_->getCameraInfo()));
    cam_info->binning_y = pylon_camera_->currentBinningY();
    camera_info_manager_->setCameraInfo(*cam_info);
    img_raw_msg_.height = pylon_camera_->imageRows();
    // step = full row length in bytes, img_size = (step * rows), imagePixelDepth
    // already contains the number of channels
    img_raw_msg_.step = img_raw_msg_.width * pylon_camera_->imagePixelDepth();
    setupSamplingIndices(sampling_indices_,
                         pylon_camera_->imageRows(),
                         pylon_camera_->imageCols(),
                         pylon_camera_parameter_set_.downsampling_factor_exp_search_);
    return true;
}

bool PylonCameraNode::setBinningCallback(camera_control_msgs::SetBinning::Request &req,
                                         camera_control_msgs::SetBinning::Response &res)
{
    size_t reached_binning_x, reached_binning_y;
    bool success_x = setBinningX(req.target_binning_x,
                                 reached_binning_x);
    bool success_y = setBinningY(req.target_binning_y,
                                 reached_binning_y);
    res.reached_binning_x = static_cast<uint32_t>(reached_binning_x);
    res.reached_binning_y = static_cast<uint32_t>(reached_binning_y);
    res.success = success_x && success_y;
    return true;
}

bool PylonCameraNode::setROICallback(camera_control_msgs::SetROI::Request &req,
                                     camera_control_msgs::SetROI::Response &res)
{
    res.success = setROI(req.target_roi, res.reached_roi);
    return true;
}
  

bool PylonCameraNode::setExposure(const float& target_exposure,
                                  float& reached_exposure)
{
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    if ( !pylon_camera_->isReady() )
    {
        ROS_WARN("Error in setExposure(): pylon_camera_ is not ready!");
        return false;
    }

    if ( pylon_camera_->setExposure(target_exposure, reached_exposure) )
    {
        // success if the delta is smaller then the exposure step
        return true;
    }
    else  // retry till timeout
    {
        // wait for max 5s till the cam has updated the exposure
        ros::Rate r(10.0);
        ros::Time timeout(ros::Time::now() + ros::Duration(5.0));
        while ( ros::ok() )
        {
            if ( pylon_camera_->setExposure(target_exposure, reached_exposure) )
            {
                // success if the delta is smaller then the exposure step
                return true;
            }

            if ( ros::Time::now() > timeout )
            {
                break;
            }
            r.sleep();
        }
        ROS_ERROR_STREAM("Error in setExposure(): Unable to set target"
            << " exposure before timeout");
        return false;
    }
}

bool PylonCameraNode::setExposureCallback(camera_control_msgs::SetExposure::Request &req,
                                          camera_control_msgs::SetExposure::Response &res)
{
    res.success = setExposure(req.target_exposure, res.reached_exposure);
    return true;
}

bool PylonCameraNode::setGain(const float& target_gain, float& reached_gain)
{
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    if ( !pylon_camera_->isReady() )
    {
        ROS_WARN("Error in setGain(): pylon_camera_ is not ready!");
        return false;
    }

    if ( pylon_camera_->setGain(target_gain, reached_gain) )
    {
        return true;
    }
    else  // retry till timeout
    {
        // wait for max 5s till the cam has updated the exposure
        ros::Rate r(10.0);
        ros::Time timeout(ros::Time::now() + ros::Duration(5.0));
        while ( ros::ok() )
        {
            if ( pylon_camera_->setGain(target_gain, reached_gain) )
            {
                return true;
            }

            if ( ros::Time::now() > timeout )
            {
                break;
            }
            r.sleep();
        }
        ROS_ERROR_STREAM("Error in setGain(): Unable to set target "
            << "gain before timeout");
        return false;
     }
}

bool PylonCameraNode::setGainCallback(camera_control_msgs::SetGain::Request &req,
                                      camera_control_msgs::SetGain::Response &res)
{
    res.success = setGain(req.target_gain, res.reached_gain);
    return true;
}

bool PylonCameraNode::setGamma(const float& target_gamma, float& reached_gamma)
{
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    if ( !pylon_camera_->isReady() )
    {
        ROS_WARN("Error in setGamma(): pylon_camera_ is not ready!");
        return false;
    }

    if ( pylon_camera_->setGamma(target_gamma, reached_gamma) )
    {
        return true;
    }
    else  // retry till timeout
    {
        // wait for max 5s till the cam has updated the gamma value
        ros::Rate r(10.0);
        ros::Time timeout(ros::Time::now() + ros::Duration(5.0));
        while ( ros::ok() )
        {
            if ( pylon_camera_->setGamma(target_gamma, reached_gamma) )
            {
                return true;
            }

            if ( ros::Time::now() > timeout )
            {
                break;
            }
            r.sleep();
        }
        ROS_ERROR_STREAM("Error in setGamma(): Unable to set target "
            << "gamma before timeout");
        return false;
    }
}

bool PylonCameraNode::setGammaCallback(camera_control_msgs::SetGamma::Request &req,
                                       camera_control_msgs::SetGamma::Response &res)
{
    res.success = setGamma(req.target_gamma, res.reached_gamma);
    return true;
}

bool PylonCameraNode::setBrightness(const int& target_brightness,
                                    int& reached_brightness,
                                    const bool& exposure_auto,
                                    const bool& gain_auto)
{
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    ros::Time begin = ros::Time::now();  // time measurement for the exposure search

    // brightness service can only work, if an image has already been grabbed,
    // because it calculates the mean on the current image. The interface is
    // ready if the grab-result-pointer of the first acquisition contains
    // valid data
    if ( !waitForCamera(ros::Duration(3.0)) )
    {
        ROS_ERROR("Setting brightness failed: interface not ready, although waiting for 3 sec!");
        return false;
    }

    int target_brightness_co = std::min(255, target_brightness);
    // smart brightness search initially sets the last rememberd exposure time
    if ( brightness_exp_lut_.at(target_brightness_co) != 0.0 )
    {
        float reached_exp;
        if ( !setExposure(brightness_exp_lut_.at(target_brightness_co),
                          reached_exp) )
        {
            ROS_WARN_STREAM("Tried to speed-up exposure search with initial"
                    << " guess, but setting the exposure failed!");
        }
        else
        {
            ROS_DEBUG_STREAM("Speed-up exposure search with initial exposure"
                    << " guess of " << reached_exp);
        }
    }

    // get actual image -> fills img_raw_msg_.data vector
    if ( !grabImage() )
    {
        ROS_ERROR("Failed to grab image, can't calculate current brightness!");
        return false;
    }

    // calculates current brightness by generating the mean over all pixels
    // stored in img_raw_msg_.data vector
    float current_brightness = calcCurrentBrightness();

    ROS_DEBUG_STREAM("New brightness request for target brightness "
            << target_brightness_co << ", current brightness = "
            << current_brightness);

    if ( std::fabs(current_brightness - static_cast<float>(target_brightness_co)) <= 1.0 )
    {
        reached_brightness = static_cast<int>(current_brightness);
        ros::Time end = ros::Time::now();
        ROS_DEBUG_STREAM("Brightness reached without exposure search, duration: "
                << (end-begin).toSec());
        return true;  // target brightness already reached
    }

    // initially cancel all running exposure search by deactivating
    // ExposureAuto & AutoGain
    pylon_camera_->disableAllRunningAutoBrightessFunctions();

    if ( target_brightness_co <= 50 )
    {
        // own binary-exp search: we need to have the upper bound -> PylonAuto
        // exposure to a initial start value of 50 provides it
        if ( brightness_exp_lut_.at(50) != 0.0 )
        {
            float reached_exp;
            if ( !setExposure(brightness_exp_lut_.at(50), reached_exp) )
            {
                ROS_WARN_STREAM("Tried to speed-up exposure search with initial"
                    << " guess, but setting the exposure failed!");
            }
            else
            {
                ROS_DEBUG_STREAM("Speed-up exposure search with initial exposure"
                    << " guess of " << reached_exp);
            }
        }
    }

    if ( !exposure_auto && !gain_auto )
    {
        ROS_WARN_STREAM("Neither Auto Exposure Time ('exposure_auto') nor Auto "
            << "Gain ('gain_auto') are enabled! Hence gain and exposure time "
            << "are assumed to be fix and the target brightness ("
            << target_brightness_co << ") can not be reached!");
        return false;
    }

    bool is_brightness_reached = false;
    size_t fail_safe_ctr = 0;
    size_t fail_safe_ctr_limit = 10;
    if ( pylon_camera_->typeName() == "DART" )
    {
        // DART Cameras may need up to 50 images till the desired brightness
        // value can be reached. USB & GigE Cameras can achieve that much faster
        fail_safe_ctr_limit = 50;
    }
    float last_brightness = std::numeric_limits<float>::max();

    // timeout for the exposure search -> need more time for great exposure values
    ros::Time start_time = ros::Time::now();
    ros::Time timeout = start_time;
    if ( target_brightness_co < 205)
    {
        timeout += ros::Duration(pylon_camera_parameter_set_.exposure_search_timeout_);
    }
    else
    {
        timeout += ros::Duration(10.0 + pylon_camera_parameter_set_.exposure_search_timeout_);
    }

    while ( ros::ok() )
    {
        // calling setBrightness in every cycle would not be necessary for the pylon auto
        // brightness search. But for the case that the target brightness is out of the
        // pylon range which is from [50 - 205] a binary exposure search will be executed
        // where we have to update the search parameter in every cycle
        if ( !pylon_camera_->setBrightness(target_brightness_co,
                                           current_brightness,
                                           exposure_auto,
                                           gain_auto) )
        {
            pylon_camera_->disableAllRunningAutoBrightessFunctions();
            break;
        }

        if ( !grabImage() )
        {
            return false;
        }

        if ( pylon_camera_->isPylonAutoBrightnessFunctionRunning() )
        {
            // do nothing if the pylon auto function is running, we need to
            // wait till it's finished
            /*
            ROS_DEBUG_STREAM("PylonAutoBrightnessFunction still running! "
                << " Current brightness: " << calcCurrentBrightness()
                << ", current exposure: " << pylon_camera_->currentExposure());
            */
            continue;
        }

        current_brightness = calcCurrentBrightness();
        is_brightness_reached = fabs(current_brightness - static_cast<float>(target_brightness_co))
                                < pylon_camera_->maxBrightnessTolerance();

        if ( is_brightness_reached )
        {
            pylon_camera_->disableAllRunningAutoBrightessFunctions();
            break;
        }

        if ( std::fabs(last_brightness - current_brightness) <= 1.0 )
        {
            fail_safe_ctr++;
        }
        else
        {
            fail_safe_ctr = 0;
        }

        last_brightness = current_brightness;

        if ( ( fail_safe_ctr > fail_safe_ctr_limit ) && !is_brightness_reached )
        {
            ROS_WARN_STREAM("Seems like the desired brightness (" << target_brightness_co
                    << ") is not reachable! Stuck at brightness " << current_brightness
                    << " and exposure " << pylon_camera_->currentExposure() << "us");
            pylon_camera_->disableAllRunningAutoBrightessFunctions();
            reached_brightness = static_cast<int>(current_brightness);
            return false;
        }

        if ( ros::Time::now() > timeout )
        {
            // cancel all running brightness search by deactivating ExposureAuto
            pylon_camera_->disableAllRunningAutoBrightessFunctions();
            ROS_WARN_STREAM("Did not reach the target brightness before "
                << "timeout of " << (timeout - start_time).sec
                << " sec! Stuck at brightness " << current_brightness
                << " and exposure " << pylon_camera_->currentExposure() << "us");
            reached_brightness = static_cast<int>(current_brightness);
            return false;
        }
    }

    ROS_DEBUG_STREAM("Finally reached brightness: " << current_brightness);
    reached_brightness = static_cast<int>(current_brightness);

    // store reached brightness - exposure tuple for next times search
    if ( is_brightness_reached )
    {
        if ( brightness_exp_lut_.at(reached_brightness) == 0.0 )
        {
            brightness_exp_lut_.at(reached_brightness) = pylon_camera_->currentExposure();
        }
        else
        {
            brightness_exp_lut_.at(reached_brightness) += pylon_camera_->currentExposure();
            brightness_exp_lut_.at(reached_brightness) *= 0.5;
        }
        if ( brightness_exp_lut_.at(target_brightness_co) == 0.0 )
        {
            brightness_exp_lut_.at(target_brightness_co) = pylon_camera_->currentExposure();
        }
        else
        {
            brightness_exp_lut_.at(target_brightness_co) += pylon_camera_->currentExposure();
            brightness_exp_lut_.at(target_brightness_co) *= 0.5;
        }
    }
    ros::Time end = ros::Time::now();
    ROS_DEBUG_STREAM("Brightness search duration: " << (end-begin).toSec());
    return is_brightness_reached;
}

bool PylonCameraNode::setBrightnessCallback(camera_control_msgs::SetBrightness::Request &req,
                                            camera_control_msgs::SetBrightness::Response &res)
{
    res.success = setBrightness(req.target_brightness,
                                res.reached_brightness,
                                req.exposure_auto,
                                req.gain_auto);
    if ( req.brightness_continuous )
    {
        if ( req.exposure_auto )
        {
            pylon_camera_->enableContinuousAutoExposure();
        }
        if ( req.gain_auto )
        {
            pylon_camera_->enableContinuousAutoGain();
        }
    }
    res.reached_exposure_time = pylon_camera_->currentExposure();
    res.reached_gain_value = pylon_camera_->currentGain();
    return true;
}

void PylonCameraNode::setupSamplingIndices(std::vector<std::size_t>& indices,
                                           std::size_t rows,
                                           std::size_t cols,
                                           int downsampling_factor)
{
    indices.clear();
    std::size_t min_window_height = static_cast<float>(rows) /
                                    static_cast<float>(downsampling_factor);
    cv::Point2i start_pt(0, 0);
    cv::Point2i end_pt(cols, rows);
    // add the iamge center point only once
    sampling_indices_.push_back(0.5 * rows * cols);
    genSamplingIndicesRec(indices,
                          min_window_height,
                          start_pt,
                          end_pt);
    std::sort(indices.begin(), indices.end());
    return;
}

void PylonCameraNode::genSamplingIndicesRec(std::vector<std::size_t>& indices,
                                            const std::size_t& min_window_height,
                                            const cv::Point2i& s,   // start
                                            const cv::Point2i& e)   // end
{
    if ( static_cast<std::size_t>(std::abs(e.y - s.y)) <= min_window_height )
    {
        return;  // abort criteria -> shrinked window has the min_col_size
    }
    /*
     * sampled img:      point:                             idx:
     * s 0 0 0 0 0 0  a) [(e.x-s.x)*0.5, (e.y-s.y)*0.5]     a.x*a.y*0.5
     * 0 0 0 d 0 0 0  b) [a.x,           1.5*a.y]           b.y*img_rows+b.x
     * 0 0 0 0 0 0 0  c) [0.5*a.x,       a.y]               c.y*img_rows+c.x
     * 0 c 0 a 0 f 0  d) [a.x,           0.5*a.y]           d.y*img_rows+d.x
     * 0 0 0 0 0 0 0  f) [1.5*a.x,       a.y]               f.y*img_rows+f.x
     * 0 0 0 b 0 0 0
     * 0 0 0 0 0 0 e
     */
    cv::Point2i a, b, c, d, f, delta;
    a = s + 0.5 * (e - s);  // center point
    delta = 0.5 * (e - s);
    b = s + cv::Point2i(delta.x,       1.5 * delta.y);
    c = s + cv::Point2i(0.5 * delta.x, delta.y);
    d = s + cv::Point2i(delta.x,       0.5 * delta.y);
    f = s + cv::Point2i(1.5 * delta.x, delta.y);
    indices.push_back(b.y * pylon_camera_->imageCols() + b.x);
    indices.push_back(c.y * pylon_camera_->imageCols() + c.x);
    indices.push_back(d.y * pylon_camera_->imageCols() + d.x);
    indices.push_back(f.y * pylon_camera_->imageCols() + f.x);
    genSamplingIndicesRec(indices, min_window_height, s, a);
    genSamplingIndicesRec(indices, min_window_height, a, e);
    genSamplingIndicesRec(indices, min_window_height, cv::Point2i(s.x, a.y), cv::Point2i(a.x, e.y));
    genSamplingIndicesRec(indices, min_window_height, cv::Point2i(a.x, s.y), cv::Point2i(e.x, a.y));
    return;
}

float PylonCameraNode::calcCurrentBrightness()
{
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    if ( img_raw_msg_.data.empty() )
    {
        return 0.0;
    }
    float sum = 0.0;
    if ( sensor_msgs::image_encodings::isMono(img_raw_msg_.encoding) )
    {
        // The mean brightness is calculated using a subset of all pixels
        for ( const std::size_t& idx : sampling_indices_ )
        {
           sum += img_raw_msg_.data.at(idx);
        }
        if ( sum > 0.0 )
        {
            sum /= static_cast<float>(sampling_indices_.size());
        }
    }
    else
    {
        // The mean brightness is calculated using all pixels and all channels
        sum = std::accumulate(img_raw_msg_.data.begin(), img_raw_msg_.data.end(), 0);
        if ( sum > 0.0 )
        {
            sum /= static_cast<float>(img_raw_msg_.data.size());
        }
    }
    return sum;
}

bool PylonCameraNode::setSleepingCallback(camera_control_msgs::SetSleeping::Request &req,
                                          camera_control_msgs::SetSleeping::Response &res)
{
    is_sleeping_ = req.set_sleeping;

    if ( is_sleeping_ )
    {
        ROS_INFO("Seting Pylon Camera Node to sleep...");
    }
    else
    {
        ROS_INFO("Pylon Camera Node continues grabbing");
    }

    res.success = true;
    return true;
}

bool PylonCameraNode::isSleeping()
{
    return is_sleeping_;
}

bool PylonCameraNode::setOffsetXCallback(camera_control_msgs::SetIntegerValue::Request &req, camera_control_msgs::SetIntegerValue::Response &res)
{
    res.message = setOffsetXY(req.value, true);
    if ((res.message.find("done") != std::string::npos) != 0)
    {
        res.success = true;
    }
    else 
    {
        res.success = false;
        if (res.message == "Node is not writable.")
        {
          res.message = "Using this feature require stop image grabbing";
        }
    }
    return true;
}

bool PylonCameraNode::setOffsetYCallback(camera_control_msgs::SetIntegerValue::Request &req, camera_control_msgs::SetIntegerValue::Response &res)
{
    res.message = setOffsetXY(req.value, false);
    if ((res.message.find("done") != std::string::npos) != 0)
    {
        res.success = true;
    }
    else 
    {
        res.success = false;
        if (res.message == "Node is not writable.")
        {
          res.message = "Using this feature require stop image grabbing";;
        }
    }
    return true;
}


std::string PylonCameraNode::setOffsetXY(const int& offsetValue, bool xAxis)
{
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    if ( !pylon_camera_->isReady() )
    {
        ROS_WARN("Error in setOffsetXY(): pylon_camera_ is not ready!");
        return "pylon camera is not ready!";
    }
    return pylon_camera_->setOffsetXY(offsetValue,xAxis) ;
}

bool PylonCameraNode::setReverseXCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    res.message = reverseXY(req.data, true);
    if (res.message == "done")
    {
        res.success = true;
    }
    else 
    {
        res.success = false;
        if (res.message == "Node is not writable.")
        {
          res.message = "Using this feature require stop image grabbing";
        }
    }
    return true;
}

bool PylonCameraNode::setReverseYCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    res.message = reverseXY(req.data, false);
    if (res.message == "done")
    {
        res.success = true;
    }
    else 
    {
        res.success = false;
        if (res.message == "Node is not writable.")
        {
          res.message = "Using this feature require stop image grabbing";
        }
    }
    return true;
}

std::string PylonCameraNode::reverseXY(const bool& data, bool around_x)
{
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    if ( !pylon_camera_->isReady() )
    {
        ROS_WARN("Error in reverseXY(): pylon_camera_ is not ready!");
        return "pylon camera is not ready!";
    }
    return pylon_camera_->reverseXY(data, around_x);
}

bool PylonCameraNode::setBlackLevelCallback(camera_control_msgs::SetIntegerValue::Request &req, camera_control_msgs::SetIntegerValue::Response &res)
{   
    res.message = setBlackLevel(req.value);
    if (res.message == "done")
    {
        res.success = true;
    }
    else 
    {
        res.success = false;
        if (res.message == "Node is not writable.")
        {
          res.message = "Using this feature require stop image grabbing";
        }
    }
    return true;
}

std::string PylonCameraNode::setBlackLevel(const int& value)
{
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    if ( !pylon_camera_->isReady() )
    {
        ROS_WARN("Error in setBlackLevel(): pylon_camera_ is not ready!");
        return "pylon camera is not ready!";
    }

    return pylon_camera_->setBlackLevel(value) ;
}

bool PylonCameraNode::setPGIModeCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    res.message = setPGIMode(req.data);
    if (res.message == "done")
    {
        res.success = true;
    }
    else 
    {
        res.success = false;
        if (res.message == "Node is not writable.")
        {
          res.message = "Using this feature require stop image grabbing";
        }
    }
    return true;
}

std::string PylonCameraNode::setPGIMode(const bool& on)
{   // mode 0 = Simple
    // mode 1 = Basler PGI
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    if ( !pylon_camera_->isReady() )
    {
        ROS_WARN("Error in setPGIMode(): pylon_camera_ is not ready!");
        return "pylon camera is not ready!";
    }
    return pylon_camera_->setPGIMode(on);
}

bool PylonCameraNode::setDemosaicingModeCallback(camera_control_msgs::SetIntegerValue::Request &req, camera_control_msgs::SetIntegerValue::Response &res)
{
    res.message = setDemosaicingMode(req.value);
    if (res.message == "done")
    {
        res.success = true;
    }
    else 
    {
        res.success = false;
        if (res.message == "Node is not writable.")
        {
          res.message = "Using this feature require stop image grabbing";
        }
        else if ((res.message.find("EnumEntry") != std::string::npos) != 0)
        {
          res.message = "The passed demosaicing mode number is not supported by the connected camera";
        }
    }
    return true;
}

std::string PylonCameraNode::setDemosaicingMode(const int& mode)
{   // mode 0 = Simple
    // mode 1 = Basler PGI
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    if ( !pylon_camera_->isReady() )
    {
        ROS_WARN("Error in setDemosaicingMode(): pylon_camera_ is not ready!");
        return "pylon camera is not ready!";
    }
    return pylon_camera_->setDemosaicingMode(mode);
}

bool PylonCameraNode::setNoiseReductionCallback(camera_control_msgs::SetFloatValue::Request &req, camera_control_msgs::SetFloatValue::Response &res)
{
    res.message = setNoiseReduction(req.value);
    if (res.message == "done")
    {
        res.success = true;
    }
    else 
    {
        res.success = false;
        if (res.message == "Node is not writable.")
        {
          res.message = "Using this feature require stop image grabbing";
        }
    }
    return true;
}

std::string PylonCameraNode::setNoiseReduction(const float& value)
{
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    if ( !pylon_camera_->isReady() )
    {
        ROS_WARN("Error in setNoiseReduction(): pylon_camera_ is not ready!");
        return "pylon camera is not ready!";
    }
    return pylon_camera_->setNoiseReduction(value);
}

bool PylonCameraNode::setSharpnessEnhancementCallback(camera_control_msgs::SetFloatValue::Request &req, camera_control_msgs::SetFloatValue::Response &res)
{
    res.message = setSharpnessEnhancement(req.value);
    if (res.message == "done")
    {
        res.success = true;
    }
    else 
    {
        res.success = false;
        if (res.message == "Node is not writable.")
        {
          res.message = "Using this feature require stop image grabbing";
        }
    }
    return true;
}

std::string PylonCameraNode::setSharpnessEnhancement(const float& value)
{
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    if ( !pylon_camera_->isReady() )
    {
        ROS_WARN("Error in setSharpnessEnhancement(): pylon_camera_ is not ready!");
        return "pylon camera is not ready!";
    }
    return pylon_camera_->setSharpnessEnhancement(value);
}

bool PylonCameraNode::setLightSourcePresetCallback(camera_control_msgs::SetIntegerValue::Request &req, camera_control_msgs::SetIntegerValue::Response &res)
{
    res.message = setLightSourcePreset(req.value);
    if ((res.message.find("done") != std::string::npos) != 0)
    {
        res.success = true;
    }
    else 
    {
        res.success = false;
        if (res.message == "Node is not writable.")
        {
          res.message = "Using this feature require stop image grabbing";
        }
        else if ((res.message.find("EnumEntry") != std::string::npos) != 0)
        {
          res.message = "The passed light source preset number is not supported by the connected camera";
        }
    }
    return true;
}

std::string PylonCameraNode::setLightSourcePreset(const int& mode)
{   // mode 0 = Off
    // mode 1 = Daylight5000K
    // mode 2 = Daylight6500K
    // mode 3 = Tungsten2800K
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    if ( !pylon_camera_->isReady() )
    {
        ROS_WARN("Error in setLightSourcePreset(): pylon_camera_ is not ready!");
        return "pylon camera is not ready!";
    }
      return pylon_camera_->setLightSourcePreset(mode) ;
}

bool PylonCameraNode::setBalanceWhiteAutoCallback(camera_control_msgs::SetIntegerValue::Request &req, camera_control_msgs::SetIntegerValue::Response &res)
{
    res.message = setBalanceWhiteAuto(req.value);
    if ((res.message.find("done") != std::string::npos) != 0)
    {
        res.success = true;
    }
    else 
    {
        res.success = false;
        if (res.message == "Node is not writable.")
        {
          res.message = "Using this feature require stop image grabbing";
        }
        else if ((res.message.find("EnumEntry") != std::string::npos) != 0)
        {
          res.message = "The passed balance white auto number is not supported by the connected camera";
        }
    }
    return true;
}

std::string PylonCameraNode::setBalanceWhiteAuto(const int& mode)
{   // mode 0 = Off
    // mode 1 = Once
    // mode 2 = Continuous
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    if ( !pylon_camera_->isReady() )
    {
        ROS_WARN("Error in setBalanceWhiteAuto(): pylon_camera_ is not ready!");
        return "pylon camera is not ready!";
    }
      return pylon_camera_->setBalanceWhiteAuto(mode) ;
}

bool PylonCameraNode::setSensorReadoutModeCallback(camera_control_msgs::SetIntegerValue::Request &req, camera_control_msgs::SetIntegerValue::Response &res)
{
    res.message = setSensorReadoutMode(req.value);
    if ((res.message.find("done") != std::string::npos) != 0)
    {
        res.success = true;
    }
    else 
    {
        res.success = false;
        if (res.message == "Node is not writable.")
        {
          res.message = "Using this feature require stop image grabbing";
        }
        else if ((res.message.find("EnumEntry") != std::string::npos) != 0)
        {
          res.message = "The passed sensor readout mode number is not supported by the connected camera";
        }
    }
    return true;
}

std::string PylonCameraNode::setSensorReadoutMode(const int& mode)
{   // mode = 0 : normal readout mode
    // mode = 1 : fast readout mode
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    if ( !pylon_camera_->isReady() )
    {
        ROS_WARN("Error in setSensorReadoutMode(): pylon_camera_ is not ready!");
        return "pylon camera is not ready!";
    }
    return pylon_camera_->setSensorReadoutMode(mode) ;
}

bool PylonCameraNode::setAcquisitionFrameCountCallback(camera_control_msgs::SetIntegerValue::Request &req, camera_control_msgs::SetIntegerValue::Response &res)
{
    res.message = setAcquisitionFrameCount(req.value);
    if ((res.message.find("done") != std::string::npos) != 0)
    {
        res.success = true;
    }
    else 
    {
        res.success = false;
        if (res.message == "Node is not writable.")
        {
          res.message = "Using this feature require stop image grabbing";
        }
    }
    return true;
}

std::string PylonCameraNode::setAcquisitionFrameCount(const int& frameCount)
{   
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    if ( !pylon_camera_->isReady() )
    {
        ROS_WARN("Error in setAcquisitionFrameCount(): pylon_camera_ is not ready!");
        return "pylon camera is not ready!";
    }
    return pylon_camera_->setAcquisitionFrameCount(frameCount) ;
}

bool PylonCameraNode::setTriggerSelectorCallback(camera_control_msgs::SetIntegerValue::Request &req, camera_control_msgs::SetIntegerValue::Response &res)
{
    res.message = setTriggerSelector(req.value);
    if ((res.message.find("done") != std::string::npos) != 0)
    {
        res.success = true;
    }
    else 
    {
        res.success = false;
        if (res.message == "Node is not writable.")
        {
          res.message = "Using this feature require stop image grabbing";
        }
        else if ((res.message.find("EnumEntry") != std::string::npos) != 0)
        {
          res.message = "The passed trigger selector number is not supported by the connected camera";
        }
    }
    return true;
}

std::string PylonCameraNode::setTriggerSelector(const int& mode)
{   // mode 0 = Frame start
    // mode 1 = Frame burst start (ace USB cameras) / Acquisition Start (ace GigE cameras)
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    if ( !pylon_camera_->isReady() )
    {
        ROS_WARN("Error in setTriggerSelector(): pylon_camera_ is not ready!");
        return "pylon camera is not ready!";
    }
    return pylon_camera_->setTriggerSelector(mode) ;
}

bool PylonCameraNode::setTriggerModeCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    res.message = setTriggerMode(req.data);
    if ((res.message.find("done") != std::string::npos) != 0)
    {
        res.success = true;
    }
    else 
    {
        res.success = false;
        if (res.message == "Node is not writable.")
        {
          res.message = "Using this feature require stop image grabbing";
        }
        else if ((res.message.find("EnumEntry") != std::string::npos) != 0)
        {
          res.message = "The passed trigger mode number is not supported by the connected camera";
        }
    }
    return true;
}

std::string PylonCameraNode::setTriggerMode(const bool& value)
{   
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    if ( !pylon_camera_->isReady() )
    {
        ROS_WARN("Error in setTriggerMode(): pylon_camera_ is not ready!");
        return "pylon camera is not ready!";
    }
    return pylon_camera_->setTriggerMode(value) ;
}

bool PylonCameraNode::executeSoftwareTriggerCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    res.message = executeSoftwareTrigger();
    if ((res.message.find("done") != std::string::npos) != 0)
    {
        res.success = true;
    }
    else 
    {
        res.success = false;
        if (res.message == "Node is not writable.")
        {
          res.message = "Using this feature require stop image grabbing";
        }
    }
    return true;
}

std::string PylonCameraNode::executeSoftwareTrigger()
{   
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    if ( !pylon_camera_->isReady() )
    {
        ROS_WARN("Error in executeSoftwareTrigger(): pylon_camera_ is not ready!");
        return "pylon camera is not ready!";
    }
    return pylon_camera_->executeSoftwareTrigger() ;
}

bool PylonCameraNode::setTriggerSourceCallback(camera_control_msgs::SetIntegerValue::Request &req, camera_control_msgs::SetIntegerValue::Response &res)
{
    res.message = setTriggerSource(req.value);
    if ((res.message.find("done") != std::string::npos) != 0)
    {
        res.success = true;
    }
    else 
    {
        res.success = false;
        if (res.message == "Node is not writable.")
        {
          res.message = "Using this feature require stop image grabbing";
        }
        else if ((res.message.find("EnumEntry") != std::string::npos) != 0)
        {
          res.message = "The passed trigger source number is not supported by the connected camera";
        }
    }
    return true;
}

std::string PylonCameraNode::setTriggerSource(const int& source)
{   // source 0 = Software
    // source 1 = Line1
    // source 2 = Line3
    // source 2 = Line4
    // source 4 = Action1(only selected GigE Camera)
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    if ( !pylon_camera_->isReady() )
    {
        ROS_WARN("Error in setTriggerSource(): pylon_camera_ is not ready!");
        return "pylon camera is not ready!";
    }
    return pylon_camera_->setTriggerSource(source) ;
}

bool PylonCameraNode::setTriggerActivationCallback(camera_control_msgs::SetIntegerValue::Request &req, camera_control_msgs::SetIntegerValue::Response &res)
{
    res.message = setTriggerActivation(req.value);
    if ((res.message.find("done") != std::string::npos) != 0)
    {
        res.success = true;
    }
    else 
    {
        res.success = false;
        if (res.message == "Node is not writable.")
        {
          res.message = "Using this feature require stop image grabbing";
        }
    }
    return true;
}

std::string PylonCameraNode::setTriggerActivation(const int& value)
{   
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    if ( !pylon_camera_->isReady() )
    {
        ROS_WARN("Error in setTriggerActivation(): pylon_camera_ is not ready!");
        return "pylon camera is not ready!";
    }
    return pylon_camera_->setTriggerActivation(value) ;
}

bool PylonCameraNode::setTriggerDelayCallback(camera_control_msgs::SetFloatValue::Request &req, camera_control_msgs::SetFloatValue::Response &res)
{
    res.message = setTriggerDelay(req.value);
    if ((res.message.find("done") != std::string::npos) != 0)
    {
        res.success = true;
    }
    else 
    {
        res.success = false;
        if (res.message == "Node is not writable.")
        {
          res.message = "Using this feature require stop image grabbing";
        }
    }
    return true;
}

std::string PylonCameraNode::setTriggerDelay(const float& value)
{   
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    if ( !pylon_camera_->isReady() )
    {
        ROS_WARN("Error in setTriggerDelay(): pylon_camera_ is not ready!");
        return "pylon camera is not ready!";
    }
    if ((value >= 0) && (value <= 1000000))
    {
      return pylon_camera_->setTriggerDelay(value) ;
    }
    else
    {
      return "Error: the trigger delay value is out of range (0 - 1,000,000)";
    }
}

bool PylonCameraNode::setLineSelectorCallback(camera_control_msgs::SetIntegerValue::Request &req, camera_control_msgs::SetIntegerValue::Response &res)
{
    res.message = setLineSelector(req.value);
    if ((res.message.find("done") != std::string::npos) != 0)
    {
        res.success = true;
    }
    else 
    {
        res.success = false;
        if (res.message == "Node is not writable.")
        {
          res.message = "Using this feature require stop image grabbing";
        }
    }
    return true;
}

std::string PylonCameraNode::setLineSelector(const int& value)
{   
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    if ( !pylon_camera_->isReady() )
    {
        ROS_WARN("Error in setLineSelector(): pylon_camera_ is not ready!");
        return "pylon camera is not ready!";
    }
    return pylon_camera_->setLineSelector(value) ;
}

bool PylonCameraNode::setLineModeCallback(camera_control_msgs::SetIntegerValue::Request &req, camera_control_msgs::SetIntegerValue::Response &res)
{
    res.message = setLineMode(req.value);
    if ((res.message.find("done") != std::string::npos) != 0)
    {
        res.success = true;
    }
    else 
    {
        res.success = false;
        if (res.message == "Node is not writable.")
        {
          res.message = "Using this feature require stop image grabbing";
        }
    }
    return true;
}

std::string PylonCameraNode::setLineMode(const int& value)
{   
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    if ( !pylon_camera_->isReady() )
    {
        ROS_WARN("Error in setLineMode(): pylon_camera_ is not ready!");
        return "pylon camera is not ready!";
    }
    return pylon_camera_->setLineMode(value) ;
}

bool PylonCameraNode::setLineSourceCallback(camera_control_msgs::SetIntegerValue::Request &req, camera_control_msgs::SetIntegerValue::Response &res)
{
    res.message = setLineSource(req.value);
    if ((res.message.find("done") != std::string::npos) != 0)
    {
        res.success = true;
    }
    else 
    {
        res.success = false;
        if (res.message == "Node is not writable.")
        {
          res.message = "Using this feature require stop image grabbing";
        }
    }
    return true;
}

std::string PylonCameraNode::setLineSource(const int& value)
{   
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    if ( !pylon_camera_->isReady() )
    {
        ROS_WARN("Error in setLineSource(): pylon_camera_ is not ready!");
        return "pylon camera is not ready!";
    }
    return pylon_camera_->setLineSource(value) ;
}

bool PylonCameraNode::setLineInverterCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    res.message = setLineInverter(req.data);
    if ((res.message.find("done") != std::string::npos) != 0)
    {
        res.success = true;
    }
    else 
    {
        res.success = false;
        if (res.message == "Node is not writable.")
        {
          res.message = "Using this feature require stop image grabbing";
        }
    }
    return true;
}

std::string PylonCameraNode::setLineInverter(const bool& value)
{   
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    if ( !pylon_camera_->isReady() )
    {
        ROS_WARN("Error in setLineInverter(): pylon_camera_ is not ready!");
        return "pylon camera is not ready!";
    }
    return pylon_camera_->setLineInverter(value) ;
}

bool PylonCameraNode::setLineDebouncerTimeCallback(camera_control_msgs::SetFloatValue::Request &req, camera_control_msgs::SetFloatValue::Response &res)
{
    res.message = setLineDebouncerTime(req.value);
    if ((res.message.find("done") != std::string::npos) != 0)
    {
        res.success = true;
    }
    else 
    {
        res.success = false;
        if (res.message == "Node is not writable.")
        {
          res.message = "Using this feature require stop image grabbing";
        }
    }
    return true;
}

std::string PylonCameraNode::setLineDebouncerTime(const float& value)
{   
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    if ( !pylon_camera_->isReady() )
    {
        ROS_WARN("Error in setLineDebouncerTime(): pylon_camera_ is not ready!");
        return "pylon camera is not ready!";
    }
    if ((value >= 0) && (value <= 20000))
    {
      return pylon_camera_->setLineDebouncerTime(value) ;
    }
    else
    {
      return "Error : given line debouncer time value is out of rane (0-20,000)";
    }
}

bool PylonCameraNode::setUserSetSelectorCallback(camera_control_msgs::SetIntegerValue::Request &req, camera_control_msgs::SetIntegerValue::Response &res)
{
    res.message = setUserSetSelector(req.value);
    if ((res.message.find("done") != std::string::npos) != 0)
    {
        res.success = true;
    }
    else 
    {
        res.success = false;
        if (res.message == "Node is not writable.")
        {
          res.message = "Using this feature require stop image grabbing";
        }
        else if ((res.message.find("EnumEntry") != std::string::npos) != 0)
        {
          res.message = "The passed user set number is not supported by the connected camera";
        }
    }
    return true;
}

std::string PylonCameraNode::setUserSetSelector(const int& set)
{   // set 0 = Default
    // set 1 = UserSet1
    // set 2 = UserSet2
    // set 3 = UserSet3
    // set 4 = HighGain
    // set 5 = AutoFunctions
    // set 6 = ColorRaw
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    if ( !pylon_camera_->isReady() )
    {
        ROS_WARN("Error in setUserSetSelector(): pylon_camera_ is not ready!");
        return "pylon camera is not ready!";
    }
    return pylon_camera_->setUserSetSelector(set) ;
}


bool PylonCameraNode::saveUserSetCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    res.message = saveUserSet();
    if ((res.message.find("done") != std::string::npos) != 0)
    {
        res.success = true;
    }
    else 
    {
        res.success = false;
        if (res.message == "Node is not writable.")
        {
          res.message = "Using this feature require stop image grabbing";
        }
    }
    return true;
}

std::string PylonCameraNode::saveUserSet()
{  
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    if ( !pylon_camera_->isReady() )
    {
        ROS_WARN("Error in saveUserSet(): pylon_camera_ is not ready!");
        return "pylon camera is not ready!";
    }
    return pylon_camera_->saveUserSet() ;
}

bool PylonCameraNode::loadUserSetCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    res.message = loadUserSet();
    if ((res.message.find("done") != std::string::npos) != 0)
    {
        res.success = true;
    }
    else 
    {
        res.success = false;
        if (res.message == "Node is not writable.")
        {
          res.message = "Using this feature require stop image grabbing";
        }
    }
    return true;
}

std::string PylonCameraNode::loadUserSet()
{  
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    if ( !pylon_camera_->isReady() )
    {
        ROS_WARN("Error in loadUserSet(): pylon_camera_ is not ready!");
        return "pylon camera is not ready!";
    }
    return pylon_camera_->loadUserSet() ;
}

bool PylonCameraNode::setUserSetDefaultSelectorCallback(camera_control_msgs::SetIntegerValue::Request &req, camera_control_msgs::SetIntegerValue::Response &res)
{
    res.message = setUserSetDefaultSelector(req.value);
    if ((res.message.find("done") != std::string::npos) != 0)
    {
        res.success = true;
    }
    else 
    {
        res.success = false;
        if (res.message == "Node is not writable.")
        {
          res.message = "Using this feature require stop image grabbing";
        }
        else if ((res.message.find("EnumEntry") != std::string::npos) != 0)
        {
          res.message = "The passed default user set number is not supported by the connected camera";
        }
    }
    return true;
}

std::string PylonCameraNode::setUserSetDefaultSelector(const int& set)
{   // set 0 = Default
    // set 1 = UserSet1
    // set 2 = UserSet2
    // set 3 = UserSet3
    // set 4 = HighGain
    // set 5 = AutoFunctions
    // set 6 = ColorRaw
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    if ( !pylon_camera_->isReady() )
    {
        ROS_WARN("Error in setUserSetDefaultSelector(): pylon_camera_ is not ready!");
        return "pylon camera is not ready!";
    }
    return pylon_camera_->setUserSetDefaultSelector(set) ;
}

// USER CONTROL

bool PylonCameraNode::setDeviceLinkThroughputLimitModeCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    res.message = setDeviceLinkThroughputLimitMode(req.data);
    if ((res.message.find("done") != std::string::npos) != 0)
    {
        res.success = true;
    }
    else 
    {
        res.success = false;
        if (res.message == "Node is not writable.")
        {
          res.message = "Using this feature require stop image grabbing";
        }
    }
    return true;
}

std::string PylonCameraNode::setDeviceLinkThroughputLimitMode(const bool& turnOn)
{   
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    if ( !pylon_camera_->isReady() )
    {
        ROS_WARN("Error in setDeviceLinkThroughputLimitMode(): pylon_camera_ is not ready!");
        return "pylon camera is not ready!";
    }
      return pylon_camera_->setDeviceLinkThroughputLimitMode(turnOn) ;
}

bool PylonCameraNode::setDeviceLinkThroughputLimitCallback(camera_control_msgs::SetIntegerValue::Request &req, camera_control_msgs::SetIntegerValue::Response &res)
{
    res.message = setDeviceLinkThroughputLimit(req.value);
    if ((res.message.find("done") != std::string::npos) != 0)
    {
        res.success = true;
    }
    else 
    {
        res.success = false;
        if (res.message == "Node is not writable.")
        {
          res.message = "Using this feature require stop image grabbing";
        }
    }
    return true;
}

std::string PylonCameraNode::setDeviceLinkThroughputLimit(const int& limit)
{   
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    if ( !pylon_camera_->isReady() )
    {
        ROS_WARN("Error in setDeviceLinkThroughputLimit(): pylon_camera_ is not ready!");
        return "pylon camera is not ready!";
    }
      return pylon_camera_->setDeviceLinkThroughputLimit(limit) ;
}

bool PylonCameraNode::triggerDeviceResetCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    res.message = triggerDeviceReset();
    if ((res.message.find("done") != std::string::npos) != 0)
    {
        res.success = true;
    }
    else 
    {
        res.success = false;
    }
    return true;
}

std::string PylonCameraNode::triggerDeviceReset()
{  
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    if ( !pylon_camera_->isReady() )
    {
        ROS_WARN("Error in triggerDeviceReset(): pylon_camera_ is not ready!");
        return "pylon camera is not ready!";
    }
    return pylon_camera_->triggerDeviceReset() ;
}

bool PylonCameraNode::StartGrabbingCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    res.message = grabbingStarting();
    if ((res.message.find("done") != std::string::npos) != 0)
    {
        res.success = true;
    }
    else 
    {
        res.success = false;
    }
    return true;
}

std::string PylonCameraNode::grabbingStarting()
{  
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    if ( !pylon_camera_->isReady() )
    {
        ROS_WARN("Error in grabbingStarting(): pylon_camera_ is not ready!");
        return "pylon camera is not ready!";
    }
    if (startGrabbing())
    {
      return "done";
    }
    else 
    {
      return "Error";
    }
}

bool PylonCameraNode::StopGrabbingCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    res.message = grabbingStopping();
    if ((res.message.find("done") != std::string::npos) != 0)
    {
        res.success = true;
    }
    else 
    {
        res.success = false;
    }
    return true;
}

std::string PylonCameraNode::grabbingStopping()
{  
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    if ( !pylon_camera_->isReady() )
    {
        ROS_WARN("Error in grabbingStopping(): pylon_camera_ is not ready!");
        return "pylon camera is not ready!";
    }
    return pylon_camera_->grabbingStopping() ;
}

bool PylonCameraNode::setImageEncodingCallback(camera_control_msgs::SetStringValue::Request &req, camera_control_msgs::SetStringValue::Response &res)
{
    grabbingStopping(); // Stop grabbing for better user experience
    res.message = setImageEncoding(req.value);
    if ((res.message.find("done") != std::string::npos) != 0)
    {
        res.success = true;
        pylon_camera_parameter_set_.setimageEncodingParam(nh_,req.value);
    }
    else 
    {
        res.success = false;
        if (res.message == "Node is not writable")
        {
          res.message = "Using this feature require stop image grabbing";
        }
    }
    grabbingStarting(); // Start grabbing for better usser experience
    return true;
}

std::string PylonCameraNode::setImageEncoding(const std::string& target_ros_encoding)
{  
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    if ( !pylon_camera_->isReady() )
    {
        ROS_WARN("Error in setImageEncoding(): pylon_camera_ is not ready!");
        return "pylon camera is not ready!";
    }
    return pylon_camera_->setImageEncoding(target_ros_encoding) ;
}

bool PylonCameraNode::setMaxTransferSizeCallback(camera_control_msgs::SetIntegerValue::Request &req, camera_control_msgs::SetIntegerValue::Response &res)
{
    res.message = setMaxTransferSize(req.value);
    if ((res.message.find("done") != std::string::npos) != 0)
    {
        res.success = true;
    }
    else 
    {
        res.success = false;
        if (res.message == "Node is not writable")
        {
          res.message = "Using this feature require stop image grabbing";
        }
    }
    return true;
}

std::string PylonCameraNode::setMaxTransferSize(const int& maxTransferSize)
{  
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    if ( !pylon_camera_->isReady() )
    {
        ROS_WARN("Error in setMaxTransferSize(): pylon_camera_ is not ready!");
        return "pylon camera is not ready!";
    }
    return pylon_camera_->setMaxTransferSize(maxTransferSize) ;
}

bool PylonCameraNode::setGammaSelectorCallback(camera_control_msgs::SetIntegerValue::Request &req, camera_control_msgs::SetIntegerValue::Response &res)
{
    res.message = setGammaSelector(req.value);
    if ((res.message.find("done") != std::string::npos) != 0)
    {
        res.success = true;
    }
    else 
    {
        res.success = false;
        if (res.message == "Node is not writable")
        {
          res.message = "Using this feature require stop image grabbing";
        }
    }
    return true;
}

std::string PylonCameraNode::setGammaSelector(const int& gammaSelector)
{  
    // gammaSelector 0 = User
    // gammaSelector 1 = sRGB
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    if ( !pylon_camera_->isReady() )
    {
        ROS_WARN("Error in setGammaSelector(): pylon_camera_ is not ready!");
        return "pylon camera is not ready!";
    }
    return pylon_camera_->setGammaSelector(gammaSelector) ;
}

bool PylonCameraNode::gammaEnableCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    res.message = gammaEnable(req.data);
    if ((res.message.find("done") != std::string::npos) != 0)
    {
        res.success = true;
    }
    else 
    {
        res.success = false;
        if (res.message == "Node is not writable")
        {
          res.message = "Using this feature require stop image grabbing";
        }
    }
    return true;
}

std::string PylonCameraNode::gammaEnable(const int& enable)
{  
    boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
    if ( !pylon_camera_->isReady() )
    {
        ROS_WARN("Error in gammaEnable(): pylon_camera_ is not ready!");
        return "pylon camera is not ready!";
    }
    return pylon_camera_->gammaEnable(enable) ;
}


bool PylonCameraNode::setGrabTimeoutCallback(camera_control_msgs::SetIntegerValue::Request &req, camera_control_msgs::SetIntegerValue::Response &res)
{
    grabbingStopping();
    try {
      pylon_camera_parameter_set_.grab_timeout_ = req.value;
      res.success = true;
    } catch (...){
      res.success = false;
    }
    grabbingStarting(); // start grabbing is required to set the new trigger timeout
    return true;
}

bool PylonCameraNode::setTriggerTimeoutCallback(camera_control_msgs::SetIntegerValue::Request &req, camera_control_msgs::SetIntegerValue::Response &res)
{
    grabbingStopping();
    try {
      pylon_camera_parameter_set_.trigger_timeout_ = req.value;
      res.success = true;
    } catch (...){
      res.success = false;
    }
    grabbingStarting(); // start grabbing is required to set the new trigger timeout
    return true;
}

bool PylonCameraNode::setWhiteBalanceCallback(camera_control_msgs::SetWhiteBalance::Request &req, camera_control_msgs::SetWhiteBalance::Response &res)
{
    try {
        res.message = pylon_camera_->setWhiteBalance(req.balance_ratio_red, req.balance_ratio_green, req.balance_ratio_blue);
        if (res.message == "done") {
            res.success = true;
        } else {
            res.success = false;
        }
    } catch (...){
      res.success = false;
    }
    return true;
}

bool PylonCameraNode::setGrabbingStrategyCallback(camera_control_msgs::SetIntegerValue::Request &req, camera_control_msgs::SetIntegerValue::Response &res)
{   // set 0 = GrabStrategy_OneByOne
    // set 1 = GrabStrategy_LatestImageOnly
    // set 2 = GrabStrategy_LatestImages

    if(req.value >= 0 && req.value <= 2) {
        grabbingStopping();
        res.success = pylon_camera_->setGrabbingStrategy(req.value);
        if(res.success){
          pylon_camera_parameter_set_.grab_strategy_ = req.value;
        }
        grabbingStarting();

    } else {
      res.success = false;
      res.message = "Unknown grabbing strategy";
    }
    
    return true;
}


bool PylonCameraNode::setOutputQueueSizeCallback(camera_control_msgs::SetIntegerValue::Request &req, camera_control_msgs::SetIntegerValue::Response &res)
{

    grabbingStopping();
    res.message = pylon_camera_->setOutputQueueSize(req.value);
    grabbingStarting();
    if ((res.message.find("done") != std::string::npos) != 0)
    {
        res.success = true;
    } else {
        res.success = false;
    }
    
    return true;
}
void PylonCameraNode::currentParamPub()
{
  boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
  if ( !pylon_camera_->isReady() )
    {
      ROS_WARN("Error in triggerDeviceReset(): pylon_camera_ is not ready!");
      params.message = "pylon camera is not ready!";
      params.sucess = false;
    }
  else 
  {
    try
    {
      params.black_level = pylon_camera_->getBlackLevel();
      params.reverse_x =  pylon_camera_->getReverseXY(true);
      params.reverse_y =  pylon_camera_->getReverseXY(false);
      params.offset_x = static_cast<uint32_t>(pylon_camera_->currentOffsetX());
      params.offset_y = static_cast<uint32_t>(pylon_camera_->currentOffsetY());
      params.pgi_mode = pylon_camera_->getPGIMode();
      params.demosaicing_mode = pylon_camera_->getDemosaicingMode();
      params.noise_reduction = pylon_camera_->getNoiseReduction();
      params.sharpness_enhancement = pylon_camera_->getSharpnessEnhancement();
      params.light_source_preset = pylon_camera_->getLightSourcePreset();
      params.balance_white_auto = pylon_camera_->getBalanceWhiteAuto();
      params.sensor_readout_mode = pylon_camera_->getSensorReadoutMode();
      params.acquisition_frame_count = pylon_camera_->getAcquisitionFrameCount();
      params.trigger_selector = pylon_camera_->getTriggerSelector();
      params.trigger_mode = pylon_camera_->getTriggerMode();
      params.trigger_source = pylon_camera_->getTriggerSource();
      params.trigger_activation = pylon_camera_->getTriggerActivation();
      params.trigger_delay = pylon_camera_->getTriggerDelay();
      params.user_set_selector = pylon_camera_->getUserSetSelector();
      params.user_set_default_selector = pylon_camera_->getUserSetDefaultSelector();
      params.is_sleeping = isSleeping();
      params.brightness = calcCurrentBrightness();
      params.exposure = pylon_camera_->currentExposure();
      params.gain = pylon_camera_->currentGain();
      params.gamma = pylon_camera_->currentGamma();
      params.binning_x = static_cast<uint32_t>(pylon_camera_->currentBinningX());
      params.binning_y = static_cast<uint32_t>(pylon_camera_->currentBinningY());
      params.roi = pylon_camera_->currentROI();
      params.available_image_encoding = pylon_camera_->detectAvailableImageEncodings(false);
      params.current_image_encoding = pylon_camera_->currentBaslerEncoding();
      params.current_image_ros_encoding = pylon_camera_->currentROSEncoding();
      params.temperature = pylon_camera_->getTemperature();

      params.sucess = true;
    }
    catch ( const GenICam::GenericException &e )
    {
      ROS_ERROR_STREAM("An exception while getting the camera current params occurred:" << e.GetDescription());
      params.sucess = false;
      params.message = "An exception while getting the camera current parameters occurred";
    }
  }
  currentParamsPublisher.publish(params);
}

PylonCameraNode::~PylonCameraNode()
{
    if ( pylon_camera_ )
    {
        delete pylon_camera_;
        pylon_camera_ = nullptr;
    }
    if ( it_ )
    {
        delete it_;
        it_ = nullptr;
    }
    if ( grab_imgs_rect_as_ )
    {
        grab_imgs_rect_as_->shutdown();
        delete grab_imgs_rect_as_;
        grab_imgs_rect_as_ = nullptr;
    }

    if ( img_rect_pub_ )
    {
        delete img_rect_pub_;
        img_rect_pub_ = nullptr;
    }

    if ( cv_bridge_img_rect_ )
    {
        delete cv_bridge_img_rect_;
        cv_bridge_img_rect_ = nullptr;
    }

    if ( pinhole_model_ )
    {
        delete pinhole_model_;
        pinhole_model_ = nullptr;
    }
}

}  // namespace pylon_camera
