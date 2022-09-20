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

#include <GenApi/GenApi.h>

#include "pylon_ros2_camera_node.hpp"


namespace pylon_ros2_camera
{

namespace
{
    static const rclcpp::Logger LOGGER = rclcpp::get_logger("basler.pylon.ros2.pylon_ros2_camera_node");
}

PylonROS2CameraNode::PylonROS2CameraNode(const rclcpp::NodeOptions& options)
  : Node("pylon_ros2_camera_node", options)
  , pylon_camera_(nullptr)
  , pinhole_model_(nullptr)
  , pylon_camera_parameter_set_()
  , camera_info_manager_(new camera_info_manager::CameraInfoManager(this))
  , cv_bridge_img_rect_(nullptr)
  , img_rect_pub_(nullptr)
  , set_user_output_srvs_()
  , grab_imgs_rect_as_(nullptr)
  , sampling_indices_()
  , brightness_exp_lut_()
  , is_sleeping_(false)
  , diagnostics_updater_(this)
{
  // information logging severity mode
  rcutils_ret_t __attribute__((unused)) res = rcutils_logging_set_logger_level(LOGGER.get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
  //RCUTILS_LOG_SEVERITY_DEBUG
  //RCUTILS_LOG_SEVERITY_INFO
  //RCUTILS_LOG_SEVERITY_WARN
  //RCUTILS_LOG_SEVERITY_ERROR
  //RCUTILS_LOG_SEVERITY_FATAL

  // initializing the interfaces
  this->initInterfaces();

  // initialize camera instance and start grabbing
  if (!this->init())
    return;

  // starting spinning thread
  RCLCPP_INFO_STREAM(LOGGER, "Start image grabbing if node connects to topic with " << "a frame_rate of: " << this->frameRate() << " Hz");
  timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1. / this->frameRate()),
            std::bind(&PylonROS2CameraNode::spin, this));
}

PylonROS2CameraNode::~PylonROS2CameraNode()
{
  if (this->pylon_camera_)
  {
    delete this->pylon_camera_;
    this->pylon_camera_ = nullptr;
  }

  if (this->img_rect_pub_)
  {
    delete this->img_rect_pub_;
    this->img_rect_pub_ = nullptr;
  }

  if (this->cv_bridge_img_rect_)
  {
    delete this->cv_bridge_img_rect_;
    this->cv_bridge_img_rect_ = nullptr;
  }

  if (this->pinhole_model_)
  {
    delete this->pinhole_model_;
    this->pinhole_model_ = nullptr;
  }
}

const double& PylonROS2CameraNode::frameRate() const
{
  return this->pylon_camera_parameter_set_.frameRate();
}

const std::string& PylonROS2CameraNode::cameraFrame() const
{
  return this->pylon_camera_parameter_set_.cameraFrame();
}

bool PylonROS2CameraNode::init()
{
  // reading all necessary parameter to open the desired camera from the
  // ros-parameter-server. In case that invalid parameter values can be
  // detected, the interface will reset them to the default values.
  // These parameters furthermore contain the intrinsic calibration matrices,
  // in case they are provided
  this->pylon_camera_parameter_set_.readFromRosParameterServer(*this);
  
  // creating the target PylonCamera-Object with the specified
  // device_user_id, registering the Software-Trigger-Mode, starting the
  // communication with the device and enabling the desired startup-settings
  if (!this->initAndRegister())
  {
    RCLCPP_ERROR(LOGGER, "Error when trying to init and register. Shutting down now.");
    rclcpp::shutdown();
    return false;
  }

  // starting the grabbing procedure with the desired image-settings
  if (!this->startGrabbing())
  {
    RCLCPP_ERROR(LOGGER, "Error when trying to start grabbing. Shutting down now.");
    rclcpp::shutdown();
    return false;
  }

  return true;
}

void PylonROS2CameraNode::initInterfaces()
{
  this->initPublishers();

  this->initServices();

  this->initActions();

  this->initDiagnostics();
}

void PylonROS2CameraNode::initPublishers()
{
  std::string msg_name;
  std::string msg_prefix = "~/";

  msg_name = msg_prefix + "current_params";
  current_params_pub_ = this->create_publisher<pylon_ros2_camera_interfaces::msg::CurrentParams>(msg_name, 5);
  msg_name = msg_prefix + "status";
  component_status_pub_ = this->create_publisher<pylon_ros2_camera_interfaces::msg::ComponentStatus>(msg_name, 5);

  msg_name = msg_prefix + "image_raw";
  img_raw_pub_ = image_transport::create_camera_publisher(this, msg_name);
}

void PylonROS2CameraNode::initServices()
{
  using namespace std::placeholders;

  std::string srv_name;
  std::string srv_prefix = "~/";

  srv_name = srv_prefix + "get_max_num_buffer";
  this->get_max_num_buffer_srv_ = this->create_service<GetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::getMaxNumBufferCallback, this, _1, _2));
  
  srv_name = srv_prefix + "get_statistic_total_buffer_count";
  this->get_statistic_total_buffer_count_srv_ = this->create_service<GetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::getStatisticTotalBufferCountCallback, this, _1, _2));
  
  srv_name = srv_prefix + "get_statistic_failed_buffer_count";
  this->get_statistic_failed_buffer_count_srv_ = this->create_service<GetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::getStatisticFailedBufferCountCallback, this, _1, _2));
  
  srv_name = srv_prefix + "get_statistic_buffer_underrun_count";
  this->get_statistic_buffer_underrun_count_srv_ = this->create_service<GetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::getStatisticBufferUnderrunCountCallback, this, _1, _2));
  
  srv_name = srv_prefix + "get_statistic_failed_packet_count";
  this->get_statistic_failed_packet_count_srv_ = this->create_service<GetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::getStatisticFailedPacketCountCallback, this, _1, _2));
  
  srv_name = srv_prefix + "get_statistic_resend_request_count";
  this->get_statistic_resend_request_count_srv_ = this->create_service<GetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::getStatisticResendRequestCountCallback, this, _1, _2));
  
  srv_name = srv_prefix + "get_statistic_missed_frame_count";
  this->get_statistic_missed_frame_count_srv_ = this->create_service<GetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::getStatisticMissedFrameCountCallback, this, _1, _2));
  
  srv_name = srv_prefix + "get_statistic_resynchronization_count";
  this->get_statistic_resynchronization_count_srv_ = this->create_service<GetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::getStatisticResynchronizationCountCallback, this, _1, _2));
  
  srv_name = srv_prefix + "get_chunk_mode_active";
  this->get_chunk_mode_active_srv_ = this->create_service<GetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::getChunkModeActiveCallback, this, _1, _2));
  
  srv_name = srv_prefix + "get_chunk_selector";
  this->get_chunk_selector_srv_ = this->create_service<GetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::getChunkSelectorCallback, this, _1, _2));
  
  srv_name = srv_prefix + "get_chunk_enable";
  this->get_chunk_enable_srv_ = this->create_service<GetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::getChunkEnableCallback, this, _1, _2));
  
  srv_name = srv_prefix + "get_chunk_timestamp";
  this->get_chunk_timestamp_srv_ = this->create_service<GetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::getChunkTimestampCallback, this, _1, _2));
  
  srv_name = srv_prefix + "get_chunk_line_status_all";
  this->get_chunk_line_status_all_srv_ = this->create_service<GetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::getChunkLineStatusAllCallback, this, _1, _2));
  
  srv_name = srv_prefix + "get_chunk_frame_counter";
  this->get_chunk_frame_counter_srv_ = this->create_service<GetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::getChunkFramecounterCallback, this, _1, _2));
  
  srv_name = srv_prefix + "get_chunk_counter_value";
  this->get_chunk_counter_value_srv_ = this->create_service<GetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::getChunkCounterValueCallback, this, _1, _2));
  
  srv_name = srv_prefix + "get_chunk_exposure_time";
  this->get_chunk_exposure_time_srv_ = this->create_service<GetFloatSrv>(srv_name, std::bind(&PylonROS2CameraNode::getChunkExposureTimeCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_binning";
  this->set_binning_srv_ = this->create_service<SetBinningSrv>(srv_name, std::bind(&PylonROS2CameraNode::setBinningCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_brightness";
  this->set_brightness_srv_ = this->create_service<SetBrightnessSrv>(srv_name, std::bind(&PylonROS2CameraNode::setBrightnessCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_exposure";
  this->set_exposure_srv_ = this->create_service<SetExposureSrv>(srv_name, std::bind(&PylonROS2CameraNode::setExposureCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_gain";
  this->set_gain_srv_ = this->create_service<SetGainSrv>(srv_name, std::bind(&PylonROS2CameraNode::setGainCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_gamma";
  this->set_gamma_srv_ = this->create_service<SetGammaSrv>(srv_name, std::bind(&PylonROS2CameraNode::setGammaCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_roi";
  this->set_roi_srv_ = this->create_service<SetROISrv>(srv_name, std::bind(&PylonROS2CameraNode::setROICallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_sleeping";
  this->set_sleeping_srv_ = this->create_service<SetSleepingSrv>(srv_name, std::bind(&PylonROS2CameraNode::setSleepingCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_white_balance";
  this->set_white_balance_srv_ = this->create_service<SetWhiteBalanceSrv>(srv_name, std::bind(&PylonROS2CameraNode::setWhiteBalanceCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_action_trigger_configuration";
  this->set_ac_trigger_config_srv_ = this->create_service<SetActionTriggerConfiguration>(srv_name, std::bind(&PylonROS2CameraNode::setActionTriggerConfigurationCallback, this, _1, _2));

  srv_name = srv_prefix + "issue_action_command";
  this->issue_action_command_srv_ = this->create_service<IssueActionCommand>(srv_name, std::bind(&PylonROS2CameraNode::issueActionCommandCallback, this, _1, _2));

  srv_name = srv_prefix + "issue_scheduled_action_command";
  this->issue_scheduled_action_command_srv_ = this->create_service<IssueScheduledActionCommand>(srv_name, std::bind(&PylonROS2CameraNode::issueScheduledActionCommandCallback, this, _1, _2));

  srv_name = srv_prefix + "set_offset_x";
  this->set_offset_x_srv_ = this->create_service<SetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::setOffsetXCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_offset_y";
  this->set_offset_y_srv_ = this->create_service<SetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::setOffsetYCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_black_level";
  this->set_black_level_srv_ = this->create_service<SetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::setBlackLevelCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_demosaicing_mode";
  this->set_demosaicing_mode_srv_ = this->create_service<SetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::setDemosaicingModeCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_light_source_preset";
  this->set_light_source_preset_srv_ = this->create_service<SetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::setLightSourcePresetCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_white_balance_auto";
  this->set_white_balance_auto_srv_ = this->create_service<SetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::setWhiteBalanceAutoCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_sensor_readout_mode";
  this->set_sensor_readout_mode_srv_ = this->create_service<SetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::setSensorReadoutModeCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_acquisition_frame_count";
  this->set_acquisition_frame_count_srv_ = this->create_service<SetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::setAcquisitionFrameCountCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_trigger_selector";
  this->set_trigger_selector_srv_ = this->create_service<SetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::setTriggerSelectorCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_trigger_source";
  this->set_trigger_source_srv_ = this->create_service<SetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::setTriggerSourceCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_trigger_activation";
  this->set_trigger_activation_srv_ = this->create_service<SetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::setTriggerActivationCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_line_selector";
  this->set_line_selector_srv_ = this->create_service<SetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::setLineSelectorCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_line_mode";
  this->set_line_mode_srv_ = this->create_service<SetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::setLineModeCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_line_source";
  this->set_line_source_srv_ = this->create_service<SetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::setLineSourceCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_user_set_selector";
  this->set_user_set_selector_srv_ = this->create_service<SetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::setUserSetSelectorCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_user_set_default_selector";
  this->set_user_set_default_selector_srv_ = this->create_service<SetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::setUserSetDefaultSelectorCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_device_link_throughput_limit";
  this->set_device_link_throughput_limit_srv_ = this->create_service<SetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::setDeviceLinkThroughputLimitCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_max_transfer_size";
  this->set_max_transfer_size_srv_ = this->create_service<SetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::setMaxTransferSizeCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_gamma_selector";
  this->set_gamma_selector_srv_ = this->create_service<SetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::setGammaSelectorCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_grab_timeout";
  this->set_grab_timeout_srv_ = this->create_service<SetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::setGrabTimeoutCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_trigger_timeout";
  this->set_trigger_timeout_srv_ = this->create_service<SetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::setTriggerTimeoutCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_grabbing_strategy";
  this->set_grabbing_strategy_srv_ = this->create_service<SetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::setGrabbingStrategyCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_output_queue_size";
  this->set_output_queue_size_srv_ = this->create_service<SetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::setOutputQueueSizeCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_max_num_buffer";
  this->set_max_num_buffer_srv_ = this->create_service<SetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::setMaxNumBufferCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_chunk_selector";
  this->set_chunk_selector_srv_ = this->create_service<SetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::setChunkSelectorCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_timer_selector";
  this->set_timer_selector_srv_ = this->create_service<SetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::setTimerSelectorCallback, this, _1, _2));

  srv_name = srv_prefix + "set_timer_trigger_source";
  this->set_timer_trigger_source_srv_ = this->create_service<SetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::setTimerTriggerSourceCallback, this, _1, _2));

  srv_name = srv_prefix + "set_ptp_priority";
  this->set_ptp_priority_srv_ = this->create_service<SetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::setPTPPriorityCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_ptp_profile";
  this->set_ptp_profile_srv_ = this->create_service<SetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::setPTPProfileCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_ptp_network_mode";
  this->set_ptp_network_mode_srv_ = this->create_service<SetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::setPTPNetworkModeCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_ptp_uc_port_address_index";
  this->set_ptp_uc_port_address_index_srv_ = this->create_service<SetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::setPTPUCPortAddressIndexCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_ptp_uc_port_address";
  this->set_ptp_uc_port_address_srv_ = this->create_service<SetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::setPTPUCPortAddressCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_sync_free_run_timer_start_time_low";
  this->set_sync_free_run_timer_start_time_low_srv_ = this->create_service<SetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::setSyncFreeRunTimerStartTimeLowCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_sync_free_run_timer_start_time_high";
  this->set_sync_free_run_timer_start_time_high_srv_ = this->create_service<SetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::setSyncFreeRunTimerStartTimeHighCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_noise_reduction";
  this->set_noise_reduction_srv_ = this->create_service<SetFloatSrv>(srv_name, std::bind(&PylonROS2CameraNode::setNoiseReductionCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_sharpness_enhancement";
  this->set_sharpness_enhancement_srv_ = this->create_service<SetFloatSrv>(srv_name, std::bind(&PylonROS2CameraNode::setSharpnessEnhancementCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_trigger_delay";
  this->set_trigger_delay_srv_ = this->create_service<SetFloatSrv>(srv_name, std::bind(&PylonROS2CameraNode::setTriggerDelayCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_line_debouncer_time";
  this->set_line_debouncer_time_srv_ = this->create_service<SetFloatSrv>(srv_name, std::bind(&PylonROS2CameraNode::setLineDebouncerTimeCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_chunk_exposure_time";
  this->set_chunk_exposure_time_srv_ = this->create_service<SetFloatSrv>(srv_name, std::bind(&PylonROS2CameraNode::setChunkExposureTimeCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_timer_duration";
  this->set_timer_duration_srv_ = this->create_service<SetFloatSrv>(srv_name, std::bind(&PylonROS2CameraNode::setTimerDurationCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_periodic_signal_period";
  this->set_periodic_signal_period_srv_ = this->create_service<SetFloatSrv>(srv_name, std::bind(&PylonROS2CameraNode::setPeriodicSignalPeriodCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_periodic_signal_delay";
  this->set_periodic_signal_delay_srv_ = this->create_service<SetFloatSrv>(srv_name, std::bind(&PylonROS2CameraNode::setPeriodicSignalDelayCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_sync_free_run_timer_trigger_rate_abs";
  this->set_sync_free_run_timer_trigger_rate_abs_srv_ = this->create_service<SetFloatSrv>(srv_name, std::bind(&PylonROS2CameraNode::setSyncFreeRunTimerTriggerRateAbsCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_image_encoding";
  this->set_image_encoding_srv_ = this->create_service<SetStringSrv>(srv_name, std::bind(&PylonROS2CameraNode::setImageEncodingCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_reverse_x";
  this->set_reverse_x_srv_ = this->create_service<SetBoolSrv>(srv_name, std::bind(&PylonROS2CameraNode::setReverseXCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_reverse_y";
  this->set_reverse_y_srv_ = this->create_service<SetBoolSrv>(srv_name, std::bind(&PylonROS2CameraNode::setReverseYCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_PGI_mode";
  this->set_PGI_mode_srv_ = this->create_service<SetBoolSrv>(srv_name, std::bind(&PylonROS2CameraNode::setPGIModeCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_trigger_mode";
  this->set_trigger_mode_srv_ = this->create_service<SetBoolSrv>(srv_name, std::bind(&PylonROS2CameraNode::setTriggerModeCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_line_inverter";
  this->set_line_inverter_srv_ = this->create_service<SetBoolSrv>(srv_name, std::bind(&PylonROS2CameraNode::setLineInverterCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_device_link_throughput_limit_mode";
  this->set_device_link_throughput_limit_mode_srv_ = this->create_service<SetBoolSrv>(srv_name, std::bind(&PylonROS2CameraNode::setDeviceLinkThroughputLimitModeCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_gamma_activation";
  this->set_gamma_activation_srv_ = this->create_service<SetBoolSrv>(srv_name, std::bind(&PylonROS2CameraNode::setGammaEnableCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_chunk_mode_active";
  this->set_chunk_mode_active_srv_ = this->create_service<SetBoolSrv>(srv_name, std::bind(&PylonROS2CameraNode::setChunkModeActiveCallback, this, _1, _2));
  
  srv_name = srv_prefix + "set_chunk_enable";
  this->set_chunk_enable_srv_ = this->create_service<SetBoolSrv>(srv_name, std::bind(&PylonROS2CameraNode::setChunkEnableCallback, this, _1, _2));

  srv_name = srv_prefix + "enable_ptp_management_protocol";
  this->enable_ptp_management_protocol_srv_ = this->create_service<SetBoolSrv>(srv_name, std::bind(&PylonROS2CameraNode::enablePTPManagementProtocolCallback, this, _1, _2));
  
  srv_name = srv_prefix + "enable_two_step_operation";
  this->enable_two_step_operation_srv_ = this->create_service<SetBoolSrv>(srv_name, std::bind(&PylonROS2CameraNode::enablePTPTwoStepOperationCallback, this, _1, _2));

  srv_name = srv_prefix + "enable_ptp";
  this->enable_ptp_srv_ = this->create_service<SetBoolSrv>(srv_name, std::bind(&PylonROS2CameraNode::enablePTPCallback, this, _1, _2));

  srv_name = srv_prefix + "enable_sync_free_run_timer";
  this->enable_sync_free_run_timer_srv_ = this->create_service<SetBoolSrv>(srv_name, std::bind(&PylonROS2CameraNode::enableSyncFreeRunTimerCallback, this, _1, _2));
  
  srv_name = srv_prefix + "execute_software_trigger";
  this->execute_software_trigger_srv_ = this->create_service<TriggerSrv>(srv_name, std::bind(&PylonROS2CameraNode::executeSoftwareTriggerCallback, this, _1, _2));
  
  srv_name = srv_prefix + "save_user_set";
  this->save_user_set_srv_ = this->create_service<TriggerSrv>(srv_name, std::bind(&PylonROS2CameraNode::saveUserSetCallback, this, _1, _2));
  
  srv_name = srv_prefix + "load_user_set";
  this->load_user_set_srv_ = this->create_service<TriggerSrv>(srv_name, std::bind(&PylonROS2CameraNode::loadUserSetCallback, this, _1, _2));
  
  srv_name = srv_prefix + "reset_device";
  this->reset_device_srv_ = this->create_service<TriggerSrv>(srv_name, std::bind(&PylonROS2CameraNode::triggerDeviceResetCallback, this, _1, _2));
  
  srv_name = srv_prefix + "start_grabbing";
  this->start_grabbing_srv_ = this->create_service<TriggerSrv>(srv_name, std::bind(&PylonROS2CameraNode::startGrabbingCallback, this, _1, _2));
  
  srv_name = srv_prefix + "stop_grabbing";
  this->stop_grabbing_srv_ = this->create_service<TriggerSrv>(srv_name, std::bind(&PylonROS2CameraNode::stopGrabbingCallback, this, _1, _2));

  srv_name = srv_prefix + "update_sync_free_run_timer";
  this->update_sync_free_run_timer_srv_ = this->create_service<TriggerSrv>(srv_name, std::bind(&PylonROS2CameraNode::updateSyncFreeRunTimerCallback, this, _1, _2));
}

void PylonROS2CameraNode::initActions()
{
  using namespace std::placeholders;

  this->grab_imgs_raw_as_ = rclcpp_action::create_server<GrabImagesAction>(
    this,
    "~/grab_images_raw",
    std::bind(&PylonROS2CameraNode::handleGrabRawImagesActionGoal, this, _1, _2),
    std::bind(&PylonROS2CameraNode::handleGrabRawImagesActionGoalCancel, this, _1),
    std::bind(&PylonROS2CameraNode::handleGrabRawImagesActionGoalAccepted, this, _1));
}

void PylonROS2CameraNode::initDiagnostics()
{
  using namespace std::chrono_literals;

  this->diagnostics_updater_.setHardwareID("none");
  this->diagnostics_updater_.add("camera_availability", this, &PylonROS2CameraNode::createDiagnostics);
  this->diagnostics_updater_.add("intrinsic_calibration", this, &PylonROS2CameraNode::createCameraInfoDiagnostics);

  auto diagnostics_trigger = this->create_wall_timer(2000ms, std::bind(&PylonROS2CameraNode::diagnosticsTimerCallback, this));
}

bool PylonROS2CameraNode::initAndRegister()
{
  this->pylon_camera_ = PylonROS2Camera::create(this->pylon_camera_parameter_set_.deviceUserID());

  if (this->pylon_camera_ == nullptr)
  {
    this->cm_status_.status_id = pylon_ros2_camera_interfaces::msg::ComponentStatus::ERROR;
    this->cm_status_.status_msg = "No available camera";

    if (this->pylon_camera_parameter_set_.enable_status_publisher_)
    {
      this->component_status_pub_->publish(this->cm_status_);
    }

    RCLCPP_WARN_STREAM(LOGGER, "Failed to connect camera device with device user id: "<< this->pylon_camera_parameter_set_.deviceUserID() << ". "
                            << "Wait and retry to connect until the specified camera is available...");

    // wait and retry until a camera is present
    rclcpp::Time end = rclcpp::Node::now() + std::chrono::duration<double>(15);
    rclcpp::Rate r(0.5);
    while (rclcpp::ok() && this->pylon_camera_ == nullptr)
    {
      this->pylon_camera_ = PylonROS2Camera::create(this->pylon_camera_parameter_set_.deviceUserID());
      if (this->pylon_camera_ == nullptr)
      {
        RCLCPP_WARN_STREAM(LOGGER, "Failed to connect camera device with device user id: "<< this->pylon_camera_parameter_set_.deviceUserID() << ". "
                            << "Trying again in a bit...");
      }

      if (rclcpp::Node::now() > end)
      {
        RCLCPP_WARN_STREAM(LOGGER, "No available camera. Keep waiting and trying...");
        this->cm_status_.status_id = pylon_ros2_camera_interfaces::msg::ComponentStatus::ERROR;
        this->cm_status_.status_msg = "No available camera";

        if (this->pylon_camera_parameter_set_.enable_status_publisher_)
        {
          this->component_status_pub_->publish(this->cm_status_);
        }

        end = rclcpp::Node::now() + std::chrono::duration<double>(15);
      }

      r.sleep();
    }
  }

  if (this->pylon_camera_ != nullptr && this->pylon_camera_parameter_set_.deviceUserID().empty())
  {
    this->pylon_camera_parameter_set_.setDeviceUserId(*this, this->pylon_camera_->deviceUserID());
    this->cm_status_.status_id = pylon_ros2_camera_interfaces::msg::ComponentStatus::RUNNING;
    this->cm_status_.status_msg = "running";
    if (this->pylon_camera_parameter_set_.enable_status_publisher_)
    {
      this->component_status_pub_->publish(this->cm_status_);
    }
  }

  if (!rclcpp::ok())
  {
    return false;
  }

  if (!this->pylon_camera_->registerCameraConfiguration())
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Error while registering the camera configuration to software-trigger mode!");
    this->cm_status_.status_id = pylon_ros2_camera_interfaces::msg::ComponentStatus::ERROR;
    this->cm_status_.status_msg = "Error while registering the camera configuration";
    if (this->pylon_camera_parameter_set_.enable_status_publisher_)
    {
      this->component_status_pub_->publish(this->cm_status_);
    }
    return false;
  }

  if (!this->pylon_camera_->openCamera())
  {
    RCLCPP_ERROR(LOGGER, "Error while trying to open the user-specified camera!");
    this->cm_status_.status_id = pylon_ros2_camera_interfaces::msg::ComponentStatus::ERROR;
    this->cm_status_.status_msg = "Error while trying to open the user-specified camera!";
    if (this->pylon_camera_parameter_set_.enable_status_publisher_)
    {
      this->component_status_pub_->publish(this->cm_status_);
    }
    return false;
  }

  if (!this->pylon_camera_->applyCamSpecificStartupSettings(this->pylon_camera_parameter_set_))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Error while applying the user-specified startup settings " << "(e.g., mtu size for GigE, ...) to the camera!");
    this->cm_status_.status_id = pylon_ros2_camera_interfaces::msg::ComponentStatus::ERROR;
    this->cm_status_.status_msg = "Error while applying the user-specified startup settings";
    if (this->pylon_camera_parameter_set_.enable_status_publisher_)
    {
      this->component_status_pub_->publish(this->cm_status_);
    }
    return false;
  }

  return true;
}

bool PylonROS2CameraNode::startGrabbing()
{
  if (!this->pylon_camera_->startGrabbing(pylon_camera_parameter_set_))
  {
    RCLCPP_ERROR(LOGGER, "Error while start grabbing");
    return false;
  }

  size_t num_user_outputs = this->pylon_camera_->numUserOutputs();
  this->set_user_output_srvs_.resize(2 * num_user_outputs);
  
  for (int i = 0; i < (int)num_user_outputs; ++i)
  {
    std::string srv_name = std::string("~/set_user_output_") + std::to_string(i);
    if (!this->serviceExists(srv_name))
    {
      this->set_user_output_srvs_.at(i) = this->create_service<SetBoolSrv>(
        srv_name,
        [this, i](const std::shared_ptr<SetBoolSrv::Request> request, std::shared_ptr<SetBoolSrv::Response> response)
        {
          this->setUserOutputCallback(i, request, response);
        });
    }
    
    std::string srv_name_af = std::string("~/activate_autoflash_output_") + std::to_string(i);
    if (!this->serviceExists(srv_name_af))
    {
      this->set_user_output_srvs_.at(num_user_outputs + i) = this->create_service<SetBoolSrv>(
        srv_name_af,
        [this, i](const std::shared_ptr<SetBoolSrv::Request> request, std::shared_ptr<SetBoolSrv::Response> response)
        {
            this->setAutoflashCallback(i + 2, request, response); // ! using lines 2 and 3
        });
    }
  }

  this->img_raw_msg_.header.frame_id = this->pylon_camera_parameter_set_.cameraFrame();
  // Encoding of pixels -- channel meaning, ordering, size
  // taken from the list of strings in include/sensor_msgs/image_encodings.h
  this->img_raw_msg_.encoding = this->pylon_camera_->currentROSEncoding();
  this->img_raw_msg_.height = this->pylon_camera_->imageRows();
  this->img_raw_msg_.width = this->pylon_camera_->imageCols();
  // step = full row length in bytes, img_size = (step * rows), imagePixelDepth
  // already contains the number of channels
  this->img_raw_msg_.step = this->img_raw_msg_.width * this->pylon_camera_->imagePixelDepth();

  if (!this->camera_info_manager_->setCameraName(this->pylon_camera_->deviceUserID()))
  { 
    // valid name contains only alphanumeric signs and '_'
    RCLCPP_WARN_STREAM(LOGGER, "[" << this->pylon_camera_->deviceUserID() << "] name not valid for camera_info_manager");
  }

  this->setupSamplingIndices(sampling_indices_,
                             this->pylon_camera_->imageRows(),
                             this->pylon_camera_->imageCols(),
                             this->pylon_camera_parameter_set_.downsampling_factor_exposure_search_);


  // Initial setting of the CameraInfo-msg, assuming no calibration given
  sensor_msgs::msg::CameraInfo initial_cam_info;
  this->setupInitialCameraInfo(initial_cam_info);
  this->camera_info_manager_->setCameraInfo(initial_cam_info);
  if (this->pylon_camera_parameter_set_.cameraInfoURL().empty() ||
      !this->camera_info_manager_->validateURL(this->pylon_camera_parameter_set_.cameraInfoURL()))
  { 
    RCLCPP_INFO_STREAM(LOGGER, "CameraInfoURL needed for rectification! ROS2-Param: "
        << "'" << this->get_namespace() << "/camera_info_url' = '"
        << this->pylon_camera_parameter_set_.cameraInfoURL() << "' is invalid!");
    RCLCPP_DEBUG_STREAM(LOGGER, "CameraInfoURL should have following style: "
        << "'file:///full/path/to/local/file.yaml' or "
        << "'file://${ROS_HOME}/camera_info/${NAME}.yaml'");
    RCLCPP_WARN(LOGGER, "Will only provide distorted /image_raw images!");
  }
  else
  { 
    // override initial camera info if the url is valid
    if (this->camera_info_manager_->loadCameraInfo(this->pylon_camera_parameter_set_.cameraInfoURL()))
    {
      this->setupRectification();
      // set the correct tf frame_id
      sensor_msgs::msg::CameraInfo cam_info = this->camera_info_manager_->getCameraInfo();
      cam_info.header.frame_id = this->img_raw_msg_.header.frame_id;
      this->camera_info_manager_->setCameraInfo(cam_info);
    }
    else
    { 
      RCLCPP_WARN(LOGGER, "Will only provide distorted /image_raw images!");
    }
  }

  if (this->pylon_camera_parameter_set_.binning_x_given_)
  {   
    size_t reached_binning_x;
    this->setBinningX(this->pylon_camera_parameter_set_.binning_x_, reached_binning_x);
    RCLCPP_INFO_STREAM(LOGGER, "Setting horizontal binning_x to "
            << this->pylon_camera_parameter_set_.binning_x_);
    RCLCPP_WARN_STREAM(LOGGER, "The image width of the camera_info-msg will "
            << "be adapted, so that the binning_x value in this msg remains 1");
  }

  if (this->pylon_camera_parameter_set_.binning_y_given_)
  {   
    size_t reached_binning_y;
    this->setBinningY(this->pylon_camera_parameter_set_.binning_y_, reached_binning_y);
    RCLCPP_INFO_STREAM(LOGGER, "Setting vertical binning_y to "
            << pylon_camera_parameter_set_.binning_y_);
    RCLCPP_WARN_STREAM(LOGGER, "The image height of the camera_info-msg will "
            << "be adapted, so that the binning_y value in this msg remains 1");
  }

  if (this->pylon_camera_parameter_set_.exposure_given_)
  {   
    float reached_exposure;
    this->setExposure(this->pylon_camera_parameter_set_.exposure_, reached_exposure);
    RCLCPP_INFO_STREAM(LOGGER, "Setting exposure to "
            << this->pylon_camera_parameter_set_.exposure_ << ", reached: "
            << reached_exposure);
  }
  
  if (this->pylon_camera_parameter_set_.gain_given_)
  {   
    float reached_gain;
    this->setGain(this->pylon_camera_parameter_set_.gain_, reached_gain);
    RCLCPP_INFO_STREAM(LOGGER, "Setting gain to: "
            << this->pylon_camera_parameter_set_.gain_ << ", reached: "
            << reached_gain);
  }

  if (pylon_camera_parameter_set_.gamma_given_)
  {   
    float reached_gamma;
    this->setGamma(pylon_camera_parameter_set_.gamma_, reached_gamma);
    RCLCPP_INFO_STREAM(LOGGER, "Setting gamma to " << this->pylon_camera_parameter_set_.gamma_
            << ", reached: " << reached_gamma);
  }

  if ( pylon_camera_parameter_set_.brightness_given_ )
  {   
    int reached_brightness;
    this->setBrightness(this->pylon_camera_parameter_set_.brightness_,
                        reached_brightness,
                        this->pylon_camera_parameter_set_.exposure_auto_,
                        this->pylon_camera_parameter_set_.gain_auto_);
    RCLCPP_INFO_STREAM(LOGGER, "Setting brightness to: "
            << this->pylon_camera_parameter_set_.brightness_ << ", reached: "
            << reached_brightness);
    if (this->pylon_camera_parameter_set_.brightness_continuous_)
    {   
      if ( this->pylon_camera_parameter_set_.exposure_auto_)
      {
        this->pylon_camera_->enableContinuousAutoExposure();
      }
      if (this->pylon_camera_parameter_set_.gain_auto_)
      {
        this->pylon_camera_->enableContinuousAutoGain();
      }
    }
    else
    { 
      this->pylon_camera_->disableAllRunningAutoBrightessFunctions();
    }
  }

  RCLCPP_INFO_STREAM(LOGGER, "Startup settings: "
      << "encoding = '" << this->pylon_camera_->currentROSEncoding() << "', "
      << "binning = [" << this->pylon_camera_->currentBinningX() << ", "
      << this->pylon_camera_->currentBinningY() << "], "
      << "exposure = " << this->pylon_camera_->currentExposure() << ", "
      << "gain = " << this->pylon_camera_->currentGain() << ", "
      << "gamma = " <<  this->pylon_camera_->currentGamma() << ", "
      << "shutter mode = " << pylon_camera_parameter_set_.shutterModeString());
  
  // Framerate Settings
  if (this->pylon_camera_->maxPossibleFramerate() < this->pylon_camera_parameter_set_.frameRate())
  {
    RCLCPP_INFO(LOGGER, "Desired framerate %.2f is higher than max possible. Will limit framerate to: %.2f Hz",
              this->pylon_camera_parameter_set_.frameRate(),
              this->pylon_camera_->maxPossibleFramerate());
    this->pylon_camera_parameter_set_.setFrameRate(*this, this->pylon_camera_->maxPossibleFramerate());
  }
  else if (this->pylon_camera_parameter_set_.frameRate() == -1)
  {
    this->pylon_camera_parameter_set_.setFrameRate(*this, this->pylon_camera_->maxPossibleFramerate());
    RCLCPP_INFO(LOGGER, "Max possible framerate is %.2f Hz", this->pylon_camera_->maxPossibleFramerate());
  }

  return true;
}

void PylonROS2CameraNode::spin()
{
  if (this->camera_info_manager_->isCalibrated())
  {
    RCLCPP_INFO_ONCE(LOGGER, "Camera is calibrated");
  }
  else
  {
    RCLCPP_INFO_ONCE(LOGGER, "Camera not calibrated");
  }

  if (this->pylon_camera_->isCamRemoved())
  {
    RCLCPP_ERROR(LOGGER, "Pylon camera has been removed, trying to reset");
    
    this->cm_status_.status_id = pylon_ros2_camera_interfaces::msg::ComponentStatus::ERROR;;
    this->cm_status_.status_msg = "Pylon camera has been removed, trying to reset";
      
    if (this->pylon_camera_parameter_set_.enable_status_publisher_)
    {
      this->component_status_pub_->publish(this->cm_status_);
    }
      
    if (this->pylon_camera_)
    {
      delete this->pylon_camera_;
      this->pylon_camera_ = nullptr;
    }
      
    // Possible issue here: ROS2 does not allow to shutdown services
    // Services are shutdown in the ROS 1 pylon version at this level
    this->set_user_output_srvs_.clear();

    rclcpp::Rate r(0.5);
    r.sleep();

    this->init();
    
    return;
  }
  
  if (!this->isSleeping() && (this->img_raw_pub_.getNumSubscribers() || this->getNumSubscribersRectImagePub()))
  {
    if (this->img_raw_pub_.getNumSubscribers() || this->getNumSubscribersRectImagePub())
    {
      if (!this->grabImage())
      {
        return;
      }
    }

    if (this->img_raw_pub_.getNumSubscribers() > 0)
    {
      // get actual cam_info-object in every frame, because it might have
      // changed due to a 'set_camera_info'-service call
      sensor_msgs::msg::CameraInfo cam_info = this->camera_info_manager_->getCameraInfo();
      cam_info.header.stamp = this->img_raw_msg_.header.stamp;
      // publish via image_transport
      this->img_raw_pub_.publish(this->img_raw_msg_, cam_info);
    }

    // this->getNumSubscribersRectImagePub() involves that this->camera_info_manager_->isCalibrated() == true
    if (this->getNumSubscribersRectImagePub() > 0)
    {
      this->cv_bridge_img_rect_->header.stamp = this->img_raw_msg_.header.stamp;
      assert(this->pinhole_model_->initialized());
      cv_bridge::CvImagePtr cv_img_raw = cv_bridge::toCvCopy(
            this->img_raw_msg_,
            this->img_raw_msg_.encoding);
      this->pinhole_model_->fromCameraInfo(this->camera_info_manager_->getCameraInfo());
      this->pinhole_model_->rectifyImage(cv_img_raw->image, this->cv_bridge_img_rect_->image);
      this->img_rect_pub_->publish(this->cv_bridge_img_rect_->toImageMsg());
    }
  }

  // Check if the image encoding changed , then save the new image encoding and restart the image grabbing to fix the ros sensor message type issue.
  if (this->pylon_camera_parameter_set_.imageEncoding() != this->pylon_camera_->currentROSEncoding()) 
  {
    this->pylon_camera_parameter_set_.setimageEncodingParam(*this, this->pylon_camera_->currentROSEncoding());
    this->grabbingStopping();
    this->grabbingStarting();
  }

  if (this->pylon_camera_parameter_set_.enable_status_publisher_)
  {
    this->component_status_pub_->publish(this->cm_status_);
  }

  if (this->pylon_camera_parameter_set_.enable_current_params_publisher_)
  {
    this->currentParamPub();
  }
}

bool PylonROS2CameraNode::grabImage()
{
  using namespace std::chrono_literals;

  std::lock_guard<std::recursive_mutex> lock(this->grab_mutex_);
  // Store current time before the image is transmitted for a more accurate grab time estimation
  auto grab_time = rclcpp::Node::now();
  if (!this->pylon_camera_->grab(img_raw_msg_.data))
  {
    return false;
  }
  img_raw_msg_.header.stamp = grab_time;
  return true;
}

bool PylonROS2CameraNode::setExposure(const float& target_exposure, float& reached_exposure)
{
  std::lock_guard<std::recursive_mutex> lock(this->grab_mutex_);
  if (!this->pylon_camera_->isReady())
  {
    RCLCPP_WARN(LOGGER, "Error in setExposure(): pylon_camera_ is not ready!");
    return false;
  }

  if (this->pylon_camera_->setExposure(target_exposure, reached_exposure))
  {
    // success if the delta is smaller then the exposure step
    return true;
  }
  else  // retry till timeout
  {
    // wait for max 5s till the cam has updated the exposure
    rclcpp::Rate r(10.0);
    rclcpp::Time timeout(rclcpp::Node::now() + std::chrono::duration<double>(5));
    while (rclcpp::ok())
    {
      if (this->pylon_camera_->setExposure(target_exposure, reached_exposure))
      {
          // success if the delta is smaller then the exposure step
          return true;
      }

      if (rclcpp::Node::now() > timeout)
      {
          break;
      }
      r.sleep();
    }
    RCLCPP_ERROR_STREAM(LOGGER, "Error in setExposure(): Unable to set target"
      << " exposure before timeout");
    return false;
  }
}

bool PylonROS2CameraNode::setBrightness(const int& target_brightness,
                                        int& reached_brightness,
                                        const bool& exposure_auto,
                                        const bool& gain_auto)
{
  std::lock_guard<std::recursive_mutex> lock(this->grab_mutex_);
  rclcpp::Time begin = rclcpp::Node::now(); // time measurement for the exposure search

  // brightness service can only work, if an image has already been grabbed,
  // because it calculates the mean on the current image. The interface is
  // ready if the grab-result-pointer of the first acquisition contains
  // valid data
  if (!this->waitForCamera(std::chrono::duration<double>(3)))
  {
    RCLCPP_ERROR(LOGGER, "Setting brightness failed: interface not ready, although waiting for 3 sec!");
    return false;
  }

  int target_brightness_co = std::min(255, target_brightness);
  // smart brightness search initially sets the last rememberd exposure time
  if (this->brightness_exp_lut_.at(target_brightness_co) != 0.0)
  {
    float reached_exp;
    if (!this->setExposure(this->brightness_exp_lut_.at(target_brightness_co), reached_exp))
    {
      RCLCPP_WARN_STREAM(LOGGER, "Tried to speed-up exposure search with initial"
              << " guess, but setting the exposure failed!");
    }
    else
    {
      RCLCPP_DEBUG_STREAM(LOGGER, "Speed-up exposure search with initial exposure"
              << " guess of " << reached_exp);
    }
  }

  // get actual image -> fills img_raw_msg_.data vector
  if (!this->grabImage())
  {
    RCLCPP_ERROR(LOGGER, "Failed to grab image, can't calculate current brightness!");
    return false;
  }

  // calculates current brightness by generating the mean over all pixels
  // stored in img_raw_msg_.data vector
  float current_brightness = this->calcCurrentBrightness();

  RCLCPP_DEBUG_STREAM(LOGGER, "New brightness request for target brightness "
          << target_brightness_co << ", current brightness = "
          << current_brightness);

  if (std::fabs(current_brightness - static_cast<float>(target_brightness_co)) <= 1.0)
  {
    reached_brightness = static_cast<int>(current_brightness);
    rclcpp::Time end = rclcpp::Node::now();
    RCLCPP_DEBUG_STREAM(LOGGER, "Brightness reached without exposure search, duration: "
            << ((end-begin).nanoseconds()/ 1e9));
    return true;  // target brightness already reached
  }

  // initially cancel all running exposure search by deactivating
  // ExposureAuto & AutoGain
  this->pylon_camera_->disableAllRunningAutoBrightessFunctions();

  if (target_brightness_co <= 50)
  {
    // own binary-exp search: we need to have the upper bound -> PylonAuto
    // exposure to a initial start value of 50 provides it
    if (brightness_exp_lut_.at(50) != 0.0)
    {
      float reached_exp;
      if (!this->setExposure(brightness_exp_lut_.at(50), reached_exp))
      {
        RCLCPP_WARN_STREAM(LOGGER, "Tried to speed-up exposure search with initial"
            << " guess, but setting the exposure failed!");
      }
      else
      {
        RCLCPP_DEBUG_STREAM(LOGGER, "Speed-up exposure search with initial exposure"
            << " guess of " << reached_exp);
      }
    }
  }

  if (!exposure_auto && !gain_auto)
  {
    RCLCPP_WARN_STREAM(LOGGER, "Neither Auto Exposure Time ('exposure_auto') nor Auto "
        << "Gain ('gain_auto') are enabled! Hence gain and exposure time "
        << "are assumed to be fix and the target brightness ("
        << target_brightness_co << ") can not be reached!");
    return false;
  }

  bool is_brightness_reached = false;
  size_t fail_safe_ctr = 0;
  size_t fail_safe_ctr_limit = 10;
  if (this->pylon_camera_->typeName() == "DART")
  {
    // DART Cameras may need up to 50 images till the desired brightness
    // value can be reached. USB & GigE Cameras can achieve that much faster
    fail_safe_ctr_limit = 50;
  }
  float last_brightness = std::numeric_limits<float>::max();

  // timeout for the exposure search -> need more time for great exposure values
  rclcpp::Time start_time = rclcpp::Node::now();
  rclcpp::Time timeout = start_time;
  if (target_brightness_co < 205)
  {
    timeout += std::chrono::duration<double>(this->pylon_camera_parameter_set_.exposure_search_timeout_);
  }
  else
  {
    timeout += std::chrono::duration<double>(10.0 + this->pylon_camera_parameter_set_.exposure_search_timeout_);
  }

  while (rclcpp::ok())
  {
    // calling setBrightness in every cycle would not be necessary for the pylon auto
    // brightness search. But for the case that the target brightness is out of the
    // pylon range which is from [50 - 205] a binary exposure search will be executed
    //  where we have to update the search parameter in every cycle
    if (!this->pylon_camera_->setBrightness(target_brightness_co,
                                            current_brightness,
                                            exposure_auto,
                                            gain_auto))
    {
      this->pylon_camera_->disableAllRunningAutoBrightessFunctions();
      break;
    }

    if (!this->grabImage())
    {
      return false;
    }

    if (this->pylon_camera_->isPylonAutoBrightnessFunctionRunning())
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

    current_brightness = this->calcCurrentBrightness();
    is_brightness_reached = fabs(current_brightness - static_cast<float>(target_brightness_co)) < this->pylon_camera_->maxBrightnessTolerance();

    if (is_brightness_reached)
    {
      this->pylon_camera_->disableAllRunningAutoBrightessFunctions();
      break;
    }

    if (std::fabs(last_brightness - current_brightness) <= 1.0)
    {
      fail_safe_ctr++;
    }
    else
    {
      fail_safe_ctr = 0;
    }

    last_brightness = current_brightness;

    if ((fail_safe_ctr > fail_safe_ctr_limit ) && !is_brightness_reached)
    {
      RCLCPP_WARN_STREAM(LOGGER, "Seems like the desired brightness (" << target_brightness_co
              << ") is not reachable! Stuck at brightness " << current_brightness
              << " and exposure " << this->pylon_camera_->currentExposure() << "us");
      this->pylon_camera_->disableAllRunningAutoBrightessFunctions();
      reached_brightness = static_cast<int>(current_brightness);
      return false;
    }

    if (rclcpp::Node::now() > timeout)
    {
      // cancel all running brightness search by deactivating ExposureAuto
      this->pylon_camera_->disableAllRunningAutoBrightessFunctions();
      RCLCPP_WARN_STREAM(LOGGER, "Did not reach the target brightness before "
          << "timeout of " << ((timeout - start_time).nanoseconds()/ 1e9)
          << " sec! Stuck at brightness " << current_brightness
          << " and exposure " << this->pylon_camera_->currentExposure() << "us");
      reached_brightness = static_cast<int>(current_brightness);
      return false;
    }
  }

  RCLCPP_DEBUG_STREAM(LOGGER, "Finally reached brightness: " << current_brightness);
  reached_brightness = static_cast<int>(current_brightness);

  // store reached brightness - exposure tuple for next times search
  if (is_brightness_reached)
  {
    if (this->brightness_exp_lut_.at(reached_brightness) == 0.0)
    {
      this->brightness_exp_lut_.at(reached_brightness) = this->pylon_camera_->currentExposure();
    }
    else
    {
      this->brightness_exp_lut_.at(reached_brightness) += this->pylon_camera_->currentExposure();
      this->brightness_exp_lut_.at(reached_brightness) *= 0.5;
    }
    if (this->brightness_exp_lut_.at(target_brightness_co) == 0.0)
    {
      this->brightness_exp_lut_.at(target_brightness_co) = this->pylon_camera_->currentExposure();
    }
    else
    {
      this->brightness_exp_lut_.at(target_brightness_co) += this->pylon_camera_->currentExposure();
      this->brightness_exp_lut_.at(target_brightness_co) *= 0.5;
    }
  }

  rclcpp::Time end = rclcpp::Node::now();
  RCLCPP_DEBUG_STREAM(LOGGER, "Brightness search duration: " << ((end-begin).nanoseconds()/ 1e9));
  
  return is_brightness_reached;
}

bool PylonROS2CameraNode::setGain(const float& target_gain, float& reached_gain)
{
  std::lock_guard<std::recursive_mutex> lock(this->grab_mutex_);
  if (!this->pylon_camera_->isReady())
  {
    RCLCPP_WARN(LOGGER, "Error in setGain(): pylon_camera_ is not ready!");
    return false;
  }

  if (this->pylon_camera_->setGain(target_gain, reached_gain))
  {
    return true;
  }
  else  // retry till timeout
  {
    // wait for max 10s till the cam has updated the exposure
    rclcpp::Rate r(10.0);
    rclcpp::Time timeout(rclcpp::Node::now() + std::chrono::duration<double>(5));
    while (rclcpp::ok())
    {
      if (this->pylon_camera_->setGain(target_gain, reached_gain))
      {
        return true;
      }

      if (rclcpp::Node::now() > timeout)
      {
        break;
      }
      r.sleep();
    }
    RCLCPP_ERROR_STREAM(LOGGER, "Error in setGain(): Unable to set target "
        << "gain before timeout");
    return false;
  }
}

bool PylonROS2CameraNode::setGamma(const float& target_gamma, float& reached_gamma)
{
  std::lock_guard<std::recursive_mutex> lock(this->grab_mutex_);
  if (!this->pylon_camera_->isReady())
  {
    RCLCPP_WARN(LOGGER, "Error in setGamma(): pylon_camera_ is not ready!");
    return false;
  }

  if (this->pylon_camera_->setGamma(target_gamma, reached_gamma))
  {
    return true;
  }
  else  // retry till timeout
  {
    // wait for max 10s till the cam has updated the gamma value
    rclcpp::Rate r(10.0);
    rclcpp::Time timeout(rclcpp::Node::now() + std::chrono::duration<double>(5));
    while (rclcpp::ok())
    {
      if (this->pylon_camera_->setGamma(target_gamma, reached_gamma))
      {
        return true;
      }

      if (rclcpp::Node::now() > timeout)
      {
        break;
      }
      r.sleep();
    }
    RCLCPP_ERROR_STREAM(LOGGER, "Error in setGamma(): Unable to set target "
        << "gamma before timeout");
    return false;
  }
}

bool PylonROS2CameraNode::setROI(const sensor_msgs::msg::RegionOfInterest target_roi,
                                 sensor_msgs::msg::RegionOfInterest& reached_roi)
{
  std::lock_guard<std::recursive_mutex> lock(this->grab_mutex_);
  if (!this->pylon_camera_->setROI(target_roi, reached_roi) )
  {
    // retry till timeout
    rclcpp::Rate r(10.0);
    rclcpp::Time timeout(rclcpp::Node::now() + std::chrono::duration<double>(2));
    while (rclcpp::ok())
    {
      if (this->pylon_camera_->setROI(target_roi, reached_roi))
      {
        break;
      }
      if (rclcpp::Node::now() > timeout)
      {
        RCLCPP_ERROR_STREAM(LOGGER, "Error in setROI(): Unable to set target "
                << "roi before timeout");
        sensor_msgs::msg::CameraInfo cam_info = this->camera_info_manager_->getCameraInfo();
        cam_info.roi = this->pylon_camera_->currentROI();
        this->camera_info_manager_->setCameraInfo(cam_info);
        this->img_raw_msg_.width = this->pylon_camera_->imageCols();
        this->img_raw_msg_.height = this->pylon_camera_->imageRows();
        // step = full row length in bytes, img_size = (step * rows), imagePixelDepth
        // already contains the number of channels
        this->img_raw_msg_.step = this->img_raw_msg_.width * this->pylon_camera_->imagePixelDepth();
        return false;
      }
      r.sleep();
    }
  }

  sensor_msgs::msg::CameraInfo cam_info = this->camera_info_manager_->getCameraInfo();
  cam_info.roi = this->pylon_camera_->currentROI();
  this->camera_info_manager_->setCameraInfo(cam_info);
  this->img_raw_msg_.height = this->pylon_camera_->imageRows();
  this->img_raw_msg_.width = this->pylon_camera_->imageCols();
  // step = full row length in bytes, img_size = (step * rows), imagePixelDepth
  // already contains the number of channels
  this->img_raw_msg_.step = this->img_raw_msg_.width * this->pylon_camera_->imagePixelDepth();

  return true;
}
  
bool PylonROS2CameraNode::setBinningX(const size_t& target_binning_x,
                                      size_t& reached_binning_x)
{
  std::lock_guard<std::recursive_mutex> lock(this->grab_mutex_);
  if (!this->pylon_camera_->setBinningX(target_binning_x, reached_binning_x))
  {
    // retry till timeout
    rclcpp::Rate r(10.0);
    rclcpp::Time timeout(rclcpp::Node::now() + std::chrono::duration<double>(2));
    while (rclcpp::ok())
    {
      if (this->pylon_camera_->setBinningX(target_binning_x, reached_binning_x))
      {
        break;
      }

      if (rclcpp::Node::now() > timeout)
      {
        RCLCPP_ERROR_STREAM(LOGGER, "Error in setBinningX(): Unable to set target "
                << "binning_x factor before timeout");
        sensor_msgs::msg::CameraInfo cam_info = this->camera_info_manager_->getCameraInfo();
        cam_info.binning_x = this->pylon_camera_->currentBinningX();
        this->camera_info_manager_->setCameraInfo(cam_info);
        this->img_raw_msg_.width = this->pylon_camera_->imageCols();
        // step = full row length in bytes, img_size = (step * rows), imagePixelDepth
        // already contains the number of channels
        this->img_raw_msg_.step = this->img_raw_msg_.width * this->pylon_camera_->imagePixelDepth();
        return false;
      }

      r.sleep();
    }
  }
  
  sensor_msgs::msg::CameraInfo cam_info = this->camera_info_manager_->getCameraInfo();
  cam_info.binning_x = this->pylon_camera_->currentBinningX();
  this->camera_info_manager_->setCameraInfo(cam_info);
  this->img_raw_msg_.width = this->pylon_camera_->imageCols();
  // step = full row length in bytes, img_size = (step * rows), imagePixelDepth
  // already contains the number of channels
  this->img_raw_msg_.step = this->img_raw_msg_.width * this->pylon_camera_->imagePixelDepth();
  this->setupSamplingIndices(this->sampling_indices_,
                             this->pylon_camera_->imageRows(),
                             this->pylon_camera_->imageCols(),
                             this->pylon_camera_parameter_set_.downsampling_factor_exposure_search_);

  return true;
}
  
bool PylonROS2CameraNode::setBinningY(const size_t& target_binning_y,
                                      size_t& reached_binning_y)
{
  std::lock_guard<std::recursive_mutex> lock(this->grab_mutex_);
  if (!this->pylon_camera_->setBinningY(target_binning_y, reached_binning_y))
  {
    // retry till timeout
    rclcpp::Rate r(10.0);
    rclcpp::Time timeout(rclcpp::Node::now() + std::chrono::duration<double>(2));
    while (rclcpp::ok())
    {
      if (this->pylon_camera_->setBinningY(target_binning_y, reached_binning_y))
      {
        break;
      }
      if (rclcpp::Node::now() > timeout)
      {
        RCLCPP_ERROR_STREAM(LOGGER, "Error in setBinningY(): Unable to set target "
                << "binning_y factor before timeout");
        sensor_msgs::msg::CameraInfo cam_info = this->camera_info_manager_->getCameraInfo();
        cam_info.binning_y = this->pylon_camera_->currentBinningY();
        this->camera_info_manager_->setCameraInfo(cam_info);
        this->img_raw_msg_.height = this->pylon_camera_->imageRows();
        // step = full row length in bytes, img_size = (step * rows), imagePixelDepth
        // already contains the number of channels
        this->img_raw_msg_.step = this->img_raw_msg_.width * this->pylon_camera_->imagePixelDepth();
        return false;
      }
      r.sleep();
    }
  }

  sensor_msgs::msg::CameraInfo cam_info = this->camera_info_manager_->getCameraInfo();
  cam_info.binning_y = this->pylon_camera_->currentBinningY();
  this->camera_info_manager_->setCameraInfo(cam_info);
  this->img_raw_msg_.height = this->pylon_camera_->imageRows();
  // step = full row length in bytes, img_size = (step * rows), imagePixelDepth
  // already contains the number of channels
  this->img_raw_msg_.step = this->img_raw_msg_.width * this->pylon_camera_->imagePixelDepth();
  this->setupSamplingIndices(this->sampling_indices_,
                             this->pylon_camera_->imageRows(),
                             this->pylon_camera_->imageCols(),
                             this->pylon_camera_parameter_set_.downsampling_factor_exposure_search_);
  
  return true;
}

std::string PylonROS2CameraNode::setOffsetXY(const int& offsetValue, bool xAxis)
{
  std::lock_guard<std::recursive_mutex> lock(this->grab_mutex_);
  if (!this->pylon_camera_->isReady())
  {
    RCLCPP_WARN(LOGGER, "Error in setOffsetXY(): pylon_camera_ is not ready!");
    return "pylon camera is not ready!";
  }
  return this->pylon_camera_->setOffsetXY(offsetValue, xAxis);
}

std::string PylonROS2CameraNode::reverseXY(const bool& data, bool around_x)
{
  std::lock_guard<std::recursive_mutex> lock(this->grab_mutex_);
  if (!this->pylon_camera_->isReady())
  {
    RCLCPP_WARN(LOGGER, "Error in reverseXY(): pylon_camera_ is not ready!");
    return "pylon camera is not ready!";
  }

  return this->pylon_camera_->reverseXY(data, around_x);
}

std::string PylonROS2CameraNode::setBlackLevel(const int& value)
{
  std::lock_guard<std::recursive_mutex> lock(this->grab_mutex_);
  if (!this->pylon_camera_->isReady())
  {
    RCLCPP_WARN(LOGGER, "Error in setBlackLevel(): pylon_camera_ is not ready!");
    return "pylon camera is not ready!";
  }

  return this->pylon_camera_->setBlackLevel(value);
}

std::string PylonROS2CameraNode::setPGIMode(const bool& on)
{   
  // mode 0 = Simple
  // mode 1 = Basler PGI
  std::lock_guard<std::recursive_mutex> lock(this->grab_mutex_);
  if (!this->pylon_camera_->isReady())
  {
    RCLCPP_WARN(LOGGER, "Error in setPGIMode(): pylon_camera_ is not ready!");
    return "pylon camera is not ready!";
  }

  return this->pylon_camera_->setPGIMode(on);
}

std::string PylonROS2CameraNode::setDemosaicingMode(const int& mode)
{   
  // mode 0 = Simple
  // mode 1 = Basler PGI
  std::lock_guard<std::recursive_mutex> lock(this->grab_mutex_);
  if (!this->pylon_camera_->isReady())
  {
    RCLCPP_WARN(LOGGER, "Error in setDemosaicingMode(): pylon_camera_ is not ready!");
    return "pylon camera is not ready!";
  }
  
  return this->pylon_camera_->setDemosaicingMode(mode);
}

std::string PylonROS2CameraNode::setNoiseReduction(const float& value)
{
  std::lock_guard<std::recursive_mutex> lock(this->grab_mutex_);
  if (!this->pylon_camera_->isReady())
  {
    RCLCPP_WARN(LOGGER, "Error in setNoiseReduction(): pylon_camera_ is not ready!");
    return "pylon camera is not ready!";
  }
  
  return this->pylon_camera_->setNoiseReduction(value);
}

std::string PylonROS2CameraNode::setSharpnessEnhancement(const float& value)
{
  std::lock_guard<std::recursive_mutex> lock(this->grab_mutex_);
  if (!this->pylon_camera_->isReady())
  {
    RCLCPP_WARN(LOGGER, "Error in setSharpnessEnhancement(): pylon_camera_ is not ready!");
    return "pylon camera is not ready!";
  }

  return this->pylon_camera_->setSharpnessEnhancement(value);
}

std::string PylonROS2CameraNode::setLightSourcePreset(const int& mode)
{
  // mode 0 = Off
  // mode 1 = Daylight5000K
  // mode 2 = Daylight6500K
  // mode 3 = Tungsten2800K
  std::lock_guard<std::recursive_mutex> lock(this->grab_mutex_);
  if (!this->pylon_camera_->isReady())
  {
    RCLCPP_WARN(LOGGER, "Error in setLightSourcePreset(): pylon_camera_ is not ready!");
    return "pylon camera is not ready!";
  }
  
  return this->pylon_camera_->setLightSourcePreset(mode);
}

std::string PylonROS2CameraNode::setWhiteBalanceAuto(const int& mode)
{  
  // mode 0 = Off
  // mode 1 = Once
  // mode 2 = Continuous
  std::lock_guard<std::recursive_mutex> lock(this->grab_mutex_);
  if (!this->pylon_camera_->isReady())
  {
    RCLCPP_WARN(LOGGER, "Error in setBalanceWhiteAuto(): pylon_camera_ is not ready!");
    return "pylon camera is not ready!";
  }
  
  return this->pylon_camera_->setBalanceWhiteAuto(mode);
}

std::string PylonROS2CameraNode::setSensorReadoutMode(const int& mode)
{   
  // mode = 0 : normal readout mode
  // mode = 1 : fast readout mode
  std::lock_guard<std::recursive_mutex> lock(this->grab_mutex_);
  if (!this->pylon_camera_->isReady())
  {
    RCLCPP_WARN(LOGGER, "Error in setSensorReadoutMode(): pylon_camera_ is not ready!");
    return "pylon camera is not ready!";
  }

  return this->pylon_camera_->setSensorReadoutMode(mode);
}

std::string PylonROS2CameraNode::setAcquisitionFrameCount(const int& frameCount)
{   
  std::lock_guard<std::recursive_mutex> lock(this->grab_mutex_);
  if (!this->pylon_camera_->isReady())
  {
    RCLCPP_WARN(LOGGER, "Error in setAcquisitionFrameCount(): pylon_camera_ is not ready!");
    return "pylon camera is not ready!";
  }

  return this->pylon_camera_->setAcquisitionFrameCount(frameCount);
}

std::string PylonROS2CameraNode::setTriggerSelector(const int& mode)
{   
  // mode 0 = Frame start
  // mode 1 = Frame burst start (ace USB cameras) / Acquisition Start (ace GigE cameras)
  std::lock_guard<std::recursive_mutex> lock(this->grab_mutex_);
  if (!this->pylon_camera_->isReady())
  {
    RCLCPP_WARN(LOGGER, "Error in setTriggerSelector(): pylon_camera_ is not ready!");
    return "pylon camera is not ready!";
  }

  return this->pylon_camera_->setTriggerSelector(mode);
}

std::string PylonROS2CameraNode::setTriggerMode(const bool& value)
{   
  std::lock_guard<std::recursive_mutex> lock(this->grab_mutex_);
  if (!this->pylon_camera_->isReady())
  {
    RCLCPP_WARN(LOGGER, "Error in setTriggerMode(): pylon_camera_ is not ready!");
    return "pylon camera is not ready!";
  }

  return this->pylon_camera_->setTriggerMode(value);
}

std::string PylonROS2CameraNode::executeSoftwareTrigger()
{   
  std::lock_guard<std::recursive_mutex> lock(this->grab_mutex_);
  if (!this->pylon_camera_->isReady())
  {
    RCLCPP_WARN(LOGGER, "Error in executeSoftwareTrigger(): pylon_camera_ is not ready!");
    return "pylon camera is not ready!";
  }

  return this->pylon_camera_->executeSoftwareTrigger();
}

std::string PylonROS2CameraNode::setTriggerSource(const int& source)
{   
  // source 0 = Software
  // source 1 = Line1
  // source 2 = Line3
  // source 2 = Line4
  // source 4 = Action1(only selected GigE Camera)
  std::lock_guard<std::recursive_mutex> lock(this->grab_mutex_);
  if (!this->pylon_camera_->isReady())
  {
    RCLCPP_WARN(LOGGER, "Error in setTriggerSource(): pylon_camera_ is not ready!");
    return "pylon camera is not ready!";
  }

  return this->pylon_camera_->setTriggerSource(source);
}

std::string PylonROS2CameraNode::setTriggerActivation(const int& value)
{   
  std::lock_guard<std::recursive_mutex> lock(this->grab_mutex_);
  if (!this->pylon_camera_->isReady())
  {
    RCLCPP_WARN(LOGGER, "Error in setTriggerActivation(): pylon_camera_ is not ready!");
    return "pylon camera is not ready!";
  }

  return this->pylon_camera_->setTriggerActivation(value);
}

std::string PylonROS2CameraNode::setTriggerDelay(const float& value)
{   
  std::lock_guard<std::recursive_mutex> lock(this->grab_mutex_);
  if (!this->pylon_camera_->isReady())
  {
    RCLCPP_WARN(LOGGER, "Error in setTriggerDelay(): pylon_camera_ is not ready!");
    return "pylon camera is not ready!";
  }
  if ((value >= 0) && (value <= 1000000))
  {
    return this->pylon_camera_->setTriggerDelay(value);
  }
  else
  {
    return "Error: the trigger delay value is out of range (0 - 1,000,000)";
  }
}

std::string PylonROS2CameraNode::setLineSelector(const int& value)
{   
  std::lock_guard<std::recursive_mutex> lock(this->grab_mutex_);
  if (!this->pylon_camera_->isReady())
  {
    RCLCPP_WARN(LOGGER, "Error in setLineSelector(): pylon_camera_ is not ready!");
    return "pylon camera is not ready!";
  }

  return this->pylon_camera_->setLineSelector(value);
}

std::string PylonROS2CameraNode::setLineMode(const int& value)
{   
  std::lock_guard<std::recursive_mutex> lock(this->grab_mutex_);
  if (!this->pylon_camera_->isReady())
  {
    RCLCPP_WARN(LOGGER, "Error in setLineMode(): pylon_camera_ is not ready!");
    return "pylon camera is not ready!";
  }

  return this->pylon_camera_->setLineMode(value);
}

std::string PylonROS2CameraNode::setLineSource(const int& value)
{   
  std::lock_guard<std::recursive_mutex> lock(this->grab_mutex_);
  if (!this->pylon_camera_->isReady())
  {
    RCLCPP_WARN(LOGGER, "Error in setLineSource(): pylon_camera_ is not ready!");
    return "pylon camera is not ready!";
  }

  return this->pylon_camera_->setLineSource(value);
}

std::string PylonROS2CameraNode::setLineInverter(const bool& value)
{   
  std::lock_guard<std::recursive_mutex> lock(this->grab_mutex_);
  if (!this->pylon_camera_->isReady())
  {
    RCLCPP_WARN(LOGGER, "Error in setLineInverter(): pylon_camera_ is not ready!");
    return "pylon camera is not ready!";
  }

  return this->pylon_camera_->setLineInverter(value);
}

std::string PylonROS2CameraNode::setLineDebouncerTime(const float& value)
{   
  std::lock_guard<std::recursive_mutex> lock(this->grab_mutex_);
  if (!this->pylon_camera_->isReady())
  {
    RCLCPP_WARN(LOGGER, "Error in setLineDebouncerTime(): pylon_camera_ is not ready!");
    return "pylon camera is not ready!";
  }
  if ((value >= 0) && (value <= 20000))
  {
    return this->pylon_camera_->setLineDebouncerTime(value);
  }
  else
  {
    return "Error : given line debouncer time value is out of rane (0-20,000)";
  }
}

std::string PylonROS2CameraNode::setUserSetSelector(const int& set)
{   
  // set 0 = Default
  // set 1 = UserSet1
  // set 2 = UserSet2
  // set 3 = UserSet3
  // set 4 = HighGain
  // set 5 = AutoFunctions
  // set 6 = ColorRaw
  std::lock_guard<std::recursive_mutex> lock(this->grab_mutex_);
  if (!this->pylon_camera_->isReady())
  {
    RCLCPP_WARN(LOGGER, "Error in setUserSetSelector(): pylon_camera_ is not ready!");
    return "pylon camera is not ready!";
  }

  return this->pylon_camera_->setUserSetSelector(set);
}

std::string PylonROS2CameraNode::saveUserSet()
{  
  std::lock_guard<std::recursive_mutex> lock(this->grab_mutex_);
  if (!this->pylon_camera_->isReady())
  {
    RCLCPP_WARN(LOGGER, "Error in saveUserSet(): pylon_camera_ is not ready!");
    return "pylon camera is not ready!";
  }

  return this->pylon_camera_->saveUserSet();
}

std::string PylonROS2CameraNode::loadUserSet()
{  
  std::lock_guard<std::recursive_mutex> lock(this->grab_mutex_);
  if (!this->pylon_camera_->isReady())
  {
    RCLCPP_WARN(LOGGER, "Error in loadUserSet(): pylon_camera_ is not ready!");
    return "pylon camera is not ready!";
  }

  return this->pylon_camera_->loadUserSet();
}

std::string PylonROS2CameraNode::setUserSetDefaultSelector(const int& set)
{   
  // set 0 = Default
  // set 1 = UserSet1
  // set 2 = UserSet2
  // set 3 = UserSet3
  // set 4 = HighGain
  // set 5 = AutoFunctions
  // set 6 = ColorRaw
  std::lock_guard<std::recursive_mutex> lock(this->grab_mutex_);
  if (!this->pylon_camera_->isReady())
  {
    RCLCPP_WARN(LOGGER, "Error in setUserSetDefaultSelector(): pylon_camera_ is not ready!");
    return "pylon camera is not ready!";
  }

  return this->pylon_camera_->setUserSetDefaultSelector(set);
}

std::string PylonROS2CameraNode::setDeviceLinkThroughputLimitMode(const bool& turnOn)
{   
  std::lock_guard<std::recursive_mutex> lock(this->grab_mutex_);
  if (!this->pylon_camera_->isReady())
  {
    RCLCPP_WARN(LOGGER, "Error in setDeviceLinkThroughputLimitMode(): pylon_camera_ is not ready!");
    return "pylon camera is not ready!";
  }

  return this->pylon_camera_->setDeviceLinkThroughputLimitMode(turnOn);
}

std::string PylonROS2CameraNode::setDeviceLinkThroughputLimit(const int& limit)
{   
  std::lock_guard<std::recursive_mutex> lock(this->grab_mutex_);
  if (!this->pylon_camera_->isReady())
  {
    RCLCPP_WARN(LOGGER, "Error in setDeviceLinkThroughputLimit(): pylon_camera_ is not ready!");
    return "pylon camera is not ready!";
  }

  return this->pylon_camera_->setDeviceLinkThroughputLimit(limit);
}

std::string PylonROS2CameraNode::triggerDeviceReset()
{  
  std::lock_guard<std::recursive_mutex> lock(this->grab_mutex_);
  if (!this->pylon_camera_->isReady())
  {
    RCLCPP_WARN(LOGGER, "Error in triggerDeviceReset(): pylon_camera_ is not ready!");
    return "pylon camera is not ready!";
  }
  
  return this->pylon_camera_->triggerDeviceReset();
}

std::string PylonROS2CameraNode::setImageEncoding(const std::string& target_ros_encoding)
{  
  std::lock_guard<std::recursive_mutex> lock(this->grab_mutex_);
  if (!this->pylon_camera_->isReady())
  {
    RCLCPP_WARN(LOGGER, "Error in setImageEncoding(): pylon_camera_ is not ready!");
    return "pylon camera is not ready!";
  }
  
  return this->pylon_camera_->setImageEncoding(target_ros_encoding);
}

std::string PylonROS2CameraNode::setMaxTransferSize(const int& maxTransferSize)
{  
  std::lock_guard<std::recursive_mutex> lock(this->grab_mutex_);
  if (!this->pylon_camera_->isReady())
  {
    RCLCPP_WARN(LOGGER, "Error in setMaxTransferSize(): pylon_camera_ is not ready!");
    return "pylon camera is not ready!";
  }

  return this->pylon_camera_->setMaxTransferSize(maxTransferSize);
}

std::string PylonROS2CameraNode::setGammaSelector(const int& gammaSelector)
{  
  // gammaSelector 0 = User
  // gammaSelector 1 = sRGB
  std::lock_guard<std::recursive_mutex> lock(this->grab_mutex_);
  if (!this->pylon_camera_->isReady())
  {
    RCLCPP_WARN(LOGGER, "Error in setGammaSelector(): pylon_camera_ is not ready!");
    return "pylon camera is not ready!";
  }

  return this->pylon_camera_->setGammaSelector(gammaSelector);
}

std::string PylonROS2CameraNode::gammaEnable(const int& enable)
{  
  std::lock_guard<std::recursive_mutex> lock(this->grab_mutex_);
  if (!this->pylon_camera_->isReady())
  {
    RCLCPP_WARN(LOGGER, "Error in gammaEnable(): pylon_camera_ is not ready!");
    return "pylon camera is not ready!";
  }
  
  return this->pylon_camera_->gammaEnable(enable);
}

uint32_t PylonROS2CameraNode::getNumSubscribersRectImagePub() const
{
  return this->camera_info_manager_->isCalibrated() ? this->img_rect_pub_->getNumSubscribers() : 0;
}

void PylonROS2CameraNode::getMaxNumBufferCallback(const std::shared_ptr<GetIntegerSrv::Request> request,
                                                  std::shared_ptr<GetIntegerSrv::Response> response)
{
  (void)request;
  int value = this->pylon_camera_->getMaxNumBuffer();
  if (value == -1 ) 
  {
    response->success = false;
    response->message = "The connected Camera not supporting this feature";
  } 
  else if (value == -2)
  {
    response->success = false;
    response->message = "Error, Refer to the ROS console";
  }
  else 
  {
    response->success = true;
    response->value = value;
  }
}

void PylonROS2CameraNode::getStatisticTotalBufferCountCallback(const std::shared_ptr<GetIntegerSrv::Request> request,
                                                               std::shared_ptr<GetIntegerSrv::Response> response)
{
  (void)request;
  int value = this->pylon_camera_->getStatisticTotalBufferCount();
  if (value == -1 ) 
  {
    response->success = false;
    response->message = "The connected Camera not supporting this feature";
  } 
  else if (value == -2)
  {
    response->success = false;
    response->message = "Error, Refer to the ROS console";
  }
  else 
  {
    response->success = true;
    response->value = value;
  }
}

void PylonROS2CameraNode::getStatisticFailedBufferCountCallback(const std::shared_ptr<GetIntegerSrv::Request> request,
                                                                std::shared_ptr<GetIntegerSrv::Response> response)
{
  (void)request;
  int value = this->pylon_camera_->getStatisticFailedBufferCount();
  if (value == -1 ) 
  {
    response->success = false;
    response->message = "The connected Camera not supporting this feature";
  }
  else if (value == -2)
  {
    response->success = false;
    response->message = "Error, Refer to the ROS console";
  }
  else 
  {
    response->success = true;
    response->value = value;
  }
}

void PylonROS2CameraNode::getStatisticBufferUnderrunCountCallback(const std::shared_ptr<GetIntegerSrv::Request> request,
                                                                  std::shared_ptr<GetIntegerSrv::Response> response)
{
  (void)request;
  int value = this->pylon_camera_->getStatisticBufferUnderrunCount();
  if (value == -1 )
  {
    response->success = false;
    response->message = "The connected Camera not supporting this feature";
  }
  else if (value == -2)
  {
    response->success = false;
    response->message = "Error, Refer to the ROS console";
  }
  else
  {
    response->success = true;
    response->value = value;
  }
}

void PylonROS2CameraNode::getStatisticFailedPacketCountCallback(const std::shared_ptr<GetIntegerSrv::Request> request,
                                                                std::shared_ptr<GetIntegerSrv::Response> response)
{
  (void)request;
  int value = this->pylon_camera_->getStatisticFailedPacketCount();
  if (value == -1 )
  {
    response->success = false;
    response->message = "The connected Camera not supporting this feature";
  }
  else if (value == -2)
  {
    response->success = false;
    response->message = "Error, Refer to the ROS console";
  }
  else
  {
    response->success = true;
    response->value = value;
  }
}

void PylonROS2CameraNode::getStatisticResendRequestCountCallback(const std::shared_ptr<GetIntegerSrv::Request> request,
                                                                 std::shared_ptr<GetIntegerSrv::Response> response)
{
  (void)request;
  int value = this->pylon_camera_->getStatisticResendRequestCount();
  if (value == -1 )
  {
    response->success = false;
    response->message = "The connected Camera not supporting this feature";
  }
  else if (value == -2)
  {
    response->success = false;
    response->message = "Error, Refer to the ROS console";
  }
  else
  {
    response->success = true;
    response->value = value;
  }
}

void PylonROS2CameraNode::getStatisticMissedFrameCountCallback(const std::shared_ptr<GetIntegerSrv::Request> request,
                                                               std::shared_ptr<GetIntegerSrv::Response> response)
{
  (void)request;
  int value = this->pylon_camera_->getStatisticMissedFrameCount();
  if (value == -1 )
  {
    response->success = false;
    response->message = "The connected Camera not supporting this feature";
  }
  else if (value == -2)
  {
    response->success = false;
    response->message = "Error, Refer to the ROS console";
  }
  else
  {
    response->success = true;
    response->value = value;
  }
}

void PylonROS2CameraNode::getStatisticResynchronizationCountCallback(const std::shared_ptr<GetIntegerSrv::Request> request,
                                                                     std::shared_ptr<GetIntegerSrv::Response> response)
{
  (void)request;
  int value = this->pylon_camera_->getStatisticResynchronizationCount();
  if (value == -1 )
  {
    response->success = false;
    response->message = "The connected Camera not supporting this feature";
  }
  else if (value == -2)
  {
    response->success = false;
    response->message = "Error, Refer to the ROS console";
  }
  else
  {
    response->success = true;
    response->value = value;
  }
}

void PylonROS2CameraNode::getChunkModeActiveCallback(const std::shared_ptr<GetIntegerSrv::Request> request,
                                                     std::shared_ptr<GetIntegerSrv::Response> response)
{
  (void)request;
  int value = this->pylon_camera_->getChunkModeActive();
  if (value >= 0 &&  value <= 1)
  {
    response->success = true;
    response->value = value;
  }
  else if (value == -1)
  {
    response->success = false;
    response->message = "The connected Camera not supporting this feature";
  }
  else if (value == -2)
  {
    response->success = false;
    response->message = "Error, Refer to the ROS console";
  }
  else
  {
    response->success = false;
    response->message = "Unknown error";
  }
}

void PylonROS2CameraNode::getChunkSelectorCallback(const std::shared_ptr<GetIntegerSrv::Request> request,
                                                   std::shared_ptr<GetIntegerSrv::Response> response)
{
  (void)request;
  int value = pylon_camera_->getChunkSelector();
  if (value >= 1 &&  value <= 32)
  {
    response->success = true;
    response->value = value;
  }
  else if (value == -1) 
  {
    response->success = false;
    response->message = "The connected Camera not supporting this feature";
  }
  else if (value == -2)
  {
    response->success = false;
    response->message = "Error, Refer to the ROS console";
  }
  else 
  {
    response->success = false;
    response->message = "Unknown error";
  }
}

void PylonROS2CameraNode::getChunkEnableCallback(const std::shared_ptr<GetIntegerSrv::Request> request,
                                                 std::shared_ptr<GetIntegerSrv::Response> response)
{
  (void)request;
  int value = this->pylon_camera_->getChunkEnable();
  if (value >= 0 &&  value <= 1)
  {
    response->success = true;
    response->value = value;
  } 
  else if (value == -1)
  {
    response->success = false;
    response->message = "The connected Camera not supporting this feature";
  }
  else if (value == -2)
  {
    response->success = false;
    response->message = "Error, Refer to the ROS console";
  }
  else
  {
    response->success = false;
    response->message = "Unknown error";
  }
}

void PylonROS2CameraNode::getChunkTimestampCallback(const std::shared_ptr<GetIntegerSrv::Request> request,
                                                    std::shared_ptr<GetIntegerSrv::Response> response)
{
  (void)request;
  int value = this->pylon_camera_->getChunkTimestamp();
  if (value == -1)
  {
    response->success = false;
    response->message = "The connected Camera not supporting this feature";
  }
  else if (value == -2)
  {
    response->success = false;
    response->message = "Error, Refer to the ROS console";
  }
  else
  {
    response->success = true;
    response->value = value;
  }
}

void PylonROS2CameraNode::getChunkLineStatusAllCallback(const std::shared_ptr<GetIntegerSrv::Request> request,
                                                        std::shared_ptr<GetIntegerSrv::Response> response)
{
  (void)request;
  int value = this->pylon_camera_->getChunkLineStatusAll();
  if (value == -1)
  {
    response->success = false;
    response->message = "The connected Camera not supporting this feature";
  }
  else if (value == -2)
  {
    response->success = false;
    response->message = "Error, Refer to the ROS console";
  }
  else
  {
    response->success = true;
    response->value = value;
  }
}

void PylonROS2CameraNode::getChunkFramecounterCallback(const std::shared_ptr<GetIntegerSrv::Request> request,
                                                       std::shared_ptr<GetIntegerSrv::Response> response)
{
  (void)request;
  int value = this->pylon_camera_->getChunkFramecounter();
  if (value == -1) 
  {
    response->success = false;
    response->message = "The connected Camera not supporting this feature";
  } 
  else if (value == -2)
  {
    response->success = false;
    response->message = "Error, Refer to the ROS console";
  } 
  else 
  {
    response->success = true;
    response->value = value;
  }
}

void PylonROS2CameraNode::getChunkCounterValueCallback(const std::shared_ptr<GetIntegerSrv::Request> request,
                                                       std::shared_ptr<GetIntegerSrv::Response> response)
{
  (void)request;
  int value = this->pylon_camera_->getChunkCounterValue();
  if (value == -1 )
  {
    response->success = false;
    response->message = "The connected Camera not supporting this feature";
  }
  else if (value == -2)
  {
    response->success = false;
    response->message = "Error, Refer to the ROS console";
  } 
  else 
  {
    response->success = true;
    response->value = value;
  }
}

void PylonROS2CameraNode::getChunkExposureTimeCallback(const std::shared_ptr<GetFloatSrv::Request> request,
                                                       std::shared_ptr<GetFloatSrv::Response> response)
{
  (void)request;
  float value = this->pylon_camera_->getChunkExposureTime();
  if (value == -1.0)
  {
    response->success = false;
    response->message = "The connected Camera not supporting this feature";
  }
  else if (value == -2.0)
  {
    response->success = false;
    response->message = "Error, Refer to the ROS console";
  }
  else
  {
    response->success = true;
    response->value = value;
  }
}

void PylonROS2CameraNode::setBinningCallback(const std::shared_ptr<SetBinningSrv::Request> request,
                                             std::shared_ptr<SetBinningSrv::Response> response)
{
  size_t reached_binning_x, reached_binning_y;
  bool success_x = this->setBinningX(request->target_binning_x,
                                     reached_binning_x);
  bool success_y = this->setBinningY(request->target_binning_y,
                                     reached_binning_y);
  response->reached_binning_x = static_cast<uint32_t>(reached_binning_x);
  response->reached_binning_y = static_cast<uint32_t>(reached_binning_y);
  response->success = success_x && success_y;
}

void PylonROS2CameraNode::setBrightnessCallback(const std::shared_ptr<SetBrightnessSrv::Request> request,
                                                std::shared_ptr<SetBrightnessSrv::Response> response)
{
  response->success = this->setBrightness(request->target_brightness,
                                          response->reached_brightness,
                                          request->exposure_auto,
                                          request->gain_auto);
    if (request->brightness_continuous)
    {
      if (request->exposure_auto)
      {
        this->pylon_camera_->enableContinuousAutoExposure();
      }
      if (request->gain_auto)
      {
        this->pylon_camera_->enableContinuousAutoGain();
      }
    }
    response->reached_exposure_time = this->pylon_camera_->currentExposure();
    response->reached_gain_value = this->pylon_camera_->currentGain();
}

void PylonROS2CameraNode::setExposureCallback(const std::shared_ptr<SetExposureSrv::Request> request,
                                              std::shared_ptr<SetExposureSrv::Response> response)
{
  response->success = this->setExposure(request->target_exposure, response->reached_exposure);
}

void PylonROS2CameraNode::setGainCallback(const std::shared_ptr<SetGainSrv::Request> request,
                                          std::shared_ptr<SetGainSrv::Response> response)
{
  response->success = this->setGain(request->target_gain, response->reached_gain);
}

void PylonROS2CameraNode::setGammaCallback(const std::shared_ptr<SetGammaSrv::Request> request,
                                           std::shared_ptr<SetGammaSrv::Response> response)
{
  response->success = this->setGamma(request->target_gamma, response->reached_gamma);
}

void PylonROS2CameraNode::setROICallback(const std::shared_ptr<SetROISrv::Request> request,
                                         std::shared_ptr<SetROISrv::Response> response)
{
  response->success = this->setROI(request->target_roi, response->reached_roi);
}

void PylonROS2CameraNode::setSleepingCallback(const std::shared_ptr<SetSleepingSrv::Request> request,
                                              std::shared_ptr<SetSleepingSrv::Response> response)
{
  is_sleeping_ = request->set_sleeping;

  if (is_sleeping_)
  {
    RCLCPP_INFO(LOGGER, "Seting Pylon Camera Node to sleep...");
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Pylon Camera Node continues grabbing");
  }

  response->success = true;
}

void PylonROS2CameraNode::setWhiteBalanceCallback(const std::shared_ptr<SetWhiteBalanceSrv::Request> request,
                                                  std::shared_ptr<SetWhiteBalanceSrv::Response> response)
{
  try 
  {
    response->message = this->pylon_camera_->setWhiteBalance(request->balance_ratio_red, request->balance_ratio_green, request->balance_ratio_blue);
    if (response->message == "done")
    {
      response->success = true;
    } 
    else
    {
      response->success = false;
    }
  } catch (...)
  {
    response->success = false;
  }
}

void PylonROS2CameraNode::setActionTriggerConfigurationCallback(const std::shared_ptr<SetActionTriggerConfiguration::Request> request,
                                                                std::shared_ptr<SetActionTriggerConfiguration::Response> response)
{
  response->message = this->pylon_camera_->setActionTriggerConfiguration(request->action_device_key, request->action_group_key, request->action_group_mask,
                                                                         request->registration_mode, request->cleanup);

  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else
  {
    response->success = false;
  }
}

void PylonROS2CameraNode::issueActionCommandCallback(const std::shared_ptr<IssueActionCommand::Request> request,
                                                     std::shared_ptr<IssueActionCommand::Response> response)
{
  response->message = this->pylon_camera_->issueActionCommand(request->device_key, request->group_key, request->group_mask, request->broadcast_address);

  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else
  {
    response->success = false;
  }
}

void PylonROS2CameraNode::issueScheduledActionCommandCallback(const std::shared_ptr<IssueScheduledActionCommand::Request> request,
                                                              std::shared_ptr<IssueScheduledActionCommand::Response> response)
{
  response->message = this->pylon_camera_->issueScheduledActionCommand(request->device_key, request->group_key, request->group_mask, request->action_time_ns_from_current_timestamp, request->broadcast_address);

  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else
  {
    response->success = false;
  }
}

void PylonROS2CameraNode::setOffsetXCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                             std::shared_ptr<SetIntegerSrv::Response> response)
{
  response->message = this->setOffsetXY(request->value, true);
  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else 
  {
    response->success = false;
    if (response->message == "Node is not writable.")
    {
      response->message = "Using this feature requires stopping image grabbing";
    }
  }
}

void PylonROS2CameraNode::setOffsetYCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                             std::shared_ptr<SetIntegerSrv::Response> response)
{
  response->message = this->setOffsetXY(request->value, true);
  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else 
  {
    response->success = false;
    if (response->message == "Node is not writable.")
    {
      response->message = "Using this feature requires stopping image grabbing";
    }
  }
}

void PylonROS2CameraNode::setBlackLevelCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                                std::shared_ptr<SetIntegerSrv::Response> response)
{
  response->message = this->setBlackLevel(request->value);
  if (response->message == "done")
  {
    response->success = true;
  }
  else 
  {
    response->success = false;
    if (response->message == "Node is not writable.")
    {
      response->message = "Using this feature requires stopping image grabbing";
    }
  }
}

void PylonROS2CameraNode::setDemosaicingModeCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                                     std::shared_ptr<SetIntegerSrv::Response> response)
{
  response->message = this->setDemosaicingMode(request->value);
  if (response->message == "done")
  {
    response->success = true;
  }
  else 
  {
    response->success = false;
    if (response->message == "Node is not writable.")
    {
      response->message = "Using this feature requires stopping image grabbing";
    }
    else if ((response->message.find("EnumEntry") != std::string::npos) != 0)
    {
      response->message = "The passed demosaicing mode number is not supported by the connected camera";
    }
  }
}

void PylonROS2CameraNode::setLightSourcePresetCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                                       std::shared_ptr<SetIntegerSrv::Response> response)
{
  response->message = this->setLightSourcePreset(request->value);
  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else 
  {
    response->success = false;
    if (response->message == "Node is not writable.")
    {
      response->message = "Using this feature requires stopping image grabbing";
    }
    else if ((response->message.find("EnumEntry") != std::string::npos) != 0)
    {
      response->message = "The passed light source preset number is not supported by the connected camera";
    }
  }
}

void PylonROS2CameraNode::setWhiteBalanceAutoCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                                      std::shared_ptr<SetIntegerSrv::Response> response)
{
  response->message = this->setWhiteBalanceAuto(request->value);
  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else 
  {
    response->success = false;
    if (response->message == "Node is not writable.")
    {
      response->message = "Using this feature requires stopping image grabbing";
    }
    else if ((response->message.find("EnumEntry") != std::string::npos) != 0)
    {
      response->message = "The passed balance white auto number is not supported by the connected camera";
    }
  }
}

void PylonROS2CameraNode::setSensorReadoutModeCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                                       std::shared_ptr<SetIntegerSrv::Response> response)
{
  response->message = this->setSensorReadoutMode(request->value);
  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else 
  {
    response->success = false;
    if (response->message == "Node is not writable.")
    {
      response->message = "Using this feature requires stopping image grabbing";
    }
    else if ((response->message.find("EnumEntry") != std::string::npos) != 0)
    {
      response->message = "The passed sensor readout mode number is not supported by the connected camera";
    }
  }
}

void PylonROS2CameraNode::setAcquisitionFrameCountCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                                           std::shared_ptr<SetIntegerSrv::Response> response)
{
  response->message = this->setAcquisitionFrameCount(request->value);
  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else 
  {
    response->success = false;
    if (response->message == "Node is not writable.")
    {
      response->message = "Using this feature requires stopping image grabbing";
    }
  }
}

void PylonROS2CameraNode::setTriggerSelectorCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                                     std::shared_ptr<SetIntegerSrv::Response> response)
{
  response->message = this->setTriggerSelector(request->value);
  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else 
  {
    response->success = false;
    if (response->message == "Node is not writable.")
    {
      response->message = "Using this feature requires stopping image grabbing";
    }
    else if ((response->message.find("EnumEntry") != std::string::npos) != 0)
    {
      response->message = "The passed trigger selector number is not supported by the connected camera";
    }
  }
}

void PylonROS2CameraNode::setTriggerSourceCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                                   std::shared_ptr<SetIntegerSrv::Response> response)
{
  response->message = this->setTriggerSource(request->value);
  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else 
  {
    response->success = false;
    if (response->message == "Node is not writable.")
    {
      response->message = "Using this feature requires stopping image grabbing";
    }
    else if ((response->message.find("EnumEntry") != std::string::npos) != 0)
    {
      response->message = "The passed trigger source number is not supported by the connected camera";
    }
  }
}

void PylonROS2CameraNode::setTriggerActivationCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                                       std::shared_ptr<SetIntegerSrv::Response> response)
{
  response->message = this->setTriggerActivation(request->value);
  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else 
  {
    response->success = false;
    if (response->message == "Node is not writable.")
    {
      response->message = "Using this feature requires stopping image grabbing";
    }
  }
}

void PylonROS2CameraNode::setLineSelectorCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                                  std::shared_ptr<SetIntegerSrv::Response> response)
{
  response->message = this->setLineSelector(request->value);
  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else 
  {
    response->success = false;
    if (response->message == "Node is not writable.")
    {
      response->message = "Using this feature requires stopping image grabbing";
    }
  }
}

void PylonROS2CameraNode::setLineModeCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                              std::shared_ptr<SetIntegerSrv::Response> response)
{
  response->message = this->setLineMode(request->value);
  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else 
  {
    response->success = false;
    if (response->message == "Node is not writable.")
    {
      response->message = "Using this feature requires stopping image grabbing";
    }
  }
}

void PylonROS2CameraNode::setLineSourceCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                                std::shared_ptr<SetIntegerSrv::Response> response)
{
  response->message = this->setLineSource(request->value);
  if ((response->message.find("done") != std::string::npos) != 0)
  {
      response->success = true;
  }
  else 
  {
    response->success = false;
    if (response->message == "Node is not writable.")
    {
      response->message = "Using this feature requires stopping image grabbing";
    }
  }
}

void PylonROS2CameraNode::setUserSetSelectorCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                                     std::shared_ptr<SetIntegerSrv::Response> response)
{
  response->message = this->setUserSetSelector(request->value);
  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else 
  {
    response->success = false;
    if (response->message == "Node is not writable.")
    {
      response->message = "Using this feature requires stopping image grabbing";
    }
    else if ((response->message.find("EnumEntry") != std::string::npos) != 0)
    {
      response->message = "The passed user set number is not supported by the connected camera";
    }
  }
}

void PylonROS2CameraNode::setUserSetDefaultSelectorCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                                            std::shared_ptr<SetIntegerSrv::Response> response)
{
  response->message = this->setUserSetDefaultSelector(request->value);
  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else 
  {
    response->success = false;
    if (response->message == "Node is not writable.")
    {
      response->message = "Using this feature requires stopping image grabbing";
    }
    else if ((response->message.find("EnumEntry") != std::string::npos) != 0)
    {
      response->message = "The passed default user set number is not supported by the connected camera";
    }
  }
}

void PylonROS2CameraNode::setDeviceLinkThroughputLimitCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                                               std::shared_ptr<SetIntegerSrv::Response> response)
{
  response->message = this->setDeviceLinkThroughputLimit(request->value);
  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else 
  {
    response->success = false;
    if (response->message == "Node is not writable.")
    {
      response->message = "Using this feature requires stopping image grabbing";
    }
  }
}

void PylonROS2CameraNode::setMaxTransferSizeCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                                     std::shared_ptr<SetIntegerSrv::Response> response)
{
  response->message = this->setMaxTransferSize(request->value);
  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else 
  {
    response->success = false;
    if (response->message == "Node is not writable")
    {
      response->message = "Using this feature requires stopping image grabbing";
    }
  }
}

void PylonROS2CameraNode::setGammaSelectorCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                                   std::shared_ptr<SetIntegerSrv::Response> response)
{
  response->message = this->setGammaSelector(request->value);
  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else 
  {
    response->success = false;
    if (response->message == "Node is not writable")
    {
      response->message = "Using this feature requires stopping image grabbing";
    }
  }
}

void PylonROS2CameraNode::setGrabTimeoutCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                                 std::shared_ptr<SetIntegerSrv::Response> response)
{
  this->grabbingStopping();
  try 
  {
    this->pylon_camera_parameter_set_.grab_timeout_ = request->value;
    response->success = true;
  } catch (...)
  {
    response->success = false;
  }
  this->grabbingStarting(); // start grabbing is required to set the new trigger timeout
}

void PylonROS2CameraNode::setTriggerTimeoutCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                                    std::shared_ptr<SetIntegerSrv::Response> response)
{
  this->grabbingStopping();
  try 
  {
    this->pylon_camera_parameter_set_.trigger_timeout_ = request->value;
    response->success = true;
  } catch (...)
  {
    response->success = false;
  }
  this->grabbingStarting(); // start grabbing is required to set the new trigger timeout
}

void PylonROS2CameraNode::setGrabbingStrategyCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                                      std::shared_ptr<SetIntegerSrv::Response> response)
{
  // set 0 = GrabStrategy_OneByOne
  // set 1 = GrabStrategy_LatestImageOnly
  // set 2 = GrabStrategy_LatestImages

  if(request->value >= 0 && request->value <= 2) 
  {
    this->grabbingStopping();
    response->success = this->pylon_camera_->setGrabbingStrategy(request->value);
    if(response->success)
    {
      this->pylon_camera_parameter_set_.grab_strategy_ = request->value;
    }
    this->grabbingStarting();

  } 
  else 
  {
    response->success = false;
    response->message = "Unknown grabbing strategy";
  }
}

void PylonROS2CameraNode::setOutputQueueSizeCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                                     std::shared_ptr<SetIntegerSrv::Response> response)
{
  this->grabbingStopping();
  response->message = this->pylon_camera_->setOutputQueueSize(request->value);
  this->grabbingStarting();

  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  } 
  else 
  {
    response->success = false;
  }
}

void PylonROS2CameraNode::setMaxNumBufferCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                                  std::shared_ptr<SetIntegerSrv::Response> response)
{
  response->message = this->pylon_camera_->setMaxNumBuffer(request->value);

  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  } 
  else
  {
    response->success = false;
  }
}

void PylonROS2CameraNode::setChunkSelectorCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                                   std::shared_ptr<SetIntegerSrv::Response> response)
{
  response->message = this->pylon_camera_->setChunkSelector(request->value);
  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else
  {
    response->success = false;
  }
}

void PylonROS2CameraNode::setTimerSelectorCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                                   std::shared_ptr<SetIntegerSrv::Response> response)
{
  response->message = this->pylon_camera_->setTimerSelector(request->value);

  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else
  {
    response->success = false;
  }
}

void PylonROS2CameraNode::setTimerTriggerSourceCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                                        std::shared_ptr<SetIntegerSrv::Response> response)
{
  response->message = this->pylon_camera_->setTimerTriggerSource(request->value);

  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else
  {
    response->success = false;
  }
}

void PylonROS2CameraNode::setPTPPriorityCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                                 std::shared_ptr<SetIntegerSrv::Response> response)
{
  response->message = this->pylon_camera_->setPTPPriority(request->value);
  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else
  {
    response->success = false;
  }
}

void PylonROS2CameraNode::setPTPProfileCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                                std::shared_ptr<SetIntegerSrv::Response> response)
{
  response->message = this->pylon_camera_->setPTPProfile(request->value);
  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else
  {
    response->success = false;
  }
}
  
void PylonROS2CameraNode::setPTPNetworkModeCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                                    std::shared_ptr<SetIntegerSrv::Response> response)
{
  response->message = this->pylon_camera_->setPTPNetworkMode(request->value);
  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else
  {
    response->success = false;
  }
}

void PylonROS2CameraNode::setPTPUCPortAddressIndexCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                                           std::shared_ptr<SetIntegerSrv::Response> response)
{
  response->message = this->pylon_camera_->setPTPUCPortAddressIndex(request->value);
  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else
  {
    response->success = false;
  }
}

void PylonROS2CameraNode::setPTPUCPortAddressCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                                      std::shared_ptr<SetIntegerSrv::Response> response)
{
  response->message = this->pylon_camera_->setPTPUCPortAddress(request->value);
  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else
  {
    response->success = false;
  }
}

void PylonROS2CameraNode::setSyncFreeRunTimerStartTimeLowCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                                                  std::shared_ptr<SetIntegerSrv::Response> response)
{
  response->message = this->pylon_camera_->setSyncFreeRunTimerStartTimeLow(request->value);
  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else
  {
    response->success = false;
  }
}

void PylonROS2CameraNode::setSyncFreeRunTimerStartTimeHighCallback(const std::shared_ptr<SetIntegerSrv::Request> request,
                                                                   std::shared_ptr<SetIntegerSrv::Response> response)
{
  response->message = this->pylon_camera_->setSyncFreeRunTimerStartTimeHigh(request->value);
  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else
  {
    response->success = false;
  }
}

void PylonROS2CameraNode::setNoiseReductionCallback(const std::shared_ptr<SetFloatSrv::Request> request,
                                                    std::shared_ptr<SetFloatSrv::Response> response)
{
  response->message = this->setNoiseReduction(request->value);
  if (response->message == "done")
  {
    response->success = true;
  }
  else 
  {
    response->success = false;
    if (response->message == "Node is not writable.")
    {
      response->message = "Using this feature requires stopping image grabbing";
    }
  }
}

void PylonROS2CameraNode::setSharpnessEnhancementCallback(const std::shared_ptr<SetFloatSrv::Request> request,
                                                          std::shared_ptr<SetFloatSrv::Response> response)
{
  response->message = this->setSharpnessEnhancement(request->value);
  if (response->message == "done")
  {
    response->success = true;
  }
  else 
  {
    response->success = false;
    if (response->message == "Node is not writable.")
    {
      response->message = "Using this feature requires stopping image grabbing";
    }
  }
}

void PylonROS2CameraNode::setTriggerDelayCallback(const std::shared_ptr<SetFloatSrv::Request> request,
                                                  std::shared_ptr<SetFloatSrv::Response> response)
{
  response->message = this->setTriggerDelay(request->value);
  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else 
  {
    response->success = false;
    if (response->message == "Node is not writable.")
    {
      response->message = "Using this feature requires stopping image grabbing";
    }
  }
}

void PylonROS2CameraNode::setLineDebouncerTimeCallback(const std::shared_ptr<SetFloatSrv::Request> request,
                                                       std::shared_ptr<SetFloatSrv::Response> response)
{
  response->message = this->setLineDebouncerTime(request->value);
  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else 
  {
    response->success = false;
    if (response->message == "Node is not writable.")
    {
      response->message = "Using this feature requires stopping image grabbing";
    }
  }
}

void PylonROS2CameraNode::setChunkExposureTimeCallback(const std::shared_ptr<SetFloatSrv::Request> request,
                                                       std::shared_ptr<SetFloatSrv::Response> response)
{
  response->message = this->pylon_camera_->setChunkExposureTime(request->value);
  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  } 
  else
  {
    response->success = false;
  }
}

void PylonROS2CameraNode::setTimerDurationCallback(const std::shared_ptr<SetFloatSrv::Request> request,
                                                   std::shared_ptr<SetFloatSrv::Response> response)
{
  response->message = this->pylon_camera_->setTimerDuration(request->value);

  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else
  {
    response->success = false;
  }
}
  
void PylonROS2CameraNode::setPeriodicSignalPeriodCallback(const std::shared_ptr<SetFloatSrv::Request> request,
                                                          std::shared_ptr<SetFloatSrv::Response> response)
{
  response->message = this->pylon_camera_->setPeriodicSignalPeriod(request->value);

  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else
  {
    response->success = false;
  }
}

void PylonROS2CameraNode::setPeriodicSignalDelayCallback(const std::shared_ptr<SetFloatSrv::Request> request,
                                                         std::shared_ptr<SetFloatSrv::Response> response)
{
  response->message = this->pylon_camera_->setPeriodicSignalDelay(request->value);

  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else
  {
    response->success = false;
  }
}

void PylonROS2CameraNode::setSyncFreeRunTimerTriggerRateAbsCallback(const std::shared_ptr<SetFloatSrv::Request> request,
                                                                    std::shared_ptr<SetFloatSrv::Response> response)
{
  response->message = this->pylon_camera_->setSyncFreeRunTimerTriggerRateAbs(request->value);

  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else
  {
    response->success = false;
  }
}

void PylonROS2CameraNode::setImageEncodingCallback(const std::shared_ptr<SetStringSrv::Request> request,
                                                   std::shared_ptr<SetStringSrv::Response> response)
{
  this->grabbingStopping(); // Stop grabbing for better user experience
  response->message = this->setImageEncoding(request->value);
  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
    pylon_camera_parameter_set_.setimageEncodingParam(*this,request->value);
  }
  else 
  {
    response->success = false;
    if (response->message == "Node is not writable")
    {
      response->message = "Using this feature requires stopping image grabbing";
    }
  }
  this->grabbingStarting(); // Start grabbing for better usser experience
}

void PylonROS2CameraNode::setReverseXCallback(const std::shared_ptr<SetBoolSrv::Request> request,
                                              std::shared_ptr<SetBoolSrv::Response> response)
{
  response->message = this->reverseXY(request->data, true);
  if (response->message == "done")
  {
    response->success = true;
  }
  else 
  {
    response->success = false;
    if (response->message == "Node is not writable.")
    {
      response->message = "Using this feature requires stopping image grabbing";
    }
  }
}

void PylonROS2CameraNode::setReverseYCallback(const std::shared_ptr<SetBoolSrv::Request> request,
                                              std::shared_ptr<SetBoolSrv::Response> response)
{
  response->message = this->reverseXY(request->data, false);
  if (response->message == "done")
  {
    response->success = true;
  }
  else 
  {
    response->success = false;
    if (response->message == "Node is not writable.")
    {
      response->message = "Using this feature requires stopping image grabbing";
    }
  }
}

void PylonROS2CameraNode::setPGIModeCallback(const std::shared_ptr<SetBoolSrv::Request> request,
                                             std::shared_ptr<SetBoolSrv::Response> response)
{
  response->message = this->setPGIMode(request->data);
  if (response->message == "done")
  {
    response->success = true;
  }
  else 
  {
    response->success = false;
    if (response->message == "Node is not writable.")
    {
      response->message = "Using this feature requires stopping image grabbing";
    }
  }
}

void PylonROS2CameraNode::setTriggerModeCallback(const std::shared_ptr<SetBoolSrv::Request> request,
                                                 std::shared_ptr<SetBoolSrv::Response> response)
{
  response->message = this->setTriggerMode(request->data);
  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else 
  {
    response->success = false;
    if (response->message == "Node is not writable.")
    {
      response->message = "Using this feature requires stopping image grabbing";
    }
    else if ((response->message.find("EnumEntry") != std::string::npos) != 0)
    {
      response->message = "The passed trigger mode number is not supported by the connected camera";
    }
  }
}

void PylonROS2CameraNode::setLineInverterCallback(const std::shared_ptr<SetBoolSrv::Request> request,
                                                  std::shared_ptr<SetBoolSrv::Response> response)
{
  response->message = this->setLineInverter(request->data);
  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else 
  {
    response->success = false;
    if (response->message == "Node is not writable.")
    {
      response->message = "Using this feature requires stopping image grabbing";
    }
  }
}

void PylonROS2CameraNode::setDeviceLinkThroughputLimitModeCallback(const std::shared_ptr<SetBoolSrv::Request> request,
                                                                   std::shared_ptr<SetBoolSrv::Response> response)
{
  response->message = this->setDeviceLinkThroughputLimitMode(request->data);
  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else 
  {
    response->success = false;
    if (response->message == "Node is not writable.")
    {
      response->message = "Using this feature requires stopping image grabbing";
    }
  }
}

void PylonROS2CameraNode::setGammaEnableCallback(const std::shared_ptr<SetBoolSrv::Request> request,
                                                 std::shared_ptr<SetBoolSrv::Response> response)
{
  response->message = this->gammaEnable(request->data);
  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else 
  {
    response->success = false;
    if (response->message == "Node is not writable")
    {
      response->message = "Using this feature requires stopping image grabbing";
    }
  }
}

void PylonROS2CameraNode::setChunkModeActiveCallback(const std::shared_ptr<SetBoolSrv::Request> request,
                                                     std::shared_ptr<SetBoolSrv::Response> response)
{
  response->message = this->pylon_camera_->setChunkModeActive(request->data);
  if (response->message == "done")
  {
      response->success = true;
  }
  else if (response->message == "Node is not writable.")
  {
    response->message = "Using this feature requires stopping image grabbing";
  } 
  else
  {
    response->success = false;
  }
}

void PylonROS2CameraNode::setChunkEnableCallback(const std::shared_ptr<SetBoolSrv::Request> request,
                                                 std::shared_ptr<SetBoolSrv::Response> response)
{
  response->message = this->pylon_camera_->setChunkEnable(request->data);
  if (response->message == "done")
  {
      response->success = true;
  }
  else if (response->message == "Node is not writable.")
  {
    response->message = "Using this feature requires stopping image grabbing";
  } 
  else
  {
    response->success = false;
  }
}

void PylonROS2CameraNode::setUserOutputCallback(const int output_id,
                                                const std::shared_ptr<SetBoolSrv::Request> request,
                                                std::shared_ptr<SetBoolSrv::Response> response)
{
  response->success = this->pylon_camera_->setUserOutput(output_id, request->data);
}

void PylonROS2CameraNode::setAutoflashCallback(const int output_id,
                                               const std::shared_ptr<SetBoolSrv::Request> request,
                                               std::shared_ptr<SetBoolSrv::Response> response)
{
  RCLCPP_INFO(LOGGER, "AUtoFlashCB: %i -> %i", output_id, request->data);
  std::map<int, bool> auto_flashs;
  auto_flashs[output_id] = request->data;
  this->pylon_camera_->setAutoflash(auto_flashs);
  response->success = true;
}

void PylonROS2CameraNode::enablePTPManagementProtocolCallback(const std::shared_ptr<SetBoolSrv::Request> request,
                                                              std::shared_ptr<SetBoolSrv::Response> response)
{
  response->message = this->pylon_camera_->enablePTPManagementProtocol(request->data);
  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else
  {
    response->success = false;
  }
}

void PylonROS2CameraNode::enablePTPTwoStepOperationCallback(const std::shared_ptr<SetBoolSrv::Request> request,
                                                            std::shared_ptr<SetBoolSrv::Response> response)
{
  response->message = this->pylon_camera_->enablePTPTwoStepOperation(request->data);
  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else
  {
    response->success = false;
  }
}

void PylonROS2CameraNode::enablePTPCallback(const std::shared_ptr<SetBoolSrv::Request> request,
                                            std::shared_ptr<SetBoolSrv::Response> response)
{
  response->message = this->pylon_camera_->enablePTP(request->data);
  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else
  {
    response->success = false;
  }
}

void PylonROS2CameraNode::enableSyncFreeRunTimerCallback(const std::shared_ptr<SetBoolSrv::Request> request,
                                                         std::shared_ptr<SetBoolSrv::Response> response)
{
  response->message = this->pylon_camera_->enableSyncFreeRunTimer(request->data);
  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else
  {
    response->success = false;
  }
}

void PylonROS2CameraNode::executeSoftwareTriggerCallback(const std::shared_ptr<TriggerSrv::Request> request,
                                                         std::shared_ptr<TriggerSrv::Response> response)
{
  (void)request;
  response->message = this->executeSoftwareTrigger();
  if ((response->message.find("done") != std::string::npos) != 0)
  {
      response->success = true;
  }
  else 
  {
    response->success = false;
    if (response->message == "Node is not writable.")
    {
      response->message = "Using this feature requires stopping image grabbing";
    }
  }
}

void PylonROS2CameraNode::saveUserSetCallback(const std::shared_ptr<TriggerSrv::Request> request,
                                              std::shared_ptr<TriggerSrv::Response> response)
{
  (void)request;
  response->message = this->saveUserSet();
  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else 
  {
    response->success = false;
    if (response->message == "Node is not writable.")
    {
      response->message = "Using this feature requires stopping image grabbing";
    }
  }
}

void PylonROS2CameraNode::loadUserSetCallback(const std::shared_ptr<TriggerSrv::Request> request,
                                              std::shared_ptr<TriggerSrv::Response> response)
{
  (void)request;
  response->message = this->loadUserSet();
  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else 
  {
    response->success = false;
    if (response->message == "Node is not writable.")
    {
      response->message = "Using this feature requires stopping image grabbing";
    }
  }
}

void PylonROS2CameraNode::triggerDeviceResetCallback(const std::shared_ptr<TriggerSrv::Request> request,
                                                     std::shared_ptr<TriggerSrv::Response> response)
{
  (void)request;
  response->message = this->triggerDeviceReset();
  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else 
  {
    response->success = false;
  }
}

void PylonROS2CameraNode::startGrabbingCallback(const std::shared_ptr<TriggerSrv::Request> request,
                                                std::shared_ptr<TriggerSrv::Response> response)
{
  (void)request;
  response->message = this->grabbingStarting();
  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else 
  {
    response->success = false;
  }
}

void PylonROS2CameraNode::stopGrabbingCallback(const std::shared_ptr<TriggerSrv::Request> request,
                                               std::shared_ptr<TriggerSrv::Response> response)
{
  (void)request;
  response->message = this->grabbingStopping();
  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else 
  {
    response->success = false;
  }
}

void PylonROS2CameraNode::updateSyncFreeRunTimerCallback(const std::shared_ptr<TriggerSrv::Request> request,
                                                         std::shared_ptr<TriggerSrv::Response> response)
{
  (void)request;
  response->message = this->pylon_camera_->updateSyncFreeRunTimer();
  if ((response->message.find("done") != std::string::npos) != 0)
  {
    response->success = true;
  }
  else 
  {
    response->success = false;
  }
}

rclcpp_action::GoalResponse PylonROS2CameraNode::handleGrabRawImagesActionGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const GrabImagesAction::Goal> goal)
{
  RCLCPP_DEBUG(LOGGER, "PylonROS2CameraNode::handleGrabRawImagesActionGoal -> Received goal request");
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PylonROS2CameraNode::handleGrabRawImagesActionGoalCancel(const std::shared_ptr<GrabImagesGoalHandle> goal_handle)
{
  RCLCPP_DEBUG(this->get_logger(), "PylonROS2CameraNode::handleGrabRawImagesActionGoalCancel -> Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PylonROS2CameraNode::handleGrabRawImagesActionGoalAccepted(const std::shared_ptr<GrabImagesGoalHandle> goal_handle)
{
  RCLCPP_DEBUG(LOGGER, "PylonROS2CameraNode::handleGrabRawImagesActionGoalAccepted -> Goal has been accepted, starting the thread");

  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&PylonROS2CameraNode::executeGrabRawImagesAction, this, _1), goal_handle}.detach();
}

void PylonROS2CameraNode::executeGrabRawImagesAction(const std::shared_ptr<GrabImagesGoalHandle> goal_handle)
{
  auto result = this->grabRawImages(goal_handle);
  goal_handle->succeed(result);
}

rclcpp_action::GoalResponse PylonROS2CameraNode::handleGrabRectImagesActionGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const GrabImagesAction::Goal> goal)
{
  RCLCPP_DEBUG(LOGGER, "PylonROS2CameraNode::handleGrabRectImagesActionGoal -> Received goal request");
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PylonROS2CameraNode::handleGrabRectImagesActionGoalCancel(const std::shared_ptr<GrabImagesGoalHandle> goal_handle)
{
  RCLCPP_DEBUG(this->get_logger(), "PylonROS2CameraNode::handleGrabRectImagesActionGoalCancel -> Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PylonROS2CameraNode::handleGrabRectImagesActionGoalAccepted(const std::shared_ptr<GrabImagesGoalHandle> goal_handle)
{
  RCLCPP_DEBUG(this->get_logger(), "PylonROS2CameraNode::handleGrabRectImagesActionGoalAccepted -> Goal has been accepted, starting the thread");

  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&PylonROS2CameraNode::executeGrabRectImagesAction, this, _1), goal_handle}.detach();
}

void PylonROS2CameraNode::executeGrabRectImagesAction(const std::shared_ptr<GrabImagesGoalHandle> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<GrabImagesAction::Result>();
  auto feedback = std::make_shared<GrabImagesAction::Feedback>();

  if (!this->camera_info_manager_->isCalibrated())
  {
    result->success = false;
    goal_handle->succeed(result);
    return;
  }
  else
  {
    result = this->grabRawImages(goal_handle);
    if (!result->success)
    {
      goal_handle->succeed(result);
      return;
    }

    for ( std::size_t i = 0; i < result->images.size(); ++i)
    {
      cv_bridge::CvImagePtr cv_img_raw = cv_bridge::toCvCopy(result->images[i],
                                                             result->images[i].encoding);
      this->pinhole_model_->fromCameraInfo(this->camera_info_manager_->getCameraInfo());
      cv_bridge::CvImage cv_bridge_img_rect;
      cv_bridge_img_rect.header = result->images[i].header;
      cv_bridge_img_rect.encoding = result->images[i].encoding;
      this->pinhole_model_->rectifyImage(cv_img_raw->image, cv_bridge_img_rect.image);
      cv_bridge_img_rect.toImageMsg(result->images[i]);
    }
    goal_handle->succeed(result);
  }
}

void PylonROS2CameraNode::createDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (this->pylon_camera_parameter_set_.deviceUserID().empty())
  {
      return;
  }

  if (this->pylon_camera_)
  {
      this->diagnostics_updater_.setHardwareID(this->pylon_camera_->deviceUserID());
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Device is connected");
  }
  else
  {
      this->diagnostics_updater_.setHardwareID(this->pylon_camera_parameter_set_.deviceUserID());
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "No camera connected");
      this->cm_status_.status_id = pylon_ros2_camera_interfaces::msg::ComponentStatus::ERROR;
      this->cm_status_.status_msg = "No camera connected";
      if (this->pylon_camera_parameter_set_.enable_status_publisher_)
      {
          this->component_status_pub_->publish(this->cm_status_);
      }
  }
}

void PylonROS2CameraNode::createCameraInfoDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (this->camera_info_manager_->isCalibrated())
  {
    stat.summaryf(diagnostic_msgs::msg::DiagnosticStatus::OK, "Intrinsic calibration found");
  }
  else
  {
    stat.summaryf(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "No intrinsic calibration found");
  }
}

void PylonROS2CameraNode::diagnosticsTimerCallback()
{
  this->diagnostics_updater_.force_update();
}

bool PylonROS2CameraNode::serviceExists(const std::string& service_name)
{
  std::map<std::string, std::vector<std::string>> results = rclcpp::Node::get_service_names_and_types();
  std::map<std::string, std::vector<std::string>>::iterator it;
  for (it = results.begin(); it != results.end(); it++)
  {
    //std::cout << it->first << " vs " << service_name << std::endl;
    if (it->first == service_name)
    {
      return true;
    }
  }

  return false;
}

void PylonROS2CameraNode::setupSamplingIndices(std::vector<std::size_t>& indices,
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
  this->sampling_indices_.push_back(0.5 * rows * cols);
  this->genSamplingIndicesRec(indices,
                              min_window_height,
                              start_pt,
                              end_pt);
  std::sort(indices.begin(), indices.end());
}

void PylonROS2CameraNode::genSamplingIndicesRec(std::vector<std::size_t>& indices,
                                                const std::size_t& min_window_height,
                                                const cv::Point2i& s,   // start
                                                const cv::Point2i& e)   // end
{
  if (static_cast<std::size_t>(std::abs(e.y - s.y)) <= min_window_height)
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
  indices.push_back(b.y * this->pylon_camera_->imageCols() + b.x);
  indices.push_back(c.y * this->pylon_camera_->imageCols() + c.x);
  indices.push_back(d.y * this->pylon_camera_->imageCols() + d.x);
  indices.push_back(f.y * this->pylon_camera_->imageCols() + f.x);
  this->genSamplingIndicesRec(indices, min_window_height, s, a);
  this->genSamplingIndicesRec(indices, min_window_height, a, e);
  this->genSamplingIndicesRec(indices, min_window_height, cv::Point2i(s.x, a.y), cv::Point2i(a.x, e.y));
  this->genSamplingIndicesRec(indices, min_window_height, cv::Point2i(a.x, s.y), cv::Point2i(e.x, a.y));
}

float PylonROS2CameraNode::calcCurrentBrightness()
{
  std::lock_guard<std::recursive_mutex> lock(this->grab_mutex_);
  if (this->img_raw_msg_.data.empty())
  {
    return 0.0;
  }
  float sum = 0.0;
  if (sensor_msgs::image_encodings::isMono(this->img_raw_msg_.encoding))
  {
      // The mean brightness is calculated using a subset of all pixels
      for (const std::size_t& idx : this->sampling_indices_)
      {
        sum += this->img_raw_msg_.data.at(idx);
      }
      if (sum > 0.0)
      {
        sum /= static_cast<float>(this->sampling_indices_.size());
      }
  }
  else
  {
    // The mean brightness is calculated using all pixels and all channels
    sum = std::accumulate(this->img_raw_msg_.data.begin(), this->img_raw_msg_.data.end(), 0);
    if (sum > 0.0)
    {
      sum /= static_cast<float>(this->img_raw_msg_.data.size());
    }
  }

  return sum;
}

void PylonROS2CameraNode::setupInitialCameraInfo(sensor_msgs::msg::CameraInfo& cam_info_msg)
{
  std_msgs::msg::Header header;
  header.frame_id = this->pylon_camera_parameter_set_.cameraFrame();
  header.stamp = rclcpp::Node::now();

  // https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/CameraInfo.msg

  // If the camera is uncalibrated, the matrices D, K, R, P should be left
  // zeroed out. In particular, clients may assume that K[0] == 0.0
  // indicates an uncalibrated camera.
  cam_info_msg.header = header;

  // The image dimensions with which the camera was calibrated. Normally
  // this will be the full camera resolution in pixels.
  cam_info_msg.height = this->pylon_camera_->imageRows();
  cam_info_msg.width = this->pylon_camera_->imageCols();

  // The distortion model used. Supported models are listed in
  // sensor_msgs/distortion_models.h. For most cameras, "plumb_bob" - a
  // simple model of radial and tangential distortion - is sufficient.
  cam_info_msg.distortion_model = "";

  // The distortion parameters, size depending on the distortion model.
  // For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3) -> float64[] d.
  cam_info_msg.d = std::vector<double>(5, 0.);

  // Intrinsic camera matrix for the raw (distorted) images.
  //     [fx  0 cx]
  // K = [ 0 fy cy]  --> 3x3 row-major matrix
  //     [ 0  0  1]
  // Projects 3D points in the camera coordinate frame to 2D pixel coordinates
  // using the focal lengths (fx, fy) and principal point (cx, cy) -> float64[9] k.
  cam_info_msg.k.fill(0.0);

  // Rectification matrix (stereo cameras only)
  // A rotation matrix aligning the camera coordinate system to the ideal
  // stereo image plane so that epipolar lines in both stereo images are parallel -> float64[9] r, 3x3 row-major matrix.
  cam_info_msg.r.fill(0.0);

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
  //  This holds for both images of a stereo pair -> float64[12]
  cam_info_msg.p.fill(0.0);

  // Binning refers here to any camera setting which combines rectangular
  // neighborhoods of pixels into larger "super-pixels." It reduces the
  // resolution of the output image to (width / binning_x) x (height / binning_y).
  // The default values binning_x = binning_y = 0 is considered the same as
  // binning_x = binning_y = 1 (no subsampling).
  cam_info_msg.binning_x = this->pylon_camera_->currentBinningX();
  cam_info_msg.binning_y = this->pylon_camera_->currentBinningY();

  // Region of interest (subwindow of full camera resolution), given in full
  // resolution (unbinned) image coordinates. A particular ROI always denotes
  // the same window of pixels on the camera sensor, regardless of binning
  // settings. The default setting of roi (all values 0) is considered the same
  // as full resolution (roi.width = width, roi.height = height).
  cam_info_msg.roi.x_offset = cam_info_msg.roi.y_offset = 0;
  cam_info_msg.roi.height = cam_info_msg.roi.width = 0;
}

void PylonROS2CameraNode::setupRectification()
{
  using namespace std::placeholders;

  if (!this->img_rect_pub_)
  {
    this->img_rect_pub_ = new image_transport::Publisher(image_transport::create_publisher(this, "~/image_rect"));
  }

  if (!this->grab_imgs_rect_as_)
  {
    this->grab_imgs_rect_as_ = rclcpp_action::create_server<GrabImagesAction>(
      this,
      "~/grab_images_rect",
      std::bind(&PylonROS2CameraNode::handleGrabRectImagesActionGoal, this, _1, _2),
      std::bind(&PylonROS2CameraNode::handleGrabRectImagesActionGoalCancel, this, _1),
      std::bind(&PylonROS2CameraNode::handleGrabRectImagesActionGoalAccepted, this, _1));
  }

  if (!this->pinhole_model_)
  {
    this->pinhole_model_ = new image_geometry::PinholeCameraModel();
  }

  this->pinhole_model_->fromCameraInfo(this->camera_info_manager_->getCameraInfo());
  if (!this->cv_bridge_img_rect_)
  {
    this->cv_bridge_img_rect_ = new cv_bridge::CvImage();
  }
  this->cv_bridge_img_rect_->header = img_raw_msg_.header;
  this->cv_bridge_img_rect_->encoding = img_raw_msg_.encoding;
}

std::shared_ptr<GrabImagesAction::Result> PylonROS2CameraNode::grabRawImages(const std::shared_ptr<GrabImagesGoalHandle> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<GrabImagesAction::Result>();
  auto feedback = std::make_shared<GrabImagesAction::Feedback>();
  
  if (goal->exposure_given && goal->exposure_times.empty())
  {
    RCLCPP_ERROR_STREAM(LOGGER, "GrabRawImages action server received request and "
        << "'exposure_given' is true, but the 'exposure_times' vector is "
        << "empty! Not enough information to execute acquisition!");
    result->success = false;
    return result;
  }

  if (goal->gain_given && goal->gain_values.empty())
  {
    RCLCPP_ERROR_STREAM(LOGGER, "GrabRawImages action server received request and "
        << "'gain_given' is true, but the 'gain_values' vector is "
        << "empty! Not enough information to execute acquisition!");
    result->success = false;
    return result;
  }

  if (goal->brightness_given && goal->brightness_values.empty())
  {
    RCLCPP_ERROR_STREAM(LOGGER, "GrabRawImages action server received request and "
        << "'brightness_given' is true, but the 'brightness_values' vector"
        << " is empty! Not enough information to execute acquisition!");
    result->success = false;
    return result;
  }

  if (goal->gamma_given && goal->gamma_values.empty())
  {
    RCLCPP_ERROR_STREAM(LOGGER, "GrabRawImages action server received request and "
        << "'gamma_given' is true, but the 'gamma_values' vector is "
        << "empty! Not enough information to execute acquisition!");
    result->success = false;
    return result;
  }

  std::vector<size_t> candidates;
  candidates.resize(4);  // gain, exposure, gamma, brightness
  candidates.at(0) = goal->gain_given ? goal->gain_values.size() : 0;
  candidates.at(1) = goal->exposure_given ? goal->exposure_times.size() : 0;
  candidates.at(2) = goal->brightness_given ? goal->brightness_values.size() : 0;
  candidates.at(3) = goal->gamma_given ? goal->gamma_values.size() : 0;

  size_t n_images = *std::max_element(candidates.begin(), candidates.end());

  if (goal->exposure_given && goal->exposure_times.size() != n_images)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Size of requested exposure times does not match to "
        << "the size of the requested vaules of brightness, gain or "
        << "gamma! Can't grab!");
    result->success = false;
    return result;
  }

  if (goal->gain_given && goal->gain_values.size() != n_images)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Size of requested gain values does not match to "
        << "the size of the requested exposure times or the vaules of "
        << "brightness or gamma! Can't grab!");
    result->success = false;
    return result;
  }

  if (goal->gamma_given && goal->gamma_values.size() != n_images)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Size of requested gamma values does not match to "
        << "the size of the requested exposure times or the vaules of "
        << "brightness or gain! Can't grab!");
    result->success = false;
    return result;
  }

  if (goal->brightness_given && goal->brightness_values.size() != n_images)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Size of requested brightness values does not match to "
        << "the size of the requested exposure times or the vaules of gain or "
        << "gamma! Can't grab!");
    result->success = false;
    return result;
  }

  if (goal->brightness_given && !( goal->exposure_auto || goal->gain_auto))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Error while executing the GrabRawImagesAction: A "
        << "target brightness is provided but Exposure time AND gain are "
        << "declared as fix, so its impossible to reach the brightness");
    result->success = false;
    return result;
  }

  result->images.resize(n_images);
  result->reached_exposure_times.resize(n_images);
  result->reached_gain_values.resize(n_images);
  result->reached_gamma_values.resize(n_images);
  result->reached_brightness_values.resize(n_images);

  result->success = true;

  std::lock_guard<std::recursive_mutex> lock(this->grab_mutex_);

  float previous_exp, previous_gain, previous_gamma;
  if (goal->exposure_given)
  {
    previous_exp = this->pylon_camera_->currentExposure();
  }
  if (goal->gain_given)
  {
    previous_gain = this->pylon_camera_->currentGain();
  }
  if (goal->gamma_given)
  {
    previous_gamma = this->pylon_camera_->currentGamma();
  }
  if (goal->brightness_given)
  {
    previous_gain = this->pylon_camera_->currentGain();
    previous_exp = this->pylon_camera_->currentExposure();
  }

  RCLCPP_DEBUG_STREAM(LOGGER, "Number of grabbed images: " << n_images);
  for (std::size_t i = 0; i < n_images; ++i)
  {
    // user cancel request
    if (goal_handle->is_canceling())
    {
      goal_handle->canceled(result);
      RCLCPP_INFO_STREAM(LOGGER, "Acquisition is stopped (action is cancelled).");
      return result;
    }

    if (goal->exposure_given)
    {
      result->success = this->setExposure(goal->exposure_times[i],
                                          result->reached_exposure_times[i]);
    }

    if (goal->gain_given)
    {
      result->success = this->setGain(goal->gain_values[i],
                                      result->reached_gain_values[i]);
    }
    
    if (goal->gamma_given)
    {
      result->success = this->setGamma(goal->gamma_values[i],
                                       result->reached_gamma_values[i]);
    }

    if (goal->brightness_given)
    {
      int reached_brightness;
      result->success = this->setBrightness(goal->brightness_values[i],
                                            reached_brightness,
                                            goal->exposure_auto,
                                            goal->gain_auto);
      result->reached_brightness_values[i] = static_cast<float>(reached_brightness);
      result->reached_exposure_times[i] = this->pylon_camera_->currentExposure();
      result->reached_gain_values[i] = this->pylon_camera_->currentGain();
    }
  
    if (!result->success)
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Error while setting one of the desired image properties during acquisition (action). Aborting!");
      break;
    }

    sensor_msgs::msg::Image& img = result->images[i];
    img.encoding = this->pylon_camera_->currentROSEncoding();
    img.height = this->pylon_camera_->imageRows();
    img.width = this->pylon_camera_->imageCols();
    // step = full row length in bytes, img_size = (step * rows), imagePixelDepth
    // already contains the number of channels
    img.step = img.width * this->pylon_camera_->imagePixelDepth();

    // Store current time before the image is transmitted for a more accurate grab time estimation
    img.header.stamp = rclcpp::Node::now();
    img.header.frame_id = cameraFrame();

    if (!this->pylon_camera_->grab(img.data))
    {
      result->success = false;
      break;
    }

    feedback->curr_nr_images_taken = i + 1;
    RCLCPP_DEBUG_STREAM(LOGGER, "Publishing feedback...");
    goal_handle->publish_feedback(feedback);
  }

  if (this->camera_info_manager_)
  {
    result->cam_info = this->camera_info_manager_->getCameraInfo();
  }

  // restore previous settings:
  float reached_val;
  if (goal->exposure_given)
  {
    this->setExposure(previous_exp, reached_val);
  }
  if (goal->gain_given)
  {
    this->setGain(previous_gain, reached_val);
  }
  if (goal->gamma_given)
  {
    this->setGamma(previous_gamma, reached_val);
  }
  if (goal->brightness_given)
  {
    this->setGain(previous_gain, reached_val);
    this->setExposure(previous_exp, reached_val);
  }

  return result;
}

std::string PylonROS2CameraNode::grabbingStarting()
{  
  std::lock_guard<std::recursive_mutex> lock(this->grab_mutex_);
  if (!this->pylon_camera_->isReady())
  {
    RCLCPP_WARN(LOGGER, "Error in grabbingStarting(): pylon_camera_ is not ready!");
    return "pylon camera is not ready!";
  }
  if (this->startGrabbing())
  {
    return "done";
  }
  else 
  {
    return "Error";
  }
}

std::string PylonROS2CameraNode::grabbingStopping()
{  
  std::lock_guard<std::recursive_mutex> lock(this->grab_mutex_);
  if (!this->pylon_camera_->isReady())
  {
    RCLCPP_WARN(LOGGER, "Error in grabbingStopping(): pylon_camera_ is not ready!");
    return "pylon camera is not ready!";
  }
  return this->pylon_camera_->grabbingStopping();
}

bool PylonROS2CameraNode::waitForCamera(const std::chrono::duration<double>& timeout) const
{
  bool result = false;
  rclcpp::Time start_time = rclcpp::Node::now();
  rclcpp::Rate r(0.02);

  while (rclcpp::ok())
  {
    if (this->pylon_camera_->isReady())
    {
      result = true;
      break;
    }
    else
    {
      if (timeout >= std::chrono::duration<double>(0))
      {
        if (rclcpp::Node::now() - start_time >= timeout)
        {
          RCLCPP_ERROR_STREAM(LOGGER, "Setting brightness failed, because the "
              << "interface is not ready. This happens although "
              << "waiting for " << timeout.count() << " seconds!");
          return false;
        }
      }
      r.sleep();
    }
  }
  return result;
}

void PylonROS2CameraNode::currentParamPub()
{
  std::lock_guard<std::recursive_mutex> lock(this->grab_mutex_);
  if (!this->pylon_camera_->isReady())
  {
    RCLCPP_WARN(LOGGER, "Error in triggerDeviceReset(): pylon_camera_ is not ready!");
    this->current_params_.message = "pylon camera is not ready!";
    this->current_params_.success = false;
  }
  else 
  {
    try
    {
      this->current_params_.black_level = this->pylon_camera_->getBlackLevel();
      this->current_params_.reverse_x =  this->pylon_camera_->getReverseXY(true);
      this->current_params_.reverse_y =  this->pylon_camera_->getReverseXY(false);
      this->current_params_.offset_x = static_cast<uint32_t>(this->pylon_camera_->currentOffsetX());
      this->current_params_.offset_y = static_cast<uint32_t>(this->pylon_camera_->currentOffsetY());
      this->current_params_.pgi_mode = this->pylon_camera_->getPGIMode();
      this->current_params_.demosaicing_mode = this->pylon_camera_->getDemosaicingMode();
      this->current_params_.noise_reduction = this->pylon_camera_->getNoiseReduction();
      this->current_params_.sharpness_enhancement = this->pylon_camera_->getSharpnessEnhancement();
      this->current_params_.light_source_preset = this->pylon_camera_->getLightSourcePreset();
      this->current_params_.balance_white_auto = this->pylon_camera_->getBalanceWhiteAuto();
      this->current_params_.sensor_readout_mode = this->pylon_camera_->getSensorReadoutMode();
      this->current_params_.acquisition_frame_count = this->pylon_camera_->getAcquisitionFrameCount();
      this->current_params_.trigger_selector = this->pylon_camera_->getTriggerSelector();
      this->current_params_.trigger_mode = this->pylon_camera_->getTriggerMode();
      this->current_params_.trigger_source = this->pylon_camera_->getTriggerSource();
      this->current_params_.trigger_activation = this->pylon_camera_->getTriggerActivation();
      this->current_params_.trigger_delay = this->pylon_camera_->getTriggerDelay();
      this->current_params_.user_set_selector = this->pylon_camera_->getUserSetSelector();
      this->current_params_.user_set_default_selector = this->pylon_camera_->getUserSetDefaultSelector();
      this->current_params_.is_sleeping = this->isSleeping();
      this->current_params_.brightness = this->calcCurrentBrightness();
      this->current_params_.exposure = this->pylon_camera_->currentExposure();
      this->current_params_.gain = this->pylon_camera_->currentGain();
      this->current_params_.gamma = this->pylon_camera_->currentGamma();
      this->current_params_.binning_x = static_cast<uint32_t>(this->pylon_camera_->currentBinningX());
      this->current_params_.binning_y = static_cast<uint32_t>(this->pylon_camera_->currentBinningY());
      this->current_params_.roi = this->pylon_camera_->currentROI();
      this->current_params_.available_image_encoding = this->pylon_camera_->detectAvailableImageEncodings(false);
      this->current_params_.current_image_encoding = this->pylon_camera_->currentBaslerEncoding();
      this->current_params_.current_image_ros_encoding = this->pylon_camera_->currentROSEncoding();
      this->current_params_.temperature = this->pylon_camera_->getTemperature();
      this->current_params_.max_num_buffer = this->pylon_camera_->getMaxNumBuffer();

      this->current_params_.success = true;
    }
    catch (const GenICam::GenericException &e)
    {
      RCLCPP_ERROR_STREAM(LOGGER, "An exception while getting the camera current params occurred:" << e.GetDescription());
      this->current_params_.success = false;
      this->current_params_.message = "An exception while getting the camera current parameters occurred";
    }
  }
  this->current_params_pub_->publish(this->current_params_);
}

bool PylonROS2CameraNode::isSleeping()
{
  return this->is_sleeping_;
}

} // namespace pylon_ros2_camera

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(pylon_ros2_camera::PylonROS2CameraNode)
