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

#include "pylon_ros2_camera_node.hpp"

namespace pylon_ros2_camera
{

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

  srv_name = srv_prefix + "set_depth_min";
  this->set_depth_min_srv_ = this->create_service<SetFloatSrv>(srv_name, std::bind(&PylonROS2CameraNode::setDepthMinCallback, this, _1, _2));

  srv_name = srv_prefix + "set_depth_max";
  this->set_depth_max_srv_ = this->create_service<SetFloatSrv>(srv_name, std::bind(&PylonROS2CameraNode::setDepthMaxCallback, this, _1, _2));

  srv_name = srv_prefix + "set_temporal_filter_strength";
  this->set_temporal_filter_strength_srv_ = this->create_service<SetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::setTemporalFilterStrengthCallback, this, _1, _2));

  srv_name = srv_prefix + "set_outlier_removal_threshold";
  this->set_outlier_removal_threshold_srv_ = this->create_service<SetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::setOutlierRemovalThresholdCallback, this, _1, _2));

  srv_name = srv_prefix + "set_outlier_removal_tolerance";
  this->set_outlier_removal_tolerance_srv_ = this->create_service<SetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::setOutlierRemovalToleranceCallback, this, _1, _2));

  srv_name = srv_prefix + "set_ambiguity_filter_threshold";
  this->set_ambiguity_filter_threshold_srv_ = this->create_service<SetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::setAmbiguityFilterThresholdCallback, this, _1, _2));

  srv_name = srv_prefix + "set_confidence_threshold";
  this->set_confidence_threshold_srv_ = this->create_service<SetFloatSrv>(srv_name, std::bind(&PylonROS2CameraNode::setConfidenceThresholdCallback, this, _1, _2));

  srv_name = srv_prefix + "set_intensity_calculation";
  this->set_intensity_calculation_srv_ = this->create_service<SetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::setIntensityCalculationCallback, this, _1, _2));

  srv_name = srv_prefix + "set_exposure_time_selector";
  this->set_exposure_time_selector_srv_ = this->create_service<SetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::setExposureTimeSelectorCallback, this, _1, _2));

  srv_name = srv_prefix + "set_operating_mode";
  this->set_operating_mode_srv_ = this->create_service<SetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::setOperatingModeCallback, this, _1, _2));

  srv_name = srv_prefix + "set_multi_camera_channel";
  this->set_multi_camera_channel_srv_ = this->create_service<SetIntegerSrv>(srv_name, std::bind(&PylonROS2CameraNode::setMultiCameraChannelCallback, this, _1, _2));

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

  srv_name = srv_prefix + "set_acquisition_frame_rate";
  this->set_acquisition_frame_rate_srv_ = this->create_service<SetFloatSrv>(srv_name, std::bind(&PylonROS2CameraNode::setAcquisitionFrameRateCallback, this, _1, _2));

  srv_name = srv_prefix + "set_scan_3d_calibration_offset";
  this->set_scan_3d_calibration_offset_srv_ = this->create_service<SetFloatSrv>(srv_name, std::bind(&PylonROS2CameraNode::setScan3dCalibrationOffsetCallback, this, _1, _2));

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

  srv_name = srv_prefix + "get_ptp_status";
  this->get_ptp_status_srv_ = this->create_service<GetPtpStatusSrv>(srv_name, std::bind(&PylonROS2CameraNode::getPTPStatusCallback, this, _1, _2));

  srv_name = srv_prefix + "enable_sync_free_run_timer";
  this->enable_sync_free_run_timer_srv_ = this->create_service<SetBoolSrv>(srv_name, std::bind(&PylonROS2CameraNode::enableSyncFreeRunTimerCallback, this, _1, _2));

  srv_name = srv_prefix + "enable_spatial_filter";
  this->enable_spatial_filter_srv_ = this->create_service<SetBoolSrv>(srv_name, std::bind(&PylonROS2CameraNode::enableSpatialFilterCallback, this, _1, _2));

  srv_name = srv_prefix + "enable_temporal_filter";
  this->enable_temporal_filter_srv_ = this->create_service<SetBoolSrv>(srv_name, std::bind(&PylonROS2CameraNode::enableTemporalFilterCallback, this, _1, _2));

  srv_name = srv_prefix + "enable_outlier_removal";
  this->enable_outlier_removal_srv_ = this->create_service<SetBoolSrv>(srv_name, std::bind(&PylonROS2CameraNode::enableOutlierRemovalCallback, this, _1, _2));

  srv_name = srv_prefix + "enable_ambiguity_filter";
  this->enable_ambiguity_filter_srv_ = this->create_service<SetBoolSrv>(srv_name, std::bind(&PylonROS2CameraNode::enableAmbiguityFilterCallback, this, _1, _2));

  srv_name = srv_prefix + "enable_thermal_drift_correction";
  this->enable_thermal_drift_correction_srv_ = this->create_service<SetBoolSrv>(srv_name, std::bind(&PylonROS2CameraNode::enableThermalDriftCorrectionCallback, this, _1, _2));

  srv_name = srv_prefix + "enable_distortion_correction";
  this->enable_distortion_correction_srv_ = this->create_service<SetBoolSrv>(srv_name, std::bind(&PylonROS2CameraNode::enableDistortionCorrectionCallback, this, _1, _2));

  srv_name = srv_prefix + "enable_acquisition_frame_rate";
  this->enable_acquisition_frame_rate_srv_ = this->create_service<SetBoolSrv>(srv_name, std::bind(&PylonROS2CameraNode::enableAcquisitionFrameRateCallback, this, _1, _2));

  srv_name = srv_prefix + "enable_hdr_mode";
  this->enable_hdr_mode_srv_ = this->create_service<SetBoolSrv>(srv_name, std::bind(&PylonROS2CameraNode::enableHDRModeCallback, this, _1, _2));

  srv_name = srv_prefix + "enable_fast_mode";
  this->enable_fast_mode_srv_ = this->create_service<SetBoolSrv>(srv_name, std::bind(&PylonROS2CameraNode::enableFastModeCallback, this, _1, _2));

  srv_name = srv_prefix + "execute_software_trigger";
  this->execute_software_trigger_srv_ = this->create_service<TriggerSrv>(srv_name, std::bind(&PylonROS2CameraNode::executeSoftwareTriggerCallback, this, _1, _2));

  srv_name = srv_prefix + "save_user_set";
  this->save_user_set_srv_ = this->create_service<TriggerSrv>(srv_name, std::bind(&PylonROS2CameraNode::saveUserSetCallback, this, _1, _2));

  srv_name = srv_prefix + "load_user_set";
  this->load_user_set_srv_ = this->create_service<TriggerSrv>(srv_name, std::bind(&PylonROS2CameraNode::loadUserSetCallback, this, _1, _2));

  srv_name = srv_prefix + "get_pfs";
  this->get_pfs_srv_ = this->create_service<GetStringSrv>(srv_name, std::bind(&PylonROS2CameraNode::getPfsCallback, this, _1, _2));

  srv_name = srv_prefix + "save_pfs";
  this->save_pfs_srv_ = this->create_service<SetStringSrv>(srv_name, std::bind(&PylonROS2CameraNode::savePfsCallback, this, _1, _2));

  srv_name = srv_prefix + "load_pfs";
  this->load_pfs_srv_ = this->create_service<SetStringSrv>(srv_name, std::bind(&PylonROS2CameraNode::loadPfsCallback, this, _1, _2));

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

} // namespace pylon_ros2_camera
