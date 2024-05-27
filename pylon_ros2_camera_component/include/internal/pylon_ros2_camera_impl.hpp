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

#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#pragma GCC diagnostic ignored "-Wliteral-suffix"

#include <pylon/BaslerUniversalGrabResultPtr.h>
#include <pylon/PylonIncludes.h>
#include <GenApi/IEnumEntry.h>
#include <string>
#include <vector>
#include <map>

#include "pylon_ros2_camera_parameter.hpp"
#include "pylon_ros2_camera.hpp"


namespace pylon_ros2_camera
{

template <typename CameraTraitT>
class PylonROS2CameraImpl : public PylonROS2Camera
{

public:

    explicit PylonROS2CameraImpl(Pylon::IPylonDevice* device);

    virtual ~PylonROS2CameraImpl();

    virtual bool registerCameraConfiguration() override;

    virtual bool openCamera() override;

    virtual bool isCamRemoved() override;

    virtual bool setupSequencer(const std::vector<float>& exposure_times) override;

    virtual bool applyCamSpecificStartupSettings(const PylonROS2CameraParameter& parameters) override;

    virtual void getInitialCameraInfo(sensor_msgs::msg::CameraInfo& cam_info_msg) override;

    virtual bool startGrabbing(const PylonROS2CameraParameter& parameters) override;

    virtual bool grab(std::vector<uint8_t>& image, rclcpp::Time &stamp) override;

    virtual bool grab(uint8_t* image) override;

    virtual bool setShutterMode(const pylon_ros2_camera::SHUTTER_MODE& mode) override;

    virtual bool setROI(const sensor_msgs::msg::RegionOfInterest target_roi,
                        sensor_msgs::msg::RegionOfInterest& reached_roi) override;

    virtual bool setBinningX(const size_t& target_binning_x,
                             size_t& reached_binning_x) override;

    virtual bool setBinningY(const size_t& target_binning_y,
                             size_t& reached_binning_y) override;

    virtual std::string setImageEncoding(const std::string& target_ros_encoding) const override;

    virtual bool setExposure(const float& target_exposure, float& reached_exposure) override;

    virtual bool setAutoflash(const std::map<int, bool> flash_on_lines) override;

    virtual bool setGain(const float& target_gain, float& reached_gain) override;

    virtual bool setGamma(const float& target_gamma, float& reached_gamma) override;

    virtual bool setBrightness(const int& target_brightness,
                               const float& current_brightness,
                               const bool& exposure_auto,
                               const bool& gain_auto) override;

    virtual std::vector<int> detectAndCountNumUserOutputs() override;

    virtual bool setUserOutput(const int& output_id, const bool& value) override;

    virtual size_t currentOffsetX() override;

    virtual size_t currentOffsetY() override;

    virtual sensor_msgs::msg::RegionOfInterest currentROI() override;

    virtual size_t currentBinningX() override;

    virtual size_t currentBinningY() override;

    virtual std::vector<std::string> detectAvailableImageEncodings(const bool& show_message) override;

    virtual std::string currentROSEncoding() const override;

    virtual std::string currentBaslerEncoding() const override;

    virtual int imagePixelDepth() const override;

    virtual float currentExposure() override;

    virtual float currentAutoExposureTimeLowerLimit() override;

    virtual float currentAutoExposureTimeUpperLimit() override;

    virtual float currentGain() override;

    virtual float currentAutoGainLowerLimit() override;

    virtual float currentAutoGainUpperLimit() override;

    virtual float currentGamma() override;

    virtual float maxPossibleFramerate() override;

    virtual bool isPylonAutoBrightnessFunctionRunning() override;

    virtual bool isBrightnessSearchRunning() override;

    virtual void disableAllRunningAutoBrightessFunctions() override;

    virtual void enableContinuousAutoExposure() override;

    virtual void enableContinuousAutoGain() override;

    virtual std::string typeName() const override;

    virtual float exposureStep() override;

    virtual std::string setOffsetXY(const int& offsetValue, bool xAxis) override;

    virtual std::string reverseXY(const bool& reverse_x,bool around_x) override;

    virtual bool getReverseXY(const bool& returnX) override;

    virtual std::string setBlackLevel(const int& value) override;

    virtual int getBlackLevel() override;

    virtual std::string setPGIMode(const bool& on) override;

    virtual int getPGIMode() override;

    virtual std::string setDemosaicingMode(const int& mode) override;

    virtual int getDemosaicingMode() override;

    virtual std::string setNoiseReduction(const float& value) override;

    virtual float getNoiseReduction() override;

    virtual std::string setSharpnessEnhancement(const float& value) override;

    virtual float getSharpnessEnhancement() override;

    virtual std::string setLightSourcePreset(const int& mode) override;

    virtual int getLightSourcePreset() override;

    virtual std::string setBalanceWhiteAuto(const int& mode) override;

    virtual int getBalanceWhiteAuto() override;

    virtual std::string setSensorReadoutMode(const int& mode) override;

    virtual int getSensorReadoutMode() override;

    virtual std::string setAcquisitionFrameCount(const int& frameCount) override;

    virtual int getAcquisitionFrameCount() override;

    virtual std::string setTriggerSelector(const int& mode) override;

    virtual int getTriggerSelector() override;

    virtual std::string setTriggerMode(const bool& value) override;

    virtual int getTriggerMode() override;

    virtual std::string executeSoftwareTrigger() override;

    virtual std::string setTriggerSource(const int& source) override;

    virtual int getTriggerSource() override;

    virtual std::string setTriggerActivation(const int& value) override;

    virtual int getTriggerActivation() override;

    virtual std::string setTriggerDelay(const float& delayValue) override;

    virtual float getTriggerDelay() override;

    virtual std::string setLineSelector(const int& value) override;

    virtual std::string setLineMode(const int& value) override;

    virtual std::string setLineSource(const int& value) override;

    virtual std::string setLineInverter(const bool& value) override;

    virtual std::string setLineDebouncerTime(const float& value) override;

    virtual std::string setUserSetSelector(const int& set) override;

    virtual int getUserSetSelector() override;

    virtual std::string saveUserSet() override;

    virtual std::string loadUserSet() override;

    virtual std::pair<std::string, std::string> getPfs() override;

    virtual std::string savePfs(const std::string& fileName) override;

    virtual std::string loadPfs(const std::string& fileName) override;

    virtual std::string setUserSetDefaultSelector(const int& set) override;

    virtual int getUserSetDefaultSelector() override;

    virtual std::string setDeviceLinkThroughputLimitMode(const bool& turnOn) override;

    virtual int getDeviceLinkThroughputLimitMode() override;

    virtual std::string setDeviceLinkThroughputLimit(const int& limit) override;

    virtual std::string triggerDeviceReset() override;

    virtual std::string grabbingStarting() const override;

    virtual std::string grabbingStopping() override;

    virtual std::string setMaxTransferSize(const int& maxTransferSize) override;

    virtual std::string setGammaSelector(const int& gammaSelector) override;

    virtual std::string gammaEnable(const bool& enable) override;

    virtual float getTemperature() override;

    virtual std::string setWhiteBalance(const double& redValue, const double& greenValue, const double& blueValue) override;

    virtual bool setGrabbingStrategy(const int& strategy) override;

    virtual std::string setOutputQueueSize(const int& size) override;

    virtual std::string setMaxNumBuffer(const int& size) override;

    virtual int getMaxNumBuffer() override;

    virtual int getStatisticTotalBufferCount() override;

    virtual int getStatisticFailedBufferCount() override;

    virtual int getStatisticBufferUnderrunCount() override;

    virtual int getStatisticFailedPacketCount() override;

    virtual int getStatisticResendRequestCount() override;

    virtual int getStatisticMissedFrameCount() override;

    virtual int getStatisticResynchronizationCount() override;

    virtual std::string setChunkModeActive(const bool& enable) override;

    virtual int getChunkModeActive() override;

    virtual std::string setChunkSelector(const int& value) override;

    virtual int getChunkSelector() override;

    virtual std::string setChunkEnable(const bool& enable) override;

    virtual int getChunkEnable() override;

    virtual int64_t getChunkTimestamp() override;

    virtual float getChunkExposureTime() override;

    virtual std::string setChunkExposureTime(const float& value) override;

    virtual int64_t getChunkLineStatusAll() override;

    virtual int64_t getChunkFramecounter() override;

    virtual int64_t getChunkCounterValue() override;

    virtual std::string setTimerSelector(const int& selector) override;

    virtual std::string setTimerTriggerSource(const int& source) override;

    virtual std::string setTimerDuration(const float& duration) override;

    virtual std::string setPTPPriority(const int& value) override;

    virtual std::string setPTPProfile(const int& value) override;

    virtual std::string setPTPNetworkMode(const int& value) override;

    virtual std::string setPTPUCPortAddressIndex(const int& value) override;

    virtual std::string setPTPUCPortAddress(const int& value) override;

    virtual std::string setPeriodicSignalPeriod(const float& value) override;

    virtual std::string setPeriodicSignalDelay(const float& value) override;

    virtual std::string setSyncFreeRunTimerStartTimeLow(const int& value) override;

    virtual std::string setSyncFreeRunTimerStartTimeHigh(const int& value) override;

    virtual std::string setSyncFreeRunTimerTriggerRateAbs(const float& value) override;

    virtual std::string enablePTPManagementProtocol(const bool& value) override;

    virtual std::string enablePTPTwoStepOperation(const bool& value) override;

    virtual std::string enablePTP(const bool& value) override;

    virtual std::string enableSyncFreeRunTimer(const bool& value) override;

    virtual std::string updateSyncFreeRunTimer() override;

    virtual std::string setActionTriggerConfiguration(const int& action_device_key, const int& action_group_key, const unsigned int& action_group_mask,
                                                      const int& registration_mode, const int& cleanup) override;

    virtual std::string issueActionCommand(const int& device_key, const int& group_key, const unsigned int& group_mask, const std::string& broadcast_address) override;

    virtual std::string issueScheduledActionCommand(const int& device_key, const int& group_key, const unsigned int& group_mask, const int64_t& action_time_ns_from_current_timestamp, const std::string& broadcast_address) override;



    // blaze related functions

    virtual bool isBlaze() override;

    virtual bool grabBlaze(sensor_msgs::msg::PointCloud2& cloud_msg,
                           sensor_msgs::msg::Image& intensity_map_msg,
                           sensor_msgs::msg::Image& depth_map_msg,
                           sensor_msgs::msg::Image& depth_map_color_msg,
                           sensor_msgs::msg::Image& confidence_map_msg) override;

    virtual std::string setDepthMin(const int& depth_min) override;

    virtual std::string setDepthMax(const int& depth_max) override;

    virtual std::string setTemporalFilterStrength(const int& strength) override;

    virtual std::string setOutlierRemovalThreshold(const int& threshold) override;

    virtual std::string setOutlierRemovalTolerance(const int& tolerance) override;

    virtual std::string setAmbiguityFilterThreshold(const int& threshold) override;

    virtual std::string setConfidenceThreshold(const int& threshold) override;

    virtual std::string setIntensityCalculation(const int& calculation) override;

    virtual std::string setExposureTimeSelector(const int& selector) override;

    virtual std::string setOperatingMode(const int& mode) override;

    virtual std::string setMultiCameraChannel(const int& channel) override;

    virtual std::string setAcquisitionFrameRate(const float& framerate) override;

    virtual std::string setScan3dCalibrationOffset(const float& offset) override;

    virtual std::string enableSpatialFilter(const bool& enable) override;

    virtual std::string enableTemporalFilter(const bool& enable) override;

    virtual std::string enableOutlierRemoval(const bool& enable) override;

    virtual std::string enableAmbiguityFilter(const bool& enable) override;

    virtual std::string enableThermalDriftCorrection(const bool& enable) override;

    virtual std::string enableDistortionCorrection(const bool& enable) override;

    virtual std::string enableAcquisitionFrameRate(const bool& enable) override;

    virtual std::string enableHDRMode(const bool& enable) override;

    virtual std::string enableFastMode(const bool& enable) override;


protected:

    typedef typename CameraTraitT::CBaslerInstantCameraT CBaslerInstantCameraT;
    typedef typename CameraTraitT::ExposureAutoEnums ExposureAutoEnums;
    typedef typename CameraTraitT::GainAutoEnums GainAutoEnums;
    typedef typename CameraTraitT::PixelFormatEnums PixelFormatEnums;
    typedef typename CameraTraitT::PixelSizeEnums PixelSizeEnums;
    typedef typename CameraTraitT::AutoTargetBrightnessType AutoTargetBrightnessType;
    typedef typename CameraTraitT::GainType GainType;
    typedef typename CameraTraitT::ShutterModeEnums ShutterModeEnums;
    typedef typename CameraTraitT::UserOutputSelectorEnums UserOutputSelectorEnums;
    typedef typename CameraTraitT::SensorReadoutModeEnums SensorReadoutModeEnums;
    typedef typename CameraTraitT::AcquisitionStatusSelectorEnums AcquisitionStatusSelectorEnums;
    typedef typename CameraTraitT::TriggerSelectorEnums TriggerSelectorEnums;
    typedef typename CameraTraitT::TriggerModeEnums TriggerModeEnums;
    typedef typename CameraTraitT::TriggerSourceEnums TriggerSourceEnums;
    typedef typename CameraTraitT::TriggerActivationEnums TriggerActivationEnums;
    typedef typename CameraTraitT::LineSelectorEnums LineSelectorEnums;
    typedef typename CameraTraitT::LineModeEnums LineModeEnums;
    typedef typename CameraTraitT::DeviceLinkThroughputLimitModeEnums DeviceLinkThroughputLimitModeEnums;
    typedef typename CameraTraitT::AutoFunctionROISelectorEnums AutoFunctionROISelectorEnums;
    typedef typename CameraTraitT::BalanceWhiteAutoEnums BalanceWhiteAutoEnums;
    typedef typename CameraTraitT::LightSourcePresetEnums LightSourcePresetEnums;
    typedef typename CameraTraitT::LineSourceEnums LineSourceEnums;
    typedef typename CameraTraitT::DemosaicingModeEnums DemosaicingModeEnums;
    typedef typename CameraTraitT::PgiModeEnums PgiModeEnums;
    typedef typename CameraTraitT::UserSetSelectorEnums UserSetSelectorEnums;
    typedef typename CameraTraitT::UserSetDefaultSelectorEnums UserSetDefaultSelectorEnums;
    typedef typename CameraTraitT::LineFormatEnums LineFormatEnums;
    typedef typename CameraTraitT::BalanceRatioSelectorEnums BalanceRatioSelectorEnums;
    typedef typename CameraTraitT::TimerSelectorEnums TimerSelectorEnums;
    typedef typename CameraTraitT::TimerTriggerSourceEnums TimerTriggerSourceEnums;

    CBaslerInstantCameraT* cam_;

    // Each camera has it's own getter for GenApi accessors that are named
    // differently for USB and GigE
    GenApi::IFloat& exposureTime();
    GainType& gain();
    GenApi::IFloat& gamma();
    GenApi::IFloat& autoExposureTimeLowerLimit();
    GenApi::IFloat& autoExposureTimeUpperLimit();
    GainType& autoGainLowerLimit();
    GainType& autoGainUpperLimit();
    GenApi::IFloat& resultingFrameRate();
    AutoTargetBrightnessType& autoTargetBrightness();

    virtual bool setExtendedBrightness(const int& target_brightness,
                                       const float& current_brightness) override;

    virtual bool setupSequencer(const std::vector<float>& exposure_times,
                                std::vector<float>& exposure_times_set);

    bool grab(Pylon::CBaslerUniversalGrabResultPtr& grab_result);
};

}  // namespace pylon_ros2_camera

#include "internal/impl/pylon_ros2_camera_base.hpp"
#include "internal/impl/pylon_ros2_camera_usb.hpp"
#include "internal/impl/pylon_ros2_camera_dart.hpp"
#include "internal/impl/pylon_ros2_camera_gige.hpp"
#include "internal/impl/pylon_ros2_camera_gige_ace2.hpp"
#include "internal/impl/pylon_ros2_camera_blaze.hpp"
