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

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/region_of_interest.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "pylon_ros2_camera_parameter.hpp"
#include "binary_exposure_search.hpp"


namespace pylon_ros2_camera
{

// Number of channels for image encoding
#define CHANNEL_MONO8 1
#define CHANNEL_RGB8  3

/**
 * The PylonROS2Camera base class. Create a new instance using the static create() functions.
 */
class PylonROS2Camera
{
public:

    /**
     * Create a new PylonROS2Camera instance. It will return the first camera that could be found.
     * @return new PylonROS2Camera instance or NULL if no camera was found.
     */
    static std::unique_ptr<PylonROS2Camera> create();

    /**
     * Create a new PylonROS2Camera instance based on the DeviceUserID of the camera.
     * @param device_user_id Pylon DeviceUserID. If the string is empty, the
     * first camera that could be found is returned.
     * @return new PylonROS2Camera instance or NULL if the camera was not found.
     */
    static std::unique_ptr<PylonROS2Camera> create(const std::string& device_user_id);

    /**
     * Configures the camera according to the software trigger mode.
     * @return true if all the configuration could be set up.
     */
    virtual bool registerCameraConfiguration() = 0;

    /**
     * Opens the desired camera, the communication starts from now on.
     * @return true if the camera could be opened.
     */
    virtual bool openCamera() = 0;

    /**
     * Returns the connection state of the camera device.
     * @return true if the camera device removal from the PC has been detected.
     */
    virtual bool isCamRemoved() = 0;

    /**
     * Configure the sequencer exposure times.
     * @param exposure_times the list of exposure times.
     * @return true if all parameters could be sent to the camera.
     */
    virtual bool setupSequencer(const std::vector<float>& exposure_times) = 0;

    /**
     * Configures the camera according to the provided ros parameters.
     * This will use the device specific parameters as e.g., the mtu size for
     * GigE cameras
     * @param parameters The PylonROS2CameraParameter set to use
     * @return true if all parameters could be sent to the camera.
     */
    virtual bool applyCamSpecificStartupSettings(const PylonROS2CameraParameter& parameters) = 0;

    /**
     * Get initial camera info
     * @return initial camera info.
     */
    virtual void getInitialCameraInfo(sensor_msgs::msg::CameraInfo& cam_info_msg) = 0;

    /**
     * Initializes the internal parameters of the PylonROS2Camera instance.
     * @param parameters The PylonROS2CameraParameter set to use
     * @return true if all parameters could be sent to the camera.
     */
    virtual bool startGrabbing(const PylonROS2CameraParameter& parameters) = 0;

    /**
     * Grab a camera frame and copy the result into image
     * @param image reference to the output image.
     * @param stamp if chunk timestamp is enabled, overwrite input stamp with the acquisition timestamp.
     * @return true if the image was grabbed successfully.
     */
    virtual bool grab(std::vector<uint8_t>& image, rclcpp::Time &stamp) = 0;

    /**
     * Grab a camera frame and copy the result into image
     * @param image pointer to the image buffer.
     *              Caution: Make sure the buffer is initialized correctly!
     * @return true if the image was grabbed successfully.
     */
    virtual bool grab(uint8_t* image) = 0;

    /**
     * Dedicated to blaze integration within the pylon driver - specify if the connected camera is a blaze
     * @return true if a blaze is connected.
     */
    virtual bool isBlaze() = 0;

    /**
     * Dedicated to blaze integration within the pylon driver - grab data from blaze and return ros messages
     * @return true if the process is successful.
     */
    virtual bool grabBlaze(sensor_msgs::msg::PointCloud2& cloud_msg,
                           sensor_msgs::msg::Image& intensity_map_msg,
                           sensor_msgs::msg::Image& depth_map_msg,
                           sensor_msgs::msg::Image& depth_map_color_msg,
                           sensor_msgs::msg::Image& confidence_map_msg) = 0;

    /**
     * @brief sets shutter mode for the camera (rolling or global_reset)
     * @param mode
     * @return
     */
    virtual bool setShutterMode(const pylon_ros2_camera::SHUTTER_MODE& mode) = 0;

    /**
     * Update area of interest in the camera image
     * @param target_roi the target roi
     * @param reached_roi the roi that could be set
     * @return true if the targeted roi could be reached
     */
    virtual bool setROI(const sensor_msgs::msg::RegionOfInterest target_roi,
			sensor_msgs::msg::RegionOfInterest& reached_roi) = 0;

    /**
     * Sets the target horizontal binning_x factor
     * @param target_binning_x the target horizontal binning_x factor.
     * @param reached_binning_x the reached horizontal binning_x factor.
     * @return false if a communication error occurred or true otherwise.
     */
    virtual bool setBinningX(const size_t& target_binning_x,
                             size_t& reached_binning_x) = 0;

    /**
     * Sets the target vertical binning_y factor
     * @param target_binning_y the target vertical binning_y factor.
     * @param reached_binning_y the reached vertical binning_y factor.
     * @return false if a communication error occurred or true otherwise.
     */
    virtual bool setBinningY(const size_t& target_binning_y,
                             size_t& reached_binning_y) = 0;

    /**
     * Detects the supported image pixel encodings of the camera an stores
     * them in a vector.
     * @return a list of strings describing the supported encodings in GenAPI
     *         language.
     */
    virtual std::vector<std::string> detectAvailableImageEncodings(const bool& show_message) = 0;

    /**
     * Sets the desired image pixel encoding (channel meaning, ordering, size)
     * taken from the list of strings in include/sensor_msgs/image_encodings.h
     * The supported encodings are 'mono8', 'bgr8', 'rgb8', 'bayer_bggr8',
     * 'bayer_gbrg8', 'bayer_rggb8' and 'yuv422'
     * @param target_ros_endcoding: string describing the encoding.
     * @return false if a communication error occurred or true otherwise.
     */
    virtual std::string setImageEncoding(const std::string& target_ros_encoding) const= 0;

    /**
     * Sets the exposure time in microseconds
     * @param target_exposure the desired exposure time to set in microseconds.
     * @param reached_exposure time in microseconds
     * @return false if a communication error occurred or true otherwise.
     */
    virtual bool setExposure(const float& target_exposure,
                             float& reached_exposure) = 0;

    /**
     * Sets autoflash active for the specified lines
     * @param flash_on_lines map from line e.g., 1 or 2 to a boolean to
              activate or deactivate the autoflash for this line .
     * @return false if a communication error occurred or true otherwise.
     */
    virtual bool setAutoflash(const std::map<int, bool> flash_on_lines) = 0;

    /**
     * Sets the gain in percent independent of the camera type
     * @param target_gain the target gain in percent.
     * @param reached_gain the reached gain in percent.
     * @return false if a communication error occurred or true otherwise.
     */
    virtual bool setGain(const float& target_gain, float& reached_gain) = 0;

    /**
     * Sets the target gamma value
     * @param target_gamma the target gamma value.
     * @param reached_gamma the reached gamma value.
     * @return false if a communication error occurred or true otherwise.
     */
    virtual bool setGamma(const float& target_gamma, float& reached_gamma) = 0;

    /**
     * Sets the target brightness
     * Setting the exposure time to -1 enables the AutoExposureContinuous mode.
     * Setting the exposure time to  0 disables the AutoExposure function.
     * If the target exposure time is not in the range of Pylon's auto target brightness range
     * the extended brightness search is started.
     * @param target_brightness is the desired brightness. Range is [1...255].
     * @param current_brightness is the current brightness with the given settings.
     * @param exposure_auto flag which indicates if the target_brightness
     *                      should be reached adapting the exposure time
     * @param gain_auto flag which indicates if the target_brightness should be
     *                      reached adapting the gain.
     * @return true if the brightness could be reached or false otherwise.
     */
    virtual bool setBrightness(const int& target_brightness,
                               const float& current_brightness,
                               const bool& exposure_auto,
                               const bool& gain_auto) = 0;

    /**
     * @brief Detects and counts the number of user-settable-outputs the cam
     *        provides. This might be zero for some cameras. The size affects
     *        the number of 'set' ros-services the camera_node will provide.
     *        A vector which length equals the number of user-settable outputs
     *        will be generated. Hence e.g., output '1' can be accessed via
     *        user_output_selector_enums_.at(1).
     * @return the UserOutputSelector enum list
     */
    virtual std::vector<int> detectAndCountNumUserOutputs() = 0;

    /**
     * @brief setUserOutput sets the digital output
     * @param output_id
     * @param value goal value for output
     * @return true if value was set
     */
    virtual bool setUserOutput(const int& output_id, const bool& value) = 0;

    /**
     * Returns the current x offset setting.
     * @return the horizontal x offset setting.
     */
    virtual size_t currentOffsetX() = 0;

    /**
     * Returns the current y offset setting.
     * @return the horizontal y offset setting.
     */
    virtual size_t currentOffsetY() = 0;

    /**
     * Returns the current roi setting.
     * @return the roi setting.
     */
    virtual sensor_msgs::msg::RegionOfInterest currentROI() = 0;

    /**
     * Returns the current horizontal binning_x setting.
     * @return the horizontal binning_x setting.
     */
    virtual size_t currentBinningX() = 0;

    /**
     * Returns the current vertical binning_y setting.
     * @return the vertical binning_y setting.
     */
    virtual size_t currentBinningY() = 0;

    /**
     * Get the camera image encoding according to sensor_msgs::image_encodings
     * The supported encodings are 'mono8', 'bgr8', 'rgb8', 'bayer_bggr8',
     * 'bayer_gbrg8', 'bayer_rggb8' and 'yuv422'
     * @return the current ros image pixel encoding.
     */
    virtual std::string currentROSEncoding() const = 0;

    /**
     * Get the camera image encoding according to Basler encodings
     * @return the current basler image pixel encoding.
     */
    virtual std::string currentBaslerEncoding() const = 0;

    /**
     * Get the number of bytes per pixel
     * @return number of bytes per pixel
     */
    virtual int imagePixelDepth() const = 0;

    /**
     * Returns the current exposure time in microseconds.
     * @return the exposure time in microseconds.
     */
    virtual float currentExposure() = 0;

    /**
     * Returns the current auto exposure time lower limit
     * @return the current auto exposure time lower limit
     */
    virtual float currentAutoExposureTimeLowerLimit() = 0;

    /**
     * Returns the current auto exposure time upper limit
     * @return the current auto exposure time upper limit
     */
    virtual float currentAutoExposureTimeUpperLimit() = 0;

    /**
     * Returns the current gain in percent.
     * @return the gain time percent.
     */
    virtual float currentGain() = 0;

    /**
     * Returns the current auto gain lower limit
     * @return the current auto gain lower limit
     */
    virtual float currentAutoGainLowerLimit() = 0;

    /**
     * Returns the current auto gain upper limit
     * @return the current auto gain upper limit
     */
    virtual float currentAutoGainUpperLimit() = 0;

    /**
     * Returns the current gamma value.
     * @return the gamma value.
     */
    virtual float currentGamma() = 0;

    /**
     * Checks if the camera currently tries to regulate towards a target brightness.
     * This can either be done by pylon for the range [50 - 205] or the own extendended binary search one
     * for the ranges [1 - 49] and [206 - 254].
     * @return true if the brightness-search is running
     */
    virtual bool isBrightnessSearchRunning() = 0;

    /**
     * Checks if the auto brightness function from the Pylon API is enabled.
     * @return true if AutoExposure is set to AutoExposureContinuous or AutoExposureOnce.
     */
    virtual bool isPylonAutoBrightnessFunctionRunning() = 0;

    /**
     * Getter for is_binary_exposure_search_running_
     * @return true if the extended exposure search is running
     */
    const bool& isBinaryExposureSearchRunning() const;

    /**
     * Disables all currently running brightness search methods in case that
     * the desired brightness is reached or a timeout occurred
     */
    virtual void disableAllRunningAutoBrightessFunctions() = 0;

    /**
     * Enables the continuous auto exposure mode
     */
    virtual void enableContinuousAutoExposure() = 0;

    /**
     * Enables the continuous auto gain mode
     */
    virtual void enableContinuousAutoGain() = 0;

    /**
     * Get the camera type. Currently supported cameras are USB, DART and GigE
     * @return camera type as string
     */
    virtual std::string typeName() const = 0;

    /**
     * Minimum possible increment between two possible exposure values
     * @return the minimum possible increment between two possible exposure values
     */
    virtual float exposureStep() = 0;

    /**
     * Getter for the device user id of the used camera
     * @return the device_user_id
     */
    const std::string& deviceUserID() const;

    /**
     * Getter for the image height
     * @return number of rows in the image
     */
    const size_t& imageRows() const;

    /**
     * Getter for the image width
     * @return number of columns in the image
     */
    const size_t& imageCols() const;

    /**
     * Getter for the is_ready_ flag. This is set in case that the
     * grab-result-pointer of the first acquisition contains valid data.
     * Hence this is the current state of the interface
     * @return true if the interface is ready
     */
    const bool& isReady() const;

    /**
     * Returns the number of digital user outputs, which can be set by the
     * camera. Might be zero for some cameras. The size affects the number of
     * 'set' ros-services the camera node will provide
     * @return number of digital user outputs
     */
    std::size_t numUserOutputs() const;

    /**
     * Returns the image size in bytes
     * @return the image size in bytes
     */
    const size_t& imageSize() const;

    /**
     * Get the maximum achievable frame rate
     * @return float
     */
    virtual float maxPossibleFramerate() = 0;

    /**
     * Checks if the camera has the auto exposure feature.
     * @return true if the camera supports auto exposure.
     */
    const bool& hasAutoExposure() const;

    /**
     * Max allowed delta between target and reached brightness
     * @return the allowed tolerance.
     */
    const float& maxBrightnessTolerance() const;

    /**
     * Getter for the sequencer exposure times.
     * @return the list of exposure times
     */
    const std::vector<float>& sequencerExposureTimes() const;

    /**
     * set the x-axis, y-axis offset value
     * @param offsetValue the offset value in the x-axis.
     * @param xAxis if true set the offset in the x-axis otherwise in y-axis.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setOffsetXY(const int& offsetValue, bool xAxis) = 0;

    /**
     * Reverse the image around x-axis and/or y-axis
     * @param data activate / diactivate reversing of the image.
     * @param around_x true-> reverse around x-axis , false-> reverse around y-axis
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string reverseXY(const bool& data, bool around_x) = 0;

    /**
     * Get current status of reverse X/Y
     * @param returnX true-> return reverse status of x-axis, false->return reverse status of y-axis
     * @return current reverse status of x or y
     */
    virtual bool getReverseXY(const bool& returnX) = 0;

    /**
     * Set the image black level
     * @param data the new black level.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setBlackLevel(const int& data) = 0;

    /**
     * Get the image black level
     * @return current black level
     */
    virtual int getBlackLevel() = 0;

    /**
     * Set the PGI mode
     * @param on : true = on, false = off.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setPGIMode(const bool& on) = 0;

    /**
     * Get the PGI mode
     * @return -3 = Unknown, -2 = Error, -1 = Not available, 0 = Off, 1 = On
     */
    virtual int getPGIMode() = 0;

    /**
     * Set the demosaicing mode
     * @param mode : 0 = simple, 1 = Basler PGI.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setDemosaicingMode(const int& mode) = 0;

    /**
     * Get current demosaicing mode status
     * @return -3 = Unknown, -2 = Error, -1 = Not available, 0 = Simple, 1 = BaslerPGI
     */
    virtual int getDemosaicingMode() = 0;

    /**
     * Set the noise reduction value
     * @param value : targeted noise reduction value (range : 0.0 to 0.2).
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setNoiseReduction(const float& value) = 0;

    /**
     * get current noise reduction value
     * @return current noise reduction value, -20000.0 = Error, -10000.0 = Not available
     */
    virtual float getNoiseReduction() = 0;

    /**
     * Set the sharpness enhancement value.
     * @param value : targeted sharpness enhancement value (range : 1.0 to 3.98438).
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setSharpnessEnhancement(const float& value) = 0;

    /**
     * Get current sharpness enhancement value.
     * @return current sharpness enhancement value, -20000.0 = Error, -10000.0 = Not available
     */
    virtual float getSharpnessEnhancement() = 0;

    /**
     * set light source preset
     * @param value : 0 = off, 1 = Daylight5000K, 2 = Daylight6500K, 3 = Tungsten2800K
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setLightSourcePreset(const int& mode) = 0;

    /**
     * get current light source preset
     * @return -3 = Unknown, -2 = Error, -1 = Not available, 0 = Off, 1 = Daylight5000K, 2 = Daylight6500K, 3 = Tungsten2800K
     */
    virtual int getLightSourcePreset() = 0;

    /**
     * set the camera balance white auto
     * @param mode : 0 = off , 1 = once, 2 = continuous
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setBalanceWhiteAuto(const int& mode) = 0;

    /**
     * get curent camera balance white auto status
     * @return -3 = Unknown, -2 = Error, -1 = Not available, 0 = Off, 1 = Once, 2 = Continuous
     */
    virtual int getBalanceWhiteAuto() = 0;

     /**
     * set the sensor readout mode
     * @param mode : 0 = normal , 1 = fast.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setSensorReadoutMode(const int& mode) = 0;

     /**
     * get current sensor readout mode status
     * @return -3 = Unknown, -2 = Error, -1 = Not available, 0 = Normal, 1 = Fast
     */
    virtual int getSensorReadoutMode() = 0;

     /**
     * set the acquisition frame count
     * @param frameCount targeted frame count.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setAcquisitionFrameCount(const int& frameCount) = 0 ;

     /**
     * get current acquisition frame count
     * @return current acquisition frame count value, -20000.0 = Error, -10000.0 = Not available
     */
    virtual int getAcquisitionFrameCount() = 0 ;

     /**
     * set the trigger selector
     * @param mode : 0 = Frame Start, 1 = Frame Burst Start (ace USB) / Acquisition Start (ace GigE)
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setTriggerSelector(const int& mode) = 0 ;

     /**
     * get current trigger selector status
     * @return -3 = Unknown, -2 = Error, -1 = Not available, 0 = FrameStart, 1 = FrameBurstStart(USB)/AcquisitionStart(GigE)
     */
    virtual int getTriggerSelector() = 0 ;

    /**
     * set the trigger mode
     * @param value : false = off, true = on
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setTriggerMode(const bool& value) = 0 ;

    /**
     * get current trigger mode
     * @return true when on, otherwise false
     */
    virtual int getTriggerMode() = 0 ;

    /**
     * execute a software trigger
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string executeSoftwareTrigger() = 0;

    /**
     * set camera trigger source
     * @param source : 0 = software, 1 = Line1, 2 = Line2, 3 = Line3, 4 = Line4, 5 = Action1, 6 = PeriodicSignal1 (only selected GigE Camera and Blaze)
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setTriggerSource(const int& source) = 0;

    /**
     * get current camera trigger source
     * @return -3 = Unknown, -2 = Error, -1 = Not available, 0 = Software, 1 = Line1, 2 = Line3, 3 = Line4, 4 = Action1(only selected GigE Camera and Blaze)
     */
    virtual int getTriggerSource() = 0;

    /**
     * set camera trigger activation type
     * @param value : 0 = RigingEdge, 1 = FallingEdge
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setTriggerActivation(const int& value) = 0;

    /**
     * get current camera trigger activation type
     * @return -3 = Unknown, -2 = Error, -1 = Not available, 0 = RisingEdge, 1 = FallingEdge
     */
    virtual int getTriggerActivation() = 0;

    /**
     * set camera trigger delay value
     * @param delayValue required dely value in µs
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setTriggerDelay(const float& delayValue) = 0;

    /**
     * get current camera trigger delay value
     * @return current trigger delay value, -20000.0 = Error, -10000.0 = Not available
     */
    virtual float getTriggerDelay() = 0;

    /**
     * set camera line selector
     * @param value : 0 = line1, 1 = line2, 2 = line3, 3 = line4
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setLineSelector(const int& value) = 0;

    /**
     * set camera line mode
     * @param value : 0 = input, 1 = output
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setLineMode(const int& value) = 0;

    /**
     * set camera line source
     * @param value : 0 = exposure active
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setLineSource(const int& value) = 0;

    /**
     * set device link throughput limit mode
     * @param value : ture = invert line
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setLineInverter(const bool& value) = 0;

    /**
     * set camera line debouncer time
     * @param value delay time in microseconds
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setLineDebouncerTime(const float& value) = 0;

    /**
     * set camera user set selector
     * @param set : 0 = Default, 1 = UserSet1, 2 = UserSet2, 3 = UserSet3, 4 = HighGain, 5 = AutoFunctions, 6 = ColorRaw
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setUserSetSelector(const int& set) = 0;

    /**
     * get current selected camera user set
     * @return -3 = Unknown, -2 = Error, -1 = Not available, 0 = Default, 1 = UserSet1, 2 = UserSet2, 3 = UserSet3, 4 = HighGain, 5 = AutoFunctions, 6 = ColorRaw
     */
    virtual int getUserSetSelector() = 0;

    /**
     * save user set
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string saveUserSet() = 0;

    /**
     * load user set
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string loadUserSet() = 0;

    /**
     * get camera configuration as pfs
     * @return  A pair of strings. First string: error message if an error occurred or done message otherwise. Second string: camera configuration as pfs (if first is done).
     */
    virtual std::pair<std::string, std::string> getPfs() = 0;

    /**
     * save pfs file
     * @param fileName : Path to the pfs file to be saved
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string savePfs(const std::string& fileName) = 0;

    /**
     * load pfs file
     * @param fileName : Path to the pfs file to be loaded
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string loadPfs(const std::string& fileName) = 0;

    /**
     * set camera user set default selector
     * @param set : 0 = Default, 1 = UserSet1, 2 = UserSet2, 3 = UserSet3, 4 = HighGain, 5 = AutoFunctions, 6 = ColorRaw
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setUserSetDefaultSelector(const int& set) = 0;

    /**
     * get current camera user set default selector
     * @return -3 = Unknown, -2 = Error, -1 = Not available, 0 = Default, 1 = UserSet1, 2 = UserSet2, 3 = UserSet3, 4 = HighGain, 5 = AutoFunctions, 6 = ColorRaw
     */
    virtual int getUserSetDefaultSelector() = 0;

    /**
     * set device link throughput limit mode
     * @param turnOn : true = on , false = Off
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setDeviceLinkThroughputLimitMode(const bool& turnOn) = 0;

    /**
     * get current device link throughput limit mode
     * @return true when on, flase otherwise
     */
    virtual int getDeviceLinkThroughputLimitMode() = 0;

    /**
     * set device link throughput limit
     * @param limit : device link throughput limit  Bytes/second
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setDeviceLinkThroughputLimit(const int& limit) = 0;

    /**
     * reset the camera device
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string triggerDeviceReset() = 0;

    /**
     * start camera aqcuisition
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string grabbingStarting() const = 0;

    /**
     * stop camera aqcuisition
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string grabbingStopping() = 0;

    /**
     * set the camera Maximum USB data transfer size in bytes
     * @param maxTransferSize targeted new camera Maximum USB data transfer size in bytes
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setMaxTransferSize(const int& maxTransferSize) = 0 ;

    /**
     * set the camera Gamma selector (GigE Camera only)
     * @param gammaSelector : 0 = User, 1 = sRGB
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setGammaSelector(const int& gammaSelector) = 0;

    /**
     * enable/disable the camera Gamma (GigE Camera only)
     * @param enable
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string gammaEnable(const bool& enable) = 0;

    /**
     * returns the current internal camera temperature
     * @return 0.0 if error or unknown
     */
    virtual float getTemperature() = 0;

    /**
     * manual correction of the color shifts so that white objects appear white in images acquired.
     * The increase or decrease in intensity is proportional. For example, if the balance ratio for a color is set to 1.2, the intensity of that color is increased by 20 %
     * @param redValue : balancd ratio of red channel
     * @param greenValue : balancd ratio of green channel
     * @param blueValue : balancd ratio of blue channel
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setWhiteBalance(const double& redValue, const double& greenValue, const double& blueValue) = 0;

    /**
     * set the camera grapping strategy
     * @param strategy : 0 = GrabStrategy_OneByOne, 1 = GrabStrategy_LatestImageOnly, 2 = GrabStrategy_LatestImages
     * @return error message if an error occurred or done message otherwise.
     */
    virtual bool setGrabbingStrategy(const int& strategy) = 0;

    /**
     * set The size of the grab result buffer output queue
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setOutputQueueSize(const int& size) = 0;

    /**
     * Set the maximum number of buffers that can be used simultaneously for grabbing images - Applies to: BCON, GigE, USB and blaze.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setMaxNumBuffer(const int& size) = 0;

    /**
     * Return the maximum number of buffers that can be used simultaneously for grabbing images - Applies to: BCON, GigE, USB and blaze.
     * @return maximum number of buffers or -1/-2 if an error occurred.
     */
    virtual int getMaxNumBuffer() = 0;

    /**
     * Return the GigE cameras: Number of frames received Other cameras: Number of buffers processed - Applies to: BCON, GigE, USB and blaze.
     * @return value or -1/-2 if an error occurred.
     */
    virtual int getStatisticTotalBufferCount() = 0;

    /**
     * Return the GigE cameras: Number of buffers with at least one failed packet. A packet is considered failed if its status is not 'success'. Other cameras: Number of buffers that returned an error.
     * @return value or -1/-2 if an error occurred.
     */
    virtual int getStatisticFailedBufferCount() = 0;

    /**
     * Return the Number of frames lost because there were no buffers in the queue - Applies to: GigE and blaze.
     * @return value or -1/-2 if an error occurred.
     */
    virtual int getStatisticBufferUnderrunCount() = 0;

    /**
     * Return the Number of failed packets, i.e., the number of packets whose status is not 'success'.
     * @return value or -1/-2 if an error occurred.
     */
    virtual int getStatisticFailedPacketCount() = 0;

    /**
     * Return the Number of emitted packet resend commands sent - Applies to: GigE and blaze.
     * @return value or -1/-2 if an error occurred.
     */
    virtual int getStatisticResendRequestCount() = 0;

    /**
     * Return the Number of corrupt or lost frames between successfully grabbed images - Applies to: BCON and USB.
     * @return value or -1/-2 if an error occurred.
     */
    virtual int getStatisticMissedFrameCount() = 0;

    /**
     * Return the Number of stream resynchronizations - Applies to: USB.
     * @return value or -1/-2 if an error occurred.
     */
    virtual int getStatisticResynchronizationCount() = 0;

    /**
     * Enable/Disable the chunk mode - Applies to: GigE, ace 2 GigE, ace 2 USB and ace USB.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setChunkModeActive(const bool& enable) = 0;

        /**
     * Enable/Disable the chunk mode - Applies to: GigE, ace 2 GigE, ace 2 USB and ace USB.
     * @return error code message if an error occurred or value message otherwise.
     */
    virtual int getChunkModeActive() = 0;

    /**
     * Sets which chunk can be enabled - Applies to: GigE, ace 2 GigE, ace 2 USB and ace USB.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setChunkSelector(const int& value) = 0;

    /**
     * Sets which chunk can be enabled - Applies to: GigE, ace 2 GigE, ace 2 USB and ace USB.
     * @return error code message if an error occurred or value message otherwise.
     */
    virtual int getChunkSelector() = 0;

    /**
     * Includes the currently selected chunk in the payload data - Applies to: GigE, ace 2 GigE, ace 2 USB and ace USB.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setChunkEnable(const bool& enable) = 0;

    /**
     * Includes the currently selected chunk in the payload data - Applies to: GigE, ace 2 GigE, ace 2 USB and ace USB.
     * @return error code message if an error occurred or value message otherwise.
     */
    virtual int getChunkEnable() = 0;

    /**
     * Value of the timestamp when the image was acquired - Applies to: GigE and ace USB.
     * @return error code message if an error occurred or value message otherwise.
     */
    virtual int64_t getChunkTimestamp() = 0;

    /**
     * Value of the Exposure time used to acquire the image - Applies to: GigE, ace 2 GigE, ace 2 USB and ace USB.
     * @return error code message if an error occurred or value message otherwise.
     */
    virtual float getChunkExposureTime() = 0;

    /**
     * Value of the Exposure time used to acquire the image - Applies to: GigE, ace 2 GigE, ace 2 USB and ace USB.
     * @return error  message if an error occurred or done message otherwise.
     */
    virtual std::string setChunkExposureTime(const float& value) = 0;

    /**
     * Bit field that indicates the status of all of the camera's input and output lines when the image was acquired - Applies to: GigE, ace 2 GigE, ace 2 USB and ace USB.
     * @return error  message if an error occurred or done message otherwise.
     */
    virtual int64_t getChunkLineStatusAll() = 0;

    /**
    * Value of the Frame counter when the image was acquired - Applies to: GigE.
    * @return error  message if an error occurred or done message otherwise.
    */
    virtual int64_t getChunkFramecounter() = 0;

    /**
    * Value of the selected chunk counter - Applies to: ace 2 GigE, ace 2 USB and ace USB.
    * @return error  message if an error occurred or done message otherwise.
    */
    virtual int64_t getChunkCounterValue() = 0;

    /**
    * Set timer selector - Applies to: GigE, ace 2 GigE, ace 2 USB and ace USB.
    * @return error  message if an error occurred or done message otherwise.
    */
    virtual std::string setTimerSelector(const int& selector) = 0;

    /**
    * Service callback for setting the internal camera signal used to trigger the selected timer - Applies to: GigE, ace 2 GigE, ace 2 USB, ace USB and dart 2 USB.
    * @return error  message if an error occurred or done message otherwise.
    */
    virtual std::string setTimerTriggerSource(const int& source) = 0;

    /**
    * Set timer duration - Applies to: ace 2 GigE, ace 2 USB, ace USB and dart 2 USB.
    * @return error  message if an error occurred or done message otherwise.
    */
    virtual std::string setTimerDuration(const float& duration) = 0;

    /**
     * Sets the PTP priority - Applies to: ace 2 GigE.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setPTPPriority(const int& value) = 0;

    /**
     * Sets the PTP profile - Applies to: ace 2 GigE.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setPTPProfile(const int& value) = 0;

    /**
     * Sets the PTP network mode - Applies to: ace 2 GigE.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setPTPNetworkMode(const int& value) = 0;

    /**
     * Sets the PTP unicast port address index - Applies to: ace 2 GigE.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setPTPUCPortAddressIndex(const int& value) = 0;

    /**
     * Sets the PTP unicast port address - Applies to: ace 2 GigE.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setPTPUCPortAddress(const int& value) = 0;

    /**
     * Length of the periodic signal in microseconds - Applies to: ace 2 GigE.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setPeriodicSignalPeriod(const float& value) = 0;

    /**
     * Delay to be applied to the periodic signal in microseconds - Applies to: ace 2 GigE.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setPeriodicSignalDelay(const float& value) = 0;

    /**
     * Low 32 bits of the synchronous free run trigger start time - Applies to: GigE and blaze.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setSyncFreeRunTimerStartTimeLow(const int& value) = 0;

    /**
     * High 32 bits of the synchronous free run trigger start time - Applies to: GigE and blaze.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setSyncFreeRunTimerStartTimeHigh(const int& value) = 0;

    /**
     * Synchronous free run trigger rate - Applies to: GigE.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setSyncFreeRunTimerTriggerRateAbs(const float& value) = 0;

    /**
     * Enables/Disables PTP management protocol - Applies to: ace 2 GigE.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string enablePTPManagementProtocol(const bool& value) = 0;

    /**
     * Enables/Disables PTP two step operation - Applies to: ace 2 GigE.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string enablePTPTwoStepOperation(const bool& value) = 0;

    /**
     * Enables/Disables PTP - Applies to: GigE, ace 2 GigE.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string enablePTP(const bool& value) = 0;

    /**
     * Enables the synchronous free run mode - Applies to: GigE and blaze.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string enableSyncFreeRunTimer(const bool& value) = 0;

    /**
     * Updates synchronous free run settings - Applies to: GigE and blaze.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string updateSyncFreeRunTimer() = 0;

    /**
     * Set the action trigger configuration of the camera. Needed when issuing scheduled or not action commands - Applies to: ace 2 GigE.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setActionTriggerConfiguration(const int& action_device_key, const int& action_group_key, const unsigned int& action_group_mask,
                                                      const int& registration_mode, const int& cleanup) = 0;

    /**
     * Issue an action command via broadcast - Applies to: GigE, ace 2 GigE.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string issueActionCommand(const int& device_key, const int& group_key, const unsigned int& group_mask, const std::string& broadcast_address) = 0;

    /**
     * Issue a scheduled action command via broadcast - Applies to: GigE, ace 2 GigE.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string issueScheduledActionCommand(const int& device_key, const int& group_key, const unsigned int& group_mask, const int64_t& action_time_ns_from_current_timestamp, const std::string& broadcast_address) = 0;

    /**
     * Set depth min - Applies to: blaze.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setDepthMin(const int& depth_min) = 0;

    /**
     * Set depth max - Applies to: blaze.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setDepthMax(const int& depth_max) = 0;

    /**
     * Set temporal filter strength - Applies to: blaze.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setTemporalFilterStrength(const int& strength) = 0;

    /**
     * Set outlier removal threshold - Applies to: blaze.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setOutlierRemovalThreshold(const int& threshold) = 0;

    /**
     * Set outlier removal tolerance - Applies to: blaze.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setOutlierRemovalTolerance(const int& tolerance) = 0;

    /**
     * Set ambiguity filter threshold - Applies to: blaze.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setAmbiguityFilterThreshold(const int& threshold) = 0;

    /**
     * Set confidence threshold - Applies to: blaze.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setConfidenceThreshold(const int& threshold) = 0;

    /**
     * Set intensity calculation - Applies to: blaze.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setIntensityCalculation(const int& calculation) = 0;

    /**
     * Set exposure time selector - Applies to: blaze.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setExposureTimeSelector(const int& selector) = 0;

    /**
     * Set operating mode - Applies to: blaze.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setOperatingMode(const int& mode) = 0;

    /**
     * Set multi camera channel - Applies to: blaze.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setMultiCameraChannel(const int& channel) = 0;

    /**
     * Set acquisition frame rate - Applies to: blaze.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setAcquisitionFrameRate(const float& framerate) = 0;

    /**
     * Set scan 3d calibration offset - Applies to: blaze.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setScan3dCalibrationOffset(const float& offset) = 0;

    /**
     * Enable/Disable spatial filter - Applies to: blaze.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string enableSpatialFilter(const bool& enable) = 0;

    /**
     * Enable/Disable temporal filter - Applies to: blaze.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string enableTemporalFilter(const bool& enable) = 0;

    /**
     * Enable/Disable outlier removal - Applies to: blaze.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string enableOutlierRemoval(const bool& enable) = 0;

    /**
     * Enable/Disable ambiguity filter - Applies to: blaze.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string enableAmbiguityFilter(const bool& enable) = 0;

    /**
     * Enable/Disable thermal drift correction - Applies to: blaze.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string enableThermalDriftCorrection(const bool& enable) = 0;

    /**
     * Enable/Disable distortion correction - Applies to: blaze.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string enableDistortionCorrection(const bool& enable) = 0;

    /**
     * Enable/Disable acquisition framerate - Applies to: blaze.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string enableAcquisitionFrameRate(const bool& enable) = 0;

    /**
     * Enable/Disable HDR mode - Applies to: blaze.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string enableHDRMode(const bool& enable) = 0;

    /**
     * Enable/Disable fast mode - Applies to: blaze.
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string enableFastMode(const bool& enable) = 0;

    virtual ~PylonROS2Camera();

protected:
    /**
     * Protected default constructor.
     */
    explicit PylonROS2Camera();

    /**
     * Enables the extended brightness search.
     * @param brightness target brightness
     * @return true after reaching the target brightness.
     */
    virtual bool setExtendedBrightness(const int& target_brightness,
                                       const float& current_brightness) = 0;

    /**
     * Parameters for the extended brightness search
     */
    BinaryExposureSearch* binary_exp_search_;

    /**
     * The DeviceUserID of the found camera
     */
    std::string device_user_id_;

    /**
     * Number of image rows.
     */
    size_t img_rows_;

    /**
     * Number of image columns.
     */
    size_t img_cols_;

    /**
     * The size of the image in number of bytes.
     */
    size_t img_size_byte_;

    /**
     * The max time a single grab is allowed to take. This value should always
     * be greater then the max possible exposure time of the camera
     */
    float grab_timeout_;

    /**
     * Flag which is set in case that the grab-result-pointer of the first
     * acquisition contains valid data
     */
    bool is_ready_;

    /**
     * Camera trigger timeout in ms
     */
    int trigger_timeout_;

    /**
     * Camera grab strategy
     * 0 = GrabStrategy_OneByOne
     * 1 = GrabStrategy_LatestImageOnly
     * 2 = GrabStrategy_LatestImages
     */
    int grab_strategy_;

    /**
     * True if the extended binary exposure search is running.
     */
    bool is_binary_exposure_search_running_;

    /**
     * Max allowed delta between target and reached brightness
     */
    const float max_brightness_tolerance_;

    /**
     * Exposure times to use when in sequencer mode.
     */
    std::vector<float> seq_exp_times_;

    /**
     * Vector containing all available user outputs.
     */
    std::vector<int> user_output_selector_enums_;

    /**
     * Vector that contains the available image_encodings the camera supports.
     * The strings describe the GenAPI encoding.
     */
    std::vector<std::string> available_image_encodings_;
};

}  // namespace pylon_ros2_camera

