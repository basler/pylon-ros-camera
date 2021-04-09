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

#ifndef PYLON_CAMERA_PYLON_CAMERA_H
#define PYLON_CAMERA_PYLON_CAMERA_H

#include <string>
#include <vector>
#include <map>

#include <pylon_camera/pylon_camera_parameter.h>
#include <pylon_camera/binary_exposure_search.h>
#include <sensor_msgs/RegionOfInterest.h>

namespace pylon_camera
{

// Number of channels for image encoding
#define CHANNEL_MONO8 1
#define CHANNEL_RGB8  3

/**
 * The PylonCamera base class. Create a new instance using the static create() functions.
 */
class PylonCamera
{
public:

    /**
     * Create a new PylonCamera instance. It will return the first camera that could be found.
     * @return new PylonCamera instance or NULL if no camera was found.
     */
    static PylonCamera* create();

    /**
     * Create a new PylonCamera instance based on the DeviceUserID of the camera.
     * @param device_user_id Pylon DeviceUserID. If the string is empty, the
     * first camera that could be found is returned.
     * @return new PylonCamera instance or NULL if the camera was not found.
     */
    static PylonCamera* create(const std::string& device_user_id);

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
     * This will use the device specific parameters as e.g. the mtu size for
     * GigE-Cameras
     * @param parameters The PylonCameraParameter set to use
     * @return true if all parameters could be sent to the camera.
     */
    virtual bool applyCamSpecificStartupSettings(const PylonCameraParameter& parameters) = 0;

    /**
     * Initializes the internal parameters of the PylonCamera instance.
     * @param parameters The PylonCameraParameter set to use
     * @return true if all parameters could be sent to the camera.
     */
    virtual bool startGrabbing(const PylonCameraParameter& parameters) = 0;

    /**
     * Grab a camera frame and copy the result into image
     * @param image reference to the output image.
     * @return true if the image was grabbed successfully.
     */
    virtual bool grab(std::vector<uint8_t>& image) = 0;

    /**
     * Grab a camera frame and copy the result into image
     * @param image pointer to the image buffer.
     *              Caution: Make sure the buffer is initialized correctly!
     * @return true if the image was grabbed successfully.
     */
    virtual bool grab(uint8_t* image) = 0;

    /**
     * @brief sets shutter mode for the camera (rolling or global_reset)
     * @param mode
     * @return
     */
    virtual bool setShutterMode(const pylon_camera::SHUTTER_MODE& mode) = 0;

    /**
     * Update area of interest in the camera image
     * @param target_roi the target roi
     * @param reached_roi the roi that could be set
     * @return true if the targeted roi could be reached
     */
    virtual bool setROI(const sensor_msgs::RegionOfInterest target_roi,
			sensor_msgs::RegionOfInterest& reached_roi) = 0;
    
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
     * @param flash_on_lines map from line e.g. 1 or 2 to a boolean to 
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
     *        will be generated. Hence e.g. output '1' can be accessed via
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
    virtual sensor_msgs::RegionOfInterest currentROI() = 0;
    
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
     * @param source : 0 = software, 1 = Line1, 2 = Line3, 3 = Line4, 4 = Action1(only selected GigE Camera)
     * @return error message if an error occurred or done message otherwise.
     */
    virtual std::string setTriggerSource(const int& source) = 0;

    /**
     * get current camera trigger source  
     * @return -3 = Unknown, -2 = Error, -1 = Not available, 0 = Software, 1 = Line1, 2 = Line3, 3 = Line4, 4 = Action1(Selected Gige)
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



    virtual ~PylonCamera();
protected:
    /**
     * Protected default constructor.
     */
    PylonCamera();

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
    int trigger_timeout;

    /**
     * Camera grab strategy
     * 0 = GrabStrategy_OneByOne
     * 1 = GrabStrategy_LatestImageOnly
     * 2 = GrabStrategy_LatestImages
     */
    int grab_strategy ;

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

}  // namespace pylon_camera

#endif  // PYLON_CAMERA_PYLON_CAMERA_H
