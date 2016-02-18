// Copyright 2015 <Magazino GmbH>

#ifndef PYLON_CAMERA_PYLON_CAMERA_H
#define PYLON_CAMERA_PYLON_CAMERA_H

#include <string>
#include <vector>

#include <pylon_camera/pylon_camera_parameter.h>
#include <pylon_camera/binary_exposure_search.h>

namespace pylon_camera
{

/**
 * The PylonCamera base class. Create a new instance using the static create() functions.
 */
class PylonCamera
{
public:
    virtual ~PylonCamera();

    /**
     * Create a new PylonCamera instance. It will return the first camera that could be found.
     * @return new PylonCamera instance or NULL if no camera was found.
     */
    static PylonCamera* create();

    /**
     * Create a new PylonCamera instance based on the DeviceUserID of the camera.
     * @param device_user_id Pylon DeviceUserID. If the string is empty, the first camera that could be found is returned.
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
     * @return true if the camera could be opend.
     */
    virtual bool openCamera() = 0;

    /**
     * Configures the camera according to the provided ros parameters.
     * @param parameters The PylonCameraParameter set to use
     * @return true if all parameters could be sent to the camera.
     */
    virtual bool applyStartupSettings(const PylonCameraParameter& parameters) = 0;

    /**
     * Configure the sequencer exposure times.
     * @param exposure_times the list of exposure times.
     * @return true if all parameters could be sent to the camera.
     */
    virtual bool setupSequencer(const std::vector<float>& exposure_times) = 0;

    /**
     * @brief sets shutter mode for the camera (rolling or global_reset)
     * @param mode
     * @return
     */
    virtual bool setShutterMode(const pylon_camera::SHUTTER_MODE& mode) = 0;

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
     * Sets the exposure time in microseconds
     * Setting the exposure time to -1.0 enables the AutoExposureContinuous mode.
     * Setting the exposure time to  0.0 disables the AutoExposure function.
     * Setting the exposure time to a value greater than zero disables the auto exposure feature.
     * @param target_exposure the desired exposure time to set in microseconds.
     * @param reached_exposure time in microseconds
     * @return false if a communication error occurred or true otherwise.
     */
    virtual bool setExposure(const float_t& target_exposure, float_t& reached_exposure) = 0;

    /**
     * Sets the gain in percent independent of the camera type
     * @param gain gain in percent.
     * @return false if a communication error occurred or true otherwise.
     */
    virtual bool setGain(const double& target_gain_percent) = 0;

    /**
     * Sets the target brightness
     * Setting the exposure time to -1 enables the AutoExposureContinuous mode.
     * Setting the exposure time to  0 disables the AutoExposure function.
     * If the target exposure time is not in the range of Pylon's auto target brightness range
     * the extended brightness search is started.
     * @param brightness target brightness. Range is [-1...255].
     * @return false if a communication error occurred or true otherwise.
     */
    virtual bool setBrightness(const int& target_brightness, const float_t& current_brightness) = 0;


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
     * the desired brightness is reached or a timeout occoured
     */
    virtual void disableAllRunningAutoBrightessFunctions() = 0;

    /**
     * Get the camera image encoding according to sensor_msgs::image_encodings
     * Currently, only mono8 cameras are supported.
     * @return the image encoding.
     */
    virtual std::string imageEncoding() const = 0;

    /**
     * Get the number of bytes per pixel
     * @return number of bytes per pixel
     */
    virtual int imagePixelDepth() const = 0;

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
     * @brief setUserOutput sets the digital output
     * @param output_id
     * @param value goal value for output
     * @return true if value was set
     */
    virtual bool setUserOutput(const int& output_id, const bool& value) = 0;

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
     * Returns the current state of the interface
     * @return true in case that the grab-result-pointer of the first acquisition contains valid data
     */
    const bool& isReady() const;

    /**
     * Returns the image size in bytes
     * @return the image size in bytes
     */
    const size_t& imageSize() const;

    /**
     * Get the maximum achievable frame rate
     * @return float
     */
    const float& maxPossibleFramerate() const;

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
     * Returns the connection state of the camera device.
     * @return true if the camera device removal from the PC has been detected.
     */
    const bool& isCamRemoved() const;

    /**
     * Getter for the sequencer exposure times.
     * @return the list of exposure times
     */
    const std::vector<float>& sequencerExposureTimes() const;

protected:
    /**
     * Protected default constructor.
     */
    PylonCamera();

    /**
     * Enables the extended brightness search.
     * @param brightness target brightness
     * @return true after reaching the target brightness. If false, recall the method until the method returns true.
     */
    virtual bool setExtendedBrightness(const int& target_brightness, const float& current_brightness) = 0;

    /**
     * Parameters for the extended brightness search
     */
    BinaryExposureSearch* binary_exp_search_;

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
     * The maximum achievable frame rate reported by the camera
     */
    float max_framerate_;

    /**
     * The max time a single grab is allwed to take. This value should always
     * be greater then the max possible exposure time of the camera
     */
    float grab_timeout_;

    /**
     *
     * Boolean to store if auto exposure is possible.
     */
    bool has_auto_exposure_;

    /**
     * Flag which is set in case that the grab-result-pointer of the first acquisition contains valid data
     */
    bool is_ready_;

    /**
     * True if the camera device removal from the PC has been detected.
     */
    bool is_cam_removed_;

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
};

}  // namespace pylon_camera

#endif  // PYLON_CAMERA_PYLON_CAMERA_H
