// Copyright 2015 <Magazino GmbH>
#ifndef PYLON_CAMERA_PYLON_CAMERA_H
#define PYLON_CAMERA_PYLON_CAMERA_H

#include <string>
#include <vector>

#include <pylon_camera/pylon_camera_parameter.h>
#include <pylon_camera/exposure_search_parameter.h>

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
     * Configures the camera according to the provided ros parameters.
     * @param parameters The PylonCameraParameter set to use
     * @return true if all parameters could be sent to the camera.
     */
    virtual bool registerCameraConfiguration(const PylonCameraParameter& parameters) = 0;

    /**
     * Configure the sequencer exposure times.
     * @param exposure_times the list of exposure times.
     * @return true if all parameters could be sent to the camera.
     */
    virtual bool setupSequencer(const std::vector<float>& exposure_times) = 0;

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
     * @param image pointer to the image buffer. Caution: Make sure the buffer is initialized correctly!
     * @return true if the image was grabbed successfully.
     */
    virtual bool grab(uint8_t* image) = 0;

    /**
     * Returns the current exposure time in microseconds.
     * @return the exposure time in microseconds.
     */
    virtual float currentExposure() = 0;

    /**
     * Sets the exposure time in microseconds
     * Setting the exposure time to -1.0 enables the AutoExposureContinuous mode.
     * Setting the exposure time to  0.0 disables the AutoExposure function.
     * Setting the exposure time to a value greater than zero disables the auto exposure feature.
     * @param exposure exposure time in microseconds.
     * @return false if a communication error occurred or true otherwise.
     */
    virtual bool setExposure(const double& exposure) = 0;

    /**
     * Sets the target brightness
     * Setting the exposure time to -1 enables the AutoExposureContinuous mode.
     * Setting the exposure time to  0 disables the AutoExposure function.
     * If the target exposure time is not in the range of Pylon's auto target brightness range
     * the extended brightness search is started.
     * @param brightness target brightness. Range is [-1...255].
     * @return false if a communication error occurred or true otherwise.
     */
    virtual bool setBrightness(const int& brightness) = 0;

    /**
     * Enables the extended brightness search.
     * @param brightness target brightness
     * @return true after reaching the target brightness. If false, recall the method until the method returns true.
     */
    virtual bool setExtendedBrightness(int& brightness) = 0;

    /**
     * Initializes the extended brightness search.
     * @param brightness target brightness.
     */
    virtual void setupExtendedBrightnessSearch(const int& brightness) = 0;

    /**
     * Checks if the auto brightness function is enabled.
     * @return true if AutoExposure is set to AutoExposureContinuous.
     */
    virtual bool isAutoBrightnessFunctionRunning() = 0;

    /**
     * Check if the extended brightness search is running
     * @return true if the extended brightness search is running
     */
    const bool& isOwnBrightnessFunctionRunning() const;

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
     * Getter for the image height
     * @return number of rows in the image
     */
    const int& imageRows() const;

    /**
     * Getter for the image width
     * @return number of columns in the image
     */
    const int& imageCols() const;

    /**
     * Returns true if the camera was initialized correctly
     * @return true if the camera was initialized correctly
     */
    const bool& isReady() const;

    /**
     * Returns the image size in bytes
     * @return the image size in bytes
     */
    const int& imageSize() const;

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
     * Returns the connection state of the camera device.
     * @return true if the camera device removal from the PC has been detected.
     */
    const bool& isCamRemoved() const;

    /**
     * Getter for the sequencer exposure times.
     * @return the list of exposure times
     */
    const std::vector<float>& sequencerExposureTimes() const;

    /**
     * Parameters for the extended brightness search
     */
    ExposureSearchParameter exp_search_params_;

protected:
    /**
     * Protected default constructor.
     */
    PylonCamera();

    /**
     * Number of image rows.
     */
    int img_rows_;

    /**
     * Number of image columns.
     */
    int img_cols_;

    /**
     * The size of the image in number of bytes.
     */
    int img_size_byte_;

    /**
     * The maximum achievable frame rate reported by the camera
     */
    float max_framerate_;

    /**
     * Boolean to store if auto exposure is possible.
     */
    bool has_auto_exposure_;

    /**
     * Is the camera initialized?
     */
    bool is_ready_;

    /**
     * True if the camera device removal from the PC has been detected.
     */
    bool is_cam_removed_;

    /**
     * True if the extended brightness search is running.
     */
    bool is_own_brightness_function_running_;

    /**
     * Exposure times to use when in sequencer mode.
     */
    std::vector<float> seq_exp_times_;
};

}  // namespace pylon_camera

#endif  // PYLON_CAMERA_PYLON_CAMERA_H
