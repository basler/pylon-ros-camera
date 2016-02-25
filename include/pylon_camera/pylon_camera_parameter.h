// Copyright 2015 <Magazino GmbH>

#ifndef PYLON_CAMERA_PYLON_CAMERA_PARAMETER_H
#define PYLON_CAMERA_PYLON_CAMERA_PARAMETER_H

#include <string>
#include <vector>
#include <ros/ros.h>

namespace pylon_camera
{

enum SHUTTER_MODE
{
    SM_ROLLING = 0,
    SM_GLOBAL = 1,
    SM_GLOBAL_RESET_RELEASE = 2,
    SM_DEFAULT =  -1,
};

/**
 * Parameter class for the PylonCamera
 */
class PylonCameraParameter
{
public:
    PylonCameraParameter();

    virtual ~PylonCameraParameter();

    /**
     * Read the parameters from the parameter server
     * @param nh the ros::NodeHandle to use
     * @return true if the values on the parameter server are valid
     */
    bool readFromRosParameterServer(const ros::NodeHandle& nh);

    /**
     * Getter for the device_user_id_ set from ros-parameter server
     */
    const std::string& deviceUserID() const;

    /**
     * Getter for the frame_rate_ read from ros-parameter server
     */
    const double& frameRate() const;

    /**
     * Setter for the frame_rate_ initially set from ros-parameter server
     * The frame rate needs to be updated with the value the camera supports
     */
    void setFrameRate(const ros::NodeHandle& nh, const double& frame_rate);

    /**
     * Getter for the camera_frame_ set from ros-parameter server
     */
    const std::string& cameraFrame() const;

    /**
     * Getter for the string describing the shutter mode
     */
    std::string shutterModeString() const;

public:
    /**
     * Binning factor to get downsampled images. This factor will be applied
     * to the image width as well as the image height
     */
    int binning_;


    // #######################################################################
    // ###################### Image Intensity Settings  ######################
    // #######################################################################
    /**
     * The exposure time in microseconds after opening the camera.
     * This value can be overriden from the brightness search, in case that
     * the flag exposure_fixed is not true
     */
    double exposure_;

    /**
     * If the exposure_fixed flag is set, the exposure time will stay fix in
     * case of a brightness search. Hence the target brightness will be reached
     * only by varying the gain
     */
    bool exposure_fixed_;

    /**
     * Flag which indicates if the exposure time is provided and hence should
     * be set during startup
     */
    bool exposure_given_;

    /** The average intensity value of the image. It depends the exposure time
     * as well as the gain setting. The interface will only try to reach this
     * value if the brightness_enabled flag is set to true.
     */
    int brightness_;

    /**
     * The brightness_continuous flag controls the auto brightness function.
     * If it is set to false, the given brightness will only be reached once.
     * Hence changing light conditions lead to changing brightness values.
     * If it is set to true, the given brightness will be reached continuously,
     * trying to adapt to changing light conditions
     */
    bool brightness_continuous_;

    /**
     * Flag which indicates if the average brightness is provided and hence
     * should be set during startup
     */
    bool brightness_given_;

    /**
     * The target gain in percent of the maximal value the camera supports
     * For USB-Cameras, the gain is in dB, for GigE-Cameras it is given in so
     * called 'device specific units'. This value can be overriden from the
     * brightness search, in case that the gain_fixed flag is set to false
     */
    double gain_;

    /**
     * If the gain_fixed flag is set, the gain value will stay fix in
     * case of a brightness search. Hence the target brightness will be reached
     * only by varying the exposure
     */
    bool gain_fixed_;

    /**
     * Flag which indicates if the gain value is provided and hence should be
     * set during startup
     */
    bool gain_given_;

    /**
     * Gamma correction of pixel intensity.
     * Adjusts the brightness of the pixel values output by the camera's sensor
     * to account for a non-linearity in the human perception of brightness or
     * of the display system (such as CRT).
     */
    double gamma_;

    /**
     * Flag which indicates if the gamma correction valus is provided and
     * hence should be set during startup
     */
    bool gamma_given_;
    // #######################################################################

    /**
     * The MTU size. Only used for GigE cameras.
     * To prevent lost frames configure the camera has to be configured
     * with the MTU size the network card supports. A value greater 3000
     * should be good (1500 for RaspberryPI)
     */
    int mtu_size_;

    /**
      Shutter mode
    */
    SHUTTER_MODE shutter_mode_;

protected:
    /**
     * Validates the parameter set found on the parameter server
     * @param nh the ros::NodeHandle to use
     * @return true if the parameter set is valid
     */
    bool validateParameterSet(const ros::NodeHandle& nh);

    /**
     * The tf frame under which the images were published
     */
    std::string camera_frame_;

    /**
     * The DeviceUserID of the camera. If empty, the first camera found in the
     * device list will be used
     */
    std::string device_user_id_;

    /**
     * The desired publisher frame rate if listening to the topics.
     * This paramter can only be set once at startup
     * Calling the GrabImages-Action can result in a higher framerate
     */
    double frame_rate_;

};

}  // namespace pylon_camera

#endif // PYLON_CAMERA_PYLON_CAMERA_PARAMETER_H
