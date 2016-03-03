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
     * Read the parameters from the parameter server.
     * If invalid parameters can be detected, the interface will reset them
     * to the default values.
     * @param nh the ros::NodeHandle to use
     */
    void readFromRosParameterServer(const ros::NodeHandle& nh);

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
    // The following settings do *NOT* have to be set. Each camera has default
    // values which provide an automatic image adjustment
    // If one would like to adjust image brightness, it is not
    // #######################################################################

    /**
     * The exposure time in microseconds to be set after opening the camera.
     */
    double exposure_;

    /**
     * Flag which indicates if the exposure time is provided and hence should
     * be set during startup
     */
    bool exposure_given_;

    /**
     * The target gain in percent of the maximal value the camera supports
     * For USB-Cameras, the gain is in dB, for GigE-Cameras it is given in so
     * called 'device specific units'.
     */
    double gain_;

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

    /**
     * The average intensity value of the images. It depends the exposure time
     * as well as the gain setting. If 'exposure' is provided, the interface
     * will try to reach the desired brightness by only varying the gain.
     * (What may often fail, because the range of possible exposure vaules is
     * many times higher than the gain range).
     * If 'gain' is provided, the interface will try to reach the desired
     * brightness by only varying the exposure time. If gain AND exposure are
     * given, it is not possible to reach the brightness, because both are
     * assumed to be fix.
     */
    int brightness_;

    /**
     * Flag which indicates if the average brightness is provided and hence
     * should be set during startup
     */
    bool brightness_given_;

    /**
     * Only relevant, if 'brightness' is set as ros-parameter:
     * The brightness_continuous flag controls the auto brightness function.
     * If it is set to false, the brightness will only be reached once.
     * Hence changing light conditions lead to changing brightness values.
     * If it is set to true, the given brightness will be reached continuously,
     * trying to adapt to changing light conditions. This is only possible for
     * values in the possible auto range of the pylon API which is
     * e.g. [50 - 205] for acA2500-14um and acA1920-40gm
     */
    bool brightness_continuous_;

    /**
     * Only relevant, if 'brightness' is given as ros-parameter:
     * If the camera should try to reach and / or keep the brightness, hence
     * adapting to changing light conditions, at least one of the following
     * flags must be set. If both are set, the interface will use the profile
     * that tries to keep the  gain at minimum to reduce white noise.
     * The exposure_auto flag indicates, that the desired brightness will
     * be reached by adapting the exposure time.
     * The gain_auto flag indicates, that the desired brightness will be
     * reached by adapting the gain.
     */
    bool exposure_auto_;
    bool gain_auto_;
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
     * Validates the parameter set found on the ros parameter server.
     * If invalid parameters can be detected, the interface will reset them
     * to the default values.
     * @param nh the ros::NodeHandle to use
     */
    void validateParameterSet(const ros::NodeHandle& nh);

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

#endif  // PYLON_CAMERA_PYLON_CAMERA_PARAMETER_H
