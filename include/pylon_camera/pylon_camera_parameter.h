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
    bool readFromRosParameterServer(ros::NodeHandle& nh);

    /**
     * The DeviceUserID of the camera. If empty, the first camera is used.
     */
    std::string device_user_id_;

    /**
     * The tf frame name of the camera
     */
    std::string camera_frame_;

    /**
     * The desired publisher frame rate
     */
    double desired_frame_rate_;

    /**
     * The targeted exposure value
     */
    double target_exposure_;

    /**
     * The targeted gain value in percent
     */
    double target_gain_;

    /**
     * The initial exposure time to start the camera with
     */
    double start_exposure_;

    /**
     * The MTU size. Only used for GigE cameras.
     * To prevent lost frames configure the camera has to be configured
     * with the MTU size the network card supports
     */
    int mtu_size_;

    /**
     * Binning factor to get downsampled images
     */
    int binning_;

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
    bool validateParameterSet(ros::NodeHandle& nh);
};

}  // namespace pylon_camera

#endif  // PYLON_CAMERA_PYLON_CAMERA_PARAMETER_H
