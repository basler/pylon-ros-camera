// Copyright 2015 <Magazino GmbH>

#ifndef PYLON_CAMERA_PYLON_CAMERA_PARAMETER_H
#define PYLON_CAMERA_PYLON_CAMERA_PARAMETER_H

#include <string>
#include <vector>
#include <ros/ros.h>

namespace pylon_camera
{

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
     * The initial exposure time to start the camera with
     */
    double start_exposure_;

    /**
     * The desired exposure times if in sequencer mode
     */
    std::vector<float> desired_seq_exp_times_;

    /**
     * the MTU size. Only used for GigE cameras.
     */
    int mtu_size_;

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
