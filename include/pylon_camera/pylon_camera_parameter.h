#ifndef PYLON_CAMERA_PARAMETER_H_
#define PYLON_CAMERA_PARAMETER_H_

#include <string>
#include <vector>
#include <ros/ros.h>

namespace pylon_camera
{

class PylonCameraParameter
{
public:
    PylonCameraParameter();
    virtual ~PylonCameraParameter();

    bool writeToYamlFile(std::string& yaml_string);
    bool readFromYamlFile(const std::string& yaml_string);

    bool readFromRosParameterServer(ros::NodeHandle& nh);

    bool validateParameterSet(ros::NodeHandle& nh);

    std::string magazino_cam_id_;
    std::string camera_frame_;
    double desired_frame_rate_;
    double target_exposure_;
    int param_update_frequency_;
    double start_exposure_;
    bool use_brightness_;
    int start_brightness_;
    std::vector<float> desired_seq_exp_times_;
    int mtu_size_;
};

}

#endif
