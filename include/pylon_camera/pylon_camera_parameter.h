#ifndef PYLON_CAMERA_PARAMETER_H_
#define PYLON_CAMERA_PARAMETER_H_

#include <string>
#include <vector>

namespace pylon_camera
{

class PylonCameraParameter
{
public:
    PylonCameraParameter();
    virtual ~PylonCameraParameter();

    std::string magazino_cam_id_;
    std::string camera_frame_;
    double desired_frame_rate_;
    double target_exposure_;
    bool use_sequencer_;
    bool output_hdr_;
    int param_update_frequency_;
    double start_exposure_;
    bool use_brightness_;
    int start_brightness_;
    std::string intrinsic_yaml_file_;
    bool have_intrinsic_data_;
    std::vector<float> desired_seq_exp_times_;
    int mtu_size_;
};

}

#endif
