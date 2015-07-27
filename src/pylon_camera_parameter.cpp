/*
 * pylon_camera_parameter.cpp
 *
 *  Created on: May 21, 2015
 *      Author: md
 */

#include <pylon_camera/pylon_camera_parameter.h>

namespace pylon_camera
{

PylonCameraParameter::PylonCameraParameter() :
                    magazino_cam_id_("x"),
                    camera_frame_("camera_base"),
                    desired_frame_rate_(-1.0),
                    target_exposure_(3000),
                    use_sequencer_(false),
                    output_hdr_(false),
                    param_update_frequency_(50),
                    start_exposure_(2000.0),
                    use_brightness_(false),
                    start_brightness_(128),
                    intrinsic_yaml_file_(""),
                    have_intrinsic_data_(false),
                    desired_seq_exp_times_(),
                    mtu_size_(3000)
{
    desired_seq_exp_times_.clear();
}

PylonCameraParameter::~PylonCameraParameter()
{
    // TODO Auto-generated destructor stub
}

} /* namespace pylon_camera */
