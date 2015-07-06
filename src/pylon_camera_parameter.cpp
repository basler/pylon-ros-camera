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
                    param_update_frequency_(50),
                    exposure_(2000.0),
                    use_brightness_(false),
                    brightness_(128),
                    intrinsic_yaml_file_(""),
                    have_intrinsic_data_(false)
{
    // TODO Auto-generated constructor stub
}

PylonCameraParameter::~PylonCameraParameter()
{
    // TODO Auto-generated destructor stub
}

} /* namespace pylon_camera */
