/*
 * pylon_sequencer_interface.h
 *
 *  Created on: May 26, 2015
 *      Author: md
 */

#ifndef PYLON_OEPNCV_INTERFACE_H_
#define PYLON_OEPNCV_INTERFACE_H_

#include <pylon_camera/pylon_interface.h>

namespace pylon_camera
{

class PylonOpenCVInterface : public PylonInterface
{
public:
    PylonOpenCVInterface();
    virtual ~PylonOpenCVInterface();

    int initialize(const PylonCameraParameter &params);
    virtual int initSequencer(const PylonCameraParameter &params);
//    virtual int terminate(const PylonCameraParameter &params);
    bool grab(const PylonCameraParameter &params, cv::Mat &image);

    std::vector<cv::Mat> img_sequence_;
};

} /* namespace pylon_camera */

#endif /* PYLON_OEPNCV_INTERFACE_H_ */
