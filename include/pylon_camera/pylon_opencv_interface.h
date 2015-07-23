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
    virtual bool setupSequencer(const PylonCameraParameter &params);
//    virtual int terminate(const PylonCameraParameter &params);
    bool grab(const PylonCameraParameter &params, cv::Mat &image);

    virtual bool setExtendedBrightness(int &brightness);

    std::vector<cv::Mat> img_sequence_;

    bool own_brightness_search_running_;
    bool own_brightness_search_sucess_;
    ExposureSearchParameter exp_search_params_;
    void setupExtendedBrightnessSearch(int &brightness);
    std::vector<float> seq_exp_times();

    std::vector<float> seq_exp_times_;

    bool findExposureSequence(const PylonCameraParameter &params);

//    bool isAutoBrightnessFunctionRunning();
private:
};

} /* namespace pylon_camera */

#endif /* PYLON_OEPNCV_INTERFACE_H_ */
