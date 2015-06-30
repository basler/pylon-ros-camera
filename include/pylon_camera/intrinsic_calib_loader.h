/*
 * intrinsic_calib_loader.h
 *
 *  Created on: Jun 10, 2015
 *      Author: md
 */

#ifndef INTRINSIC_CALIB_LOADER_H_
#define INTRINSIC_CALIB_LOADER_H_

#include <opencv2/opencv.hpp>

using std::cout;
using std::cerr;
using std::endl;

namespace pylon_camera
{

class IntrinsicCalibLoader
{
public:
    IntrinsicCalibLoader();
    virtual ~IntrinsicCalibLoader();
    bool init(const std::string yaml_file);
    bool loadCalib();

    cv::Mat D();
    cv::Mat K();
    int img_rows();
    int img_cols();

private:

    std::string intrinsic_yaml_file_;

    cv::Mat D_;
    cv::Mat K_;

    int img_cols_;
    int img_rows_;
};

} /* namespace pylon_camera */

#endif /* INTRINSIC_CALIB_LOADER_H_ */
