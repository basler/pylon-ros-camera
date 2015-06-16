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
  IntrinsicCalibLoader(const std::string yaml_file);
  virtual ~IntrinsicCalibLoader();
  bool loadCalib();

  cv::Mat D_;
  cv::Mat K_;

  int img_cols_;
  int img_rows_;

  std::string intrinsic_yaml_file_;
};

} /* namespace pylon_camera */

#endif /* INTRINSIC_CALIB_LOADER_H_ */
