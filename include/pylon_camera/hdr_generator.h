/*
 * hdr_generator.h
 *
 *  Created on: Jul 13, 2015
 *      Author: md
 */

#ifndef HDRGENERATOR_H_
#define HDRGENERATOR_H_

#include <opencv2/photo.hpp>
#include "opencv2/imgcodecs.hpp"
#include <opencv2/highgui.hpp>
#include <vector>
#include <iostream>
#include <fstream>

namespace pylon_camera {


using namespace cv;
using namespace std;

class HDRGenerator
{
public:
    HDRGenerator();
    virtual ~HDRGenerator();

    bool mergeMertens(vector<Mat> img_sequence, vector<float> exp_times, Mat &hdr_img);
    void loadExposureSeq(String, vector<Mat>&, vector<float>&);
};

} /* namespace pylon_camera */

#endif /* HDRGENERATOR_H_ */
