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

    bool merge(const vector<Mat>& img_sequence, const vector<float>& exp_times, Mat &hdr_img);
    void loadExposureSeq(const String&, const vector<Mat>&, vector<float>&);

private:
    Mat fusion_;
    Mat weight_map_;
};

} /* namespace pylon_camera */

#endif /* HDRGENERATOR_H_ */
