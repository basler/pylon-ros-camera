/*
 * image_rectifier.h
 *
 *  Created on: May 26, 2015
 *      Author: md
 */

#ifndef IMAGE_RECTIFIER_H_
#define IMAGE_RECTIFIER_H_

#include <opencv2/opencv.hpp>

using namespace cv;

namespace pylon_camera
{

class ImageRectifier
{
public:
    ImageRectifier();
    virtual ~ImageRectifier();

    void setupRectifyingMap(const Mat& cam_matrix, const Mat& dist_coefficients, const int& img_width,
        const int& img_height);
    void rectify(const Mat& src, Mat& dst);

private:
    cv::Mat rect_map_x_, rect_map_y_;
};

} /* namespace pylon_camera */

#endif /* IMAGE_RECTIFIER_H_ */
