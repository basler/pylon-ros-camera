#ifndef IMAGE_RECTIFIER_H_
#define IMAGE_RECTIFIER_H_

#include <opencv2/opencv.hpp>

namespace pylon_camera
{

class ImageRectifier
{
public:
    ImageRectifier();
    virtual ~ImageRectifier();

    void setupRectifyingMap(const cv::Mat& cam_matrix,
                            const cv::Mat& dist_coefficients,
                            const int& img_width,
                            const int& img_height);

    void rectify(const cv::Mat& src, cv::Mat& dst);

private:
    cv::Mat rect_map_x_, rect_map_y_;
};

}

#endif
