#ifndef INTRINSIC_CALIB_LOADER_H_
#define INTRINSIC_CALIB_LOADER_H_

#include <opencv2/opencv.hpp>

namespace pylon_camera
{

class IntrinsicCalibLoader
{
public:
    IntrinsicCalibLoader();
    virtual ~IntrinsicCalibLoader();

    bool init(const std::string& yaml_file);
    bool loadCalib();

    const cv::Mat& D() const;
    const cv::Mat& K() const;
    const int& img_rows() const;
    const int& img_cols() const;

private:

    int img_rows_;
    int img_cols_;

    cv::Mat D_;
    cv::Mat K_;

    std::string intrinsic_yaml_file_;
};

}

#endif
