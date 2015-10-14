#ifndef HDRGENERATOR_H_
#define HDRGENERATOR_H_

#include <opencv2/opencv.hpp>

namespace pylon_camera
{

class MergeMertensC1;

class HDRGenerator
{
public:
    HDRGenerator();
    virtual ~HDRGenerator();

    bool merge(const std::vector<cv::Mat>& img_sequence,
               const std::vector<float>& exp_times,
               cv::Mat &hdr_img);
    void loadExposureSeq(const cv::String&,
                         const std::vector<cv::Mat>&,
                         std::vector<float>&);

private:
    cv::Mat fusion_;

    MergeMertensC1* merge_mertens_;
};

}

#endif
