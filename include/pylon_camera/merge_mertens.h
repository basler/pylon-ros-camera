#ifndef PYLON_CAMERA_MERGE_MERTENS_H_
#define PYLON_CAMERA_MERGE_MERTENS_H_

#include <opencv2/opencv.hpp>

namespace pylon_camera
{
class MergeMertensC1
{
public:
    MergeMertensC1(const float& contrast_weight = 1.0, const float& exposure_weight = 0.0);
    virtual ~MergeMertensC1();

    void process(cv::InputArrayOfArrays src, cv::OutputArray dst,
                 cv::InputArray times, cv::InputArray response);
    void process(cv::InputArrayOfArrays src, cv::OutputArray dst);

    float getContrastWeight() const;
    void setContrastWeight(float contrast_weiht);

    float getExposureWeight() const;
    void setExposureWeight(float exposure_weight);

protected:
    float wcon_;
    float wexp_;

};

}

#endif
