#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pylon_camera/hdr_generator.h>
#include <pylon_camera/merge_mertens.h>

namespace pylon_camera
{

/* Weighted Fusion Algorithm. Not good enough for our purpose but I'll leave it here as a starting point for future optimizations
// See: Multi-exposure Imaging on Mobile Devices
// N. Gelfand, A. Adams, S.H. Park, K. Pulli
// URL: http://graphics.stanford.edu/~shpark7/projects/hdr_gelfand_mm10.pdf

    Mat weightedFusion(const std::vector<Mat>& images, const Mat& wm)
    {
        Mat fused = Mat::zeros(images[0].size(), CV_32FC1);
        Mat transformed(images[0].size(), CV_32FC1);
        std::vector<Mat>::const_iterator image_it = images.begin();
        for (; image_it != images.end(); ++image_it)
        {
            LUT(*image_it, wm, transformed);
            fused += transformed;


        }
        Mat tmp;
        cv::threshold(images[images.size() - 1], tmp, 85, 85, CV_THRESH_TRUNC);
        tmp *= 3;

        LUT(tmp, wm, transformed);
        fused += transformed;

        return fused;
    }


    Mat createWeightedFusionWeightMap()
    {
        Mat weight(LDR_SIZE, 1, CV_32FC1);
        const float sigma = 0.2 * 255;
        const float mu = 0.5 * 255;

        const float b = 2 * std::pow((sigma * 255), 2);

        for (int i = 0; i < LDR_SIZE; ++i)
        {
            weight.at<float>(i) = std::exp( - std::pow(i / 255. - mu * 255, 2) / b) / 255.;
        }
        return weight;
    }
*/


HDRGenerator::HDRGenerator() :
    merge_mertens_(new MergeMertensC1)
{
}

HDRGenerator::~HDRGenerator()
{
    delete merge_mertens_;
    merge_mertens_ = NULL;
}

bool HDRGenerator::merge(const std::vector<cv::Mat>& img_sequence, const std::vector<float>& exp_times, cv::Mat& hdr_img)
{
    double min, max;
    cv::Point min_loc, max_loc;

    merge_mertens_->process(img_sequence, fusion_);

    cv::minMaxLoc(fusion_, &min, &max, &min_loc, &max_loc);
    fusion_.convertTo(hdr_img, CV_8U, 255.0 / (max-min), -255.0 * min);
    return true;
}

}
