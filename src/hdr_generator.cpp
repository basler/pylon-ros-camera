/*
 * hdr_generator.cpp
 *
 *  Created on: Jul 13, 2015
 *      Author: md
 */

#include <ros/ros.h>
#include <pylon_camera/hdr_generator.h>

#define NPERFORMANCE_TEST

namespace pylon_camera {

Mat weightedFusion(const std::vector<Mat>& images, const Mat& wm)
{
    Mat fused = Mat::zeros(images[0].size(), CV_32FC1);
    Mat transformed(images[0].size(), CV_32FC1);
    for (std::vector<Mat>::const_iterator it = images.begin(); it != images.end(); ++it)
    {
        LUT(*it, wm, transformed);
        fused += transformed;
    }
    return fused;
}

Mat createWeightMap()
{
    Mat weight(LDR_SIZE, 1, CV_32FC1);
    const float sigma = 0.2;
    const float mu = 0.5;

    const float b = 2 * std::pow((sigma * 255), 2);

    for (int i = 0; i < LDR_SIZE; i++)
    {
        weight.at<float>(i) = std::exp( - std::pow(i / 255. - mu * 255, 2) / b) / 255.;
    }
    return weight;
}


HDRGenerator::HDRGenerator() :
    weight_map_(createWeightMap())
{
    // TODO Auto-generated constructor stub

}

HDRGenerator::~HDRGenerator()
{
    // TODO Auto-generated destructor stub
}

#ifdef PERFORMANCE_TEST
static int frame = 0;
#endif

bool HDRGenerator::merge(const vector<Mat>& img_sequence, const vector<float>& exp_times, Mat& hdr_img)
{
    double min, max;
    Point min_loc, max_loc;
#ifdef PERFORMANCE_TEST
    Ptr<MergeMertens> merge_mertens = createMergeMertens();
    ros::Time now;
    std::stringstream ss;

    now = ros::Time::now();
    merge_mertens->process(img_sequence, fusion_);
    std::cout << "mertens took " << (ros::Time::now() - now).toSec() << std::endl;

    minMaxLoc(fusion_, &min, &max, &min_loc, &max_loc);
    fusion_.convertTo(hdr_img, CV_8U, 255.0 / (max-min), -255.0 * min / (max - min));
    ss << "/tmp/hdrtest-" << frame << "-mertens.jpg";
    cv::imwrite(ss.str(), hdr_img);


    now = ros::Time::now();
#endif
    fusion_ = weightedFusion(img_sequence, weight_map_);
#ifdef PERFORMANCE_TEST
    std::cout << "weighted Fusion took " << (ros::Time::now() - now).toSec() << std::endl;

    ss.str("");
#endif
    minMaxLoc(fusion_, &min, &max, &min_loc, &max_loc);
    fusion_.convertTo(hdr_img, CV_8U, 255.0 / (max-min), -255.0 * min / (max - min));
#ifdef PERFORMANCE_TEST
    ss << "/tmp/hdrtest-" << frame << "-expfusion.jpg";
    cv::imwrite(ss.str(), hdr_img);

    for (std::size_t i = 0; i < img_sequence.size(); ++i)
    {
        ss.str("");
        ss << "/tmp/hdrtest-" << frame << "-exposure-" << i << ".jpg";
        cv::imwrite(ss.str(), img_sequence[i]);
    }

    ++frame;
#endif

    return true;
}


} /* namespace pylon_camera */
