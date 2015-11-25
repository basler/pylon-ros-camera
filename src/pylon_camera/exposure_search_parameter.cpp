// Copyright 2015 <Magazino GmbH>
#include <pylon_camera/exposure_search_parameter.h>

namespace pylon_camera
{

ExposureSearchParameter::ExposureSearchParameter() :
    target_brightness_(0.0),
    target_exposure_(0.0),
    current_exposure_(0.0),
    last_exposure_(0.0),
    last_unchanged_exposure_counter_(0),
    current_brightness_(0),
    left_limit_(0.0),
    right_limit_(0.0),
    success_(false),
    is_initialized_(false)
{
}

ExposureSearchParameter::~ExposureSearchParameter()
{
}

void ExposureSearchParameter::updateBinarySearch()
{
    if (current_brightness_ > target_brightness_)
    {
        right_limit_ = current_exposure_;
    }
    else
    {
        left_limit_ = current_exposure_;
    }

    target_exposure_ = (right_limit_ + left_limit_) / 2.0;

    if (current_exposure_ == last_exposure_)
    {
        ++last_unchanged_exposure_counter_;
    }
    else
    {
        last_exposure_ = current_exposure_;
    }
}

void ExposureSearchParameter::initialize(const double& target_brightness,
                                         const double& left_lim,
                                         const double& right_lim,
                                         const double& current_exp,
                                         const double& current_brightness)
{
    target_brightness_ = target_brightness;
    left_limit_ = left_lim;
    right_limit_ = right_lim;
    current_exposure_ = current_exp;
    current_brightness_ = current_brightness;
    target_exposure_ = 0.0;
    last_exposure_ = current_exp;
    last_unchanged_exposure_counter_ = 0;
    is_initialized_ = true;
}

}  // namespace pylon_camera
