// Copyright 2015 <Magazino GmbH>

#include <pylon_camera/binary_exposure_search.h>

namespace pylon_camera
{

BinaryExposureSearch::BinaryExposureSearch(const float_t& target_brightness,
                                           const float_t& left_lim,
                                           const float_t& right_lim,
                                           const float_t& current_exp)
    : new_exposure_(0.0)
    , last_exposure_(current_exp)
    , last_unchanged_exposure_counter_(0)
    , left_limit_(left_lim)
    , right_limit_(right_lim)
    , target_brightness_(target_brightness)
    , limit_reached_(false)
{}

bool BinaryExposureSearch::update(const float_t& current_brightness,
                                  const float_t& current_exposure)
{
    current_brightness > target_brightness_ ? right_limit_ = current_exposure : left_limit_ = current_exposure;

    new_exposure_ = (right_limit_ + left_limit_) / 2.0;

    new_exposure_ == current_exposure ? ++last_unchanged_exposure_counter_ : last_exposure_ = current_exposure;

    if (last_unchanged_exposure_counter_ > 2)
    {
        ROS_ERROR("BinaryExposureSearch failed, trying three times to set the same new exposure value");
        return false;
    }
    else
    {
        return true;
    }
}

float_t& BinaryExposureSearch::newExposure()
{
    return new_exposure_;
}

void BinaryExposureSearch::limitReached(bool reached)
{
    limit_reached_ = reached;
}

bool BinaryExposureSearch::isLimitReached()
{
    return limit_reached_;
}

BinaryExposureSearch::~BinaryExposureSearch()
{}

}  // namespace pylon_camera
