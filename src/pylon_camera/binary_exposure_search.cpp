// Copyright 2015 <Magazino GmbH>

#include <pylon_camera/binary_exposure_search.h>

namespace pylon_camera
{

BinaryExposureSearch::BinaryExposureSearch(const float& target_brightness,
                                           const float& left_lim,
                                           const float& right_lim,
                                           const float& current_exp)
    : last_exposure_(current_exp)
    , last_unchanged_exposure_counter_(0)
    , left_limit_(left_lim)
    , right_limit_(right_lim)
    , new_exposure_( (left_lim + right_lim) / 2.0 )
    , target_brightness_(target_brightness)
    , limit_reached_(false)
    , is_initial_setting_(true)
{}

BinaryExposureSearch::~BinaryExposureSearch()
{}

bool BinaryExposureSearch::update(const float& current_brightness,
                                  const float& current_exposure)
{
    if (is_initial_setting_ )
    {
        // no need to update the limits, the first time this function will
        // be called because limits were correctly set in the constructor
        is_initial_setting_ = false;
        return true;
    }

    if ( current_brightness > target_brightness_)
    {
        right_limit_ = current_exposure;
    }
    else
    {
        left_limit_ = current_exposure;
    }

    new_exposure_ = (left_limit_ + right_limit_) / 2.0;

    if ( new_exposure_ == current_exposure )
    {
       ++last_unchanged_exposure_counter_;
    }
    else
    {
       last_exposure_ = current_exposure;
    }

    if (last_unchanged_exposure_counter_ > 2)
    {
        ROS_ERROR_STREAM("BinaryExposureSearch failed, trying three times "
                << "to set the same new exposure value");
        return false;
    }
    else
    {
        return true;
    }
}

const float& BinaryExposureSearch::newExposure() const
{
    return new_exposure_;
}

void BinaryExposureSearch::limitReached(bool reached)
{
    limit_reached_ = reached;
}

bool BinaryExposureSearch::isLimitReached() const
{
    return limit_reached_;
}

}  // namespace pylon_camera
