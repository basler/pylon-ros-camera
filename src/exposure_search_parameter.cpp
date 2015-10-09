#include <pylon_camera/exposure_search_parameter.h>

namespace pylon_camera {

ExposureSearchParameter::ExposureSearchParameter() :
                    goal_brightness_(0.0),
                    goal_exposure_(0.0),
                    desired_exposure_(0.0),
                    current_exposure_(0.0),
                    last_exposure_(0.0),
                    last_unchanged_exposure_counter_(0),
                    current_brightness_(0),
                    left_limit_(0.0),
                    right_limit_(0.0),
                    success_(false),
                    is_initialized_(false),
                    first_time_(true)
{
}

ExposureSearchParameter::~ExposureSearchParameter()
{
}

void ExposureSearchParameter::updateBinarySearch()
{

    if (current_brightness_ > goal_brightness_)
    {
        right_limit_ = current_exposure_;
    }
    else
    {
        left_limit_ = current_exposure_;
    }

    desired_exposure_ = (right_limit_ + left_limit_) / 2.0;

    if (current_exposure_ == last_exposure_)
    {
        last_unchanged_exposure_counter_++;
    }
    else
    {
        last_exposure_ = current_exposure_;
    }

    if (first_time_)
    {
        first_time_ = false;
    }

}
void ExposureSearchParameter::initialize(const double& goal,
                                         const double& left_lim,
                                         const double& right_lim,
                                         const double& current_exp,
                                         const double& current_brightness)
{
    goal_brightness_ = goal;
    left_limit_ = left_lim;
    right_limit_ = right_lim;
    current_exposure_ = current_exp;
    current_brightness_ = current_brightness;
    desired_exposure_ = 0.0;
    last_exposure_ = current_exp;
    last_unchanged_exposure_counter_ = 0;
    first_time_ = true;
    is_initialized_ = true;
}

}
