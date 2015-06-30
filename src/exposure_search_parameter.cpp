/*
 * exposure_search_parameter.cpp
 *
 *  Created on: Jun 24, 2015
 *      Author: md
 */

#include <pylon_camera/exposure_search_parameter.h>

namespace pylon_camera {

ExposureSearchParameter::ExposureSearchParameter() :
                    is_initialized_(false),
                    goal_brightness_(0.0),
                    goal_exposure_(0.0),
                    current_exposure_(0.0),
                    desired_exposure_(0.0),
                    last_exposure_(0.0),
                    last_unchanged_exposure_counter_(0),
                    current_brightness_(0),
                    left_limit_(0.0),
                    right_limit_(0.0),
                    success_(false),
                    exp_update_sleep_counter_(0),
                    first_time_(true)
{
}

ExposureSearchParameter::~ExposureSearchParameter()
{
    // TODO Auto-generated destructor stub
}
void ExposureSearchParameter::updateBinarySearch()
{

    if (current_brightness_ > goal_brightness_)
    {
        right_limit_ = current_exposure_;
    } else
    {
        left_limit_ = current_exposure_;
    }

    desired_exposure_ = (right_limit_ + left_limit_) / 2.0;
    if(desired_exposure_ == last_exposure_){
        last_unchanged_exposure_counter_++;
    }

    if(first_time_){
        first_time_ = false;
    }
    exp_update_sleep_counter_ = 0;
//    last_exposure_ = current_exposure_;
    cout << "####################################################################################################" << endl;
    cout << "UPDATE: " << "goal = " << goal_brightness_ << ", current brightness = " << current_brightness_ << ", current exp = " << current_exposure_ << ", left lim = " << left_limit_ << ", right lim = " << right_limit_ << endl;
    cout << "last exp = " << last_exposure_ << ", last unchanged exp ctr = " << last_unchanged_exposure_counter_ << ", desired exp = " <<  desired_exposure_ << endl;
    cout << "####################################################################################################" << endl;

}
void ExposureSearchParameter::correctLimits(){
    if (current_brightness_ > goal_brightness_)
    {
        right_limit_ = current_exposure_;
    } else
    {
        left_limit_ = current_exposure_;
    }
    cout << "####################################################################################################" << endl;
    cout << "LIMIT CORRECTION: " << "goal = " << goal_brightness_ << ", current brightness = " << current_brightness_ << ", current exp = " << current_exposure_ << ", left lim = " << left_limit_ << ", right lim = " << right_limit_ << endl;
    cout << "####################################################################################################" << endl;
}
void ExposureSearchParameter::initialize(double goal, double left_lim, double right_lim, double current_exp, double current_brightness)
{
    goal_brightness_ = goal;
    left_limit_ = left_lim;
    right_limit_ = right_lim;
    current_exposure_ = current_exp;
    current_brightness_ = current_brightness;
    desired_exposure_ = 0.0;
    last_exposure_ = 0.0;
    last_unchanged_exposure_counter_ = -1;
    first_time_ = true;
    exp_update_sleep_counter_ = 0;
    is_initialized_ = true;
    cout << "***********************************************************************************************************" << endl;
    cout << "INITIALIZED: " << "goal = " << goal_brightness_ << ", current brightness = " << current_brightness_ << ", current exp = " << current_exposure_ << ", left lim = " << left_limit_ << ", right lim = " << right_limit_ << endl;
    cout << "last unchanged exp ctr = " << last_unchanged_exposure_counter_ << ", desired exp = " <<  desired_exposure_ << ", initialized = " << is_initialized_ << endl;
    cout << "***********************************************************************************************************" << endl;
}

} /* namespace pylon_camera */
