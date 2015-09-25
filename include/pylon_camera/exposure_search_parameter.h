/*
 * exposure_search_parameter.h
 *
 *  Created on: Jun 24, 2015
 *      Author: md
 */

#ifndef EXPOSURE_SEARCH_PARAMETER_H_
#define EXPOSURE_SEARCH_PARAMETER_H_

#include <iostream>

using std::cout;
using std::endl;

namespace pylon_camera {

class ExposureSearchParameter
{
public:
    ExposureSearchParameter();
    virtual ~ExposureSearchParameter();

    double goal_brightness_;
    double goal_exposure_;

    double desired_exposure_;
    double current_exposure_;
    double last_exposure_;
    int last_unchanged_exposure_counter_;
    double current_brightness_;

    int left_limit_;
    int right_limit_;

    bool success_;
    bool is_initialized_;

    bool first_time_;

    void updateBinarySearch();
    void initialize(const double& goal, const double& left_lim, const double& right_lim, const double& current_exp, const double& current_brightness);

};

} /* namespace pylon_camera */

#endif /* EXPOSURE_SEARCH_PARAMETER_H_ */
