// Copyright 2015 <Magazino GmbH>

#ifndef PYLON_CAMERA_BINARY_EXPOSURE_SEARCH_H
#define PYLON_CAMERA_BINARY_EXPOSURE_SEARCH_H

#include <ros/ros.h>

namespace pylon_camera
{

/**
 * Class for the extended brighntess search using a binary
 * exposure search method
 */
class BinaryExposureSearch
{
public:

    /**
     * Initialize the exposure search
     * @param target_brightness the targeted brightness value
     * @param left_lim the minimum exposure time to set
     * @param right_lim the maximum exposure time to set
     * @param current_exp the current exposure time
     */
    BinaryExposureSearch(const float_t& target_brightness,
                         const float_t& left_lim,
                         const float_t& right_lim,
                         const float_t& current_exp);

    /**
     * Update the binary search based on the current
     * brightness and exposure values
     */
    bool update(const float_t& current_brightness,
                const float_t& current_exposure);

    /**
     * Setter for limit_reached_
     */
    void limitReached(bool reached);

    /**
     * Getter for limit_reached_
     * Returns true if the search reached the physical exposure
     * limit of the camera
     */
    bool isLimitReached();

    /**
     * Getter for the new exposure calculated in out of the update step
     */
    float_t& newExposure();

    virtual ~BinaryExposureSearch();

private:

    /**
     * The new exposure out of the update step
     */
    float_t new_exposure_;

    /**
     * The targeted brightness value
     */
    const float_t target_brightness_;

    /**
     * The previous exposure value
     */
    float_t last_exposure_;

    /**
     * Counts how often the exposure remained unchanged during search
     */
    size_t last_unchanged_exposure_counter_;

    /**
     * Left limit for the binary search
     */
    float_t left_limit_;

    /**
     * Right limit for the binary search
     */
    float_t right_limit_;

    /**
     * Flag which tells, when the physical limit of the camera is reached.
     * Hence the BinaryExposureSearch will fail, if the target brightness
     * is not yet reached
     */
    bool limit_reached_;

};

}  // namespace pylon_camera

#endif  // PYLON_CAMERA_BINARY_EXPOSURE_SEARCH_H
