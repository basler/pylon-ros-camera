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
    BinaryExposureSearch(const float& target_brightness,
                         const float& left_lim,
                         const float& right_lim,
                         const float& current_exp);

    virtual ~BinaryExposureSearch();

    /**
     * Update the binary search based on the current
     * brightness and exposure values
     */
    bool update(const float& current_brightness,
                       const float& current_exposure);

    /**
     * Setter for limit_reached_
     */
    void limitReached(bool reached);

    /**
     * Getter for limit_reached_
     * Returns true if the search reached the physical exposure
     * limit of the camera
     */
    bool isLimitReached() const;

    /**
     * Getter for the new exposure calculated in out of the update step
     */
    const float& newExposure() const;

private:
    /**
     * The targeted brightness value
     */
    const float target_brightness_;

    /**
     * The previous exposure value
     */
    float last_exposure_;

    /**
     * Counts how often the exposure remained unchanged during search
     */
    size_t last_unchanged_exposure_counter_;

    /**
     * Left limit for the binary search
     */
    float left_limit_;

    /**
     * Right limit for the binary search
     */
    float right_limit_;

    /**
     * The new exposure out of the update step
     */
    float new_exposure_;

    /**
     * Flag which tells, when the physical limit of the camera is reached.
     * Hence the BinaryExposureSearch will fail, if the target brightness
     * is not yet reached
     */
    bool limit_reached_;

    /**
     * Flag which tells if it is the first time the update() function is called
     * Hence it is not necessary to update the search limits, because they
     * were set correctly in the constructor
     */
    bool is_initial_setting_;
};

}  // namespace pylon_camera

#endif  // PYLON_CAMERA_BINARY_EXPOSURE_SEARCH_H
