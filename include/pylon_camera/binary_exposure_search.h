// Copyright 2015 <Magazino GmbH>

#ifndef PYLON_CAMERA_BINARY_EXPOSURE_SEARCH_H
#define PYLON_CAMERA_BINARY_EXPOSURE_SEARCH_H

namespace pylon_camera
{

/**
 * Class for the extended brighntess search using a binary exposure search method
 */
class BinaryExposureSearch
{
public:
    BinaryExposureSearch();
    virtual ~BinaryExposureSearch();

    /**
     * The targeted brightness value
     */
    double target_brightness_;

    /**
     * The targeted exposure value
     */
    double target_exposure_;

    /**
     * The current exposure value
     */
    double current_exposure_;

    /**
     * The previous exposure value
     */
    double last_exposure_;

    /**
     * Counts how often the exposure remained unchanged during search
     */
    int last_unchanged_exposure_counter_;

    /**
     * The current brightness
     */
    double current_brightness_;

    /**
     * Left limit for the binary search
     */
    int left_limit_;

    /**
     * Right limit for the binary search
     */
    int right_limit_;

    /**
     * If true, the exposure search finished
     */
    bool success_;

    /**
     * If true, the exposure search was initialized
     * Is set to false from the outside to stop the exposure search
     */
    bool is_initialized_;

    /**
     * Update the binary search based on the current brightness and exposure values
     */
    void updateBinarySearch(const float& current_brightness);

    /**
     * Initialize the exposure search
     * @param target_brightness the targeted brightness value
     * @param left_lim the minimum exposure time to set
     * @param right_lim the maximum exposure time to set
     * @param current_exp the current exposure time
     * @param current_brightness the current brightness value
     */
    void initialize(const double& target_brightness,
                    const double& left_lim,
                    const double& right_lim,
                    const double& current_exp,
                    const double& current_brightness);
};

}  // namespace pylon_camera

#endif  // PYLON_CAMERA_BINARY_EXPOSURE_SEARCH_H
