/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (C) 2016, Magazino GmbH. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Magazino GmbH nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#ifndef PYLON_CAMERA_INTRINSIC_CALIB_LOADER_H
#define PYLON_CAMERA_INTRINSIC_CALIB_LOADER_H

#include <opencv2/opencv.hpp>
#include <string>

namespace pylon_camera
{

/**
 * Wrapper class for loading intrinsic calibration data
 */
class IntrinsicCalibLoader
{
public:
    IntrinsicCalibLoader();
    virtual ~IntrinsicCalibLoader();

    /**
     * Initialize the intrinsic calib loader class
     * @param yaml_file path to yaml file or yaml content
     * @return true
     */
    bool init(const std::string& yaml_file);

    /**
     * Tries to load the camera calibration from the yaml file / yaml content
     * @return true if successful
     */
    bool loadCalib();

    /**
     * Distortion matrix
     * @return the distortion matrix
     */
    const cv::Mat& D() const;

    /**
     * Camera matrix
     * @return the camera matrix
     */
    const cv::Mat& K() const;

    /**
     * The image height
     * @return the image height
     */
    const int& img_rows() const;

    /**
     * The image width
     * @return the image width
     */
    const int& img_cols() const;

private:
    /**
     * The image height
     */
    int img_rows_;

    /**
     * The image width
     */
    int img_cols_;

    /**
     * The distortion matrix
     */
    cv::Mat D_;

    /**
     * The camera matrix
     */
    cv::Mat K_;

    /**
     * The yaml file path or yaml content for the intrinsic calibration values.
     */
    std::string intrinsic_yaml_file_;
};

}  // namespace pylon_camera

#endif  // PYLON_CAMERA_INTRINSIC_CALIB_LOADER_H
