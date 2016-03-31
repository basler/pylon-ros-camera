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
#include <pylon_camera/intrinsic_calib_loader.h>
#include <ros/ros.h>
#include <string>

namespace pylon_camera
{

IntrinsicCalibLoader::IntrinsicCalibLoader() :
        img_rows_(-1),
        img_cols_(-1),
        intrinsic_yaml_file_("INVALID_YAML_FILE")
{}

IntrinsicCalibLoader::~IntrinsicCalibLoader()
{}

bool IntrinsicCalibLoader::init(const std::string& yaml_file)
{
    intrinsic_yaml_file_ = yaml_file;
    return true;
}

bool IntrinsicCalibLoader::loadCalib()
{
    if (intrinsic_yaml_file_ == "INVALID_YAML_FILE")
    {
        ROS_ERROR("Error loading Intrinsic calib file: call IntrinsicCalibLoader::init() first!");
        return false;
    }

    if (intrinsic_yaml_file_.size() == 0)
    {
        ROS_ERROR("yaml_string empty");
        return false;
    }

    int flags = cv::FileStorage::READ;
    if (intrinsic_yaml_file_[0] != '/')
    {
        flags += cv::FileStorage::MEMORY;
    }

    cv::FileStorage fs(intrinsic_yaml_file_, flags);
    if (!fs.isOpened())
    {
        ROS_ERROR_STREAM("Could not open cv::Filestorage with intrinsic calib "
            << "yaml file: " << intrinsic_yaml_file_);
        return false;
    }
    else
    {
        fs["distortion"] >> D_;
        fs["cam_matrix"] >> K_;

        /// two calib file versions: one with 'row', 'col' and one with 'width', 'height'
        /// if not existent: filestorage sets to 0
        int row_1 = 0, col_1 = 0, row_2 = 0, col_2 = 0;

        fs["cols"] >> col_1;
        fs["rows"] >> row_1;
        fs["width"] >> col_2;
        fs["height"] >> row_2;

        if (col_1 > 0 && col_2 > 0){
            ROS_ERROR("Intrinsic calibration file has entries 'cols' and 'width', can't be intrepreted");
            return false;
        }

        if (row_1 > 0 && row_2 > 0){
            ROS_ERROR("Intrinsic calibration file has entries 'rows' and 'height', can't be intrepreted");
            return false;
        }

        img_rows_ = std::max(row_1, row_2);
        img_cols_ = std::max(col_1, col_2);

        fs.release();

        if (img_cols_ <= 0 || img_rows_ <= 0)
        {
            ROS_ERROR_STREAM("Unexpected img size (" << img_cols_ << ", " << img_rows_
                             << ") while reading from yaml file!");
            return false;
        }
        if (!(D_.rows == 5 && D_.cols == 1))
        {
            ROS_ERROR_STREAM("Unexpected size of distortion matrix (" << D_.rows << " x " << D_.cols
                             << ") while reading from yaml file! Should be: (5 x 1)");
            return false;
        }
        if (!(K_.rows == 3 && K_.cols == 3))
        {
            ROS_ERROR_STREAM("Unexpected size of camera matrix (" << K_.rows << " x " << K_.cols
                             << ") while reading from yaml file! Should be: (3 x 3)");
            return false;
        }
    }

    return true;
}

const cv::Mat& IntrinsicCalibLoader::D() const
{
    return D_;
}

const cv::Mat& IntrinsicCalibLoader::K() const
{
    return K_;
}

const int& IntrinsicCalibLoader::img_rows() const
{
    return img_rows_;
}

const int& IntrinsicCalibLoader::img_cols() const
{
    return img_cols_;
}

}  // namespace pylon_camera
