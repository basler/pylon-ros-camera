/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (C) 2016, Magazino GmbH. All rights reserved.
 *
 * Improved by drag and bot GmbH (www.dragandbot.com), 2019
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

#ifndef PYLON_CAMERA_ENCODING_CONVERSIONS_H
#define PYLON_CAMERA_ENCODING_CONVERSIONS_H

// Notes:
// yuv422 format codes 2 pixels in 4 bytes. Y is the instensity (the first Y of the first pixel, the second Y of the second one)
// U and V are the color components code the color used in both pixels.
// - The ROS format (yuv422) expects UYVY http://docs.ros.org/jade/api/sensor_msgs/html/image__encodings_8h_source.html
// - Basler Camera format (YUV422Packed) is coded as UYVY
// https://www.baslerweb.com/en/sales-support/knowledge-base/frequently-asked-questions/how-does-the-yuv-color-coding-work/15182/
// https://docs.baslerweb.com/pixel-format#yuv-formats

#include <string>

namespace pylon_camera
{

namespace encoding_conversions
{
    /**
     * Converts an encoding from the sensor_msgs/image_encodings.h list into
     * the GenAPI language.
     * @return true in case that an corresponding conversion could be found and
     *         false otherwise.
     */
    bool ros2GenAPI(const std::string& ros_enc, std::string& gen_api_enc, bool is_16bits_available);

    /**
     * Converts an encoding described in GenAPI language into the ROS encoding
     * language. The ROS encodings are listed in sensor_msgs/image_encodings.h
     * @return true in case that an corresponding conversion could be found and
     *         false otherwise.
     */
    bool genAPI2Ros(const std::string& gen_api_enc, std::string& ros_enc);

    /**
     * Returns if the given encoding convertion is 8 or 12 bits. in case of Basler 12 bits, 4-bits shifting is required to make it compatible with ROS 16-bits encoding
     */
    bool is_12_bit_gen_api_enc(const std::string& gen_api_enc);
    bool is_12_bit_ros_enc(const std::string& ros_enc);

}  // namespace encoding_conversions
}  // namespace pylon_camera
#endif  // PYLON_CAMERA_ENCODING_CONVERSIONS_H



