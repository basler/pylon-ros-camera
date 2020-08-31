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

// Check notes in header file

#include <ros/ros.h>
#include <pylon_camera/encoding_conversions.h>
#include <sensor_msgs/image_encodings.h>

namespace pylon_camera
{

namespace encoding_conversions
{

bool ros2GenAPI(const std::string& ros_enc, std::string& gen_api_enc, bool is_16bits_available)
{
    /*
     * http://docs.ros.org/kinetic/api/sensor_msgs/html/image__encodings_8h_source.html
     */
    if ( ros_enc == sensor_msgs::image_encodings::MONO8 )
    {
        gen_api_enc = "Mono8";
    }
    else if ( ros_enc == sensor_msgs::image_encodings::MONO16 && ! is_16bits_available)
    {
        gen_api_enc = "Mono12";
    }
    else if ( ros_enc == sensor_msgs::image_encodings::MONO16 && is_16bits_available)
    {
        gen_api_enc = "Mono16";
    }
    else if ( ros_enc == sensor_msgs::image_encodings::BGR8 )
    {
        gen_api_enc = "BGR8";
    }
    else if ( ros_enc == sensor_msgs::image_encodings::RGB8 )
    {
        gen_api_enc = "RGB8";
    }
    else if ( ros_enc == sensor_msgs::image_encodings::BAYER_BGGR8 )
    {
        gen_api_enc = "BayerBG8";
    }
    else if ( ros_enc == sensor_msgs::image_encodings::BAYER_GBRG8 )
    {
        gen_api_enc = "BayerGB8";
    }
    else if ( ros_enc == sensor_msgs::image_encodings::BAYER_RGGB8 )
    {
        gen_api_enc = "BayerRG8";
    }
    else if ( ros_enc == sensor_msgs::image_encodings::BAYER_GRBG8 )
    {
        gen_api_enc = "BayerGR8";
    }
    else if ( ros_enc == sensor_msgs::image_encodings::BAYER_RGGB16 && ! is_16bits_available)
    {
        gen_api_enc = "BayerRG12";
    }
    else if ( ros_enc == sensor_msgs::image_encodings::BAYER_BGGR16 && ! is_16bits_available)
    {
        gen_api_enc = "BayerBG12";
    }
    else if ( ros_enc == sensor_msgs::image_encodings::BAYER_GBRG16 && ! is_16bits_available)
    {
        gen_api_enc = "BayerGB12";
    }
    else if ( ros_enc == sensor_msgs::image_encodings::BAYER_GRBG16 && ! is_16bits_available)
    {
        gen_api_enc = "BayerGR12";
    }
    else if ( ros_enc == sensor_msgs::image_encodings::BAYER_RGGB16 )
    {
        gen_api_enc = "BayerRG16";
    }
    else if ( ros_enc == sensor_msgs::image_encodings::BAYER_BGGR16 )
    {
        gen_api_enc = "BayerBG16";
    }
    else if ( ros_enc == sensor_msgs::image_encodings::BAYER_GBRG16 )
    {
        gen_api_enc = "BayerGB16";
    }
    else if ( ros_enc == sensor_msgs::image_encodings::BAYER_GRBG16 )
    {
        gen_api_enc = "BayerGR16";
    }
    else if ( ros_enc == sensor_msgs::image_encodings::YUV422 )
    {
        //  This is the UYVY version of YUV422 codec http://www.fourcc.org/yuv.php#UYVY
        //  with an 8-bit depth. Is the same as basler provides
        gen_api_enc = "YUV422Packed"; // --> UYVY implementation
    }
    else
    {
        /* No gen-api pendant existant for following ROS-encodings:*/
        return false;
    }

    // Notes:
    //gen_api_enc = "YCbCr422_8"; --> https://en.wikipedia.org/wiki/YCbCr currently not supported
    //gen_api_enc = "YUV422_YUYV_Packed"; --> This is a YUVY implementation. Currently not supported.

    return true;
}

bool genAPI2Ros(const std::string& gen_api_enc, std::string& ros_enc)
{
    if ( gen_api_enc == "Mono8" )
    {
        ros_enc = sensor_msgs::image_encodings::MONO8;
    }
    else if ( gen_api_enc == "Mono12" )
    {
        ros_enc = sensor_msgs::image_encodings::MONO16;
    }
    else if ( gen_api_enc == "Mono16" )
    {
        ros_enc = sensor_msgs::image_encodings::MONO16;
    }
    else if ( gen_api_enc == "BGR8" )
    {
        ros_enc = sensor_msgs::image_encodings::BGR8;
    }
    else if ( gen_api_enc == "RGB8" )
    {
        ros_enc = sensor_msgs::image_encodings::RGB8;
    }
    else if ( gen_api_enc == "BayerBG8" )
    {
        ros_enc = sensor_msgs::image_encodings::BAYER_BGGR8;
    }
    else if ( gen_api_enc == "BayerGB8" )
    {
        ros_enc = sensor_msgs::image_encodings::BAYER_GBRG8;
    }
    else if ( gen_api_enc == "BayerRG8" )
    {
        ros_enc = sensor_msgs::image_encodings::BAYER_RGGB8;
    }
    else if ( gen_api_enc == "BayerGR8" )
    {
        ros_enc = sensor_msgs::image_encodings::BAYER_GRBG8;
    }
    else if ( gen_api_enc == "BayerRG12" )
    {
        ros_enc = sensor_msgs::image_encodings::BAYER_RGGB16;
    }
    else if ( gen_api_enc == "BayerBG12" )
    {
        ros_enc = sensor_msgs::image_encodings::BAYER_BGGR16;
    }
    else if ( gen_api_enc == "BayerGB12" )
    {
        ros_enc = sensor_msgs::image_encodings::BAYER_GBRG16;
    }
    else if ( gen_api_enc == "BayerGR12" )
    {
        ros_enc = sensor_msgs::image_encodings::BAYER_GRBG16;
    }
    else if ( gen_api_enc == "BayerRG16" )
    {
        ros_enc = sensor_msgs::image_encodings::BAYER_RGGB16;
    }
    else if ( gen_api_enc == "BayerBG16" )
    {
        ros_enc = sensor_msgs::image_encodings::BAYER_BGGR16;
    }
    else if ( gen_api_enc == "BayerGB16" )
    {
        ros_enc = sensor_msgs::image_encodings::BAYER_GBRG16;
    }
    else if ( gen_api_enc == "BayerGR16" )
    {
        ros_enc = sensor_msgs::image_encodings::BAYER_GRBG16;
    }
    else if ( gen_api_enc == "YUV422Packed" )
    {
        ros_enc = sensor_msgs::image_encodings::YUV422;
    }

    /*
        // Currently no ROS equivalents:
        else if ( gen_api_enc == "YUV422_YUYV_Packed" )
        {
            ros_enc = sensor_msgs::image_encodings::YUV422;
        }
        else if ( gen_api_enc == "YCbCr422_8" )
        {
            ros_enc = sensor_msgs::image_encodings::YUV422;
        }
    */


    else
    {
        /* Unsupported are:
         * - Mono10
         * - Mono10p
         * - Mono12p
         * - BayerGR10
         * - BayerGR10p
         * - BayerRG10
         * - BayerRG10p
         * - BayerGB10
         * - BayerGB10p
         * - BayerBG10
         * - BayerBG10p
         * - BayerGR12p
         * - BayerRG12p
         * - BayerGB12p
         * - BayerBG12p
         * - YCbCr422_8
         * - YUV422_YUYV_Packed
         */
        return false;
    }
    return true;
}


bool is_12_bit_gen_api_enc(const std::string& gen_api_enc){
    return ( gen_api_enc == "Mono12" )      || 
           ( gen_api_enc == "BayerRG12" )   ||
           ( gen_api_enc == "BayerBG12" )   ||
           ( gen_api_enc == "BayerGB12" )   ||
           ( gen_api_enc == "BayerGR12" );
}

bool is_12_bit_ros_enc(const std::string& ros_enc){
    std::string gen_api_enc;
    if (ros2GenAPI(ros_enc, gen_api_enc, false)) {
        return is_12_bit_gen_api_enc(gen_api_enc);
    } else {
        return false;
    }
}

}  // namespace encoding_conversions
}  // namespace pylon_camera
