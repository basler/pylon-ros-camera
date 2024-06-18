/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (C) 2022, Basler AG. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * No contributors' name may be used to endorse or promote products derived from
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

#pragma once

#include <string>
#include <vector>

#include "internal/impl/pylon_ros2_camera_gige.hpp"

#include <pylon/BlazeInstantCamera.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

//#include <boost/make_shared.hpp>

#include "pcl_conversions/pcl_conversions.h"

#pragma pack(push, 1)
struct BGR 
{
    uint8_t b;
    uint8_t g;
    uint8_t r;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct Point
{
    float x;
    float y;
    float z;
};
#pragma pack(pop)


namespace pylon_ros2_camera
{

namespace
{
    static const rclcpp::Logger LOGGER_BLAZE = rclcpp::get_logger("basler.pylon.ros2.pylon_ros2_blaze_camera");

    // value that identifies pixel with missing depth information
    constexpr static double  s_invalid_data_value = std::numeric_limits<double>::quiet_NaN();

    // Checks whether a given 3D point represents a valid coordinate.
    // The Scan3dInvalidDataValue is used to identify a non-valid pixel.
    static inline bool isValid(const Point* point)
    {
        static constexpr bool isInvalidValueNaN = s_invalid_data_value != s_invalid_data_value; // true, when s_invalid_data_value equals NaN
        return isInvalidValueNaN ? !std::isnan(point->z) : point->z != s_invalid_data_value;
    }
}

class PylonROS2BlazeCamera : public PylonROS2GigECamera
{
public:
    explicit PylonROS2BlazeCamera(Pylon::IPylonDevice* device);
    virtual ~PylonROS2BlazeCamera();

    virtual bool isBlaze();
    virtual bool registerCameraConfiguration();
    virtual bool openCamera();
    virtual bool applyCamSpecificStartupSettings(const PylonROS2CameraParameter& parameters);

    virtual bool startGrabbing(const PylonROS2CameraParameter& parameters);
    virtual std::string grabbingStarting();
    virtual std::string grabbingStopping();
    virtual bool isCamRemoved();

    virtual bool grabBlaze(sensor_msgs::msg::PointCloud2& cloud_msg,
                           sensor_msgs::msg::Image& intensity_map_msg, 
                           sensor_msgs::msg::Image& depth_map_msg, 
                           sensor_msgs::msg::Image& depth_map_color_msg, 
                           sensor_msgs::msg::Image& confidence_map_msg);
            bool grabBlaze(Pylon::CGrabResultPtr& grab_result);
    virtual bool grab(Pylon::CGrabResultPtr& grab_result);                 // is not used but needs to be implemented
    virtual bool grab(std::vector<uint8_t>& image, rclcpp::Time &stamp);   // is not used but needs to be implemented

            bool processAndConvertBlazeData(const Pylon::CPylonDataContainer& container,
                                            sensor_msgs::msg::PointCloud2& cloud_msg,
                                            sensor_msgs::msg::Image& intensity_map_msg, 
                                            sensor_msgs::msg::Image& depth_map_msg, 
                                            sensor_msgs::msg::Image& depth_map_color_msg, 
                                            sensor_msgs::msg::Image& confidence_map_msg);
            bool convertGrabResultToPointCloud(const Pylon::CPylonDataContainer& container,
                                               sensor_msgs::msg::PointCloud2& cloud_msg);

            // Calculates a grayscale depth map from point cloud data sent from a blaze camera.
            // The buffer that pDepthMap points to must be allocated accordingly before 
            // passing it to the calculateDepthMap function.
            void calculateDepthMap(const Pylon::CPylonDataComponent& pointCloud, int min_depth, int max_depth, uint16_t* pDepthMap);
            // Calculates a color depth map from point cloud data sent from a blaze camera.
            // The buffer that pDepthMap points to must be allocated accordingly before
            // passing it to the calculateDepthMap function.
            void calculateDepthMapColor(const Pylon::CPylonDataComponent& pointCloud, int min_depth, int max_depth, BGR* pDepthMap);
    
    virtual void getInitialCameraInfo(sensor_msgs::msg::CameraInfo& cam_info_msg);
    
    virtual int imagePixelDepth() const;
    virtual float maxPossibleFramerate();
    
    //virtual bool setExposure(const float& target_exposure, float& reached_exposure);
    virtual std::string gammaEnable(const bool& enable);

    virtual std::string setTriggerSelector(const int& mode);
    virtual std::string setTriggerSource(const int& source);

    virtual std::string setLineSelector(const int& value);

    virtual std::string setDeviceLinkThroughputLimitMode(const bool& turnOn);
    virtual std::string setDeviceLinkThroughputLimit(const int& limit);

    // blaze specific
    virtual std::string setDepthMin(const int& depth_min);
    virtual std::string setDepthMax(const int& depth_max);
    virtual std::string setTemporalFilterStrength(const int& strength);
    virtual std::string setOutlierRemovalThreshold(const int& threshold);
    virtual std::string setOutlierRemovalTolerance(const int& tolerance);
    virtual std::string setAmbiguityFilterThreshold(const int& threshold);
    virtual std::string setConfidenceThreshold(const int& threshold);
    virtual std::string setIntensityCalculation(const int& calculation);
    virtual std::string setExposureTimeSelector(const int& selector);
    virtual std::string setOperatingMode(const int& mode);
    virtual std::string setMultiCameraChannel(const int& channel);
    virtual std::string setAcquisitionFrameRate(const float& framerate);
    virtual std::string setScan3dCalibrationOffset(const float& offset);
    virtual std::string enableSpatialFilter(const bool& enable);
    virtual std::string enableTemporalFilter(const bool& enable);
    virtual std::string enableOutlierRemoval(const bool& enable);
    virtual std::string enableAmbiguityFilter(const bool& enable);
    virtual std::string enableThermalDriftCorrection(const bool& enable);
    virtual std::string enableDistortionCorrection(const bool& enable);
    virtual std::string enableAcquisitionFrameRate(const bool& enable);
    virtual std::string enableHDRMode(const bool& enable);
    virtual std::string enableFastMode(const bool& enable);

public:
    Pylon::CBlazeInstantCamera* blaze_cam_;

    // remember current setting in order to restore it when node is shut down
    double invalid_data_value_old_;
};

PylonROS2BlazeCamera::PylonROS2BlazeCamera(Pylon::IPylonDevice* device) :
    PylonROS2GigECamera(device),
    blaze_cam_(new Pylon::CBlazeInstantCamera(device)),
    invalid_data_value_old_(0.0f)
{
    // information logging severity mode
    rcutils_ret_t __attribute__((unused)) res = rcutils_logging_set_logger_level(LOGGER_BLAZE.get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
    //RCUTILS_LOG_SEVERITY_DEBUG
    //RCUTILS_LOG_SEVERITY_INFO
    //RCUTILS_LOG_SEVERITY_WARN
    //RCUTILS_LOG_SEVERITY_ERROR
    //RCUTILS_LOG_SEVERITY_FATAL
}

PylonROS2BlazeCamera::~PylonROS2BlazeCamera()
{
    try
    {
        if (blaze_cam_->IsOpen()) 
        {
            for (auto axis : {  Pylon::BlazeCameraParams_Params::Scan3dCoordinateSelector_CoordinateA, 
                                Pylon::BlazeCameraParams_Params::Scan3dCoordinateSelector_CoordinateB, 
                                Pylon::BlazeCameraParams_Params::Scan3dCoordinateSelector_CoordinateC })
            {
                // Choose axis to configure.
                blaze_cam_->Scan3dCoordinateSelector.SetValue(axis);
                // Set value used to mark missing depth data.
                blaze_cam_->Scan3dInvalidDataValue.SetValue(invalid_data_value_old_);
            }
        }
    }
    catch(GenICam::GenericException& e)
    {
        RCLCPP_DEBUG_STREAM(LOGGER_BLAZE, "Destructor (blaze): Failed to reset properties to default: " << e.GetDescription());
    }
  
    try
    {
        if (blaze_cam_->IsOpen())
        {
            blaze_cam_->Close();
        }
    }
    catch(GenICam::GenericException& e)
    {
        RCLCPP_DEBUG_STREAM(LOGGER_BLAZE, "Destructor (blaze): Failed to properly close camera: " << e.GetDescription());
    }

    if (blaze_cam_)
    {
        delete blaze_cam_;
        blaze_cam_ = nullptr;
    }
}

bool PylonROS2BlazeCamera::isBlaze()
{
    return true;
}

bool PylonROS2BlazeCamera::registerCameraConfiguration()
{
    try
    {
        blaze_cam_->RegisterConfiguration(new Pylon::CBlazeDefaultConfiguration,
                                          Pylon::RegistrationMode_ReplaceAll,
                                          Pylon::Cleanup_Delete);
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "An exception occurred while registering camera configuration:" << e.GetDescription());
        return false;
    }

    return true;
}

bool PylonROS2BlazeCamera::openCamera()
{
    try
    {
        blaze_cam_->Open();
        RCLCPP_DEBUG_STREAM(LOGGER_BLAZE, "Connected to camera " << blaze_cam_->GetDeviceInfo().GetFriendlyName());
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "An exception occurred while opening camera:" << e.GetDescription());
        return false;
    }

    return true;
}

bool PylonROS2BlazeCamera::applyCamSpecificStartupSettings(const PylonROS2CameraParameter& parameters)
{
    try
    {
        this->grabbingStarting();
        blaze_cam_->StopGrabbing();

        // Display info about the connected camera.
        RCLCPP_DEBUG_STREAM(LOGGER_BLAZE, "Infos about connected camera:");
        RCLCPP_DEBUG_STREAM(LOGGER_BLAZE, "-> ID:                " << blaze_cam_->GetDeviceInfo().GetDeviceID().c_str());
        RCLCPP_DEBUG_STREAM(LOGGER_BLAZE, "-> Model name:        " << blaze_cam_->GetDeviceInfo().GetModelName().c_str());
        RCLCPP_DEBUG_STREAM(LOGGER_BLAZE, "-> Display name:      " << blaze_cam_->GetDeviceInfo().GetFriendlyName().c_str());
        RCLCPP_DEBUG_STREAM(LOGGER_BLAZE, "-> Serial number:     " << blaze_cam_->GetDeviceInfo().GetSerialNumber().c_str());
        RCLCPP_DEBUG_STREAM(LOGGER_BLAZE, "-> User defined name: " << blaze_cam_->GetDeviceInfo().GetUserDefinedName().c_str());
        RCLCPP_DEBUG_STREAM(LOGGER_BLAZE, "-> IP address:        " << blaze_cam_->GetDeviceInfo().GetIpAddress().c_str());

        // Set up acquisition.
        
        // Enable depth data
        blaze_cam_->ComponentSelector.SetValue(Pylon::BlazeCameraParams_Params::ComponentSelector_Range);
        blaze_cam_->ComponentEnable.SetValue(true);
        blaze_cam_->PixelFormat.SetValue(Pylon::BlazeCameraParams_Params::PixelFormat_Coord3D_ABC32f);

        // Enable intensity image
        blaze_cam_->ComponentSelector.SetValue(Pylon::BlazeCameraParams_Params::ComponentSelector_Intensity);
        blaze_cam_->ComponentEnable.SetValue(true);
        blaze_cam_->PixelFormat.SetValue(Pylon::BlazeCameraParams_Params::PixelFormat_Mono16);

        // Enable confidence map
        blaze_cam_->ComponentSelector.SetValue(Pylon::BlazeCameraParams_Params::ComponentSelector_Confidence);
        blaze_cam_->ComponentEnable.SetValue(true);
        blaze_cam_->PixelFormat.SetValue(Pylon::BlazeCameraParams_Params::PixelFormat_Confidence16);

        invalid_data_value_old_ = blaze_cam_->Scan3dInvalidDataValue.GetValue();

        for (auto axis : {  Pylon::BlazeCameraParams_Params::Scan3dCoordinateSelector_CoordinateA, 
                            Pylon::BlazeCameraParams_Params::Scan3dCoordinateSelector_CoordinateB, 
                            Pylon::BlazeCameraParams_Params::Scan3dCoordinateSelector_CoordinateC })
        {
            // Choose axis to configure.
            blaze_cam_->Scan3dCoordinateSelector.SetValue(axis);
            // Set value used to mark missing depth data.
            blaze_cam_->Scan3dInvalidDataValue.SetValue(s_invalid_data_value);
        }

        // set up some default parameters
        
        blaze_cam_->TriggerSource.SetValue(Pylon::BlazeCameraParams_Params::TriggerSource_Software);
        blaze_cam_->TriggerMode.SetValue(Pylon::BlazeCameraParams_Params::TriggerMode_On);

        auto value = blaze_cam_->AcquisitionFrameRate.GetValue();
        RCLCPP_INFO_STREAM_ONCE(LOGGER_BLAZE, "blaze currently set frame rate: " << value << " fps (range ["
                                                    << blaze_cam_->AcquisitionFrameRate.GetMin() << ", "
                                                    << blaze_cam_->AcquisitionFrameRate.GetMax() << "])");

        RCLCPP_INFO_STREAM(LOGGER_BLAZE, "blaze has exposure time range: ["
                    << blaze_cam_->ExposureTime.GetMin() << " - " 
                    << blaze_cam_->ExposureTime.GetMax()
                    << "] measured in microseconds.");

        RCLCPP_INFO_STREAM(LOGGER_BLAZE, "blaze has depth range: ["
                    << blaze_cam_->DepthMin.GetValue() << " - " 
                    << blaze_cam_->DepthMax.GetValue()
                    << "] measured in millimeters.");
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "An exception occurred while applying specific startup settings:" << e.GetDescription());
        return e.GetDescription();
    }

    return true;
}

bool PylonROS2BlazeCamera::startGrabbing(const PylonROS2CameraParameter& parameters)
{
    try
    {
        this->grabbingStarting();

        device_user_id_ = blaze_cam_->DeviceUserID.GetValue();
        img_rows_ = static_cast<size_t>(blaze_cam_->Height.GetValue());
        img_cols_ = static_cast<size_t>(blaze_cam_->Width.GetValue());
        img_size_byte_ = img_cols_ * img_rows_ * imagePixelDepth();

        grab_timeout_ = parameters.grab_timeout_;
        trigger_timeout = parameters.trigger_timeout_;
        RCLCPP_DEBUG_STREAM_ONCE(LOGGER_BLAZE, "Grab timeout for blaze: " << grab_timeout_);
        RCLCPP_DEBUG_STREAM_ONCE(LOGGER_BLAZE, "Trigger timeout for blaze: " << trigger_timeout);

        Pylon::CGrabResultPtr grab_result;
        this->grabBlaze(grab_result);
        
        if (grab_result.IsValid())
        {
            is_ready_ = true;
        }
        else
        {
            RCLCPP_ERROR(LOGGER_BLAZE, "PylonROS2BlazeCamera not ready because the result of the initial grab is invalid");
        }

    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "An exception occurred while starting image grabbing:" << e.GetDescription());
        return false;
    }

    return true;
}

std::string PylonROS2BlazeCamera::grabbingStarting()
{
    try
    {
        blaze_cam_->StartGrabbing();
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "An exception occurred while starting image grabbing:" << e.GetDescription());
        return e.GetDescription();
    }

    return "done";
}

std::string PylonROS2BlazeCamera::grabbingStopping()
{
    try
    {
        blaze_cam_->StopGrabbing();
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "An exception occurred while stopping image grabbing:" << e.GetDescription());
        return e.GetDescription();
    }

    return "done";
}

bool PylonROS2BlazeCamera::isCamRemoved()
{
    return cam_->IsCameraDeviceRemoved();
}

bool PylonROS2BlazeCamera::grabBlaze(sensor_msgs::msg::PointCloud2& cloud_msg,
                                     sensor_msgs::msg::Image& intensity_map_msg, 
                                     sensor_msgs::msg::Image& depth_map_msg, 
                                     sensor_msgs::msg::Image& depth_map_color_msg, 
                                     sensor_msgs::msg::Image& confidence_map_msg)
{
    Pylon::CGrabResultPtr ptr_grab_result;
    if (!this->grabBlaze(ptr_grab_result))
    {   
        RCLCPP_ERROR(LOGGER_BLAZE, "Grabbing with blaze failed");
        return false;
    }

    // process the acquired data
    auto container = ptr_grab_result->GetDataContainer();
    this->processAndConvertBlazeData(container, cloud_msg, intensity_map_msg, depth_map_msg, depth_map_color_msg, confidence_map_msg);

    return true;
}

bool PylonROS2BlazeCamera::grabBlaze(Pylon::CGrabResultPtr& grab_result)
{
    if (!blaze_cam_->IsGrabbing())
    {
        return false;
    }

    try
    {
        if (blaze_cam_->TriggerMode.GetValue() == Pylon::BlazeCameraParams_Params::TriggerMode_On)
        {
            // The blaze does not support waiting for frame trigger ready.
            blaze_cam_->ExecuteSoftwareTrigger();
        }
        
        blaze_cam_->RetrieveResult(grab_timeout_, grab_result, Pylon::TimeoutHandling_ThrowException);
    }
    catch (const GenICam::GenericException &e)
    {
        if (blaze_cam_->IsCameraDeviceRemoved())
        {   
            RCLCPP_ERROR(LOGGER_BLAZE, "Lost connection to the camera...");
        }
        else
        {   
            if ((blaze_cam_->TriggerSource.GetValue() != Pylon::BlazeCameraParams_Params::TriggerSource_Software) && (blaze_cam_->TriggerMode.GetValue() == Pylon::BlazeCameraParams_Params::TriggerMode_On))
            {
                RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "Waiting for Trigger signal");
            }
            else 
            {
                RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "An image grabbing exception in pylon camera occurred: " << e.GetDescription());
            }
        }
        
        return false;
    }
    catch (...)
    {   
        RCLCPP_ERROR(LOGGER_BLAZE, "An unspecified image grabbing exception occurred");
        return false;
    }

    if (!grab_result->GrabSucceeded())
    {   
        RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "Error: " << grab_result->GetErrorCode() << " " << grab_result->GetErrorDescription());
        return false;
    }

    return true;
}

bool PylonROS2BlazeCamera::processAndConvertBlazeData(const Pylon::CPylonDataContainer& container,
                                                      sensor_msgs::msg::PointCloud2& cloud_msg,
                                                      sensor_msgs::msg::Image& intensity_map_msg,
                                                      sensor_msgs::msg::Image& depth_map_msg,
                                                      sensor_msgs::msg::Image& depth_map_color_msg,
                                                      sensor_msgs::msg::Image& confidence_map_msg)
{
    // some first checks
    if (container.GetDataComponentCount() != 3)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "Expected 3 parts, got " << container.GetDataComponentCount());
        return false;
    }

    auto range_component = container.GetDataComponent(0);
    auto intensity_component = container.GetDataComponent(1);
    auto confidence_component = container.GetDataComponent(2);

    if (range_component.GetPixelType() != Pylon::PixelType_Coord3D_ABC32f)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "Unexpected data format for the first image part. Coord3D_ABC32f is expected.");
        return false;
    }

    if (intensity_component.GetPixelType() != Pylon::PixelType_Mono16)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "Unexpected data format for the second image part. Mono16 is expected.");
        return false;
    }

    // processing and converting data

    const int width = range_component.GetWidth();
    const int height = range_component.GetHeight();
    // Get min and max depth values.
    int min_depth = blaze_cam_->DepthMin.GetValue();
    int max_depth = blaze_cam_->DepthMax.GetValue();

    // point cloud
    this->convertGrabResultToPointCloud(container, cloud_msg);
    
    // intensity
    cv::Mat intensity_map = cv::Mat(height, width, CV_16UC1, (void*) intensity_component.GetData());
    // Scale the intensity image since it often looks quite dark.
    double  max;
    cv::minMaxLoc(intensity_map, NULL, &max);
    intensity_map /= (max / std::numeric_limits<uint16_t>::max());
    // convert
    cv_bridge::CvImage intensity_cv_img;
    intensity_cv_img.encoding = sensor_msgs::image_encodings::MONO16;
    intensity_cv_img.image    = intensity_map;
    // define message
    intensity_map_msg.header = intensity_cv_img.toImageMsg()->header;
    intensity_map_msg.height = intensity_cv_img.toImageMsg()->height;
    intensity_map_msg.width = intensity_cv_img.toImageMsg()->width;
    intensity_map_msg.encoding = intensity_cv_img.toImageMsg()->encoding;
    intensity_map_msg.is_bigendian = intensity_cv_img.toImageMsg()->is_bigendian;
    intensity_map_msg.step = intensity_cv_img.toImageMsg()->step;
    intensity_map_msg.data = intensity_cv_img.toImageMsg()->data;

    // depth map
    uint16_t* pdepth_data = new uint16_t[width * height];
    this->calculateDepthMap(range_component, min_depth, max_depth, pdepth_data);
    cv::Mat depth_map = cv::Mat(height, width, CV_16UC1, pdepth_data);
    // convert
    cv_bridge::CvImage depth_map_cv_img;
    depth_map_cv_img.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    depth_map_cv_img.image    = depth_map;
    // define message
    depth_map_msg.header = depth_map_cv_img.toImageMsg()->header;
    depth_map_msg.height = depth_map_cv_img.toImageMsg()->height;
    depth_map_msg.width = depth_map_cv_img.toImageMsg()->width;
    depth_map_msg.encoding = depth_map_cv_img.toImageMsg()->encoding;
    depth_map_msg.is_bigendian = depth_map_cv_img.toImageMsg()->is_bigendian;
    depth_map_msg.step = depth_map_cv_img.toImageMsg()->step;
    depth_map_msg.data = depth_map_cv_img.toImageMsg()->data;
    // free memory
    free(pdepth_data);

    // depth map color
    BGR* pdepth_data_color = new BGR[width * height];
    this->calculateDepthMapColor(range_component, min_depth, max_depth, pdepth_data_color);
    cv::Mat depth_map_color = cv::Mat(height, width, CV_8UC3, pdepth_data_color);
    // convert
    cv_bridge::CvImage depth_map_color_cv_img;
    depth_map_color_cv_img.encoding = sensor_msgs::image_encodings::BGR8;
    depth_map_color_cv_img.image    = depth_map_color;
    // define message
    depth_map_color_msg.header = depth_map_color_cv_img.toImageMsg()->header;
    depth_map_color_msg.height = depth_map_color_cv_img.toImageMsg()->height;
    depth_map_color_msg.width = depth_map_color_cv_img.toImageMsg()->width;
    depth_map_color_msg.encoding = depth_map_color_cv_img.toImageMsg()->encoding;
    depth_map_color_msg.is_bigendian = depth_map_color_cv_img.toImageMsg()->is_bigendian;
    depth_map_color_msg.step = depth_map_color_cv_img.toImageMsg()->step;
    depth_map_color_msg.data = depth_map_color_cv_img.toImageMsg()->data;
    // free memory
    free(pdepth_data_color);

    // confidence map
    cv::Mat confidence_map = cv::Mat(height, width, CV_16UC1, (void*) confidence_component.GetData());
    // convert
    cv_bridge::CvImage confidence_cv_img;
    confidence_cv_img.encoding = sensor_msgs::image_encodings::MONO16;
    confidence_cv_img.image    = confidence_map;
    // define message
    confidence_map_msg.header = confidence_cv_img.toImageMsg()->header;
    confidence_map_msg.height = confidence_cv_img.toImageMsg()->height;
    confidence_map_msg.width = confidence_cv_img.toImageMsg()->width;
    confidence_map_msg.encoding = confidence_cv_img.toImageMsg()->encoding;
    confidence_map_msg.is_bigendian = confidence_cv_img.toImageMsg()->is_bigendian;
    confidence_map_msg.step = confidence_cv_img.toImageMsg()->step;
    confidence_map_msg.data = confidence_cv_img.toImageMsg()->data;

    return true;
}

bool PylonROS2BlazeCamera::convertGrabResultToPointCloud(const Pylon::CPylonDataContainer& container,
                                                         sensor_msgs::msg::PointCloud2& cloud_msg)
{
    // An organized point cloud is used, i.e., for each camera pixel there is an entry 
    // in the data structure indicating the 3D coordinates calculated from that pixel.
    // If the camera wasn't able to create depth information for a pixel, the x, y, and z coordinates 
    // are set to NaN. These NaNs will be retained in the PCL point cloud.

    auto range_component = container.GetDataComponent(0);
    auto intensity_component = container.GetDataComponent(1);

    const size_t width = range_component.GetWidth();
    const size_t height = range_component.GetHeight();

    // allocate PCL point cloud.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ppoint_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    ppoint_cloud->width = width;
    ppoint_cloud->height = height;
    ppoint_cloud->points.resize(width * height);
    ppoint_cloud->is_dense = false; // organized point cloud

    // Create a pointer to the 3D coordinates of the first point.
    // imgParts[0] always refers to the point cloud data.
    Point* psrc_point = (Point*) range_component.GetData();

    // Create a pointer to the intensity information stored in the second buffer part.
    uint16_t* pintensity = (uint16_t*)intensity_component.GetData();

    // Set the points.
    for (size_t i = 0; i < height * width; ++i, ++psrc_point, ++pintensity)
    {
        // Set the X/Y/Z cordinates.
        pcl::PointXYZRGB& dst_point = ppoint_cloud->points[i];

        dst_point.x = psrc_point->x * 0.001;
        dst_point.y = psrc_point->y * 0.001;
        dst_point.z = psrc_point->z * 0.001;

        // Use the intensity value of the pixel for coloring the point.
        dst_point.r = dst_point.g = dst_point.b = (uint8_t)(*pintensity >> 8);
    }

    // convert from pcl to ros
    pcl::toROSMsg(*ppoint_cloud, cloud_msg);

    return true;
}

void PylonROS2BlazeCamera::calculateDepthMap(const Pylon::CPylonDataComponent& pointCloud, int min_depth, int max_depth, uint16_t* pDepthMap)
{
    const int width = pointCloud.GetWidth();
    const int height = pointCloud.GetHeight();
    const Point *pPoint = reinterpret_cast<const Point*>(pointCloud.GetData());

    const double scale = 65535.0 / (max_depth - min_depth);

    for (int row = 0; row < height; ++row)
    {
        for (int col = 0; col < width; ++col, ++pPoint, ++pDepthMap)
        {
            if (isValid(pPoint))
            {
                // Calculate the radial distance.
                //double distance = sqrt(pPoint->x * pPoint->x + pPoint->y * pPoint->y + pPoint->z * pPoint->z);
                double distance = pPoint->z * this->blaze_cam_->Scan3dCoordinateScale.GetValue();
                // Clip to [min_depth..MaxDept].
                if (distance < min_depth)
                    distance = min_depth;
                else if (distance > max_depth)
                    distance = max_depth;
                *pDepthMap = (uint16_t) ( ( distance - min_depth ) * scale );
            }
            else
            {
                // No depth information available for this pixel. Zero it.
                *pDepthMap = 0;
            }
        }
    }
}

void PylonROS2BlazeCamera::calculateDepthMapColor(const Pylon::CPylonDataComponent& pointCloud, int min_depth, int max_depth, BGR* pDepthMap)
{
    const int width = pointCloud.GetWidth();
    const int height = pointCloud.GetHeight();
    const Point *pPoint = reinterpret_cast<const Point*>(pointCloud.GetData());

    const double scale = 65535.0 / (max_depth - min_depth);

    for (int row = 0; row < height; ++row)
    {
        for (int col = 0; col < width; ++col, ++pPoint, ++pDepthMap)
        {
            if (isValid(pPoint))
            {
                // Calculate the radial distance.
                double distance = sqrt(pPoint->x * pPoint->x + pPoint->y * pPoint->y + pPoint->z * pPoint->z);

                // Clip to [min_depth..MaxDept].
                if (distance < min_depth)
                    distance = min_depth;
                else if (distance > max_depth)
                    distance = max_depth;
                
                // Calculate the color.
                BGR bgr;
                const uint16_t g = (uint16_t)((distance - min_depth) * scale);
                const uint16_t val = g >> 6 & 0xff;
                const uint16_t sel = g >> 14;
                uint32_t res = val << 8 | 0xff;
                if (sel & 0x01)
                {
                    res = (~res) >> 8 & 0xffff;
                }
                if (sel & 0x02)
                {
                    res = res << 8;
                }
                bgr.r = res & 0xff;
                res = res >> 8;
                bgr.g = res & 0xff;
                res = res >> 8;
                bgr.b = res & 0xff;

                *pDepthMap = bgr;
            }
            else
            {
                // No depth information available for this pixel. Set it to black.
                BGR bgr;
                bgr.r = bgr.g = bgr.b = 0;
                *pDepthMap = bgr;
            }
        }
    }
}

void PylonROS2BlazeCamera::getInitialCameraInfo(sensor_msgs::msg::CameraInfo& cam_info_msg)
{
    // https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/CameraInfo.msg

    // The image dimensions with which the camera was calibrated. Normally
    // this will be the full camera resolution in pixels.
    cam_info_msg.height = this->imageRows();
    cam_info_msg.width = this->imageCols();

    // The distortion model used. Supported models are listed in
    // sensor_msgs/distortion_models.h. For most cameras, "plumb_bob" - a
    // simple model of radial and tangential distortion - is sufficient.
    cam_info_msg.distortion_model = "plumb_bob";

    // The distortion parameters, size depending on the distortion model.
    // For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3) -> float64[] d.
    cam_info_msg.d = std::vector<double>(5, 0.);

    const double f = blaze_cam_->Scan3dFocalLength.GetValue();
    const double cx = blaze_cam_->Scan3dPrincipalPointU.GetValue();
    const double cy = blaze_cam_->Scan3dPrincipalPointV.GetValue();

    // Intrinsic camera matrix for the raw (distorted) images.
    //     [fx  0 cx]
    // K = [ 0 fy cy]  --> 3x3 row-major matrix
    //     [ 0  0  1]
    // Projects 3D points in the camera coordinate frame to 2D pixel coordinates
    // using the focal lengths (fx, fy) and principal point (cx, cy) -> float64[9] k.
    cam_info_msg.k = {f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0};
    
    // Rectification matrix (stereo cameras only)
    // A rotation matrix aligning the camera coordinate system to the ideal
    // stereo image plane so that epipolar lines in both stereo images are parallel -> float64[9] r, 3x3 row-major matrix.
    cam_info_msg.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

    // Projection/camera matrix
    //     [fx'  0  cx' Tx]
    // P = [ 0  fy' cy' Ty]  --> # 3x4 row-major matrix
    //     [ 0   0   1   0]
    // By convention, this matrix specifies the intrinsic (camera) matrix of the
    // processed (rectified) image. That is, the left 3x3 portion is the normal
    // camera intrinsic matrix for the rectified image. It projects 3D points
    // in the camera coordinate frame to 2D pixel coordinates using the focal
    // lengths (fx', fy') and principal point (cx', cy') - these may differ from
    // the values in K. For monocular cameras, Tx = Ty = 0. Normally, monocular
    // cameras will also have R = the identity and P[1:3,1:3] = K.
    // For a stereo pair, the fourth column [Tx Ty 0]' is related to the
    // position of the optical center of the second camera in the first
    // camera's frame. We assume Tz = 0 so both cameras are in the same
    // stereo image plane. The first camera always has Tx = Ty = 0.
    // For the right (second) camera of a horizontal stereo pair,
    // Ty = 0 and Tx = -fx' * B, where B is the baseline between the cameras.
    // Given a 3D point [X Y Z]', the projection (x, y) of the point onto the
    // rectified image is given by:
    // [u v w]' = P * [X Y Z 1]'
    //        x = u / w
    //        y = v / w
    //  This holds for both images of a stereo pair -> float64[12]
    cam_info_msg.p = {f, 0.0, cx, 0.0, 0.0, f, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
}

int PylonROS2BlazeCamera::imagePixelDepth() const
{
    // there's no pixel size for the blaze
    // intensity and confidence pixels are coded over 2 bytes    
    return 2;
}

float PylonROS2BlazeCamera::maxPossibleFramerate()
{
    if (blaze_cam_->FastMode.GetValue())
    {
        RCLCPP_DEBUG_STREAM_ONCE(LOGGER_BLAZE, "blaze max frame rate (Fast Mode): " << blaze_cam_->AcquisitionFrameRate.GetMax() << " fps");
    }
    else
    {
        RCLCPP_DEBUG_STREAM_ONCE(LOGGER_BLAZE, "blaze max frame rate (Default Mode) " << blaze_cam_->AcquisitionFrameRate.GetMax() << " fps");
    }

    return blaze_cam_->AcquisitionFrameRate.GetMax();
}

std::string PylonROS2BlazeCamera::gammaEnable(const bool& enable)
{
    try
    {
        blaze_cam_->GammaCorrection.SetValue(enable);
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "An exception while enabling/disabling gamma correction occurred:" << e.GetDescription());
        return e.GetDescription();
    }

    return "done";
}

std::string PylonROS2BlazeCamera::setTriggerSelector(const int& mode)
{
    try
    {
        if (GenApi::IsAvailable(blaze_cam_->TriggerSelector))
        {  
            if (mode == 0)
            {
                blaze_cam_->TriggerSelector.SetValue(Pylon::BlazeCameraParams_Params::TriggerSelector_FrameStart);
                RCLCPP_INFO_STREAM(LOGGER_BLAZE, "Trigger selector: Frame Start");
            }
            else
            {
                return "Error: unknown value";
            }
        }
        else 
        {
             RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "Error while trying to change the Acquisition frame count. The connected Camera not supporting this feature");
             return "The connected Camera not supporting this feature";
        }

    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "An exception while changing Acquisition frame count occurred:" << e.GetDescription());
        return e.GetDescription();
    }

    return "done";
}

std::string PylonROS2BlazeCamera::setTriggerSource(const int& source)
{
    try
    {   if (GenApi::IsAvailable(blaze_cam_->TriggerSource))
        {
            switch (source)
            {
                case 0:
                    blaze_cam_->TriggerSource.SetValue(Pylon::BlazeCameraParams_Params::TriggerSource_Software);
                    RCLCPP_INFO_STREAM(LOGGER_BLAZE, "Trigger source: Software");
                    break;
                //case 1:
                //    blaze_cam_->TriggerSource.SetValue(Pylon::BlazeCameraParams_Params::TriggerSource_Line0);
                //    RCLCPP_INFO_STREAM(LOGGER_BLAZE, "Trigger source: Line 0");
                //    break;
                default:
                    //RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "Trigger source value is invalid! Please choose between 0 -> Trigger Software / 1 -> Line 0");
                    RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "Trigger source value is invalid! Please choose 0 -> Trigger Software");
                    return "Error: unknown value for trigger source";
            }
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "Error while trying to change the trigger source. The connected camera does not support this feature");
            return "The connected camera does not support this feature";
        }

    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "An exception while setting the trigger source occurred:" << e.GetDescription());
        return e.GetDescription(); 
    }
    return "done";
}

std::string PylonROS2BlazeCamera::setLineSelector(const int& value)
{
    try
    {   
        if (GenApi::IsAvailable(blaze_cam_->LineSelector))
        {
            if (value == 0)
            {
                blaze_cam_->LineSelector.SetValue(Pylon::BlazeCameraParams_Params::LineSelector_Line0);
            }
            else if (value == 1)
            {
                blaze_cam_->LineSelector.SetValue(Pylon::BlazeCameraParams_Params::LineSelector_Line1);
            }
            else 
            {
                return "Error: unknown value";
            }
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "Error while trying to set the line selector. The connected Camera not supporting this feature");
            return "The connected Camera not supporting this feature";
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "An exception while setting the line selector occurred:" << e.GetDescription());
        return e.GetDescription(); 
    }

    return "done";
}

std::string PylonROS2BlazeCamera::setDeviceLinkThroughputLimitMode(const bool& turnOn)
{
    try
    {
        if (GenApi::IsAvailable(cam_->DeviceLinkThroughputLimitMode))
        {  
            if (turnOn)
            {
                cam_->DeviceLinkThroughputLimitMode.SetValue(Basler_UniversalCameraParams::DeviceLinkThroughputLimitModeEnums::DeviceLinkThroughputLimitMode_On);
            }
            else 
            {
                cam_->DeviceLinkThroughputLimitMode.SetValue(Basler_UniversalCameraParams::DeviceLinkThroughputLimitModeEnums::DeviceLinkThroughputLimitMode_Off);
            }
        }
        else 
        {
             RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "Error while trying to change the device link throughput limit mode. The connected Camera not supporting this feature");
             return "The connected Camera not supporting this feature";
        }

    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "An exception while changing the device link throughput limit mode occurred:" << e.GetDescription());
        return e.GetDescription();
    }

    return "done";
}

std::string PylonROS2BlazeCamera::setDeviceLinkThroughputLimit(const int& limit)
{
    try
    {
        if (GenApi::IsAvailable(cam_->DeviceLinkThroughputLimit))
        {
            cam_->DeviceLinkThroughputLimit.SetValue(limit);
        }
        else 
        {
            RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "Error while trying to change the device link throughput limit. The connected Camera not supporting this feature");
            return "The connected Camera not supporting this feature";
        }
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "An exception while changing the device link throughput limit occurred:" << e.GetDescription());
        return e.GetDescription();
    }

    return "done";
}

std::string PylonROS2BlazeCamera::setDepthMin(const int& depth_min)
{
    try
    {
        blaze_cam_->DepthMin.SetValue(depth_min);
        RCLCPP_DEBUG_STREAM(LOGGER_BLAZE, "Depth min set to " << depth_min);
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "An exception while changing the depth min occurred: " << e.GetDescription());
        return e.GetDescription();
    }

    return "done";
}

std::string PylonROS2BlazeCamera::setDepthMax(const int& depth_max)
{
    try
    {
        blaze_cam_->DepthMax.SetValue(depth_max);
        RCLCPP_DEBUG_STREAM(LOGGER_BLAZE, "Depth max set to " << depth_max);
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "An exception while changing the depth max occurred: " << e.GetDescription());
        return e.GetDescription();
    }

    return "done";
}

std::string PylonROS2BlazeCamera::setTemporalFilterStrength(const int& strength)
{
    try
    {
        blaze_cam_->TemporalFilterStrength.SetValue(strength);
        RCLCPP_DEBUG_STREAM(LOGGER_BLAZE, "Temporal filter strength set to " << strength);
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "An exception while changing the temporal filter strength occurred: " << e.GetDescription());
        return e.GetDescription();
    }

    return "done";
}

std::string PylonROS2BlazeCamera::setOutlierRemovalThreshold(const int& threshold)
{
    try
    {
        blaze_cam_->OutlierRemovalThreshold.SetValue(threshold);
        RCLCPP_DEBUG_STREAM(LOGGER_BLAZE, "Outlier removal threshold set to " << threshold);
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "An exception while changing the outlier removal threshold occurred: " << e.GetDescription());
        return e.GetDescription();
    }

    return "done";
}

std::string PylonROS2BlazeCamera::setOutlierRemovalTolerance(const int& tolerance)
{
    try
    {
        blaze_cam_->OutlierRemovalTolerance.SetValue(tolerance);
        RCLCPP_DEBUG_STREAM(LOGGER_BLAZE, "Outlier removal tolerance set to " << tolerance);
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "An exception while changing the outlier removal tolerance occurred: " << e.GetDescription());
        return e.GetDescription();
    }

    return "done";
}

std::string PylonROS2BlazeCamera::setAmbiguityFilterThreshold(const int& threshold)
{
    try
    {
        blaze_cam_->AmbiguityFilterThreshold.SetValue(threshold);
        RCLCPP_DEBUG_STREAM(LOGGER_BLAZE, "Ambiguity filter threshold set to " << threshold);
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "An exception while changing the ambiguity filter threshold occurred: " << e.GetDescription());
        return e.GetDescription();
    }

    return "done";
}

std::string PylonROS2BlazeCamera::setConfidenceThreshold(const int& threshold)
{
    try
    {
        blaze_cam_->ConfidenceThreshold.SetValue(threshold);
        RCLCPP_DEBUG_STREAM(LOGGER_BLAZE, "Confidence threshold set to " << threshold);
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "An exception while changing the confidence threshold occurred: " << e.GetDescription());
        return e.GetDescription();
    }

    return "done";
}

std::string PylonROS2BlazeCamera::setIntensityCalculation(const int& calculation)
{
    try
    {
        switch (calculation)
        {
            case 0:
                RCLCPP_WARN(LOGGER_BLAZE, "IntensityCalculation_Method1 answers to calculation = 1 and not 0");
                return "IntensityCalculation_Method1 answers to calculation = 1 and not 0";
            case 1:
                blaze_cam_->IntensityCalculation.SetValue(Pylon::BlazeCameraParams_Params::IntensityCalculation_Method1);
                RCLCPP_DEBUG(LOGGER_BLAZE, "Intensity calculation is now set to Method1");
                break;
            case 2:
                blaze_cam_->IntensityCalculation.SetValue(Pylon::BlazeCameraParams_Params::IntensityCalculation_Method2);
                RCLCPP_DEBUG(LOGGER_BLAZE, "Intensity calculation is now set to Method2");
                break;
            default:
                RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "Intensity calculation value is invalid! Please choose between 1 -> Method1 / 2 -> Method2");
                return "Intensity calculation value is invalid! Please choose between 1 -> Method1 / 2 -> Method2";
        }
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "An exception while changing the intensity calculation occurred: " << e.GetDescription());
        return e.GetDescription();
    }

    return "done";
}

std::string PylonROS2BlazeCamera::setExposureTimeSelector(const int& selector)
{
    try
    {
        if (blaze_cam_->HDRMode.GetValue())
        {
            switch (selector)
            {
                case 0:
                    RCLCPP_WARN(LOGGER_BLAZE, "ExposureTimeSelector_Stage1 answers to selector = 1 and not 0");
                    return "ExposureTimeSelector_Stage1 answers to selector = 1 and not 0";
                case 1:
                    blaze_cam_->ExposureTimeSelector.SetValue(Pylon::BlazeCameraParams_Params::ExposureTimeSelector_Stage1);
                    RCLCPP_DEBUG(LOGGER_BLAZE, "Exposure time selector is now set to Stage1");
                    break;
                case 2:
                    blaze_cam_->ExposureTimeSelector.SetValue(Pylon::BlazeCameraParams_Params::ExposureTimeSelector_Stage2);
                    RCLCPP_DEBUG(LOGGER_BLAZE, "Exposure time selector is now set to Stage2");
                    break;
                default:
                    RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "Exposure time selector value is invalid! Please choose between 1 -> Stage1 / 2 -> Stage2");
                    return "Exposure time selector value is invalid! Please choose between 1 -> Stage1 / 2 -> Stage2";
            }
        }
        else
        {
            RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "To change exposure time selector, HDR mode must first be enabled");
            return "To change exposure time selector, HDR mode must first be enabled";
        }
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "An exception while changing the exposure time selector occurred: " << e.GetDescription());
        return e.GetDescription();
    }

    return "done";
}

std::string PylonROS2BlazeCamera::setOperatingMode(const int& mode)
{
    try
    {
        switch (mode)
        {
            case 0:
                blaze_cam_->OperatingMode.SetValue(Pylon::BlazeCameraParams_Params::OperatingMode_LongRange);
                RCLCPP_DEBUG(LOGGER_BLAZE, "Operating mode is now set to LongRange");
                break;
            case 1:
                blaze_cam_->OperatingMode.SetValue(Pylon::BlazeCameraParams_Params::OperatingMode_ShortRange);
                RCLCPP_DEBUG(LOGGER_BLAZE, "Operating mode is now set to ShortRange");
                break;
            default:
                RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "Operating mode value is invalid! Please choose between 0 -> LongRange / 1 -> ShortRange");
                return "Operating mode value is invalid! Please choose between 0 -> LongRange / 1 -> ShortRange";
        }
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "An exception while changing the operating mode occurred: " << e.GetDescription());
        return e.GetDescription();
    }

    return "done";
}

std::string PylonROS2BlazeCamera::setMultiCameraChannel(const int& channel)
{
    try
    {
        blaze_cam_->MultiCameraChannel.SetValue(channel);
        RCLCPP_DEBUG_STREAM(LOGGER_BLAZE, "Multi camera channel set to " << channel);
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "An exception while changing the multi camera channel occurred: " << e.GetDescription());
        return e.GetDescription();
    }

    return "done";
}

std::string PylonROS2BlazeCamera::setAcquisitionFrameRate(const float& framerate)
{
    try
    {
        if (blaze_cam_->AcquisitionFrameRateEnable.GetValue())
        {
            blaze_cam_->AcquisitionFrameRate.SetValue(framerate);
            RCLCPP_DEBUG_STREAM(LOGGER_BLAZE, "Acquisition frame rate set to " << framerate);
        }
        else
        {
            RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "To change acquisition frame rate, it must first be enabled");
            return "To change acquisition frame rate, it must first be enabled";
        }
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "An exception while changing the acquisition frame rate occurred: " << e.GetDescription());
        return e.GetDescription();
    }

    return "done";
}

std::string PylonROS2BlazeCamera::setScan3dCalibrationOffset(const float& offset)
{
    try
    {
        blaze_cam_->Scan3dCalibrationOffset.SetValue(offset);
        RCLCPP_DEBUG_STREAM(LOGGER_BLAZE, "Scan 3d calibration offset set to " << offset);
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "An exception while changing the scan 3d calibration offset occurred: " << e.GetDescription());
        return e.GetDescription();
    }

    return "done";
}

std::string PylonROS2BlazeCamera::enableSpatialFilter(const bool& enable)
{
    try
    {
        blaze_cam_->SpatialFilter.SetValue(enable);
        if (enable)
            RCLCPP_DEBUG(LOGGER_BLAZE, "Spatial filter is enabled");
        else
            RCLCPP_DEBUG(LOGGER_BLAZE, "Spatial filter is disabled");
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "An exception while enabling/disabling spatial filter occurred: " << e.GetDescription());
        return e.GetDescription();
    }

    return "done";
}

std::string PylonROS2BlazeCamera::enableTemporalFilter(const bool& enable)
{
    try
    {
        blaze_cam_->TemporalFilter.SetValue(enable);
        if (enable)
            RCLCPP_DEBUG(LOGGER_BLAZE, "Temporal filter is enabled");
        else
            RCLCPP_DEBUG(LOGGER_BLAZE, "Temporal filter is disabled");
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "An exception while enabling/disabling temporal filter occurred: " << e.GetDescription());
        return e.GetDescription();
    }

    return "done";
}

std::string PylonROS2BlazeCamera::enableOutlierRemoval(const bool& enable)
{
    try
    {
        blaze_cam_->OutlierRemoval.SetValue(enable);
        if (enable)
            RCLCPP_DEBUG(LOGGER_BLAZE, "Outlier removal is enabled");
        else
            RCLCPP_DEBUG(LOGGER_BLAZE, "Outlier removal is disabled");
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "An exception while enabling/disabling outlier removal occurred: " << e.GetDescription());
        return e.GetDescription();
    }

    return "done";
}

std::string PylonROS2BlazeCamera::enableAmbiguityFilter(const bool& enable)
{
    try
    {
        blaze_cam_->AmbiguityFilter.SetValue(enable);
        if (enable)
            RCLCPP_DEBUG(LOGGER_BLAZE, "Ambiguity filter is enabled");
        else
            RCLCPP_DEBUG(LOGGER_BLAZE, "Ambiguity filter is disabled");
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "An exception while enabling/disabling ambiguity filter occurred: " << e.GetDescription());
        return e.GetDescription();
    }

    return "done";
}

std::string PylonROS2BlazeCamera::enableThermalDriftCorrection(const bool& enable)
{
    try
    {
        blaze_cam_->ThermalDriftCorrection.SetValue(enable);
        if (enable)
            RCLCPP_DEBUG(LOGGER_BLAZE, "Thermal drift correction is enabled");
        else
            RCLCPP_DEBUG(LOGGER_BLAZE, "Thermal drift correction is disabled");
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "An exception while enabling/disabling thermal drift correction occurred: " << e.GetDescription());
        return e.GetDescription();
    }

    return "done";
}

std::string PylonROS2BlazeCamera::enableDistortionCorrection(const bool& enable)
{
    try
    {
        blaze_cam_->DistortionCorrection.SetValue(enable);
        if (enable)
            RCLCPP_DEBUG(LOGGER_BLAZE, "Distortion correction is enabled");
        else
            RCLCPP_DEBUG(LOGGER_BLAZE, "Distortion correction is disabled");
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "An exception while enabling/disabling distortion correction occurred: " << e.GetDescription());
        return e.GetDescription();
    }

    return "done";
}

std::string PylonROS2BlazeCamera::enableAcquisitionFrameRate(const bool& enable)
{
    try
    {
        blaze_cam_->AcquisitionFrameRateEnable.SetValue(enable);
        if (enable)
            RCLCPP_DEBUG(LOGGER_BLAZE, "Acquisition frame rate is enabled");
        else
            RCLCPP_DEBUG(LOGGER_BLAZE, "Acquisition frame rate is disabled");
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "An exception while enabling/disabling acquisition frame rate occurred: " << e.GetDescription());
        return e.GetDescription();
    }

    return "done";
}

std::string PylonROS2BlazeCamera::enableHDRMode(const bool& enable)
{
    try
    {
        blaze_cam_->HDRMode.SetValue(enable);
        if (enable)
            RCLCPP_DEBUG(LOGGER_BLAZE, "HDR mode is enabled");
        else
            RCLCPP_DEBUG(LOGGER_BLAZE, "HDR mode is disabled");
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "An exception while enabling/disabling HDR mode occurred: " << e.GetDescription());
        return e.GetDescription();
    }

    return "done";
}

std::string PylonROS2BlazeCamera::enableFastMode(const bool& enable)
{
    try
    {
        blaze_cam_->FastMode.SetValue(enable);
        if (enable)
            RCLCPP_DEBUG(LOGGER_BLAZE, "Fast mode is enabled");
        else
            RCLCPP_DEBUG(LOGGER_BLAZE, "Fast mode is disabled");
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_BLAZE, "An exception while enabling/disabling fast mode occurred: " << e.GetDescription());
        return e.GetDescription();
    }

    return "done";
}

bool PylonROS2BlazeCamera::grab(Pylon::CGrabResultPtr& grab_result)
{
    RCLCPP_DEBUG(LOGGER_BLAZE, "This function is not the right one to grab data from the blaze - use the function grabBlaze instead.");
    return true;
}

bool PylonROS2BlazeCamera::grab(std::vector<uint8_t>& image, rclcpp::Time &stamp)
{
    RCLCPP_DEBUG(LOGGER_BLAZE, "This function is not the right one to grab data from the blaze - use the function grabBlaze instead.");
    return true;
}

}  // namespace pylon_ros2_camera
