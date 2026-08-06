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

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "internal/impl/pylon_ros2_camera_gige.hpp"

#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/image_encodings.hpp>

#ifdef cv_bridge_HPP
    #include <cv_bridge/cv_bridge.hpp>
#else
    #include <cv_bridge/cv_bridge.h>
#endif
#include <opencv2/opencv.hpp>

#include "pcl_conversions/pcl_conversions.h"


// Interleaved BGR triplet used for false-color depth maps.
#pragma pack(push, 1)
struct BGR
{
    uint8_t b;
    uint8_t g;
    uint8_t r;
};
#pragma pack(pop)

// Single 3D point as delivered by a Coord3D_ABC32f range component (X, Y, Z floats).
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
    static const rclcpp::Logger LOGGER_3D = rclcpp::get_logger("basler.pylon.ros2.pylon_ros2_3d_camera");

    // Value that identifies a pixel with missing depth information.
    constexpr static double s_invalid_data_value = std::numeric_limits<double>::quiet_NaN();

    // Checks whether a given 3D point represents a valid coordinate.
    // The Scan3dInvalidDataValue is used to identify a non-valid pixel.
    static inline bool isValid(const Point* point)
    {
        static constexpr bool isInvalidValueNaN = s_invalid_data_value != s_invalid_data_value; // true, when s_invalid_data_value equals NaN
        return isInvalidValueNaN ? !std::isnan(point->z) : point->z != s_invalid_data_value;
    }
}

/**
 * Generic 3D camera profile.
 *
 * Abstract intermediate class shared by all 3D cameras integrated in the driver
 * (blaze, Stereo mini, Stereo ace, ...). It centralizes the parts that are common
 * to every 3D camera:
 *   - the is3D() marker used by the node to branch on 3D behavior,
 *   - acquisition start/stop and device-removal detection routed to the
 *     device-specific camera object (activeCamera()),
 *   - common 3D state (depth range, gain, gamma) that stays safely accessible on a
 *     3D device,
 *   - default 3D camera-info (Scan3d intrinsics) and max-framerate behavior,
 *   - reusable conversion primitives operating on a Coord3D_ABC32f range component
 *     (grayscale depth map, false-color depth map, point cloud, intensity,
 *     confidence),
 *   - the device-detach helper needed by the dual-wrapper pattern (a generic
 *     universal instant camera plus a device-specific instant camera wrapping the
 *     same physical device).
 *
 * NOTE: The profile currently inherits from PylonROS2GigECamera so that the
 * generic parameter access provided by PylonROS2CameraImpl (through the inherited
 * universal instant camera cam_) remains available unchanged. This couples the
 * profile to the GigE trait even though some 3D cameras (e.g. the Stereo mini) may
 * also appear on USB. This is a known, deliberate and tracked compromise to be revisited later. typeName() is intentionally left as the inherited
 * "GigE": it denotes the transport (and gates a GigE-vs-USB quirk in
 * detectAndCountNumUserOutputs()), not the sensor model, so a concrete 3D camera
 * class must not override it to its marketing name.
 */
class PylonROS23DCamera : public PylonROS2GigECamera
{
public:
    explicit PylonROS23DCamera(Pylon::IPylonDevice* device);
    virtual ~PylonROS23DCamera() = default;

    // Every camera deriving from this profile is a 3D camera.
    virtual bool is3D() override;

    // Acquisition start/stop and device-removal detection. A 3D camera grabs
    // through its device-specific camera object (activeCamera()); the inherited
    // universal cam_ is never opened for grabbing. Implementing these here makes
    // them correct for every 3D camera and fixes the base implementations that
    // would (wrongly) act on the never-opened cam_.
    // grabbingStarting() is const to match the base signature so it overrides
    // correctly when called through a PylonROS2Camera* (e.g. the start_grabbing
    // service): otherwise a non-const override in a derived class does not override
    // and the base cam_->StartGrabbing() runs on the shared, already-open stream grabber.
    virtual std::string grabbingStarting() const override;
    virtual std::string grabbingStopping() override;
    virtual bool isCamRemoved() override;

    // Returns the working depth range (mm) by reading the DepthMin/DepthMax nodes
    // from the device node map; returns -1 when the nodes are not available.
    virtual int getDepthMin() override;
    virtual int getDepthMax() override;

    // Returns the current gain/gamma by reading the device node map of the active
    // camera; returns -1 when the node is missing or the read fails. The base
    // implementations throw when the Gain/Gamma node is absent on the universal
    // cam_, which would let an exception escape publishCurrentParams().
    virtual float currentGain() override;
    virtual float currentGamma() override;

    // Default 3D camera info: intrinsics read from the device Scan3d nodes of the
    // active camera. A camera whose intrinsics live elsewhere (e.g. blaze reads them
    // manually, the Stereo ace must first select the Disparity component) overrides
    // this; a camera whose intrinsics match this default (e.g. Stereo mini) reuses it.
    virtual void getInitialCameraInfo(sensor_msgs::msg::CameraInfo& cam_info_msg) override;

    // Default maximum frame rate: the AcquisitionFrameRate node of the active
    // camera, falling back to 30 fps when the node is not readable. A camera with a
    // different notion of "max" (e.g. blaze uses AcquisitionFrameRate.GetMax())
    // overrides this.
    virtual float maxPossibleFramerate() override;

    // 2D-oriented controls. These features use sensor nodes that a 3D device does
    // not expose. A camera that supports one (e.g. the Stereo mini has analog Gain,
    // Gamma, Brightness, white balance and an acquisition frame count) overrides
    // it; a camera without support inherits the version here, which returns failure
    // without touching the never-opened universal cam_.
    virtual bool setGain(const float& target_gain, float& reached_gain) override;
    virtual bool setGamma(const float& target_gamma, float& reached_gamma) override;
    virtual bool setBrightness(const int& target_brightness,
                               const float& current_brightness,
                               const bool& exposure_auto,
                               const bool& gain_auto) override;
    virtual std::string setBalanceWhiteAuto(const int& mode) override;
    virtual std::string setAcquisitionFrameCount(const int& frameCount) override;

    // 2D image-sensor controls (sensor offset/mirror, black level,
    // PGI/demosaicing/denoise/sharpness/light-source presets, sensor readout mode,
    // gamma selector, pixel-format encoding, shutter mode, ROI, binning, sequencer)
    // exist only on 2D area-scan sensors and are absent on every 3D depth device.
    // Implementing them once here keeps every 3D camera from falling through to the
    // base implementations, which would act on the never-opened universal cam_.
    // Device-level GigE features (PTP, action commands, statistics, line I/O,
    // device-link throughput, ...) are NOT handled here: a GigE 3D camera may
    // support them through the base class, so they stay with the base and each
    // camera class.
    virtual std::string setOffsetXY(const int& offsetValue, bool xAxis) override;
    virtual std::string reverseXY(const bool& data, bool around_x) override;
    virtual std::string setBlackLevel(const int& data) override;
    virtual std::string setPGIMode(const bool& on) override;
    virtual std::string setDemosaicingMode(const int& mode) override;
    virtual std::string setNoiseReduction(const float& value) override;
    virtual std::string setSharpnessEnhancement(const float& value) override;
    virtual std::string setLightSourcePreset(const int& mode) override;
    virtual std::string setSensorReadoutMode(const int& mode) override;
    virtual std::string setGammaSelector(const int& gammaSelector) override;
    virtual std::string setImageEncoding(const std::string& target_ros_encoding) const override;
    virtual bool setShutterMode(const pylon_ros2_camera::SHUTTER_MODE& mode) override;
    virtual bool setROI(const sensor_msgs::msg::RegionOfInterest target_roi,
                        sensor_msgs::msg::RegionOfInterest& reached_roi) override;
    virtual bool setBinningX(const size_t& target_binning_x, size_t& reached_binning_x) override;
    virtual bool setBinningY(const size_t& target_binning_y, size_t& reached_binning_y) override;
    virtual bool setupSequencer(const std::vector<float>& exposure_times) override;

protected:
    /**
     * Returns the device-specific instant camera object (blaze_cam_ /
     * stereo_mini_cam_ / stereo_ace_cam_) that actually wraps and grabs from the
     * physical device. Every 3D camera class owns such an object; the inherited universal
     * cam_ is only used for generic parameter access and is never opened for
     * grabbing. Centralizing access here lets the profile implement the shared
     * acquisition/lifetime behavior once, independently of the concrete type.
     */
    virtual Pylon::CInstantCamera& activeCamera() const = 0;

    /**
     * Detaches the physical device from the inherited universal instant camera
     * (base class cam_) without destroying it.
     *
     * A 3D camera wraps the same IPylonDevice with two pylon camera objects: the
     * inherited cam_ (universal, used for generic parameter access) and a
     * device-specific instant camera (used for 3D grabbing). The device-specific
     * object owns the device lifetime and will call DestroyDevice() on it. This
     * helper must be called from the derived destructor before the device-specific
     * object is destroyed, so DestroyDevice() is not called twice on the same
     * device (which would crash inside libpylonbase).
     */
    void detachBaseDevice();

    /**
     * Calculates a grayscale (16-bit) depth map from a Coord3D_ABC32f range
     * component. The output buffer must hold width * height uint16_t values.
     * Distances are clipped to [min_depth, max_depth] and normalized to the full
     * 16-bit range. Invalid pixels are set to 0.
     *
     * @param range_component  the Coord3D_ABC32f range component.
     * @param coordinate_scale the Scan3dCoordinateScale of the Z coordinate.
     * @param min_depth        minimum depth (same unit as the scaled coordinate).
     * @param max_depth        maximum depth (same unit as the scaled coordinate).
     * @param depth_map        output buffer (width * height uint16_t).
     */
    void calculateDepthMap(const Pylon::CPylonDataComponent& range_component,
                           double coordinate_scale,
                           int min_depth,
                           int max_depth,
                           uint16_t* depth_map);

    /**
     * Calculates a false-color (BGR) depth map from a Coord3D_ABC32f range
     * component. The output buffer must hold width * height BGR values. The radial
     * distance is clipped to [min_depth, max_depth]. Invalid pixels are set to
     * black.
     *
     * @param range_component the Coord3D_ABC32f range component.
     * @param min_depth       minimum depth.
     * @param max_depth       maximum depth.
     * @param depth_map       output buffer (width * height BGR).
     */
    void calculateDepthMapColor(const Pylon::CPylonDataComponent& range_component,
                                int min_depth,
                                int max_depth,
                                BGR* depth_map);

    /**
     * Builds an organized sensor_msgs/PointCloud2 (pcl::PointXYZRGB) from a
     * Coord3D_ABC32f range component. The XYZ coordinates are assumed to be in
     * millimeters and are scaled to meters. If an intensity component is
     * provided, each point is colored from it (grayscale for Mono8/Mono16,
     * color for RGB8/RGBA8); the intensity component is assumed to have the same
     * dimensions as the range component. Invalid coordinates (NaN) are retained.
     *
     * @param range_component     the Coord3D_ABC32f range component (mm).
     * @param intensity_component optional intensity component for coloring (may be null).
     * @param cloud_msg           output point cloud message.
     */
    void buildPointCloud(const Pylon::CPylonDataComponent& range_component,
                         const Pylon::CPylonDataComponent* intensity_component,
                         sensor_msgs::msg::PointCloud2& cloud_msg);

    /**
     * Builds a sensor_msgs/Image from an intensity component, choosing the ROS
     * encoding from the component pixel type (Mono8 -> mono8, Mono16 -> mono16,
     * RGB8 -> rgb8, RGBA8 -> rgba8).
     */
    void buildIntensityImage(const Pylon::CPylonDataComponent& intensity_component,
                             sensor_msgs::msg::Image& image_msg);

    /**
     * Builds a sensor_msgs/Image from a confidence component (Confidence8 ->
     * mono8, Confidence16 -> mono16).
     */
    void buildConfidenceImage(const Pylon::CPylonDataComponent& confidence_component,
                              sensor_msgs::msg::Image& image_msg);

    /**
     * Populates the intrinsic fields of a CameraInfo message from the standard
     * Scan3d GenICam nodes (Scan3dFocalLength, Scan3dPrincipalPointU/V) of the
     * given node map. Uses the plumb_bob model with zero distortion.
     */
    void populateCameraInfoFromScan3d(GenApi::INodeMap& node_map,
                                      int width,
                                      int height,
                                      sensor_msgs::msg::CameraInfo& cam_info_msg);

    /**
     * Reads the DepthMin/DepthMax GenICam nodes (in millimeters) from the given
     * node map. Falls back to [fallback_min, fallback_max] if the nodes are not
     * available.
     */
    void readDepthRange(GenApi::INodeMap& node_map,
                        int fallback_min,
                        int fallback_max,
                        int& min_depth,
                        int& max_depth);
};

PylonROS23DCamera::PylonROS23DCamera(Pylon::IPylonDevice* device) :
    PylonROS2GigECamera(device)
{
}

bool PylonROS23DCamera::is3D()
{
    return true;
}

std::string PylonROS23DCamera::grabbingStarting() const
{
    try
    {
        // The device-specific camera is already grabbing right after startup, so a
        // repeated start_grabbing returns success instead of raising "Grabbing has
        // already been started".
        if (activeCamera().IsGrabbing())
        {
            return "done";
        }

        // Grab strategy is user-selectable (set_grabbing_strategy); each camera class sets
        // its own default in its constructor (OneByOne for blaze, LatestImageOnly
        // for the stereo cameras).
        Pylon::EGrabStrategy strategy = Pylon::GrabStrategy_OneByOne;
        if (grab_strategy_ == 1)
            strategy = Pylon::GrabStrategy_LatestImageOnly;
        else if (grab_strategy_ == 2)
            strategy = Pylon::GrabStrategy_LatestImages;

        activeCamera().StartGrabbing(strategy);
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_3D, "An exception occurred while starting image grabbing: " << e.GetDescription());
        return e.GetDescription();
    }

    return "done";
}

std::string PylonROS23DCamera::grabbingStopping()
{
    try
    {
        activeCamera().StopGrabbing();
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_3D, "An exception occurred while stopping image grabbing: " << e.GetDescription());
        return e.GetDescription();
    }

    return "done";
}

bool PylonROS23DCamera::isCamRemoved()
{
    try
    {
        return activeCamera().IsCameraDeviceRemoved();
    }
    catch (const GenICam::GenericException&)
    {
        return false;
    }
}

int PylonROS23DCamera::getDepthMin()
{
    int min_depth = -1, max_depth = -1;
    try
    {
        this->readDepthRange(activeCamera().GetNodeMap(), -1, -1, min_depth, max_depth);
    }
    catch (const GenICam::GenericException&)
    {
        return -1;
    }
    return min_depth;
}

int PylonROS23DCamera::getDepthMax()
{
    int min_depth = -1, max_depth = -1;
    try
    {
        this->readDepthRange(activeCamera().GetNodeMap(), -1, -1, min_depth, max_depth);
    }
    catch (const GenICam::GenericException&)
    {
        return -1;
    }
    return max_depth;
}

float PylonROS23DCamera::currentGain()
{
    try
    {
        GenApi::CFloatPtr gain_node(activeCamera().GetNodeMap().GetNode("Gain"));
        if (gain_node.IsValid() && GenApi::IsReadable(gain_node))
        {
            return static_cast<float>(gain_node->GetValue());
        }
    }
    catch (const GenICam::GenericException&) {}
    return -1.0f;
}

float PylonROS23DCamera::currentGamma()
{
    try
    {
        GenApi::CFloatPtr gamma_node(activeCamera().GetNodeMap().GetNode("Gamma"));
        if (gamma_node.IsValid() && GenApi::IsReadable(gamma_node))
        {
            return static_cast<float>(gamma_node->GetValue());
        }
    }
    catch (const GenICam::GenericException&) {}
    return -1.0f;
}

void PylonROS23DCamera::getInitialCameraInfo(sensor_msgs::msg::CameraInfo& cam_info_msg)
{
    this->populateCameraInfoFromScan3d(activeCamera().GetNodeMap(),
                                       static_cast<int>(this->imageCols()),
                                       static_cast<int>(this->imageRows()),
                                       cam_info_msg);
}

float PylonROS23DCamera::maxPossibleFramerate()
{
    try
    {
        GenApi::CFloatPtr frame_rate(activeCamera().GetNodeMap().GetNode("AcquisitionFrameRate"));
        if (frame_rate.IsValid() && GenApi::IsReadable(frame_rate))
        {
            return static_cast<float>(frame_rate->GetValue());
        }
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_DEBUG_STREAM(LOGGER_3D, "maxPossibleFramerate: could not read AcquisitionFrameRate: " << e.GetDescription());
    }
    return 30.0f;
}

// --- 2D-oriented controls ---------------------------------------------------
// A camera that supports the feature overrides these; a camera without support
// inherits the version here, which returns failure instead of touching the
// never-opened universal cam_.

bool PylonROS23DCamera::setGain(const float& /*target_gain*/, float& reached_gain)
{
    reached_gain = -1.0f;
    return false;
}

bool PylonROS23DCamera::setGamma(const float& /*target_gamma*/, float& reached_gamma)
{
    reached_gamma = -1.0f;
    return false;
}

bool PylonROS23DCamera::setBrightness(const int& /*target_brightness*/,
                                      const float& /*current_brightness*/,
                                      const bool& /*exposure_auto*/,
                                      const bool& /*gain_auto*/)
{
    return false;
}

std::string PylonROS23DCamera::setBalanceWhiteAuto(const int& /*mode*/)
{
    return "Feature not available for this camera type";
}

std::string PylonROS23DCamera::setAcquisitionFrameCount(const int& /*frameCount*/)
{
    return "Feature not available for this camera type";
}

// --- 2D image-sensor controls -----------------------------------------------
// Absent on every 3D depth device, so these return "not available" here.

std::string PylonROS23DCamera::setOffsetXY(const int& /*offsetValue*/, bool /*xAxis*/)
{
    return "Feature not available for this camera type";
}

std::string PylonROS23DCamera::reverseXY(const bool& /*data*/, bool /*around_x*/)
{
    return "Feature not available for this camera type";
}

std::string PylonROS23DCamera::setBlackLevel(const int& /*data*/)
{
    return "Feature not available for this camera type";
}

std::string PylonROS23DCamera::setPGIMode(const bool& /*on*/)
{
    return "Feature not available for this camera type";
}

std::string PylonROS23DCamera::setDemosaicingMode(const int& /*mode*/)
{
    return "Feature not available for this camera type";
}

std::string PylonROS23DCamera::setNoiseReduction(const float& /*value*/)
{
    return "Feature not available for this camera type";
}

std::string PylonROS23DCamera::setSharpnessEnhancement(const float& /*value*/)
{
    return "Feature not available for this camera type";
}

std::string PylonROS23DCamera::setLightSourcePreset(const int& /*mode*/)
{
    return "Feature not available for this camera type";
}

std::string PylonROS23DCamera::setSensorReadoutMode(const int& /*mode*/)
{
    return "Feature not available for this camera type";
}

std::string PylonROS23DCamera::setGammaSelector(const int& /*gammaSelector*/)
{
    return "Feature not available for this camera type";
}

std::string PylonROS23DCamera::setImageEncoding(const std::string& /*target_ros_encoding*/) const
{
    // 3D cameras fix a PixelFormat per output component (range/intensity/confidence)
    // at startup, so there is no single 2D encoding to switch: blaze intensity is
    // Mono16, the stereo mini formats are fixed by its multi-source pipeline, and
    // the stereo ace intensity (RGB8) is chosen at startup.
    return "Feature not available for this camera type";
}

bool PylonROS23DCamera::setShutterMode(const pylon_ros2_camera::SHUTTER_MODE& /*mode*/)
{
    return false;
}

bool PylonROS23DCamera::setROI(const sensor_msgs::msg::RegionOfInterest /*target_roi*/,
                               sensor_msgs::msg::RegionOfInterest& /*reached_roi*/)
{
    return false;
}

bool PylonROS23DCamera::setBinningX(const size_t& /*target_binning_x*/, size_t& /*reached_binning_x*/)
{
    return false;
}

bool PylonROS23DCamera::setBinningY(const size_t& /*target_binning_y*/, size_t& /*reached_binning_y*/)
{
    return false;
}

bool PylonROS23DCamera::setupSequencer(const std::vector<float>& /*exposure_times*/)
{
    return false;
}

void PylonROS23DCamera::detachBaseDevice()
{
    // The base class cam_ was constructed with the same IPylonDevice pointer as
    // the derived device-specific camera. The derived camera's destructor will
    // call DestroyDevice() on that shared pointer. Detach the device from cam_
    // first (without destroying it) so the base class destructor does not call
    // DestroyDevice() a second time, which would SIGSEGV inside libpylonbase.so.
    try
    {
        if (cam_->IsPylonDeviceAttached())
        {
            cam_->DetachDevice();
        }
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_DEBUG_STREAM(LOGGER_3D, "Destructor (3D): Failed to detach device from base cam: " << e.GetDescription());
    }
}

void PylonROS23DCamera::calculateDepthMap(const Pylon::CPylonDataComponent& range_component,
                                          double coordinate_scale,
                                          int min_depth,
                                          int max_depth,
                                          uint16_t* depth_map)
{
    const int width = range_component.GetWidth();
    const int height = range_component.GetHeight();
    const Point* pPoint = reinterpret_cast<const Point*>(range_component.GetData());

    const double scale = 65535.0 / (max_depth - min_depth);

    for (int row = 0; row < height; ++row)
    {
        for (int col = 0; col < width; ++col, ++pPoint, ++depth_map)
        {
            if (isValid(pPoint))
            {
                // Calculate the radial distance.
                //double distance = sqrt(pPoint->x * pPoint->x + pPoint->y * pPoint->y + pPoint->z * pPoint->z);
                // EDIT: the standard distance is enough in this context
                double distance = pPoint->z * coordinate_scale;
                // Clip to [min_depth..MaxDept].
                if (distance < min_depth)
                    distance = min_depth;
                else if (distance > max_depth)
                    distance = max_depth;
                *depth_map = (uint16_t) ( ( distance - min_depth ) * scale );
            }
            else
            {
                // No depth information available for this pixel. Zero it.
                *depth_map = 0;
            }
        }
    }
}

void PylonROS23DCamera::calculateDepthMapColor(const Pylon::CPylonDataComponent& range_component,
                                               int min_depth,
                                               int max_depth,
                                               BGR* depth_map)
{
    const int width = range_component.GetWidth();
    const int height = range_component.GetHeight();
    const Point* pPoint = reinterpret_cast<const Point*>(range_component.GetData());

    const double scale = 65535.0 / (max_depth - min_depth);

    for (int row = 0; row < height; ++row)
    {
        for (int col = 0; col < width; ++col, ++pPoint, ++depth_map)
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

                *depth_map = bgr;
            }
            else
            {
                // No depth information available for this pixel. Set it to black.
                BGR bgr;
                bgr.r = bgr.g = bgr.b = 0;
                *depth_map = bgr;
            }
        }
    }
}

void PylonROS23DCamera::buildPointCloud(const Pylon::CPylonDataComponent& range_component,
                                        const Pylon::CPylonDataComponent* intensity_component,
                                        sensor_msgs::msg::PointCloud2& cloud_msg)
{
    // An organized point cloud is used, i.e., for each camera pixel there is an
    // entry in the data structure. Coordinates for invalid pixels are NaN and are
    // retained in the cloud.
    const size_t width = range_component.GetWidth();
    const size_t height = range_component.GetHeight();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ppoint_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    ppoint_cloud->width = width;
    ppoint_cloud->height = height;
    ppoint_cloud->points.resize(width * height);
    ppoint_cloud->is_dense = false;

    // Coord3D_ABC32f: interleaved X,Y,Z floats (in millimeters).
    const Point* psrc_point = reinterpret_cast<const Point*>(range_component.GetData());

    // Determine coloring source from the intensity pixel type.
    Pylon::EPixelType intensity_type = Pylon::PixelType_Undefined;
    const uint8_t* pintensity8 = nullptr;
    const uint16_t* pintensity16 = nullptr;
    if (intensity_component != nullptr &&
        intensity_component->GetWidth() == width &&
        intensity_component->GetHeight() == height)
    {
        intensity_type = intensity_component->GetPixelType();
        pintensity8 = reinterpret_cast<const uint8_t*>(intensity_component->GetData());
        pintensity16 = reinterpret_cast<const uint16_t*>(intensity_component->GetData());
    }

    for (size_t i = 0; i < height * width; ++i)
    {
        pcl::PointXYZRGB& dst_point = ppoint_cloud->points[i];

        // Convert from millimeters to meters.
        dst_point.x = psrc_point[i].x * 0.001f;
        dst_point.y = psrc_point[i].y * 0.001f;
        dst_point.z = psrc_point[i].z * 0.001f;

        uint8_t r = 0, g = 0, b = 0;
        switch (intensity_type)
        {
            case Pylon::PixelType_Mono16:
                r = g = b = static_cast<uint8_t>(pintensity16[i] >> 8);
                break;
            case Pylon::PixelType_Mono8:
                r = g = b = pintensity8[i];
                break;
            case Pylon::PixelType_RGB8packed:
                r = pintensity8[i * 3 + 0];
                g = pintensity8[i * 3 + 1];
                b = pintensity8[i * 3 + 2];
                break;
            case Pylon::PixelType_RGBA8packed:
                r = pintensity8[i * 4 + 0];
                g = pintensity8[i * 4 + 1];
                b = pintensity8[i * 4 + 2];
                break;
            default:
                r = g = b = 0;
                break;
        }
        dst_point.r = r;
        dst_point.g = g;
        dst_point.b = b;
    }

    pcl::toROSMsg(*ppoint_cloud, cloud_msg);
}

void PylonROS23DCamera::buildIntensityImage(const Pylon::CPylonDataComponent& intensity_component,
                                            sensor_msgs::msg::Image& image_msg)
{
    const int width = intensity_component.GetWidth();
    const int height = intensity_component.GetHeight();
    void* data = const_cast<void*>(intensity_component.GetData());

    cv_bridge::CvImage cv_img;
    switch (intensity_component.GetPixelType())
    {
        case Pylon::PixelType_Mono16:
            cv_img.encoding = sensor_msgs::image_encodings::MONO16;
            cv_img.image = cv::Mat(height, width, CV_16UC1, data);
            break;
        case Pylon::PixelType_Mono8:
            cv_img.encoding = sensor_msgs::image_encodings::MONO8;
            cv_img.image = cv::Mat(height, width, CV_8UC1, data);
            break;
        case Pylon::PixelType_RGB8packed:
            cv_img.encoding = sensor_msgs::image_encodings::RGB8;
            cv_img.image = cv::Mat(height, width, CV_8UC3, data);
            break;
        case Pylon::PixelType_RGBA8packed:
            cv_img.encoding = sensor_msgs::image_encodings::RGBA8;
            cv_img.image = cv::Mat(height, width, CV_8UC4, data);
            break;
        default:
            RCLCPP_WARN_STREAM(LOGGER_3D, "buildIntensityImage: unsupported intensity pixel type 0x"
                << std::hex << static_cast<uint64_t>(intensity_component.GetPixelType()) << std::dec);
            return;
    }

    const auto msg = cv_img.toImageMsg();
    image_msg.header = msg->header;
    image_msg.height = msg->height;
    image_msg.width = msg->width;
    image_msg.encoding = msg->encoding;
    image_msg.is_bigendian = msg->is_bigendian;
    image_msg.step = msg->step;
    image_msg.data = msg->data;
}

void PylonROS23DCamera::buildConfidenceImage(const Pylon::CPylonDataComponent& confidence_component,
                                             sensor_msgs::msg::Image& image_msg)
{
    const int width = confidence_component.GetWidth();
    const int height = confidence_component.GetHeight();
    void* data = const_cast<void*>(confidence_component.GetData());

    cv_bridge::CvImage cv_img;
    switch (confidence_component.GetPixelType())
    {
        case Pylon::PixelType_Confidence16:
            cv_img.encoding = sensor_msgs::image_encodings::MONO16;
            cv_img.image = cv::Mat(height, width, CV_16UC1, data);
            break;
        case Pylon::PixelType_Confidence8:
            cv_img.encoding = sensor_msgs::image_encodings::MONO8;
            cv_img.image = cv::Mat(height, width, CV_8UC1, data);
            break;
        default:
            RCLCPP_WARN_STREAM(LOGGER_3D, "buildConfidenceImage: unsupported confidence pixel type 0x"
                << std::hex << static_cast<uint64_t>(confidence_component.GetPixelType()) << std::dec);
            return;
    }

    const auto msg = cv_img.toImageMsg();
    image_msg.header = msg->header;
    image_msg.height = msg->height;
    image_msg.width = msg->width;
    image_msg.encoding = msg->encoding;
    image_msg.is_bigendian = msg->is_bigendian;
    image_msg.step = msg->step;
    image_msg.data = msg->data;
}

void PylonROS23DCamera::populateCameraInfoFromScan3d(GenApi::INodeMap& node_map,
                                                     int width,
                                                     int height,
                                                     sensor_msgs::msg::CameraInfo& cam_info_msg)
{
    cam_info_msg.height = height;
    cam_info_msg.width = width;
    cam_info_msg.distortion_model = "plumb_bob";
    cam_info_msg.d = std::vector<double>(5, 0.0);

    double f = 0.0, cx = 0.0, cy = 0.0;
    try
    {
        GenApi::CFloatPtr focal_length(node_map.GetNode("Scan3dFocalLength"));
        GenApi::CFloatPtr principal_u(node_map.GetNode("Scan3dPrincipalPointU"));
        GenApi::CFloatPtr principal_v(node_map.GetNode("Scan3dPrincipalPointV"));
        if (focal_length.IsValid() && GenApi::IsReadable(focal_length)) f = focal_length->GetValue();
        if (principal_u.IsValid() && GenApi::IsReadable(principal_u)) cx = principal_u->GetValue();
        if (principal_v.IsValid() && GenApi::IsReadable(principal_v)) cy = principal_v->GetValue();
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_WARN_STREAM(LOGGER_3D, "populateCameraInfoFromScan3d: could not read Scan3d intrinsics: " << e.GetDescription());
    }

    cam_info_msg.k = {f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0};
    cam_info_msg.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    cam_info_msg.p = {f, 0.0, cx, 0.0, 0.0, f, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
}

void PylonROS23DCamera::readDepthRange(GenApi::INodeMap& node_map,
                                       int fallback_min,
                                       int fallback_max,
                                       int& min_depth,
                                       int& max_depth)
{
    min_depth = fallback_min;
    max_depth = fallback_max;
    try
    {
        GenApi::CIntegerPtr depth_min(node_map.GetNode("DepthMin"));
        GenApi::CIntegerPtr depth_max(node_map.GetNode("DepthMax"));
        if (depth_min.IsValid() && GenApi::IsReadable(depth_min)) min_depth = static_cast<int>(depth_min->GetValue());
        if (depth_max.IsValid() && GenApi::IsReadable(depth_max)) max_depth = static_cast<int>(depth_max->GetValue());
    }
    catch (const GenICam::GenericException& e)
    {
        RCLCPP_WARN_STREAM(LOGGER_3D, "readDepthRange: could not read DepthMin/DepthMax, using fallback: " << e.GetDescription());
    }
}

} // namespace pylon_ros2_camera
