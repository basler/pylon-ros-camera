// Copyright 2015 <Magazino GmbH>

#ifndef PYLON_CAMERA_INTERNAL_PYLON_CAMERA_H
#define PYLON_CAMERA_INTERNAL_PYLON_CAMERA_H

#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#pragma GCC diagnostic ignored "-Wliteral-suffix"

#include <pylon/PylonIncludes.h>
#include <GenApi/IEnumEntry.h>
#include <string>
#include <vector>

#include <pylon_camera/pylon_camera_parameter.h>
#include <pylon_camera/pylon_camera.h>

namespace pylon_camera
{

template <typename CameraTraitT>
class PylonCameraImpl : public PylonCamera
{
public:
    explicit PylonCameraImpl(Pylon::IPylonDevice* device);

    virtual ~PylonCameraImpl();

    virtual bool registerCameraConfiguration(const PylonCameraParameter& params);

    virtual bool startGrabbing(const PylonCameraParameter& params);

    virtual bool grab(std::vector<uint8_t>& image);

    virtual bool grab(uint8_t* image);

    virtual bool setupSequencer(const std::vector<float>& exposure_times);

    virtual bool setShutterMode(const pylon_camera::SHUTTER_MODE& mode);

    virtual float currentExposure();

    virtual bool setExposure(const double& exposure);

    virtual bool setBrightness(const int& brightness);

    virtual bool setExtendedBrightness(int& brightness);

    virtual void setupExtendedBrightnessSearch(const int& brightness);

    virtual bool isAutoBrightnessFunctionRunning();

    virtual std::string imageEncoding() const;

    virtual int imagePixelDepth() const;

    virtual std::string typeName() const;

    virtual float exposureStep();

    virtual bool setUserOutput(int output_id, bool value);

protected:
    typedef typename CameraTraitT::CBaslerInstantCameraT CBaslerInstantCameraT;
    typedef typename CameraTraitT::ExposureAutoEnums ExposureAutoEnums;
    typedef typename CameraTraitT::PixelFormatEnums PixelFormatEnums;
    typedef typename CameraTraitT::PixelSizeEnums PixelSizeEnums;
    typedef typename CameraTraitT::AutoTargetBrightnessType AutoTargetBrightnessType;
    typedef typename CameraTraitT::ShutterModeEnums ShutterModeEnums;

    // Each camera has it's own getter for GenApi accessors that are named differently for USB and GigE
    GenApi::IFloat& exposureTime();
    GenApi::IFloat& resultingFrameRate();
    AutoTargetBrightnessType& autoTargetBrightness();

    virtual bool grab(Pylon::CGrabResultPtr& grab_result);

    virtual bool setupSequencer(const std::vector<float>& exposure_times, std::vector<float>& exposure_times_set);

    CBaslerInstantCameraT* cam_;

    PixelFormatEnums image_encoding_;

    PixelSizeEnums image_pixel_depth_;
};

}  // namespace pylon_camera

#include <pylon_camera/internal/impl/pylon_camera_base.hpp>
#include <pylon_camera/internal/impl/pylon_camera_usb.hpp>
#include <pylon_camera/internal/impl/pylon_camera_dart.hpp>
#include <pylon_camera/internal/impl/pylon_camera_gige.hpp>

#endif  // PYLON_CAMERA_INTERNAL_PYLON_CAMERA_H
