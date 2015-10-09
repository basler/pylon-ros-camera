#ifndef PYLON_INTERFACE_H_
#define PYLON_INTERFACE_H_

#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#include <ros/ros.h>

#include <pylon/PylonIncludes.h>
#include <pylon/usb/BaslerUsbInstantCamera.h>
#include <pylon/gige/BaslerGigEInstantCamera.h>
#include <pylon_camera/pylon_camera_parameter.h>
#include <pylon_camera/exposure_search_parameter.h>
#include <GenApi/IEnumEntry.h>

namespace pylon_camera
{

enum PYLON_CAM_TYPE
{
    GIGE = 1, USB = 2, DART = 3, UNKNOWN = -1,
};

class PylonInterface
{
public:
    PylonInterface();
    virtual ~PylonInterface();

    bool initialize(const PylonCameraParameter &params);

    virtual int img_rows();
    virtual int img_cols();
    virtual int image_size();
    virtual std::string img_encoding();
    virtual int img_pixel_depth();
    virtual float max_possible_framerate();
    virtual bool has_auto_exposure();
    virtual bool is_cam_removed();
    virtual double last_exposure_val();
    virtual int last_brightness_val();
    void set_image_size(int size);
    virtual int setExposure(double exposure);
    virtual bool setBrightness(int brightness);
    bool registerCameraConfiguration(const PylonCameraParameter &params);
    bool startGrabbing(const PylonCameraParameter &params);
    virtual bool grab(const PylonCameraParameter &params, std::vector<uint8_t> &image);
    virtual bool setupSequencer(const PylonCameraParameter &params);
    virtual bool isAutoBrightnessFunctionRunning();
    float getCurrentExposure();

    // Con-/ De-structor automagically calls PylonInitialize and PylonTerminate to ensure the pylon runtime system
    // is initialized during the lifetime of this object.
    Pylon::PylonAutoInitTerm auto_init_term_;
    bool is_opencv_interface_;
    bool is_ready_;

protected:

    std::string pylonCamTypeToString(const PYLON_CAM_TYPE type);
    bool findDesiredCam(const PylonCameraParameter &params);
    PYLON_CAM_TYPE detectPylonCamType(const Pylon::CInstantCamera* cam);

    Basler_GigECameraParams::PixelFormatEnums gige_img_encoding_;
    Basler_GigECameraParams::PixelSizeEnums gige_img_pixel_depth_;
    Basler_UsbCameraParams::PixelFormatEnums usb_img_encoding_;
    Basler_UsbCameraParams::PixelSizeEnums usb_img_pixel_depth_;

    int img_rows_, img_cols_;
    int height_aoi_, width_aoi_, offset_height_aoi_, offset_width_aoi_;
    int img_size_byte_;
    float max_framerate_;
    bool has_auto_exposure_;

    PYLON_CAM_TYPE cam_type_;

    // This smart pointer will receive the grab result data.
    Pylon::CGrabResultPtr ptr_grab_result_;

    Pylon::CBaslerGigEInstantCamera* gige_cam_;
    Pylon::CBaslerUsbInstantCamera* usb_cam_;
    Pylon::CBaslerUsbInstantCamera* dart_cam_;

    double last_exposure_val_;
    int last_brightness_val_;

    bool is_cam_removed_;
    bool is_pylon_auto_function_running_;

private:
    virtual void setupExtendedBrightnessSearch(int &brightness);
};

}

#endif
