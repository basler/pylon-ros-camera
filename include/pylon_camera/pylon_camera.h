#ifndef PYLON_CAMERA_H_
#define PYLON_CAMERA_H_

#include <pylon_camera/pylon_camera_parameter.h>
#include <pylon_camera/exposure_search_parameter.h>

#ifdef WITH_OPENCV
#include <opencv2/opencv.hpp>
#endif

namespace pylon_camera
{

class PylonCamera
{
public:
    virtual ~PylonCamera();

    static PylonCamera* create();

    static PylonCamera* create(const std::string& name);

    virtual bool registerCameraConfiguration(const PylonCameraParameter& params) = 0;

    virtual bool setupSequencer(const std::vector<float>& exposure_times) = 0;

    virtual bool startGrabbing(const PylonCameraParameter& params) = 0;

    virtual bool grab(std::vector<uint8_t>& image) = 0;

#ifdef WITH_OPENCV
    virtual bool grab(cv::Mat& image) = 0;
#endif

    virtual float currentExposure() = 0;

    virtual bool setExposure(const double& exposure) = 0;

    virtual bool setBrightness(const int& brightness) = 0;

    virtual bool setExtendedBrightness(int& brightness) = 0;

    virtual void setupExtendedBrightnessSearch(const int& brightness) = 0;

    virtual bool isAutoBrightnessFunctionRunning() = 0;

    const bool& isOwnBrightnessFunctionRunning() const;

    virtual std::string imageEncoding() const = 0;

    virtual int imagePixelDepth() const = 0;

    virtual std::string typeName() const = 0;

    virtual float exposureStep() = 0;

    const int& imageRows() const;
    const int& imageCols() const;
    const bool& isReady() const;

    void setImageSize(const int& size);
    const int& imageSize() const;

    const float& maxPossibleFramerate() const;
    const bool& hasAutoExposure() const;
    const bool& isCamRemoved() const;
    const double& lastExposureValue() const;
    const int& lastBrightnessValue() const;

    const std::vector<float>& sequencerExposureTimes() const;

    ExposureSearchParameter exp_search_params_;

protected:

    PylonCamera();

    int img_rows_, img_cols_;
    int height_aoi_, width_aoi_, offset_height_aoi_, offset_width_aoi_;
    int img_size_byte_;
    float max_framerate_;
    bool has_auto_exposure_;

    double last_exposure_val_;
    int last_brightness_val_;

    bool is_ready_;
    bool is_cam_removed_;
    bool is_own_brightness_function_running_;

    std::vector<float> seq_exp_times_;

};

}

#endif
