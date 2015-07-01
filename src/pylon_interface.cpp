/*
 * pylon_interface.cpp
 *
 *  Created on: May 21, 2015
 *      Author: md
 */

#include <pylon_camera/pylon_interface.h>

namespace pylon_camera
{

PylonInterface::PylonInterface() :
                    img_rows_(-1),
                    img_cols_(-1),
                    height_aoi_(-1),
                    width_aoi_(-1),
                    img_size_byte_(-1),
                    offset_height_aoi_(-1),
                    offset_width_aoi_(-1),
                    max_framerate_(-1.0),
                    gige_cam_(NULL),
                    usb_cam_(NULL),
                    dart_cam_(NULL),
                    has_auto_exposure_(false),
                    last_exposure_val_(2000.0),
                    last_brightness_val_(-1),
                    is_cam_removed_(false),
                    auto_init_term_(),
                    c_instant_cam_(NULL),
                    exposure_search_running_(false),
                    usb_img_pixel_depth_(Basler_UsbCameraParams::PixelSize_Bpp8),
                    gige_img_pixel_depth_(Basler_GigECameraParams::PixelSize_Bpp8),
                    usb_img_encoding_(Basler_UsbCameraParams::PixelFormat_Mono8),
                    gige_img_encoding_(Basler_GigECameraParams::PixelFormat_Mono8),
                    cam_type_(UNKNOWN)
{
    // TODO Auto-generated constructor stub
}
PylonInterface::~PylonInterface()
{
    // TODO Auto-generated destructor stub
}
int PylonInterface::initialize(const PylonCameraParameter &params)
{
    int exit_code = 0;

    // Find the desired camera device in the list of pluged-in devices
    // Use either magazino_cam_id or the device found first
    if (findDesiredCam(params))
    {
        return 1;
    }
    if (!(cam_type_ == GIGE || cam_type_ == USB || cam_type_ == DART))
    {
        return 2;
    }
    return exit_code;
}
bool PylonInterface::grab(const PylonCameraParameter &params, std::vector<uint8_t> &image)
{
    switch (cam_type_)
    {
        case GIGE:
            try
            {
                gige_cam_->ExecuteSoftwareTrigger();
                gige_cam_->RetrieveResult(gige_cam_->ExposureTimeAbs.GetMax() * 1.05,
                                          ptr_grab_result_,
                                          TimeoutHandling_ThrowException);
            }
            catch (GenICam::GenericException &e)
            {
                if (gige_cam_->IsCameraDeviceRemoved())
                {
                    is_cam_removed_ = true;
                }
                else
                {
                    cerr << "An image grabbing exception in pylon camera occurred:" << endl
                         << e.GetDescription()
                         << endl;
                    return false;
                }
            }
            break;
        case USB:
            try
            {
                usb_cam_->ExecuteSoftwareTrigger();
                usb_cam_->RetrieveResult(usb_cam_->ExposureTime.GetMax() * 1.05,
                                         ptr_grab_result_,
                                         TimeoutHandling_ThrowException);
            }
            catch (GenICam::GenericException &e)
            {
                if (usb_cam_->IsCameraDeviceRemoved())
                {
                    is_cam_removed_ = true;
                }
                else
                {
                    cerr << "An image grabbing exception in pylon camera occurred:" << endl
                         << e.GetDescription()
                         << endl;
                    return false;
                }
            }
            break;
        case DART:
            try
            {
                dart_cam_->ExecuteSoftwareTrigger();
                dart_cam_->RetrieveResult(dart_cam_->ExposureTime.GetMax() * 1.05,
                                          ptr_grab_result_,
                                          TimeoutHandling_ThrowException);
            }
            catch (GenICam::GenericException &e)
            {
                if (dart_cam_->IsCameraDeviceRemoved())
                {
                    is_cam_removed_ = true;
                }
                else
                {
                    cerr << "An image grabbing exception in pylon camera occurred:" << endl
                         << e.GetDescription()
                         << endl;
                    return false;
                }
            }
            break;
        default:
            cerr << "Unknown Camera Type" << endl;
            break;
    }

    if (ptr_grab_result_->GrabSucceeded())
    {
        //TODO: wird hier neuer Speicher allokiert? Kann der bereits allokierte weiterverwendet werden?
        const uint8_t *pImageBuffer = (uint8_t *)ptr_grab_result_->GetBuffer();
        image = std::vector<uint8_t>(pImageBuffer, pImageBuffer + img_size_byte_);
        return true;
    }
    else
    {
        cout << "Error: " << ptr_grab_result_->GetErrorCode() << " "
             << ptr_grab_result_->GetErrorDescription()
             << endl;
        return false;
    }
    return false;
}
int PylonInterface::initSequencer(const PylonCameraParameter &params)
{
    // Dummy -> only used in PylonSequencerInterface
//    usb_cam_->SequencerMode.SetValue(Basler_UsbCameraParams::SequencerMode_Off);
    return 0;
}
bool PylonInterface::registerCameraConfiguration(const PylonCameraParameter &params)
{
    try
    {
        switch (cam_type_)
        {
            case GIGE:
                gige_cam_->RegisterConfiguration(new CSoftwareTriggerConfiguration,
                                                 RegistrationMode_ReplaceAll,
                                                 Cleanup_Delete);
                gige_cam_->Open();
                // Remove all previous settings (sequencer etc.)
                gige_cam_->UserSetSelector.SetValue(Basler_GigECameraParams::UserSetSelector_Default);
                gige_cam_->UserSetLoad.Execute();
                // UserSetSelector_Default overrides Software Trigger Mode !!
                gige_cam_->TriggerSource.SetValue(Basler_GigECameraParams::TriggerSource_Software);
                gige_cam_->TriggerMode.SetValue(Basler_GigECameraParams::TriggerMode_On);
                break;
            case USB:
                usb_cam_->RegisterConfiguration(new CSoftwareTriggerConfiguration,
                                                RegistrationMode_ReplaceAll,
                                                Cleanup_Delete);
                usb_cam_->Open();
                // Remove all previous settings (sequencer etc.)
                usb_cam_->UserSetSelector.SetValue(Basler_UsbCameraParams::UserSetSelector_Default);
                usb_cam_->UserSetLoad.Execute();
                // UserSetSelector_Default overrides Software Trigger Mode !!
                usb_cam_->TriggerSource.SetValue(Basler_UsbCameraParams::TriggerSource_Software);
                usb_cam_->TriggerMode.SetValue(Basler_UsbCameraParams::TriggerMode_On);
                break;
            case DART:

                dart_cam_->RegisterConfiguration(new CSoftwareTriggerConfiguration,
                                                 RegistrationMode_ReplaceAll,
                                                 Cleanup_Delete);
                dart_cam_->Open();
                // Remove all previous settings (sequencer etc.)
                dart_cam_->UserSetSelector.SetValue(Basler_UsbCameraParams::UserSetSelector_Default);
                dart_cam_->UserSetLoad.Execute();
                // UserSetSelector_Default overrides Software Trigger Mode !!
                dart_cam_->TriggerSource.SetValue(Basler_UsbCameraParams::TriggerSource_Software);
                dart_cam_->TriggerMode.SetValue(Basler_UsbCameraParams::TriggerMode_On);
                break;
            default:
                cerr << "Unknown Camera Type" << endl;
                break;
        }
    }
    catch (GenICam::GenericException &e)
    {
        cerr << e.GetDescription() << endl;
        return false;
    }
    return true;
}

bool PylonInterface::startGrabbing(const PylonCameraParameter &params)
{
    try
    {
        switch (cam_type_)
        {
            case GIGE:
                //                initSequencer(params);
                gige_cam_->StartGrabbing();
                img_rows_ = (int)gige_cam_->Height.GetValue();
                img_cols_ = (int)gige_cam_->Width.GetValue();
                gige_img_encoding_ = gige_cam_->PixelFormat.GetValue();
                gige_img_pixel_depth_ = gige_cam_->PixelSize.GetValue();
                max_framerate_ = gige_cam_->ResultingFrameRateAbs.GetValue();
                has_auto_exposure_ = GenApi::IsAvailable(gige_cam_->ExposureAuto);
                gige_cam_->ExecuteSoftwareTrigger();

                break;
            case USB:
                usb_cam_->StartGrabbing();
                img_rows_ = (int)usb_cam_->Height.GetValue();
                img_cols_ = (int)usb_cam_->Width.GetValue();
                usb_img_encoding_ = usb_cam_->PixelFormat.GetValue();
                usb_img_pixel_depth_ = usb_cam_->PixelSize.GetValue();
                max_framerate_ = usb_cam_->ResultingFrameRate.GetValue();
                has_auto_exposure_ = GenApi::IsAvailable(usb_cam_->ExposureAuto);
                usb_cam_->ExecuteSoftwareTrigger();
                break;
            case DART:
                dart_cam_->StartGrabbing();
                img_rows_ = (int)dart_cam_->Height.GetValue();
                img_cols_ = (int)dart_cam_->Width.GetValue();
                usb_img_encoding_ = dart_cam_->PixelFormat.GetValue();
                usb_img_pixel_depth_ = dart_cam_->PixelSize.GetValue();
                max_framerate_ = dart_cam_->ResultingFrameRate.GetValue();
                has_auto_exposure_ = GenApi::IsAvailable(dart_cam_->ExposureAuto);
                dart_cam_->ExecuteSoftwareTrigger();
                break;
            default:
                cerr << "Unknown Camera Type" << endl;
                break;
        }
    }
    catch (GenICam::GenericException &e)
    {
        cerr << e.GetDescription() << endl;
        return false;
    }

    height_aoi_ = 0.5 * img_rows_;
    width_aoi_ = 0.5 * img_cols_;
    offset_height_aoi_ = 0.25 * img_rows_;
    offset_width_aoi_ = 0.25 * img_cols_;

    return true;
}

int PylonInterface::findDesiredCam(const PylonCameraParameter &params)
{

    if (params.magazino_cam_id_ == "x")
    { // Select camera device found first
        try
        {
            CInstantCamera* cam = new CInstantCamera(CTlFactory::GetInstance().CreateFirstDevice());
            cam->Open();
            cam_type_ = detectPylonCamType(cam);
            cout << "Desired Cam is a " << pylonCamTypeToString(cam_type_) << " camera." << endl;
            cam->Close();
            switch (cam_type_)
            {
                case GIGE:
                    gige_cam_ = new Pylon::CBaslerGigEInstantCamera(CTlFactory::GetInstance()
                                                                                             .CreateFirstDevice());
                    break;
                case USB:
                    usb_cam_ = new Pylon::CBaslerUsbInstantCamera(CTlFactory::GetInstance()
                                                                                           .CreateFirstDevice());
                    break;
                case DART:
                    dart_cam_ = new Pylon::CBaslerUsbInstantCamera(CTlFactory::GetInstance()
                                                                                            .CreateFirstDevice());
                    break;
                default:
                    break;
            }

        }
        catch (GenICam::GenericException &e)
        {
            cerr << "An exception while selecting the first pylon camera found occurred:" << endl;
            cerr << e.GetDescription() << endl;
            return 1;
        }
    }
    else
    { // Select camera device with specified magazino_cam_id
        try
        {
            // Get the transport layer factory.
            CTlFactory& transport_layer_factory = CTlFactory::GetInstance();

            // Get all attached devices and exit application if no device is found.
            DeviceInfoList_t device_info_list;
            if (transport_layer_factory.EnumerateDevices(device_info_list) == 0)
            {
                throw RUNTIME_EXCEPTION( "No camera present.");
                return 2;
            }

            // Create an array of instant cameras for the found devices
            CInstantCameraArray camera_array(device_info_list.size());

            bool found_desired_device = false;

            // Create and attach all Pylon Devices.
            size_t cam_pos = -1;
            for (size_t i = 0; i < camera_array.GetSize(); ++i)
            {

                camera_array[i].Attach(transport_layer_factory.CreateDevice(device_info_list[i]));

                camera_array[i].Open();

                GenApi::INodeMap& node_map = camera_array[i].GetNodeMap();
                GenApi::CStringPtr DeviceUserID(node_map.GetNode("DeviceUserID"));

                if (std::string(DeviceUserID->GetValue()) == params.magazino_cam_id_)
                {
                    found_desired_device = true;
                    cam_pos = i;
                    cout << "Found the desired Camera with Magazino ID: " << params.magazino_cam_id_
                         << ": "
                         << camera_array[cam_pos].GetDeviceInfo().GetModelName()
                         << endl;
                    break;
                }
                camera_array[i].Close();
            }
            if (!found_desired_device)
            {
                cerr << "Maybe the given magazino_cam_id ("
                     << params.magazino_cam_id_.c_str()
                     << ") is wrong or has not yet been written to the camera using 'write_magazino_id_to_camera' ?!"
                     << endl;
                return 3;
            }
            if (!camera_array[cam_pos].IsOpen())
            {
                camera_array[cam_pos].Open();
            }
            cam_type_ = detectPylonCamType(&camera_array[cam_pos]);
            camera_array[cam_pos].Close();

            switch (cam_type_)
            {
                case GIGE:
                    gige_cam_ = new Pylon::CBaslerGigEInstantCamera(CTlFactory::GetInstance()
                                                                                             .CreateFirstDevice());
                    break;
                case USB:
                    usb_cam_ = new Pylon::CBaslerUsbInstantCamera(CTlFactory::GetInstance()
                                                                                           .CreateFirstDevice());
                    break;
                case DART:
                    dart_cam_ = new Pylon::CBaslerUsbInstantCamera(CTlFactory::GetInstance()
                                                                                            .CreateFirstDevice());
                    break;
                default:
                    break;
            }

        }
        catch (GenICam::GenericException &e)
        {
            cerr << "An exception while opening the desired camera with Magazino ID: "
                 << params.magazino_cam_id_
                 << " occurred:"
                 << endl;
            cerr << e.GetDescription() << endl;
            return 3;
        }
    }

    return 0;
}
PYLON_CAM_TYPE PylonInterface::detectPylonCamType(const CInstantCamera* cam)
{
    PYLON_CAM_TYPE cam_type;
    try
    {
        VersionInfo sfnc_version = cam->GetSfncVersion();

        if (sfnc_version < Sfnc_2_0_0)
        { // ace GigE
          //CIntegerPtr ExposureTimeRaw (nodemap.GetNode("ExposureTimeRaw"));
            cam_type = GIGE;
        }
        else if (sfnc_version == Sfnc_2_1_0)
        { // ace USB
          // CFloatPtr ExposureTime (nodemap.GetNode("ExposureTime"));
            cam_type = USB;
        }
        else if (sfnc_version == Sfnc_2_2_0)
        { // Dart Cameras
          // CFloatPtr ExposureTime (nodemap.GetNode("ExposureTime"));
            cam_type = DART;
        }
    }
    catch (GenICam::GenericException &e)
    {
        cerr << "An exception while detecting the pylon camera type from its SFNC-Version occurred:"
             << endl;
        cerr << e.GetDescription() << endl;
    }
    return cam_type;
}

bool PylonInterface::setBrightness(int brightness)
{
    // Exposure Auto is the 'automatic' counterpart to manually setting the Exposure Time Abs parameter.
    // It adjusts the Exposure Time Abs parameter value automatically within set limits until a target
    // average gray value for the pixel data of the related Auto Function AOI is reached
    // -2: AutoExposureOnce //!<Sets operation mode to 'once'.
    // -1: AutoExposureContinuous //!<Sets operation mode to 'continuous'.
    //  0: AutoExposureOff //!<Disables the Exposure Auto function.
    // else: 8 bit brightness value, regardless of the current pixel data output format
    // 0 -> black, 255 -> white.

    if ((brightness == -1 || brightness == -2 || brightness == 0) && !has_auto_exposure_)
    {
        cerr << "Error while trying to set auto brightness properties: camera has no auto exposure function!"
             << endl;
        return 1;
    }
    else if (!(brightness == -1 || brightness == -2 || brightness == 0) && brightness < 0)
    {
        cerr << "Target brightness " << brightness << " not in the allowed range:" << endl;
        cerr << "-2: AutoExposureOnce, -1: AutoExposureContinuous, 0: AutoExposureOff, else: brightness 0-> black to 255 -> white"
             << endl;
        return 2;
    }

    int truncated_brightness = brightness;
    try
    {
        switch (cam_type_)
        {
            case GIGE:
                if (brightness == -2)
                {
                    gige_cam_->ExposureAuto.SetValue(Basler_GigECameraParams::ExposureAuto_Once);
                }
                else if (brightness == -1)
                {
                    gige_cam_->ExposureAuto.SetValue(Basler_GigECameraParams::ExposureAuto_Continuous);
                }
                else if (brightness == 0)
                {
                    gige_cam_->ExposureAuto.SetValue(Basler_GigECameraParams::ExposureAuto_Off);
                }
                else
                {
                    // Set the target value for luminance control. The value is always expressed
                    // as an 8 bit value regardless of the current pixel data output format,
                    // i.e., 0 -> black, 255 -> white.
                    if (gige_cam_->AutoTargetValue.GetMin() <= brightness && brightness
                                    <= gige_cam_->AutoTargetValue.GetMax())
                    {
                        // Use Pylon Auto Funciton, whenever in possible range
                        gige_cam_->AutoTargetValue.SetValue(brightness, false);
                        gige_cam_->ExposureAuto.SetValue(Basler_GigECameraParams::ExposureAuto_Once);
                    }
                    else if (gige_cam_->AutoTargetValue.GetMin() > brightness)
                    {
                        gige_cam_->ExposureAuto.SetValue(Basler_GigECameraParams::ExposureAuto_Off);
                        cout << "Desired brightness " << brightness
                             << " out of Pylon-Auto-Range [50-205]. Starting own Auto-function!"
                             << endl;
                        exposure_search_running_ = true;
                    } else if (gige_cam_->AutoTargetValue.GetMax() < brightness)
                    {
                        gige_cam_->ExposureAuto.SetValue(Basler_GigECameraParams::ExposureAuto_Off);
                        cout << "Desired brightness " << brightness
                             << " out of Pylon-Auto-Range [50-205]. Starting own Auto-function!"
                             << endl;
                        exposure_search_running_ = true;
                    } else
                    {
                        cerr << "ERROR unexpected brightness case" << endl;
                    }
                }
                break;
            case USB:
                if (brightness == -2)
                {
                    usb_cam_->ExposureAuto.SetValue(Basler_UsbCameraParams::ExposureAuto_Once);
                }
                else if (brightness == -1)
                {
                    usb_cam_->ExposureAuto.SetValue(Basler_UsbCameraParams::ExposureAuto_Continuous);
                }
                else if (brightness == 0)
                {
                    usb_cam_->ExposureAuto.SetValue(Basler_UsbCameraParams::ExposureAuto_Off);
                }
                else
                {
                    double brightness_f = brightness / 255.0;
                    // Set the target value for luminance control. The value is always expressed
                    // as an float value regardless of the current pixel data output format,
                    // i.e., 0.0 -> black, 1.0 -> white.
                    if (usb_cam_->AutoTargetBrightness.GetMin() <= brightness_f && brightness_f
                                    <= usb_cam_->AutoTargetBrightness.GetMax())
                    {
                        // Use Pylon Auto Funciton, whenever in possible range
                        usb_cam_->AutoTargetBrightness.SetValue(brightness_f, false);
                        usb_cam_->ExposureAuto.SetValue(Basler_UsbCameraParams::ExposureAuto_Once);
                    }
                    else if (usb_cam_->AutoTargetBrightness.GetMin() > brightness_f)
                    {
                        usb_cam_->ExposureAuto.SetValue(Basler_UsbCameraParams::ExposureAuto_Off);
                        cout << "Desired brightness " << brightness
                             << " out of Pylon-Auto-Range [50-205]. Starting own Auto-function!"
                             << endl;
                        exposure_search_running_ = true;
                    } else if (usb_cam_->AutoTargetBrightness.GetMax() < brightness_f)
                    {
                        usb_cam_->ExposureAuto.SetValue(Basler_UsbCameraParams::ExposureAuto_Off);
                        cout << "Desired brightness " << brightness
                             << " out of Pylon-Auto-Range [50-205]. Starting own Auto-function!"
                             << endl;
                        exposure_search_running_ = true;
                    } else
                    {
                        cerr << "ERROR unexpected brightness case" << endl;
                    }
                }
                break;
            case DART:
                if (brightness == -2)
                {
                    dart_cam_->ExposureAuto.SetValue(Basler_UsbCameraParams::ExposureAuto_Once);
                }
                else if (brightness == -1)
                {
                    dart_cam_->ExposureAuto.SetValue(Basler_UsbCameraParams::ExposureAuto_Continuous);
                }
                else if (brightness == 0)
                {
                    dart_cam_->ExposureAuto.SetValue(Basler_UsbCameraParams::ExposureAuto_Off);
                }
                else
                {
                    double brightness_f = brightness / 255.0;
                    // Set the target value for luminance control. The value is always expressed
                    // as an float value regardless of the current pixel data output format,
                    // i.e., 0.0 -> black, 1.0 -> white.
                    if (dart_cam_->AutoTargetBrightness.GetMin() <= brightness_f && brightness_f
                                    <= dart_cam_->AutoTargetBrightness.GetMax())
                    {
                        // Use Pylon Auto Funciton, whenever in possible range
                        dart_cam_->AutoTargetBrightness.SetValue(brightness_f, false);
                        dart_cam_->ExposureAuto.SetValue(Basler_UsbCameraParams::ExposureAuto_Once);
                    }
                    else if (dart_cam_->AutoTargetBrightness.GetMin() > brightness_f)
                    {
                        dart_cam_->ExposureAuto.SetValue(Basler_UsbCameraParams::ExposureAuto_Off);
                        cout << "Desired brightness " << brightness
                             << " out of Pylon-Auto-Range [50-205]. Starting own Auto-function!"
                             << endl;
                        exposure_search_running_ = true;
                    } else if (dart_cam_->AutoTargetBrightness.GetMax() < brightness_f)
                    {
                        dart_cam_->ExposureAuto.SetValue(Basler_UsbCameraParams::ExposureAuto_Off);
                        cout << "Desired brightness " << brightness
                             << " out of Pylon-Auto-Range [50-205]. Starting own Auto-function!"
                             << endl;
                        exposure_search_running_ = true;
                    } else
                    {
                        cerr << "ERROR unexpected brightness case" << endl;
                    }
                }
                break;
            default:
                break;
        }
        last_brightness_val_ = brightness;
    }
    catch (GenICam::GenericException &e)
    {
        cerr << "An exception while setting target brightness to " << brightness << " occurred:"
             << endl;
        cerr << e.GetDescription() << endl;
        return false;
    }
    return true;
}
// Own Auto Function for average brightness values out of the Pylon Range ([50-205])
bool PylonInterface::setExtendedBrightness(int& brightness)
{
    switch (cam_type_)
    {
        case GIGE:
        {
                    if (!exp_search_params_.is_initialized_)
                    {
                        gige_cam_->ExposureAuto.SetValue(Basler_GigECameraParams::ExposureAuto_Off);
                        if (!GenApi::IsWritable(gige_cam_->ExposureTimeBaseAbs))
                        {
                            cerr << "Pylon Exposure Auto Node not writable in own auto-exp-function!" << endl;
                            return false;
                        }
                        if (gige_cam_->AutoTargetValue.GetMin() > brightness)
                        {
                            exp_search_params_.initialize(brightness,
                                                          gige_cam_->ExposureTimeAbs.GetMin(),
                                                          gige_cam_->ExposureTimeAbs.GetValue(),
                                                          gige_cam_->ExposureTimeAbs.GetValue(),
                                                          exp_search_params_.current_brightness_);
                        } else
                        {
                            exp_search_params_.initialize(brightness,
                                                          gige_cam_->ExposureTimeAbs.GetValue(),
                                                          gige_cam_->ExposureTimeAbs.GetMax(),
                                                          gige_cam_->ExposureTimeAbs.GetValue(),
                                                          exp_search_params_.current_brightness_);
                        }
                    }

                    if (fabs(exp_search_params_.goal_brightness_ - exp_search_params_.current_brightness_) < 1)
                    {
                        exposure_search_running_ = false;
                        exp_search_params_.is_initialized_ = false;
                        brightness = exp_search_params_.current_brightness_;
                        cout << "Own Auto Function: Success! Goal = " <<  brightness << endl;
                        return true;
                    }
                    if (exp_search_params_.last_unchanged_exposure_counter_ > 2)
                    {
                        exposure_search_running_ = false;
                        exp_search_params_.is_initialized_ = false;
                        exp_search_params_.last_unchanged_exposure_counter_ = 0;
                        brightness = exp_search_params_.current_brightness_;
                        cout << "Own Auto Function: Success! Goal = " <<  brightness << endl;
                        return true;
                    }

                    exp_search_params_.updateBinarySearch();

                    // truncate desired exposure if out of range
                    if (exp_search_params_.desired_exposure_ < gige_cam_->ExposureTimeAbs.GetMin() || exp_search_params_
                                    .desired_exposure_
                                                                                                  > gige_cam_->ExposureTimeAbs
                                                                                                                  .GetMax())
                    {
                        if (exp_search_params_.desired_exposure_ < gige_cam_->ExposureTimeAbs.GetMin())
                        {
                            cout << "Desired mean brightness unreachable! Min possible exposure = "
                                 << gige_cam_->ExposureTimeAbs.GetMin()
                                 << ". Will limit to this value." << endl;
                            exp_search_params_.desired_exposure_ = gige_cam_->ExposureTimeAbs.GetMin();
                        } else if (exp_search_params_.desired_exposure_ > gige_cam_->ExposureTimeAbs.GetMax())
                        {
                            cout << "Desired mean brightness unreachable! Max possible exposure = "
                                 << gige_cam_->ExposureTimeAbs.GetMax()
                                 << ". Will limit to this value." << endl;
                            exp_search_params_.desired_exposure_ = gige_cam_->ExposureTimeAbs.GetMax();
                        }
                    }
                    // Current exposure  = min/max limit value -> auto function finished -> update brightness param
                    if (exp_search_params_.current_exposure_ == gige_cam_->ExposureTimeAbs.GetMin() || exp_search_params_
                                    .current_exposure_
                                                                                                   == gige_cam_->ExposureTimeAbs
                                                                                                                   .GetMax())
                    {
                        exposure_search_running_ = false;
                        exp_search_params_.is_initialized_ = false;
                        cout << "WILL USE SMALLES EXP POSSIBLE!!!" << endl;
                        brightness = exp_search_params_.current_brightness_;
                        return true;

                    }

                    //gige_cam_->ExposureAuto.SetValue(Basler_UsbCameraParams::ExposureAuto_Off);
                    gige_cam_->ExposureTimeAbs.SetValue(exp_search_params_.desired_exposure_);
                    // Update GeniCam Cache with GetNodeMap().InvalidateNodes()
                    gige_cam_->GetNodeMap().InvalidateNodes();
                    // Attention: Setting and Getting exposure not necessary the same: Difference of up to 35.0 ms
                    exp_search_params_.last_exposure_ = exp_search_params_.current_exposure_;
                    exp_search_params_.current_exposure_ = gige_cam_->ExposureTimeAbs.GetValue();

                    return false;
                }

        case USB:
            {
            double brightness_f = brightness / 255.0;

            if (!exp_search_params_.is_initialized_)
            {
                usb_cam_->ExposureAuto.SetValue(Basler_UsbCameraParams::ExposureAuto_Off);
                if (!GenApi::IsWritable(usb_cam_->ExposureTime))
                {
                    cerr << "Pylon Exposure Auto Node not writable in own auto-exp-function!" << endl;
                    return false;
                }
                if (usb_cam_->AutoTargetBrightness.GetMin() > brightness_f)
                {
                    exp_search_params_.initialize(brightness,
                                                  usb_cam_->ExposureTime.GetMin(),
                                                  usb_cam_->ExposureTime.GetValue(),
                                                  usb_cam_->ExposureTime.GetValue(),
                                                  exp_search_params_.current_brightness_);
                } else
                {
                    exp_search_params_.initialize(brightness,
                                                  usb_cam_->ExposureTime.GetValue(),
                                                  usb_cam_->ExposureTime.GetMax(),
                                                  usb_cam_->ExposureTime.GetValue(),
                                                  exp_search_params_.current_brightness_);
                }
            }

            if (fabs(exp_search_params_.goal_brightness_ - exp_search_params_.current_brightness_) < 1)
            {
                exposure_search_running_ = false;
                exp_search_params_.is_initialized_ = false;
                brightness = exp_search_params_.current_brightness_;
                cout << "Own Auto Function: Success! Goal = " <<  brightness << endl;
                return true;
            }
            if (exp_search_params_.last_unchanged_exposure_counter_ > 2)
            {
                exposure_search_running_ = false;
                exp_search_params_.is_initialized_ = false;
                exp_search_params_.last_unchanged_exposure_counter_ = 0;
                brightness = exp_search_params_.current_brightness_;
                cout << "Own Auto Function: Success! Goal = " <<  brightness << endl;
                return true;
            }

            exp_search_params_.updateBinarySearch();

            // truncate desired exposure if out of range
            if (exp_search_params_.desired_exposure_ < usb_cam_->ExposureTime.GetMin() || exp_search_params_
                            .desired_exposure_
                                                                                          > usb_cam_->ExposureTime
                                                                                                          .GetMax())
            {
                if (exp_search_params_.desired_exposure_ < usb_cam_->ExposureTime.GetMin())
                {
                    cout << "Desired mean brightness unreachable! Min possible exposure = "
                         << usb_cam_->ExposureTime.GetMin()
                         << ". Will limit to this value." << endl;
                    exp_search_params_.desired_exposure_ = usb_cam_->ExposureTime.GetMin();
                } else if (exp_search_params_.desired_exposure_ > usb_cam_->ExposureTime.GetMax())
                {
                    cout << "Desired mean brightness unreachable! Max possible exposure = "
                         << usb_cam_->ExposureTime.GetMax()
                         << ". Will limit to this value." << endl;
                    exp_search_params_.desired_exposure_ = usb_cam_->ExposureTime.GetMax();
                }
            }
            // Current exposure  = min/max limit value -> auto function finished -> update brightness param
            if (exp_search_params_.current_exposure_ == usb_cam_->ExposureTime.GetMin() || exp_search_params_
                            .current_exposure_
                                                                                           == usb_cam_->ExposureTime
                                                                                                           .GetMax())
            {
                exposure_search_running_ = false;
                exp_search_params_.is_initialized_ = false;
                cout << "WILL USE SMALLES EXP POSSIBLE!!!" << endl;
                brightness = exp_search_params_.current_brightness_;
                return true;

            }

            //usb_cam_->ExposureAuto.SetValue(Basler_UsbCameraParams::ExposureAuto_Off);
            usb_cam_->ExposureTime.SetValue(exp_search_params_.desired_exposure_);
            // Update GeniCam Cache with GetNodeMap().InvalidateNodes()
            usb_cam_->GetNodeMap().InvalidateNodes();
            // Attention: Setting and Getting exposure not necessary the same: Difference of up to 35.0 ms
            exp_search_params_.last_exposure_ = exp_search_params_.current_exposure_;
            exp_search_params_.current_exposure_ = usb_cam_->ExposureTime.GetValue();

            return false;
        }
        case DART:
        {
        double brightness_f = brightness / 255.0;

        if (!exp_search_params_.is_initialized_)
        {
            dart_cam_->ExposureAuto.SetValue(Basler_UsbCameraParams::ExposureAuto_Off);
            if (!GenApi::IsWritable(dart_cam_->ExposureTime))
            {
                cerr << "Pylon Exposure Auto Node not writable in own auto-exp-function!" << endl;
                return false;
            }
            if (dart_cam_->AutoTargetBrightness.GetMin() > brightness_f)
            {
                exp_search_params_.initialize(brightness,
                                              dart_cam_->ExposureTime.GetMin(),
                                              dart_cam_->ExposureTime.GetValue(),
                                              dart_cam_->ExposureTime.GetValue(),
                                              exp_search_params_.current_brightness_);
            } else
            {
                exp_search_params_.initialize(brightness,
                                              dart_cam_->ExposureTime.GetValue(),
                                              dart_cam_->ExposureTime.GetMax(),
                                              dart_cam_->ExposureTime.GetValue(),
                                              exp_search_params_.current_brightness_);
            }
        }

        if (fabs(exp_search_params_.goal_brightness_ - exp_search_params_.current_brightness_) < 1)
        {
            exposure_search_running_ = false;
            exp_search_params_.is_initialized_ = false;
            brightness = exp_search_params_.current_brightness_;
            cout << "Own Auto Function: Success! Goal = " <<  brightness << endl;
            return true;
        }
        if (exp_search_params_.last_unchanged_exposure_counter_ > 2)
        {
            exposure_search_running_ = false;
            exp_search_params_.is_initialized_ = false;
            exp_search_params_.last_unchanged_exposure_counter_ = 0;
            brightness = exp_search_params_.current_brightness_;
            cout << "Own Auto Function: Success! Goal = " <<  brightness << endl;
            return true;
        }

        exp_search_params_.updateBinarySearch();

        // truncate desired exposure if out of range
        if (exp_search_params_.desired_exposure_ < dart_cam_->ExposureTime.GetMin() || exp_search_params_
                        .desired_exposure_
                                                                                      > dart_cam_->ExposureTime
                                                                                                      .GetMax())
        {
            if (exp_search_params_.desired_exposure_ < dart_cam_->ExposureTime.GetMin())
            {
                cout << "Desired mean brightness unreachable! Min possible exposure = "
                     << dart_cam_->ExposureTime.GetMin()
                     << ". Will limit to this value." << endl;
                exp_search_params_.desired_exposure_ = dart_cam_->ExposureTime.GetMin();
            } else if (exp_search_params_.desired_exposure_ > dart_cam_->ExposureTime.GetMax())
            {
                cout << "Desired mean brightness unreachable! Max possible exposure = "
                     << dart_cam_->ExposureTime.GetMax()
                     << ". Will limit to this value." << endl;
                exp_search_params_.desired_exposure_ = dart_cam_->ExposureTime.GetMax();
            }
        }
        // Current exposure  = min/max limit value -> auto function finished -> update brightness param
        if (exp_search_params_.current_exposure_ == dart_cam_->ExposureTime.GetMin() || exp_search_params_
                        .current_exposure_
                                                                                       == dart_cam_->ExposureTime
                                                                                                       .GetMax())
        {
            exposure_search_running_ = false;
            exp_search_params_.is_initialized_ = false;
            cout << "WILL USE SMALLES EXP POSSIBLE!!!" << endl;
            brightness = exp_search_params_.current_brightness_;
            return true;

        }

        //dart_cam_->ExposureAuto.SetValue(Basler_UsbCameraParams::ExposureAuto_Off);
        dart_cam_->ExposureTime.SetValue(exp_search_params_.desired_exposure_);
        // Update GeniCam Cache with GetNodeMap().InvalidateNodes()
        dart_cam_->GetNodeMap().InvalidateNodes();
        // Attention: Setting and Getting exposure not necessary the same: Difference of up to 35.0 ms
        exp_search_params_.last_exposure_ = exp_search_params_.current_exposure_;
        exp_search_params_.current_exposure_ = dart_cam_->ExposureTime.GetValue();

        return false;
    }
        default:
            break;
    }

    return false;
}
int PylonInterface::setExposure(double exposure)
{
    // Exposure Auto is the 'automatic' counterpart to manually setting the Exposure Time Abs parameter.
    // It adjusts the Exposure Time Abs parameter value automatically within set limits until a target
    // average gray value for the pixel data of the related Auto Function AOI is reached
    // -2: AutoExposureOnce //!<Sets operation mode to 'once'.
    // -1: AutoExposureContinuous //!<Sets operation mode to 'continuous'.
    //  0: AutoExposureOff //!<Disables the Exposure Auto function.

    if ((exposure == -1.0 || exposure == -2.0 || exposure == 0.0) && !has_auto_exposure_)
    {
        cerr << "Error while trying to set auto exposure properties: camera has no auto exposure function!"
             << endl;
        return 1;
    }
    else if (!(exposure == -1.0 || exposure == -2.0 || exposure == 0.0) && exposure < 0.0)
    {
        cerr << "Target Exposure " << exposure << " not in the allowed range:" << endl;
        cerr << "-2: AutoExposureOnce, -1: AutoExposureContinuous, 0: AutoExposureOff, else: exposure in mus"
             << endl;
        return 2;
    }

    try
    {
        switch (cam_type_)
        {
            case GIGE:
                if (exposure == -2.0)
                {
                    gige_cam_->ExposureAuto.SetValue(Basler_GigECameraParams::ExposureAuto_Once);
                }
                else if (exposure == -1.0)
                {
                    gige_cam_->ExposureAuto.SetValue(Basler_GigECameraParams::ExposureAuto_Continuous);
                }
                else if (exposure == 0.0)
                {
                    gige_cam_->ExposureAuto.SetValue(Basler_GigECameraParams::ExposureAuto_Off);
                }
                else
                {
                    if (has_auto_exposure_)
                    {
                        gige_cam_->ExposureAuto.SetValue(Basler_GigECameraParams::ExposureAuto_Off);
                    }
                    if (gige_cam_->ExposureTimeAbs.GetMin() > exposure)
                    {
                        exposure = gige_cam_->ExposureTimeAbs.GetMin();
                        cout << "Desired exposure time unreachable! Setting to lower limit: "
                             << exposure
                             << endl;
                    }
                    else if (gige_cam_->ExposureTimeAbs.GetMax() < exposure)
                    {
                        exposure = gige_cam_->ExposureTimeAbs.GetMax();
                        cout << "Desired exposure time unreachable! Setting to upper limit: "
                             << exposure
                             << endl;
                    }
                    gige_cam_->ExposureTimeAbs.SetValue(exposure, false);
                }
                break;
            case USB:
                if (exposure == -2.0)
                {
                    usb_cam_->ExposureAuto.SetValue(Basler_UsbCameraParams::ExposureAuto_Once);
                }
                else if (exposure == -1.0)
                {
                    usb_cam_->ExposureAuto.SetValue(Basler_UsbCameraParams::ExposureAuto_Continuous);
                }
                else if (exposure == 0.0)
                {
                    usb_cam_->ExposureAuto.SetValue(Basler_UsbCameraParams::ExposureAuto_Off);
                }
                else
                {
                    if (has_auto_exposure_)
                    {
                        usb_cam_->ExposureAuto.SetValue(Basler_UsbCameraParams::ExposureAuto_Off);
                    }
                    usb_cam_->ExposureTime.SetValue(exposure, false);
                }
                break;
            case DART:
                if (exposure == -2.0)
                {
                    dart_cam_->ExposureAuto.SetValue(Basler_UsbCameraParams::ExposureAuto_Once);
                }
                else if (exposure == -1.0)
                {
                    dart_cam_->ExposureAuto.SetValue(Basler_UsbCameraParams::ExposureAuto_Continuous);
                }
                else if (exposure == 0.0)
                {
                    dart_cam_->ExposureAuto.SetValue(Basler_UsbCameraParams::ExposureAuto_Off);
                }
                else
                {
                    if (has_auto_exposure_)
                    {
                        dart_cam_->ExposureAuto.SetValue(Basler_UsbCameraParams::ExposureAuto_Off);
                    }
                    dart_cam_->ExposureTime.SetValue(exposure, false);
                }
                break;
            default:
                cerr << "Unknown Camera Type" << endl;
                break;
        }
        last_exposure_val_ = exposure;
    }
    catch (GenICam::GenericException &e)
    {
        cerr << "An exception while setting target exposure to " << exposure << " occurred:"
             << endl;
        cerr << e.GetDescription() << endl;
    }
    return 0;
}
void PylonInterface::set_image_size(int size)
{
    this->img_size_byte_ = size;
}
int PylonInterface::image_size()
{
    return this->img_size_byte_;
}
bool PylonInterface::is_cam_removed()
{
    return is_cam_removed_;
}
double PylonInterface::last_exposure_val()
{
    return last_exposure_val_;
}
int PylonInterface::last_brightness_val()
{
    return last_brightness_val_;
}
int PylonInterface::img_rows()
{
    return this->img_rows_;
}
int PylonInterface::img_cols()
{
    return this->img_cols_;
}
float PylonInterface::max_possible_framerate()
{
    return this->max_framerate_;
}
bool PylonInterface::has_auto_exposure()
{
    return has_auto_exposure_;
}
std::string PylonInterface::img_encoding()
{
    switch (cam_type_)
    {
        case GIGE:
            switch (gige_img_encoding_)
            {
                case Basler_GigECameraParams::PixelFormat_Mono8:
                    return "mono8";
                    break;
                default:
                    cerr << "Image encoding " << gige_img_encoding_ << " not yet implemented!"
                         << endl;
                    return "";
                    break;
            }
            break;
        case USB:
            switch (usb_img_encoding_)
            {
                case Basler_UsbCameraParams::PixelFormat_Mono8:
                    return "mono8";
                    break;
                default:
                    cerr << "Image encoding " << usb_img_encoding_ << " not yet implemented!"
                         << endl;
                    return "";
                    break;
            }
            break;
        case DART:
            switch (usb_img_encoding_)
            {
                case Basler_UsbCameraParams::PixelFormat_Mono8:
                    return "mono8";
                    break;
                default:
                    cerr << "Image encoding " << usb_img_encoding_ << " not yet implemented!"
                         << endl;
                    return "";
                    break;
            }
    }
    return "";
    // List from #include <sensor_msgs/image_encodings.h>
    //    const std::string RGB8 = "rgb8";
    //    const std::string RGBA8 = "rgba8";
    //    const std::string RGB16 = "rgb16";
    //    const std::string RGBA16 = "rgba16";
    //    const std::string BGR8 = "bgr8";
    //    const std::string BGRA8 = "bgra8";
    //    const std::string BGR16 = "bgr16";
    //    const std::string BGRA16 = "bgra16";
    //    const std::string MONO8="mono8";
    //    const std::string MONO16="mono16";
}
int PylonInterface::img_pixel_depth()
{
    switch (cam_type_)
    {
        case GIGE:
            switch (gige_img_pixel_depth_)
            {
                case Basler_GigECameraParams::PixelSize_Bpp8:
                    return sizeof(uint8_t);
                    break;

                default:
                    cerr << "Image Pixel Depth not yet implemented!" << endl;
                    return -1;
                    break;
            }
            break;
        case USB:
            switch (usb_img_pixel_depth_)
            {
                case Basler_UsbCameraParams::PixelSize_Bpp8:
                    return sizeof(uint8_t);
                    break;
                    //		case Basler_UsbCameraParams::PixelSize_Bpp12:
                    //			return 12;
                    //			break;
                    //		case Basler_UsbCameraParams::PixelSize_Bpp16:
                    //			return 16;
                    //			break;
                    //		case Basler_UsbCameraParams::PixelSize_Bpp24:
                    //			return 24;
                    //			break;
                default:
                    cerr << "Image Pixel Depth not yet implemented!" << endl;
                    return -1;
                    break;
            }
            break;
        case DART:
            switch (usb_img_pixel_depth_)
            {
                case Basler_UsbCameraParams::PixelSize_Bpp8:
                    return sizeof(uint8_t);
                    break;
                default:
                    cerr << "Image Pixel Depth not yet implemented!" << endl;
                    return -1;
                    break;
            }
    }
    return -1;

//	switch (img_pixel_depth_) {
//	case Basler_UsbCameraParams::PixelSize_Bpp8:
//		return sizeof(uint8_t);
//		break;
//
//	default:
//		cerr << "Image Pixel Depth not yet implemented!" << endl;
//		return -1;
//		break;
//	}
//	return -1;
}
std::string PylonInterface::pylonCamTypeToString(const PYLON_CAM_TYPE type)
{
    switch (type)
    {
        case GIGE:
            return "GigE";
        case USB:
            return "USB";
        case DART:
            return "Dart";
    }
    return "Unknown Camera Type";
}
//int PylonInterface::terminate(const PylonCameraParameter &params)
//{
//    // Dummy -> only used in PylonSequencerInterface
//    return 0;
//}
} /* namespace pylon_camera */
