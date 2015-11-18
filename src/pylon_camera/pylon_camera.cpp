#include <pylon_camera/internal/pylon_camera.h>

using namespace Pylon;

namespace pylon_camera
{

enum PYLON_CAM_TYPE
{
    GIGE = 1,
    USB = 2,
    DART = 3,
    UNKNOWN = -1,
};

PYLON_CAM_TYPE detectPylonCamType(CInstantCamera* cam)
{
    VersionInfo sfnc_version;
    try
    {
        sfnc_version = cam->GetSfncVersion();
    }
    catch (const GenICam::GenericException &e)
    {
        std::cerr << "An exception while detecting the pylon camera type from its SFNC-Version occurred:" << std::endl;
        std::cerr << e.GetDescription() << std::endl;
        return UNKNOWN;
    }

    switch (sfnc_version.getMinor())
    {
        case 0: // GigE Camera: Sfnc_2_0_0
            return GIGE;
        case 1: // USB Camera: Sfnc_2_1_0
            return USB;
        case 2: // DART Camera: Sfnc_2_2_0
            return DART;
        default:
            std::cerr << "Unknown Camera Type!" << std::endl;
            return UNKNOWN;
    }
}

PylonCamera* createFromDevice(PYLON_CAM_TYPE cam_type, IPylonDevice* device)
{
    switch (cam_type)
    {
        case GIGE:
            return new PylonGigECamera(device);
        case USB:
            return new PylonUSBCamera(device);
        case DART:
            return new PylonDARTCamera(device);
        case UNKNOWN:
        default:
            return NULL;
    }
}


PylonCamera* PylonCamera::create()
{
    try
    {
    	// BaslerDebugDay: It is possible to detect the correct camera without opening it first
    	// Hence use CDeviceInfo info; to detect the name

        CInstantCamera* cam = new CInstantCamera(CTlFactory::GetInstance().CreateFirstDevice());
        cam->Open();

        {
            GenApi::INodeMap& node_map = cam->GetNodeMap();
            GenApi::CStringPtr DeviceUserID(node_map.GetNode("DeviceUserID"));
            std::string device_user_id(DeviceUserID->GetValue());
            std::cout << "Using camera " << device_user_id << std::endl;
        }

        PYLON_CAM_TYPE cam_type = detectPylonCamType(cam);

        cam->Close();
        delete cam;
        return createFromDevice(cam_type, CTlFactory::GetInstance().CreateFirstDevice());
    }
    catch (const GenICam::GenericException &e)
    {
        std::cerr << "Error while PylonCamera::create(). Exception: " << e.what() << std::endl;
        return NULL;
    }
}

PylonCamera* PylonCamera::create(const std::string& name)
{
    if (name.empty() || name.compare("x") == 0)
    {
        return create();
    }
    try
    {
        // Get the transport layer factory.
        CTlFactory& transport_layer_factory = CTlFactory::GetInstance();

        // Get all attached devices and exit application if no device is found.
        DeviceInfoList_t device_info_list;

        if (transport_layer_factory.EnumerateDevices(device_info_list) == 0)
        {
            throw RUNTIME_EXCEPTION( "No camera present.");
            return NULL;
        }

        // Create an array of instant cameras for the found devices
        CInstantCameraArray camera_array(device_info_list.size());

        bool found_desired_device = false;

        // Create and attach all Pylon Devices.
        size_t cam_pos = -1;
        for (size_t i = 0; i < camera_array.GetSize(); ++i)
        {
            try
            {
                camera_array[i].Attach(transport_layer_factory.CreateDevice(device_info_list[i]));

                camera_array[i].Open();

                GenApi::INodeMap& node_map = camera_array[i].GetNodeMap();
                GenApi::CStringPtr DeviceUserID(node_map.GetNode("DeviceUserID"));
                std::string device_user_id(DeviceUserID->GetValue());

                camera_array[i].Close();

//                std::cout << "device_user_id " << device_user_id << std::endl;
//                std::cout << "length " << device_user_id.size() << std::endl;

                if (device_user_id.compare(name) == 0) // ||
//                    device_user_id.compare(device_user_id.length() - name.length(), name.length(), name))
                {
                    found_desired_device = true;
                    cam_pos = i;
                    std::cout << "Found the desired Camera with Magazino ID: " << name
                              << ": "
                              << camera_array[cam_pos].GetDeviceInfo().GetModelName()
                              << std::endl;
                    break;
                }
            }
            catch (const GenICam::GenericException &e)
            {
                continue;
            }
        }

        if (!found_desired_device)
        {
            std::cerr << "Maybe the given magazino_cam_id ("
                      << name
                      << ") is wrong or has not yet been written to the camera using 'write_magazino_id_to_camera' ?!"
                      << std::endl;
            return NULL;
        }


        if (!camera_array[cam_pos].IsOpen())
        {
            camera_array[cam_pos].Open();
        }
        PYLON_CAM_TYPE cam_type = detectPylonCamType(&camera_array[cam_pos]);
        camera_array[cam_pos].Close();

        return createFromDevice(cam_type, transport_layer_factory.CreateDevice(camera_array[cam_pos].GetDeviceInfo()));
    }
    catch (GenICam::GenericException &e)
    {
        std::cerr << "An exception while opening the desired camera with Magazino ID: "
                  << name
                  << " occurred:"
                  << std::endl;
        std::cerr << e.GetDescription() << std::endl;
        return NULL;
    }
}

PylonCamera::PylonCamera()
    : img_rows_(-1)
    , img_cols_(-1)
    , height_aoi_(-1)
    , width_aoi_(-1)
    , offset_height_aoi_(-1)
    , offset_width_aoi_(-1)
    , img_size_byte_(-1)
    , max_framerate_(-1.0)
    , has_auto_exposure_(false)
    , last_exposure_val_(2000.0)
    , last_brightness_val_(-1)
    , is_ready_(false)
    , is_cam_removed_(false)
    , is_own_brightness_function_running_(false)
{
}

PylonCamera::~PylonCamera()
{
}

const int& PylonCamera::imageRows() const
{
    return img_rows_;
}

const int& PylonCamera::imageCols() const
{
    return img_cols_;
}

void PylonCamera::setImageSize(const int& size)
{
    img_size_byte_ = size;
}

const int& PylonCamera::imageSize() const
{
    return img_size_byte_;
}

const float& PylonCamera::maxPossibleFramerate() const
{
    return max_framerate_;
}

const bool& PylonCamera::hasAutoExposure() const
{
    return has_auto_exposure_;
}

const bool& PylonCamera::isCamRemoved() const
{
    return is_cam_removed_;
}

const double& PylonCamera::lastExposureValue() const
{
    return last_exposure_val_;
}

const bool& PylonCamera::isReady() const
{
    return is_ready_;
}

const int& PylonCamera::lastBrightnessValue() const
{
    return last_brightness_val_;
}

const std::vector<float>& PylonCamera::sequencerExposureTimes() const
{
    return seq_exp_times_;
}

const bool& PylonCamera::isOwnBrightnessFunctionRunning() const
{
    return is_own_brightness_function_running_;
}

}
