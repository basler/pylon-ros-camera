// Copyright 2015 <Magazino GmbH>

#include <pylon_camera/internal/pylon_camera.h>
#include <string>
#include <vector>


namespace pylon_camera
{

enum PYLON_CAM_TYPE
{
    GIGE = 1,
    USB = 2,
    DART = 3,
    UNKNOWN = -1,
};

PYLON_CAM_TYPE detectPylonCamType(Pylon::CInstantCamera* cam)
{
    Pylon::VersionInfo sfnc_version;
    try
    {
        sfnc_version = cam->GetSfncVersion();
    }
    catch (const GenICam::GenericException &e)
    {
        ROS_ERROR_STREAM("An exception while detecting the pylon camera type from its SFNC-Version occurred: "
                         << e.GetDescription());
        return UNKNOWN;
    }

    switch (sfnc_version.getMinor())
    {
    case 0:  // GigE Camera: Sfnc_2_0_0
        return GIGE;
    case 1:  // USB Camera: Sfnc_2_1_0
        return USB;
    case 2:  // DART Camera: Sfnc_2_2_0
        return DART;
    default:
        ROS_ERROR("Unknown Camera Type!");
        return UNKNOWN;
    }
}

PylonCamera* createFromDevice(PYLON_CAM_TYPE cam_type, Pylon::IPylonDevice* device)
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
        // Before using any pylon methods, the pylon runtime must be initialized.
        Pylon::PylonInitialize();

        // BaslerDebugDay: It is possible to detect the correct camera without opening it first
        // Hence use CDeviceInfo info; to detect the name

        Pylon::CInstantCamera* cam = new Pylon::CInstantCamera(Pylon::CTlFactory::GetInstance().CreateFirstDevice());
        cam->Open();

        {
            GenApi::INodeMap& node_map = cam->GetNodeMap();
            GenApi::CStringPtr DeviceUserID(node_map.GetNode("DeviceUserID"));
            std::string device_user_id(DeviceUserID->GetValue());
            ROS_INFO_STREAM("Using camera " << device_user_id);
        }

        PYLON_CAM_TYPE cam_type = detectPylonCamType(cam);

        cam->Close();
        delete cam;
        return createFromDevice(cam_type, Pylon::CTlFactory::GetInstance().CreateFirstDevice());
    }
    catch (const GenICam::GenericException &e)
    {
        ROS_ERROR_STREAM("Error while PylonCamera::create(). Exception: " << e.what());
        return NULL;
    }
}

PylonCamera* PylonCamera::create(const std::string& device_user_id_to_open)
{
    if (device_user_id_to_open.empty())
    {
        return create();
    }
    try
    {
        // Before using any pylon methods, the pylon runtime must be initialized.
        Pylon::PylonInitialize();

        // Get the transport layer factory.
        Pylon::CTlFactory& transport_layer_factory = Pylon::CTlFactory::GetInstance();

        // Get all attached devices and exit application if no device is found.
        Pylon::DeviceInfoList_t device_info_list;

        if (transport_layer_factory.EnumerateDevices(device_info_list) == 0)
        {
            ROS_ERROR("No camera present.");
            return NULL;
        }

        // Create an array of instant cameras for the found devices
        Pylon::CInstantCameraArray camera_array(device_info_list.size());

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
                GenApi::CStringPtr cam_device_user_id_ptr(node_map.GetNode("DeviceUserID"));
                std::string cam_device_user_id(cam_device_user_id_ptr->GetValue());
                camera_array[i].Close();
                if (cam_device_user_id.compare(device_user_id_to_open) == 0 ||
                    (device_user_id_to_open.length() < cam_device_user_id.length() &&
                     0 == cam_device_user_id.compare(cam_device_user_id.length() - device_user_id_to_open.length(),
                                                     device_user_id_to_open.length(),
                                                     device_user_id_to_open)))
                {
                    found_desired_device = true;
                    cam_pos = i;
                    ROS_INFO_STREAM("Found the desired Camera with DeviceUserID: " << device_user_id_to_open
                                    << ": " << camera_array[cam_pos].GetDeviceInfo().GetModelName());
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
            ROS_ERROR_STREAM("Maybe the given DeviceUserID (" <<
                             device_user_id_to_open <<
                             ") is wrong or has not yet been written to the camera");
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
        ROS_ERROR_STREAM("An exception while opening the desired camera with Device User ID: <"
                         << device_user_id_to_open << "> occurred:" << e.GetDescription());
        return NULL;
    }
}

PylonCamera::PylonCamera()
    : img_rows_(-1)
    , img_cols_(-1)
    , img_size_byte_(-1)
    , max_framerate_(-1.0)
    , has_auto_exposure_(false)
    , is_ready_(false)
    , is_cam_removed_(false)
    , is_own_brightness_function_running_(false)
{
}

PylonCamera::~PylonCamera()
{
    // Releases all pylon resources.
    Pylon::PylonTerminate();
}

const int& PylonCamera::imageRows() const
{
    return img_rows_;
}

const int& PylonCamera::imageCols() const
{
    return img_cols_;
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

/**
 * Flag which is set in case that the grab-result-pointer of the first acquisition contains valid data
 * From this point on, the interface is ready
 */
const bool& PylonCamera::isReady() const
{
    return is_ready_;
}

const std::vector<float>& PylonCamera::sequencerExposureTimes() const
{
    return seq_exp_times_;
}

const bool& PylonCamera::isOwnBrightnessFunctionRunning() const
{
    return is_own_brightness_function_running_;
}


}  // namespace pylon_camera
