#include <pylon/PylonIncludes.h>
#include <algorithm>
#include <unistd.h>
#include <string>

int main(int argc, char* argv[])
{
    if ( argc < 3 )
    {
        std::cerr << "ERROR: Not enough parameters!" << std::endl;
        std::cout << "USAGE: set_user_id_to_camera SERIAL DEVICE_USER_ID" << std::endl;
        return 1;
    }

    std::string desired_device_user_id(reinterpret_cast<char *>(argv[2]));
    if ( desired_device_user_id.empty() )
    {
        std::cout << "ERROR:" << std::endl;
        std::cout << "Your desired device_user_id is empty!" << std::endl;
        return 2;
    }

    std::string serial(reinterpret_cast<char *>(argv[1]));
    if ( serial.empty() )
    {
        std::cout << "ERROR:" << std::endl;
        std::cout << "Your serial is empty!" << std::endl;
        return 2;
    }

    Pylon::PylonInitialize();

    try {
        Pylon::CTlFactory &tl_factory = Pylon::CTlFactory::GetInstance();

        Pylon::DeviceInfoList_t devices;
        if (tl_factory.EnumerateDevices(devices) == 0) {
            std::cout << "No cameras detected!" << std::endl;
            return false;
        }
        size_t i = 0;
        for (i; i < devices.size(); i++) {
            Pylon::CDeviceInfo &dev_info = devices[i];
            std::string serial_number(dev_info.GetSerialNumber().c_str());

            if(serial_number == serial){
                Pylon::CInstantCamera dev(tl_factory.CreateDevice(dev_info));
                dev.Open();
                GenApi::INodeMap& node_map = dev.GetNodeMap();
                GenApi::CStringPtr current_device_user_id(node_map.GetNode("DeviceUserID"));
                current_device_user_id->SetValue(Pylon::String_t(desired_device_user_id.c_str()));

                std::cout << "Successfully wrote " << current_device_user_id->GetValue() << " to the camera "
                  << dev.GetDeviceInfo().GetModelName() << std::endl;
                dev.Close();
                break;
            }
        }
        if (i == devices.size())
            std::cout << "Camera with serial " << serial << " not found!\n\nVerify serial with \n\tlsusb -v | grep Serial" << std::endl;
    } catch (GenICam::GenericException &e) {
        std::cout << "Exception occured: " << e.what() << std::endl;
    return false;
    }
    // Releases all pylon resources.
    Pylon::PylonTerminate();

    return 0;
}
