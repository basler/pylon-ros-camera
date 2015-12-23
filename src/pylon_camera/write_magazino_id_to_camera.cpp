// Copyright 2015 <Magazino GmbH>
/*
 This program will open a Basler Pylon Camera and write a desired Magazino Cam ID to it.
 Naming-Convention:
 ######################
 # PROJECT_NR_CAM-POS #
 ######################
*/

// Include files to use the PYLON API.
#include <pylon/PylonIncludes.h>

#include <algorithm>
#include <unistd.h>
#include <string>

// TODO: Smoke-Test with Signal Handler
int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        std::cerr << "ERROR: No Magazino Cam ID set!" << std::endl;
        std::cout << "USAGE: write_magazino_id_to_camera MAGAZINO_ID" << std::endl;
        return 1;
    }

    // TODO: regular expression, instead of only catching 2 '_'
    std::string magazino_id(reinterpret_cast<char *>(argv[1]));
    if (std::count(magazino_id.begin(), magazino_id.end(), '_') != 2)
    {
        std::cout << "ERROR:" << std::endl;
        std::cout << "Your desired Magazino ID (" << magazino_id << ") does not follow the naming conventions:"
                  << std::endl;
        std::cout << "--------------------------------------------------------------------" << std::endl;
        std::cout << "###########################" << std::endl;
        std::cout << "# PROJECT-NAME_NR_CAM-POS #" << std::endl;
        std::cout << "###########################" << std::endl;
        std::cout << "Example:" << std::endl;
        std::cout << "ABC_0001_XYZ -> Camera XYZ for project ABC" << std::endl;
        std::cout << "--------------------------------------------------------------------" << std::endl;
        return 2;
    }

    // Before using any pylon methods, the pylon runtime must be initialized.
    Pylon::PylonInitialize();

    try
    {
        Pylon::CDeviceInfo di;

        // TODO: Multiple cameras connected? -> Don't use first device found
        // TODO: Write IP to Camera?

        // Create an instant camera object with the camera device found first.
        Pylon::CInstantCamera camera(Pylon::CTlFactory::GetInstance().CreateFirstDevice(di));

        camera.Open();

        while (!camera.IsOpen())
        {
            usleep(1000);
        }

        GenApi::INodeMap& node_map = camera.GetNodeMap();
        GenApi::CStringPtr device_user_id(node_map.GetNode("DeviceUserID"));
        device_user_id->SetValue(Pylon::String_t(magazino_id.c_str()));

        std::cout << "Successfully wrote " << device_user_id->GetValue() << " to the camera "
                  << camera.GetDeviceInfo().GetModelName() << std::endl;
        camera.Close();
    }
    catch (GenICam::GenericException &e)
    {
        // Error handling.
        std::cerr << "An exception occurred."
                  << std::endl
                  << e.GetDescription()
                  << std::endl;
        return 3;
    }

    // Releases all pylon resources.
    Pylon::PylonTerminate();

    return 0;
}
