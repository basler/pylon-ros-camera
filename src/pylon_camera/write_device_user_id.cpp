/*
 This program will open a Basler Pylon Camera and write the device_user_id to it.

TODO: Provide way to select camera e.g. via serial number or IP

 */

#include <pylon/PylonIncludes.h>

#include <unistd.h>
#include <algorithm>

using namespace Pylon;
using namespace GenApi;
using namespace std;

//TODO: Smoke-Test with Signal Handler
int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        cerr << "ERROR: no device_user_id given" << endl;
        cout << "USAGE: ./write_device_user_id new_device_user_id" << endl;
        return EXIT_FAILURE;
    }


    std::string camera_name((char *)argv[1]);

    // Automagically call PylonInitialize and PylonTerminate to ensure the pylon runtime system
    // is initialized during the lifetime of this object.
    Pylon::PylonAutoInitTerm auto_init_term;
    try
    {
        CDeviceInfo di;


        // Create an instant camera object with the camera device found first.
        CInstantCamera camera(CTlFactory::GetInstance().CreateFirstDevice(di));

        camera.Open();

        while (!camera.IsOpen())
        {
            usleep(1000);
        }

        INodeMap& node_map = camera.GetNodeMap();
        CStringPtr device_user_id(node_map.GetNode("DeviceUserID"));
        device_user_id->SetValue(String_t(camera_name.c_str()));

        cout << "Successfully wrote " << device_user_id->GetValue() << " to the camera "
             << camera.GetDeviceInfo().GetModelName()
             << endl;
        camera.Close();
    }
    catch (GenICam::GenericException &e)
    {
        cerr << "An exception occurred:" << endl << e.GetDescription() << endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
