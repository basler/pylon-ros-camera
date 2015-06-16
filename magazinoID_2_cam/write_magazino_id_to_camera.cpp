// write_magazino_id_to_camera.cpp
/*
 This program will open a Basler Pylon Camera and write a desired Magazino Cam ID to it.
 Naming-Convention:
 ######################
 # PROJECT_NR_CAM-POS #
 ######################

 Example:
 MARU_0001_a -> Insertion cam of the first Maru (Bille)
 MARU_0002_b -> Crane cam of the second Maru
 SOL_0002_b -> Left cam of second SoL-system
 */

// Include files to use the PYLON API.
#include <pylon/PylonIncludes.h>
#include <unistd.h>
#include <algorithm>
using namespace Pylon;
using namespace GenApi;
using namespace std;

int main(int argc, char* argv[])
{

  if (argc < 2)
  {
    cerr << "ERROR: No Magazino Cam ID set!" << endl;
    cout << "USAGE: write_magazino_id_to_camera MAGAZINO_ID" << endl;
    return 1;
  }
  std::string magazino_id((char *)argv[1]);
  if (std::count(magazino_id.begin(), magazino_id.end(), '_') != 2)
  {
    cout << "ERROR:" << endl;
    cout << "Your desired Magazino ID (" << magazino_id << ") does not follow the naming conventions:" << endl;
    cout << "--------------------------------------------------------------------" << endl;
    cout << "###########################" << endl;
    cout << "# PROJECT-NAME_NR_CAM-POS #" << endl;
    cout << "###########################" << endl;
    cout << "Example:" << endl;
    cout << "MARU_0001_a -> Insertion cam of the first Maru (Bille)" << endl;
    cout << "MARU_0002_b -> Crane cam of the second Maru" << endl;
    cout << "SOL_0002_b -> Left cam of second SoL-system" << endl;
    cout << "--------------------------------------------------------------------" << endl;
    return 2;
  }

  // Automagically call PylonInitialize and PylonTerminate to ensure the pylon runtime system
  // is initialized during the lifetime of this object.
  Pylon::PylonAutoInitTerm autoInitTerm;

  CInstantCamera* camera = NULL;

  try
  {
    CDeviceInfo di;
    // Create an instant camera object with the camera device found first.
    camera = new CInstantCamera(CTlFactory::GetInstance().CreateFirstDevice(di));

    camera->Open();

    while (!camera->IsOpen())
    {
      usleep(1000);
    }

    INodeMap& node_map = camera->GetNodeMap();
    CStringPtr DeviceUserID(node_map.GetNode("DeviceUserID"));
    DeviceUserID->SetValue(String_t(magazino_id.c_str()));

    cout << "Successfully wrote " << DeviceUserID->GetValue() << " to the camera "
         << camera->GetDeviceInfo().GetModelName() << endl;
    camera->Close();
  }
  catch (GenICam::GenericException &e)
  {
    // Error handling.
    cerr << "An exception occurred." << endl << e.GetDescription() << endl;
    return 3;
  }

  return 0;
}
