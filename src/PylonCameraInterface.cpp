#include "pylon_camera/PylonCameraInterface.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace Pylon;
using namespace std;


PylonCameraInterface::PylonCameraInterface(){
    camera = NULL;
}

PylonCameraInterface::~PylonCameraInterface(){
    cerr << "Closing camera";
    if (camera)
        camera->Close();
}


bool PylonCameraInterface::openCamera(){

    // The exit code of the sample application.
    int exitCode = 0;

    // Automagically call PylonInitialize and PylonTerminate to ensure the pylon runtime system
    // is initialized during the lifetime of this object.

    try{


        TlInfoList_t tl_list;
        CTlFactory::GetInstance().EnumerateTls(tl_list);
        cout << "tl size " << tl_list.size() << endl;

        Pylon::DeviceInfoList_t lstDevices;
        Pylon::CTlFactory::GetInstance().EnumerateDevices(lstDevices);


        if (tl_list.size() == 0){
            cerr << "No transport layer, check environment variables!";
            return false;
        }

        if (lstDevices.size() == 0){
            cerr << "No Camera detected";
            return false;
        }

        camera = new Pylon::CBaslerGigEInstantCamera(CTlFactory::GetInstance().CreateFirstDevice());
        // camera->MaxNumBuffer = 5;

        // Print the model name of the camera.
        cout << "Using device " << camera->GetDeviceInfo().GetModelName() << endl;

    }
    catch (GenICam::GenericException &e)
    {
        // Error handling.
        cerr << "An exception occurred." << endl
             << e.GetDescription() << endl;
        exitCode = 1;
    }

    return true;
}

void PylonCameraInterface::testRun(){


    camera->StartGrabbing( 100 );

    // This smart pointer will receive the grab result data.
    CGrabResultPtr ptrGrabResult;

    // Camera.StopGrabbing() is called automatically by the RetrieveResult() method
    // when c_countOfImagesToGrab images have been retrieved.
    while ( camera->IsGrabbing())
    {
        // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
        camera->RetrieveResult( 5000, ptrGrabResult, TimeoutHandling_ThrowException);

        // Image grabbed successfully?
        if (ptrGrabResult->GrabSucceeded())
        {
            // Access the image data.
            cout << "SizeX: " << ptrGrabResult->GetWidth() << endl;
            cout << "SizeY: " << ptrGrabResult->GetHeight() << endl;
            const uint8_t *pImageBuffer = (uint8_t *) ptrGrabResult->GetBuffer();
            cout << "Gray value of first pixel: " << (uint32_t) pImageBuffer[0] << endl << endl;

            cv::Mat img(ptrGrabResult->GetHeight(),ptrGrabResult->GetWidth(),CV_8UC1);

            memcpy(img.data,(uint8_t *) ptrGrabResult->GetBuffer(),
                   ptrGrabResult->GetHeight()*ptrGrabResult->GetWidth() );

            cv::imwrite("/opt/tmp/gig.jpg",img);





#ifdef PYLON_WIN_BUILD
            // Display the grabbed image.
            Pylon::DisplayImage(1, ptrGrabResult);
#endif
        }
        else
        {
            cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription();
        }
    }



    //        // Print the model name of the camera->


    //        // The parameter MaxNumBuffer can be used to control the count of buffers
    //        // allocated for grabbing. The default value of this parameter is 10.
    //        camera->MaxNumBuffer = 5;

    //        // Start the grabbing of c_countOfImagesToGrab images.
    //        // The camera device is parameterized with a default configuration which
    //        // sets up free-running continuous acquisition.


    //        camera->StartGrabbing( 100);

    //        // This smart pointer will receive the grab result data.
    //        CGrabResultPtr ptrGrabResult;

    //        // camera->StopGrabbing() is called automatically by the RetrieveResult() method
    //        // when c_countOfImagesToGrab images have been retrieved.
    //        while ( camera->IsGrabbing())
    //        {
    //            // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
    //            camera->RetrieveResult( 5000, ptrGrabResult, TimeoutHandling_ThrowException);

    //            // Image grabbed successfully?
    //            if (ptrGrabResult->GrabSucceeded())
    //            {
    //                // Access the image data.
    //                cout << "SizeX: " << ptrGrabResult->GetWidth() << endl;
    //                cout << "SizeY: " << ptrGrabResult->GetHeight() << endl;
    //                const uint8_t *pImageBuffer = (uint8_t *) ptrGrabResult->GetBuffer();
    //                cout << "Gray value of first pixel: " << (uint32_t) pImageBuffer[0] << endl << endl;

    //                std_msgs::Int32 msg;
    //                msg.data = (uint32_t) pImageBuffer[0];
    //                pub->publish(msg);
    //                ros::spinOnce();

    //#ifdef PYLON_WIN_BUILD
    //                // Display the grabbed image.
    //                Pylon::DisplayImage(1, ptrGrabResult);
    //#endif
    //            }
    //            else
    //            {
    //                cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription();
    //            }
    //        }
    //    }
    //    catch (GenICam::GenericException &e)
    //    {
    //        // Error handling.
    //        cerr << "An exception occurred." << endl
    //             << e.GetDescription() << endl;
    //    }

    // Comment the following two lines to disable waiting on exit.
    // cerr << endl << "Press Enter to exit." << endl;
    // while( cin.get() != '\n');

    //return exitCode;
}
