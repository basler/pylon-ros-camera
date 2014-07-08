#include "pylon_camera/PylonCameraInterface.h"

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/Int32.h"
#include "sensor_msgs/Image.h"
#include <sqlconnection/retainvariables.h>

using namespace Pylon;
using namespace std;


PylonCameraInterface::PylonCameraInterface(){
    camera = NULL;
    db.connect("Maru1");
}

void PylonCameraInterface::close(){

    if (camera){
        cout << "Closing camera" << endl;
        camera->Close();
    }
}

PylonCameraInterface::~PylonCameraInterface(){
    close();
}


bool PylonCameraInterface::openCamera(){


    // The exit code of the sample application.
    int exitCode = 0;

    // Automagically call PylonInitialize and PylonTerminate to ensure the pylon runtime system
    // is initialized during the lifetime of this object.

    try{


        TlInfoList_t tl_list;
        CTlFactory::GetInstance().EnumerateTls(tl_list);

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

        camera->Open();

        if (camera->IsOpen()){
            cerr << "Camera " << camera->GetDeviceInfo().GetModelName() << " was opened" << endl;
        }else{
            cerr << "Could not open camera!";
            return false;
        }


    }
    catch (GenICam::GenericException &e)
    {
        // Error handling.
        cerr << "An exception occurred." << endl
             << e.GetDescription() << endl;
        exitCode = 1;
    }


    ros::NodeHandle n;
    pub_img = n.advertise<sensor_msgs::Image>("pylon",100);
    camera->StartGrabbing();
    return true;
}



bool PylonCameraInterface::sendNextImage(){


    fflush(stdout);

    // This smart pointer will receive the grab result data.

    // Camera.StopGrabbing() is called automatically by the RetrieveResult() method
    // when c_countOfImagesToGrab images have been retrieved.
    if ( camera->IsGrabbing())
    {
        // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
        camera->RetrieveResult( 5000, ptrGrabResult, TimeoutHandling_ThrowException);

        // Image grabbed successfully?
        if (ptrGrabResult->GrabSucceeded())
        {
            // Access the image data.
            int w = ptrGrabResult->GetWidth();
            int h = ptrGrabResult->GetHeight();

            // cout << "SizeX: " << ptrGrabResult->GetWidth() << endl;
            // cout << "SizeY: " << ptrGrabResult->GetHeight() << endl;
            // const uint8_t *pImageBuffer = (uint8_t *) ptrGrabResult->GetBuffer();
            // cout << "Gray value of first pixel: " << (uint32_t) pImageBuffer[0] << endl << endl;

            if (img.cols != w || img.rows != h || img.type() != CV_8UC1){
                cerr << "New image size: " << w << " " << h << endl;
                img = cv::Mat(h,w,CV_8UC1);
            }

            memcpy(img.data,(uint8_t *) ptrGrabResult->GetBuffer(),w*h*sizeof(uchar) );

            //  cout << "frame: " << ptrGrabResult->GetFrameNumber() << endl;
            //  cv::imwrite("/opt/tmp/gig.jpg",img);


            /// TODO: copy directly to out_msg

            cv_bridge::CvImage out_msg;
            out_msg.header.stamp   = ros::Time::now(); // ptrGrabResult->GetTimeStamp()
            out_msg.header.frame_id   = "pylon_frame";
            out_msg.encoding = sensor_msgs::image_encodings::MONO8;
            out_msg.image    = img;

            pub_img.publish(out_msg.toImageMsg());


            DbVarSet vset("machine_state",&db);
            vset.constraints.push_back(DB_Variable("id",0)); // only one row in this table
            vset.variables.push_back(DB_Variable("last_box_cam_time",int64_t(out_msg.header.stamp.toNSec())));
            if (!vset.writeToDB()){
                qDebug() << "Could not write";
            //ROS_ERROR("Ensenso: Could not write timestamp to machine_state-table");
            }else{
                // qDebug() << "Wrote to DB";
            }


//            cout << "Pylon time " << ptrGrabResult->GetTimeStamp() << endl;
//            cout << "ROS Time " << ros::Time::now() << endl;
//            fflush(stdout);

        } else {
            cout << "Pylon Camera: Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << endl;
        }

        return true;

    }

    return false;
}
