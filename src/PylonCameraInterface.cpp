#include "pylon_camera/PylonCameraInterface.h"


#include <pylon/gige/PylonGigEIncludes.h>
#include <pylon/PylonIncludes.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sqlconnection/retainvariables.h>
#include <med_recog/ccam_stream.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

//#include <std_msgs/Int32.h>


using namespace Pylon;
using namespace std;
using namespace GenApi;
using namespace Basler_GigECameraParams;

PylonCameraInterface::PylonCameraInterface(){
    camera = NULL;
    db.connect("Maru1");
    off.open("/opt/tmp/cam_time.txt");
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

        camera->TriggerSelector.SetValue(TriggerSelector_FrameStart);
        camera->TriggerMode.SetValue(TriggerMode_On);
        camera->TriggerSource.SetValue(TriggerSource_Software);

        // qDebug() << "Activating continuous exposure";
        camera->ExposureAuto.SetValue(Basler_GigECamera::ExposureAuto_Continuous);

        // qDebug() << "Requesting 30 hz";
        // camera->AcquisitionFrameRateAbs.SetValue(30);

    }
    catch (GenICam::GenericException &e)
    {
        // Error handling.
        cerr << "An exception occurred." << endl
             << e.GetDescription() << endl;
        exitCode = 1;
    }


    // get cam info from db
    CalibRetainSet crs; crs.db_con = &db;

    ///
    int rows,cols, dev_id;
    assert(crs.readCalib(10, dist, camm,cols,rows, dev_id)); // TODO: parameter
    cam_info.width = cols; cam_info.height = rows;
    assert(dist.rows == 5);
    cam_info.distortion_model = "plumb_bob";
    for (uint i=0; i<5; ++i){  cam_info.D.push_back(dist.at<double>(0,i)); }

    int pos = 0;
    for (uint i=0; i<3; ++i){
        for (uint j=0; j<3; ++j){
            cam_info.K[pos++] = camm.at<double>(i,j);
        }
    }

    pos = 0;
    for (uint i=0; i<3; ++i){
        for (uint j=0; j<3; ++j){
            cam_info.P[pos++] = camm.at<double>(i,j);
        }
        cam_info.P[pos++] = 0;
    }

    cam_info.header.frame_id = "GripperCam";



    ros::NodeHandle n;
    pub_img = n.advertise<sensor_msgs::Image>("/pylon",100);
    pub_img_undist = n.advertise<sensor_msgs::Image>("/pylon/image_rect",100);
    pub_cam_info = n.advertise<sensor_msgs::CameraInfo>("/pylon/camera_info",100);

    camera->StartGrabbing();
    return true;
}



bool PylonCameraInterface::sendNextImage(){


    fflush(stdout);

    // This smart pointer will receive the grab result data.

    //if ( camera->IsGrabbing())

    // TODO: make sure that camera has finished reading last img (e.g. mutex)
    cv_bridge::CvImage out_msg;
    camera->ExecuteSoftwareTrigger();
    out_msg.header.stamp = ros::Time::now();

    // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
    try{
        camera->RetrieveResult( 5000, ptrGrabResult, TimeoutHandling_ThrowException);
    }catch (GenICam::TimeoutException & e){
        ROS_ERROR("Timeout in Pylon-Camera");
        return false;
    }

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


        //            ros::Duration capture_duration(0.25); // 30 fps
        //            ros::Time n = ros::Time::now();
        //            qDebug() << "ros::now " << n.toNSec();
        //            off << n.toNSec() << " " << ptrGrabResult->GetTimeStamp() << endl;
        //            double d = camera->AcquisitionFrameRateAbs.GetValue();
        //            qDebug() << "Framerate"  << d;

        //            double read = camera->ReadoutTimeAbs.GetValue();
        //            qDebug() << "read"  << read/1000.0;

        //            double exposure = camera->ExposureTimeAbs.GetValue();
        //            qDebug() << "exposure"  << exposure;


        // send distorted image
        out_msg.header.frame_id = cam_info.header.frame_id;
        out_msg.encoding = sensor_msgs::image_encodings::MONO8;
        out_msg.image    = img;
        pub_img.publish(out_msg.toImageMsg());


        // send undistorted image
        cv::Mat undistorted;
        cv::undistort(img,undistorted,camm,dist);
        out_msg.image    = undistorted;
        pub_img_undist.publish(out_msg.toImageMsg());



        // send cam_info with current stamp
        cam_info.header.stamp = out_msg.header.stamp;
        pub_cam_info.publish(cam_info);



        DbVarSet vset("machine_state",&db);
        vset.constraints.push_back(DB_Variable("id",0)); // only one row in this table
        vset.variables.push_back(DB_Variable("last_box_cam_time",int64_t(out_msg.header.stamp.toNSec())));
        if (!vset.writeToDB()){
            qDebug() << "Could not write";
            //ROS_ERROR("Ensenso: Could not write timestamp to machine_state-table");
        }else{
            // qDebug() << "Wrote to DB";
        }


    } else {
        cout << "Pylon Camera: Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << endl;
    }

    return true;


}
