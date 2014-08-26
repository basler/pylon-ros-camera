#include "pylon_camera/PylonCameraInterface.h"


#include <pylon/gige/PylonGigEIncludes.h>
#include <pylon/PylonIncludes.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sqlconnection/retainvariables.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>


using namespace Pylon;
using namespace std;
using namespace GenApi;
using namespace Basler_GigECameraParams;

struct CalibRetainSet : public DbVarSet {

    CalibRetainSet(){}

    CalibRetainSet(DB_connection *db){
        this->db_con = db;
    }

    bool readCalib(int cam_id, cv::Mat &dist_coeffs, cv::Mat &cam_matrix, int& cols, int& rows, int& dev_id);
};


bool CalibRetainSet::readCalib(int camId, cv::Mat &dist_coeffs, cv::Mat &cam_matrix, int& cols, int& rows, int& dev_id){
    this->clear();

    assert(db_con->isConnected());

    tableName = "IntrinsicCalibration";

    constraints.push_back(DB_Variable("CamID",camId));

    variables.push_back(DB_Variable("cols",int()));
    variables.push_back(DB_Variable("rows",int()));
    variables.push_back(DB_Variable("SensorId",int()));
    variables.push_back(DB_Variable("data",QString()));

    if (!doesRowExist()){
        qDebug() << "NOT FOUND" << getReadCommand();
        return false;
    }

    // qDebug() << "Reading calib";
    //qDebug() << "getRead new" << getReadCommand();

    if (!readFromDB())
        return false;

    cols = getVariable(std::string("cols"))->asInt();
    rows = getVariable("rows")->asInt();
    dev_id = getVariable("SensorId")->asInt();
    
    // ROS_INFO("read: %s", getVariable("data")->asString().toAscii().constData());

    cv::FileStorage fs(getVariable("data")->asString().toAscii().constData(),cv::FileStorage::READ + cv::FileStorage::MEMORY);

    fs["distortion"] >> dist_coeffs;
    fs["cam_matrix"] >> cam_matrix;

    assert(cam_matrix.cols == 3 && cam_matrix.rows == 3);
    assert(!dist_coeffs.empty());

    fs.release();

    //qDebug() << "got instrinc";

    return true;

}


PylonCameraInterface::PylonCameraInterface():
    cam_info()
{
    camera = NULL;
    db = new DB_connection();
}

void PylonCameraInterface::close(){

    if (camera){
        ROS_INFO("Closing camera");
        camera->Close();
    }
}

PylonCameraInterface::~PylonCameraInterface(){
    close();
}

bool PylonCameraInterface::openCamera(const std::string &camera_identifier, const std::string &camera_frame, int exposure_mu_s)
{
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
            ROS_ERROR("No transport layer, check environment variables!");
            return false;
        }

        if (lstDevices.size() == 0){
            ROS_ERROR("No Camera detected");
            return false;
        }

        if (lstDevices.size() > 0 && camera_identifier != "*")
        {
            bool found = false;
            ROS_INFO("enumerating %ld cams, searching for '%s'", lstDevices.size(), camera_identifier.c_str());
            DeviceInfoList_t::const_iterator it;
            for ( it = lstDevices.begin(); it != lstDevices.end(); ++it )
            {
                ROS_INFO("cam: '%s'", it->GetFullName().c_str());
                if (camera_identifier == it->GetFullName().c_str())
                {
                    camera = new Pylon::CBaslerGigEInstantCamera(CTlFactory::GetInstance().CreateFirstDevice(*it));
                    found = true;
                    break;
                }
            }
            if (!found)
            {
                ROS_ERROR("did not find the specified camera");
                return false;
            }

        }
        else
        {
            ROS_INFO("using first device");
            camera = new Pylon::CBaslerGigEInstantCamera(CTlFactory::GetInstance().CreateFirstDevice());
        }


        camera->Open();


        if (camera->IsOpen()){
            ROS_INFO("Camera %s was opened",camera->GetDeviceInfo().GetModelName().c_str());
        }else{
            ROS_WARN("Could not open camera!");
            return false;
        }

        camera->TriggerSelector.SetValue(TriggerSelector_FrameStart);
        camera->TriggerMode.SetValue(TriggerMode_On);
        camera->TriggerSource.SetValue(TriggerSource_Software);

        if (exposure_mu_s > 0){
            camera->ExposureAuto.SetValue(Basler_GigECamera::ExposureAuto_Off);
            camera->ExposureTimeAbs.SetValue(exposure_mu_s);
            ROS_INFO("Setting exposure to %i mu s",(int) camera->ExposureTimeAbs.GetValue());
        }else{
            camera->ExposureAuto.SetValue(Basler_GigECamera::ExposureAuto_Continuous);
            ROS_INFO("Using continuous exposure estimation");
        }


        // camera->AcquisitionFrameRateAbs.SetValue(30);

    }
    catch (GenICam::GenericException &e)
    {
        // Error handling.
        ROS_ERROR("An exception occurred: %s",e.GetDescription());
        exitCode = 1;
    }


    // get cam info from db
    CalibRetainSet crs; crs.db_con = db;

    int rows,cols, dev_id;
    if (!crs.readCalib(10, dist, camm,cols,rows, dev_id))
    {
        ROS_ERROR("Error reading calibration");
        return false;
    }
    
    cam_info.width = cols; cam_info.height = rows;
    cam_info.distortion_model = "plumb_bob";
    cam_info.D.resize(5);
    
    for (uint i=0; i<5; ++i)
    {
        double d = dist.at<double>(0,i);
        cam_info.D[i] = d;
    }

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

    cam_info.header.frame_id = camera_frame;


    ros::NodeHandle n;
    pub_img = n.advertise<sensor_msgs::Image>("/pylon/image_raw",100);
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



        DbVarSet vset("machine_state",db);
        vset.constraints.push_back(DB_Variable("id",0)); // only one row in this table
        vset.variables.push_back(DB_Variable("last_box_cam_time",int64_t(out_msg.header.stamp.toNSec())));
        if (!vset.writeToDB()){
            ROS_ERROR("Ensenso: Could not write timestamp to machine_state-table");
        }else{
            // qDebug() << "Wrote to DB";
        }


    } else {
        ROS_ERROR("Pylon Camera: Error: %i  %s", (int)ptrGrabResult->GetErrorCode(), ptrGrabResult->GetErrorDescription().c_str());
    }

    return true;


}
