#include "pylon_camera/PylonCameraInterface.h"


#include <pylon/PylonIncludes.h>
#include <opencv2/highgui/highgui.hpp>


#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <opencv/cv.h>

using namespace Pylon;
using namespace cv;
using namespace std;
using namespace GenApi;

#ifdef WITH_QT_DB


#include <sqlconnection/retainvariables.h>

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

    return true;

}
#endif


PylonCameraInterface::PylonCameraInterface():
    cam_info()
   ,nh("~")
   ,exposure_as_(nh,"calib_exposure_action",boost::bind(&PylonCameraInterface::exposure_cb, this,_1), false)
{

  //  exposure_as_.registerGoalCallback();
    exposure_as_.start();
    ROS_INFO("Starting exposure service");

#ifdef WITH_QT_DB
    db = new DB_connection();
#endif

}

void PylonCameraInterface::exposure_cb(const ExposureServer::GoalHandle handle){
    ROS_INFO("Got Exposre callback");
}

void PylonCameraInterface::close(){

    //    if (camera()){
    //        ROS_INFO("Closing camera");
    //        camera()->Close();
    //    }
}

PylonCameraInterface::~PylonCameraInterface(){
    close();
}

bool PylonCameraInterface::openCamera(const std::string &camera_identifier, const std::string &camera_frame,int camera_id, int exposure_mu_s)
{

    // Automagically call PylonInitialize and PylonTerminate to ensure the pylon runtime system
    // is initialized during the lifetime of this object.

    current_exposure = exposure_mu_s;
    calibration_loaded = false;
    calibrating_exposure = false;


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

        ROS_INFO("Number of devices: %i", int(lstDevices.size()));

        if (lstDevices.size() > 0 && camera_identifier != "x")
        {
            bool found = false;
            ROS_INFO("enumerating %ld cams, searching for '%s'", lstDevices.size(), camera_identifier.c_str());
            DeviceInfoList_t::const_iterator it;
            for ( it = lstDevices.begin(); it != lstDevices.end(); ++it )
            {
                ROS_INFO("cam: '%s'", it->GetFullName().c_str());
                if (camera_identifier == it->GetFullName().c_str())
                {

                    Pylon::CInstantCamera *cam = new Pylon::CInstantCamera(CTlFactory::GetInstance().CreateFirstDevice(*it));
                    is_usb = cam->IsUsb();
                    cam->Close();
                    delete cam;

                    if (is_usb){
                        camera_usb = new Pylon::CBaslerUsbInstantCamera(CTlFactory::GetInstance().CreateFirstDevice(*it));
                    }else{
                        camera_gige = new Pylon::CBaslerGigEInstantCamera(CTlFactory::GetInstance().CreateFirstDevice(*it));
                    }

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
            ROS_INFO("Opening first device");
            Pylon::CInstantCamera *cam = new Pylon::CInstantCamera(CTlFactory::GetInstance().CreateFirstDevice());

            is_usb = cam->IsUsb();
            cam->Close();
            delete cam;

            ROS_INFO("reopening device");
            if (is_usb){
                camera_usb = new Pylon::CBaslerUsbInstantCamera(CTlFactory::GetInstance().CreateFirstDevice());
            }else{
                camera_gige = new Pylon::CBaslerGigEInstantCamera(CTlFactory::GetInstance().CreateFirstDevice());
            }
        }


        camera()->Open();


        if (camera()->IsOpen()){
            ROS_INFO("Camera %s was opened",camera()->GetDeviceInfo().GetModelName().c_str());
        }else{
            ROS_WARN("Could not open camera!");
            return false;
        }



        /// starts a grab thread!
        //        camera->StartGrabbing();



    }
    catch (GenICam::GenericException &e)
    {
        // Error handling.
        ROS_ERROR("An exception occurred: %s",e.GetDescription());
        return false;
    }

    camera()->RegisterConfiguration( new CSoftwareTriggerConfiguration, RegistrationMode_ReplaceAll, Cleanup_Delete);




    int rows,cols;

#ifdef WITH_QT_DB
    if (camera_id > 0){

        // get cam info from db
        CalibRetainSet crs; crs.db_con = db;

        int dev_id;
        calibration_loaded = crs.readCalib(camera_id, dist, camm,cols,rows, dev_id);
        if (!calibration_loaded)
        {
            ROS_ERROR("Could not read calibration for cam_id %i from db", camera_id);
            // return false;
        }
    }
#endif

    /// try to read from file:
    if (!calibration_loaded){
        cv::FileStorage fs(intrinsic_file_path,cv::FileStorage::READ);

        if (!fs.isOpened()){
            ROS_ERROR("Could not load calib from file %s", intrinsic_file_path.c_str());
        }else{
            fs["distortion"] >> dist;
            fs["cam_matrix"] >> camm;
            fs["cols"] >> cols;
            fs["rows"] >> rows;

            calibration_loaded = true;
            ROS_INFO("Calibration was read from %s",intrinsic_file_path.c_str());
            fs.release();
        }
    }



    cam_info.width = cols; cam_info.height = rows;
    cam_info.distortion_model = "plumb_bob";
    cam_info.D.resize(5);

    if (calibration_loaded){


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
    }else{
        ROS_WARN("RUNNING PYLON NODE WITHOUT INTRINSIC CALIBRATION!");
    }


    cam_info.header.frame_id = camera_frame;

    orig_msg.header.frame_id = cam_info.header.frame_id;
    orig_msg.encoding = sensor_msgs::image_encodings::MONO8;

    undist_msg.header.frame_id = cam_info.header.frame_id;
    undist_msg.encoding = sensor_msgs::image_encodings::MONO8;


    pub_img = nh.advertise<sensor_msgs::Image>("image_raw",100);
    pub_img_undist = nh.advertise<sensor_msgs::Image>("image_rect",100);
    pub_cam_info = nh.advertise<sensor_msgs::CameraInfo>("camera_info",100);
    sub_exp_calib = nh.subscribe("calib_exposure",1,&PylonCameraInterface::calib_exposure_cb,this);




    return true;
}


void PylonCameraInterface::set_exposure(int exposure_mu_s){
    if (exposure_mu_s > 0 ){
        if (exposure_mu_s != current_exposure || calibrating_exposure){

            current_exposure = exposure_mu_s;


            ROS_INFO("Updating exposure to %i",exposure_mu_s);
            try {
                if (is_usb){
                    camera_usb->ExposureAuto.SetValue(Basler_UsbCameraParams::ExposureAuto_Off);
                    camera_usb->ExposureTime.SetValue(exposure_mu_s);
                }else{
                    camera_gige->ExposureAuto.SetValue(Basler_GigECamera::ExposureAuto_Off);
                    camera_gige->ExposureTimeAbs.SetValue(exposure_mu_s);
                }
            } catch (GenICam::OutOfRangeException e) {
                ROS_ERROR("Exposure value of %i is out of range: MSG: %s", exposure_mu_s,e.what());
            }
        }
    }else{
        if (is_usb){
            camera_usb->ExposureAuto.SetValue(Basler_UsbCameraParams::ExposureAuto_Continuous);
        }else{
            camera_gige->ExposureAuto.SetValue(Basler_GigECamera::ExposureAuto_Continuous);
        }
    }
}

void PylonCameraInterface::calib_exposure_cb(const std_msgs::Int32ConstPtr &msg){


    calibrating_exposure = true;
    goal_brightness = msg->data;


    ROS_INFO("Calibrating exposure with goal %i", goal_brightness);

    if (goal_brightness < 0 || goal_brightness > 255){
        qDebug() << "Invalid goal brightness: " << goal_brightness;
        return;
    }

    calib_threshold = 5;
    left_exp = 100;
    right_exp = 320000;
    calib_exposure = (left_exp+right_exp)/2;
}


float getMeanInCenter(const cv::Mat& img){
    int c = img.cols, r = img.rows;
    return cv::mean(img.colRange(0.25*c, 0.75*c).rowRange(0.25*r,0.75*r)).val[0];
}

bool PylonCameraInterface::sendNextImage(){


    bool send_original = pub_img.getNumSubscribers() > 0;
    bool send_undist = pub_img_undist.getNumSubscribers() > 0;

    if (! (send_original || send_undist || calibrating_exposure)){
        return true;
    }


    fflush(stdout);


    if (calibrating_exposure){
        set_exposure(calib_exposure);
    }else{
        int exposure_mu_s;
        nh.param<int>("pylon_exposure_mu_s", exposure_mu_s, -1);
        set_exposure(exposure_mu_s); // noop if no change in exposure
    }


    // TODO: make sure that camera has finished reading last img (e.g. mutex)

    camera()->StartGrabbing(1);
    camera()->ExecuteSoftwareTrigger();


    orig_msg.header.stamp = ros::Time::now();
    undist_msg.header.stamp = ros::Time::now();

    // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
    try{
        // camera->GrabOne(1000, ptrGrabResult,TimeoutHandling_ThrowException);
        camera()->RetrieveResult( 5000, ptrGrabResult, TimeoutHandling_ThrowException);
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

        if (orig_msg.image.cols != w || orig_msg.image.rows != h || orig_msg.image.type() != CV_8UC1){
            orig_msg.image = cv::Mat(h,w,CV_8UC1);
        }

        memcpy(orig_msg.image.data,(uint8_t *) ptrGrabResult->GetBuffer(),w*h*sizeof(uchar) );

        // send distorted image
        if (send_original){
            pub_img.publish(orig_msg.toImageMsg());
        }


        if ( send_undist ){

            if (!calibration_loaded){
                ROS_ERROR("Undistored image was requested, but no intrinsic calibration was loaded");
            }else{

                //            ros::Time s = ros::Time::now();

                // send undistorted image
                cv::undistort(orig_msg.image,undist_msg.image,camm,dist);
                pub_img_undist.publish(undist_msg.toImageMsg());


                //            Mat map1,map2;
                //            Mat newCamMatrix;
                //            cv::initUndistortRectifyMap(camm,dist,Mat(),newCamMatrix, cv::Size(img.cols,img.rows), CV_32FC1, map1,map2);

                //            cout << "old matrix" << endl << camm << endl;
                //            cout << "new matrix" << endl << newCamMatrix << endl;


                //            Mat undis2;

                //            cv::remap(img,undis2,map1,map2,cv::INTER_CUBIC);

                ////            cv::imwrite("/opt/tmp/undis1.png", undistorted);
                ////            cv::imwrite("/opt/tmp/undis2.png", undis2);


                //            ros::Duration d = ros::Time::now()-s;
                //            ROS_INFO("undis %f ms",d.toNSec()/1000.0/1000.0);
            }
        }

        // send cam_info with current stamp
        cam_info.header.stamp = orig_msg.header.stamp;
        pub_cam_info.publish(cam_info);


    } else {
        return false;
        // ROS_WARN("Pylon Camera: ERROR: %i  %s", (int)ptrGrabResult->GetErrorCode(), ptrGrabResult->GetErrorDescription().c_str());
    }


    // update exposure and binary search parameters:
    if (calibrating_exposure){
        float c_br = getMeanInCenter(orig_msg.image);

        // ROS_INFO("new brightness %f for exposure %f", c_br, calib_exposure);


        if (abs(c_br - goal_brightness) < calib_threshold ){
            ROS_INFO("Found new exposure as %f",calib_exposure);
            current_exposure = calib_exposure;
            calibrating_exposure = false;
            ROS_INFO("Setting exp param to %i", current_exposure);
            nh.setParam("pylon_exposure_mu_s",current_exposure);
        }

        if (c_br > goal_brightness){
            right_exp = calib_exposure;
        }else{
            left_exp = calib_exposure;
        }


        calib_exposure = (left_exp+right_exp)/2;
    }



    return true;


}
