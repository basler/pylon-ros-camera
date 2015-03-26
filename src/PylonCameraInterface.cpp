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


/**
camera.ShutterMode.SetValue(ShutterMode_GlobalResetRelease);
ShutterModeEnums e = camera.ShutterMode.GetValue();


  */

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
    // variables.push_back(DB_Variable("SensorId",int()));
    variables.push_back(DB_Variable("data",QString()));

    if (!doesRowExist()){
        qDebug() << "NOT FOUND" << getReadCommand();
        return false;
    }

    if (!readFromDB())
        return false;

    cols = getVariable(std::string("cols"))->asInt();
    rows = getVariable("rows")->asInt();
    // dev_id = getVariable("SensorId")->asInt();

    // ROS_INFO("read: %s", getVariable("data")->asString().toAscii().constData());

    cv::FileStorage fs(getVariable("data")->asString().toAscii().constData(),cv::FileStorage::READ + cv::FileStorage::MEMORY);

    fs["distortion"] >> dist_coeffs;
    fs["cam_matrix"] >> cam_matrix;
    
    std::string comment = "";
    fs["comment"] >> comment;

    if (comment.size() > 0){
        ROS_INFO("Found comment for intrinsic calibration: %s", comment.c_str());
    }else{
        ROS_INFO("Intrinsic calibration has no comment");
    }

    assert(cam_matrix.cols == 3 && cam_matrix.rows == 3);
    assert(!dist_coeffs.empty());

    fs.release();

    return true;

}
#endif


PylonCameraInterface::PylonCameraInterface():
    cam_info()
  ,nh("~")
  ,exposure_once_running(false)
  ,exposure_as_(nh,"calib_exposure_action",boost::bind(&PylonCameraInterface::exposure_native_cb, this,_1), false)
  //  ,exposure_as_(nh,"calib_exposure_action",boost::bind(&PylonCameraInterface::exposure_cb, this,_1), false)
{
    exposure_as_.start();
    write_exp_to_db = false;

#ifdef WITH_QT_DB
    db = new DB_connection();
#endif

}

void PylonCameraInterface::exposure_native_cb(const ExposureServer::GoalHandle handle){


    current_exp_handle = handle;
    current_exp_handle.setAccepted();
    calibrating_exposure = false; // only used for manual exposure search
    goal_brightness = handle.getGoal()->goal_exposure;

    //    ROS_INFO("NATIVE EXP WITH GOAL %i",goal_brightness);

    //    if (!has_auto_exposure){
    //        ROS_WARN("Auto exposure requested for camera that does not natively support Auto Exposure, using seach method");
    //        startExposureSearch(handle);
    //        return;
    //    }




    exp_once_timeout = ros::Time::now() + ros::Duration(15.0); // 15 sec timeout

    try {
        if (is_usb){

            ROS_WARN("Native Exposure not yet implemented for USB, using old method");
            startExposureSearch(handle);
            return;

            //            camera_usb->ExposureAuto.SetValue(Basler_UsbCameraParams::ExposureAuto_Once);
            //            camera_usb->AutoTargetBrightness.SetValue(goal_brightness);


            //            camera_usb->ExposureAuto.SetValue(ExposureAuto_Off);
            //            ExposureAutoEnums e = camera.ExposureAuto.GetValue();

            // camera_usb->AutoTargetValue.SetValue(10);

        }else{

            if (goal_brightness < 50 || goal_brightness > 205){
                //                ROS_INFO("Goal out of range: %i [50,205], using own search method", goal_brightness);
                startExposureSearch(handle);
                return;
            }


            camera_gige->AutoTargetValue.SetValue(goal_brightness);
            camera_gige->ExposureAuto.SetValue(Basler_GigECamera::ExposureAuto_Once);

            if (camera_gige->ExposureAuto.GetValue() != Basler_GigECamera::ExposureAuto_Once){
                ROS_ERROR("Could not activate ExposureAuto_Once");
            }
        }

    }
    catch (GenICam::GenericException &e)
    {
        ROS_ERROR("An exception occurred: %s",e.GetDescription());
    }


    /// new exposure is only computed on the next images so we can't just wait here until ExposureAuto-Value is set to Off
    exposure_once_running = true;
    last_exp_once_exposure = -1;

}


void PylonCameraInterface::startExposureSearch(const ExposureServer::GoalHandle handle){
    calibrating_exposure = true;
    goal_brightness = handle.getGoal()->goal_exposure;
    calib_threshold = handle.getGoal()->accuracy;

    int a,b;
    nh.param<int>("min_search_exp",a, 50);
    nh.param<int>("max_search_exp",b, 100000);

    left_exp = a;
    right_exp = b;

    ROS_DEBUG("Starting search for goal %i with exposure limits %.0f %.0f", goal_brightness, left_exp, right_exp);

    max_exposure = right_exp;
    calib_exposure = (left_exp+right_exp)/2;
}

void PylonCameraInterface::exposure_cb(const ExposureServer::GoalHandle handle){
    current_exp_handle = handle;
    current_exp_handle.setAccepted();
    startExposureSearch(handle);
}



void PylonCameraInterface::close(){

    if (camera()){
        ROS_INFO("Closing camera");
        camera()->Close();
    }
}

PylonCameraInterface::~PylonCameraInterface(){
    close();
}

bool hasEnding (std::string const &fullString, std::string const &ending) 
{
    if (fullString.length() >= ending.length())
    {
        return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
    }
    else
    {
        return false;
    }
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

        // ROS_INFO("Number of devices: %i", int(lstDevices.size()));

        if (lstDevices.size() > 0 && camera_identifier != "x")
        {
            bool found = false;
            ROS_INFO("enumerating %ld cams, searching for '%s'", lstDevices.size(), camera_identifier.c_str());
            DeviceInfoList_t::const_iterator it;

            for ( it = lstDevices.begin(); it != lstDevices.end(); ++it )
            {
                ROS_INFO("cam: '%s'", it->GetFullName().c_str());
            }

            for ( it = lstDevices.begin(); it != lstDevices.end(); ++it )
            {
                // ROS_INFO("cam: '%s'", it->GetFullName().c_str());
                if (camera_identifier == it->GetFullName().c_str() || hasEnding(it->GetFullName().c_str(), camera_identifier))
                {
                    ROS_INFO("Found fitting name: %s", it->GetFullName().c_str());
                    Pylon::CInstantCamera *cam = new Pylon::CInstantCamera(CTlFactory::GetInstance().CreateDevice(*it));
                    is_usb = cam->IsUsb();
                    cam->Close();
                    delete cam;

                    if (is_usb)
                    {
                        camera_usb = new Pylon::CBaslerUsbInstantCamera(CTlFactory::GetInstance().CreateDevice(*it));
                    }else
                    {
                        
                        camera_gige = new Pylon::CBaslerGigEInstantCamera(CTlFactory::GetInstance().CreateDevice(*it));
                        ROS_INFO("Opened: %s", camera_gige->GetDeviceInfo().GetFullName().c_str());
                    }

                    if (found){
                        ROS_ERROR("Camera identifier is not unique!");
                        found = false;
                        break;
                    }

                    found = true;
//                    break;
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

            /// TODO: What happens if there is no camera?

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


        /// HACK
        has_auto_exposure = true;

        if (is_usb && camera_usb == NULL)
        {
            camera_usb = new Pylon::CBaslerUsbInstantCamera(CTlFactory::GetInstance().CreateFirstDevice());
            // has_auto_exposure = GenApi::IsAvailable(camera_usb->ExposureAuto);

            camera_usb->TriggerSelector.SetValue(Basler_UsbCameraParams::TriggerSelector_FrameStart);
            camera_usb->TriggerMode.SetValue(Basler_UsbCameraParams::TriggerMode_On);
            camera_usb->TriggerSource.SetValue(Basler_UsbCameraParams::TriggerSource_Software);


        }
        else if (camera_gige == NULL)
        {
            camera_gige = new Pylon::CBaslerGigEInstantCamera(CTlFactory::GetInstance().CreateFirstDevice());
            //            has_auto_exposure =  GenApi::IsAvailable(camera_gige->ExposureAuto);
            // has_auto_exposure =  GenApi::IsAvailable(Basler_GigECameraParams::expos);

            // Basler_UsbCameraParams::ExposureAuto_Off
        }


        //        if ( ! has_auto_exposure){
        //            ROS_INFO("Camera %s has NO auto exposure",camera()->GetDeviceInfo().GetModelName().c_str() );
        //        }else{
        //            ROS_INFO("WE have auto exposure");
        //        }




        cam_name = camera()->GetDeviceInfo().GetModelName();
        camera()->Open();

        if (camera()->IsOpen()){
            ROS_INFO("Camera %s was opened",cam_name.c_str());
        }else{
            ROS_WARN("Could not open camera!");
            return false;
        }

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

            cols = rows = -1;

            // fs["cols"] >> cols;
            // fs["rows"] >> rows;

            fs["width"] >> cols;
            fs["height"] >> rows;

            if (cols < 0){
                ROS_WARN("no 'width' entry in %s", intrinsic_file_path.c_str());
            }

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
    // sub_exp_calib = nh.subscribe("calib_exposure",1,&PylonCameraInterface::calib_exposure_cb,this);


    return true;
}


void PylonCameraInterface::set_exposure(int exposure_mu_s){

    if (exposure_mu_s > 0 ){
        if (exposure_mu_s != current_exposure || calibrating_exposure){

            current_exposure = exposure_mu_s;

            // ROS_INFO("Updating exposure to %i",exposure_mu_s);
            try {
                if (is_usb){
                    if (has_auto_exposure) camera_usb->ExposureAuto.SetValue(Basler_UsbCameraParams::ExposureAuto_Off);
                    camera_usb->ExposureTime.SetValue(exposure_mu_s);

                }else{
                    //ROS_INFO("SEtting exposure to %i", exposure_mu_s);
                    // if (has_auto_exposure)
                    camera_gige->ExposureAuto.SetValue(Basler_GigECamera::ExposureAuto_Off);
                    camera_gige->ExposureTimeAbs.SetValue(exposure_mu_s);
                }
            } catch (GenICam::OutOfRangeException e) {
                ROS_ERROR("Exposure value of %i is out of range: MSG: %s", exposure_mu_s,e.what());
            }    catch (GenICam::GenericException &e)
            {
                ROS_ERROR("Setting Exp: An exception occurred: %s",e.GetDescription());
            }

        }
    }else{

        if (has_auto_exposure){
            if (is_usb){
                camera_usb->ExposureAuto.SetValue(Basler_UsbCameraParams::ExposureAuto_Continuous);
            }else{
                camera_gige->ExposureAuto.SetValue(Basler_GigECamera::ExposureAuto_Continuous);
            }
        }else{
            // ROS_INFO("No Auto Exposure");
        }
    }
}


float getMeanInCenter(const cv::Mat& img){
    int c = img.cols, r = img.rows;
    return cv::mean(img.colRange(0.25*c, 0.75*c).rowRange(0.25*r,0.75*r)).val[0];
}



void PylonCameraInterface::handleEndOfNativeExposure(){

    //            ros::Time time_out = ros::Time::now() + ros::Duration(3.0);

    // Basler_GigECamera::ExposureAutoEnums e = camera_gige->ExposureAuto.GetValue();
    //            cout << "once " << Basler_GigECamera::ExposureAuto_Once << endl;
    //            cout << "off " << Basler_GigECamera::ExposureAuto_Off << endl;

    /// send current exposure as feedback
    current_exposure = camera_gige->ExposureTimeRaw.GetValue();
    exp_feedback.exposure_mu_s = current_exposure;
    current_exp_handle.publishFeedback(exp_feedback);






    /// camera goes from Once to Off after goal brightness was found

    bool succeeded = false;
    if (is_usb){
        ROS_ERROR("NOT IMPLEMENTED YET");
    }else{
        succeeded = (camera_gige->ExposureAuto.GetValue() == Basler_GigECamera::ExposureAuto_Off);
    }

    /// also stop if exposure has converged:
    if (last_exp_once_exposure > 0){
        float change = abs(current_exposure-last_exp_once_exposure)*100.0/current_exposure;
        if (change < 3){ /// change less than 3% of current value
            succeeded = true;
        }

        //        ROS_INFO("Last exp: %i, now: %i  %f", last_exp_once_exposure, current_exposure, change);
    }
    last_exp_once_exposure = current_exposure;



    if (succeeded){

        exposure_once_running = false;

        nh.setParam("pylon_exposure_mu_s",current_exposure);

        ExposureServer::Result res;
        res.success = true;
        res.exposure_mu_s = current_exposure;
        current_exp_handle.setSucceeded(res);

        // ROS_INFO("Native exp search ended");
    }



    if (ros::Time::now() > exp_once_timeout){
        exposure_once_running = false;
        nh.setParam("pylon_exposure_mu_s",current_exposure);

        ExposureServer::Result res;
        res.success = false;
        res.exposure_mu_s = current_exposure;
        current_exp_handle.setSucceeded(res);

        ROS_WARN("Native exp timed out");
    }


}





bool PylonCameraInterface::sendNextImage(){

    static bool send_original = false;
    static bool send_undist = false;
    static int img_counter = 0;

    if (img_counter++%10 == 0){
        send_original = pub_img.getNumSubscribers() > 0;
        send_undist = pub_img_undist.getNumSubscribers() > 0;
    }



    //    if (! (send_original || send_undist || calibrating_exposure || exposure_once_running )){
    //        cout << "not sending anything" << endl;
    //        return true;
    //    }


    /// a) exposure_once_running: camera sets new exposure
    /// b) calibrating_exposure: we are using binary search to get new exposure
    /// c) else: use fixed exposure (if changed)
    if (exposure_once_running){
        handleEndOfNativeExposure();
    }else{
        if (calibrating_exposure){
            set_exposure(calib_exposure);
        }else{
            int exposure_mu_s;
            nh.param<int>("pylon_exposure_mu_s", exposure_mu_s, -1);
            set_exposure(exposure_mu_s); // noop if no change in exposure
        }
    }


    // TODO: make sure that camera has finished reading last img (e.g. mutex)

    if (!camera()->IsGrabbing()){
        camera()->StartGrabbing(1);
    }else{
        ROS_WARN("CAMERA IS NOT GRABBING");
    }
    camera()->ExecuteSoftwareTrigger();


    orig_msg.header.stamp = ros::Time::now();
    undist_msg.header.stamp = ros::Time::now();

    // Wait for an image and then retrieve it.
    try{
        // camera->GrabOne(1000, ptrGrabResult,TimeoutHandling_ThrowException);
        camera()->RetrieveResult( 2000, ptrGrabResult, TimeoutHandling_ThrowException);
    }catch (GenICam::TimeoutException & e){
        cout << "timeout" << endl;
        return false;
    }

    bool calibration_running = exposure_once_running || calibrating_exposure;


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


        /// Do not send images while calibrating exposure:



        if (calibration_running){
            // ROS_INFO("NOT SENDING WHILE CALIBRATING EXPOSURE");
        }else{

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

        }
    } else {
        // http://www.baslerweb.com/media/documents/AW00100301000%20Pylon%20for%20Linux%20Readme_.pdf
        // -> timeout bei uebertragung
        // [/pylon  WARN 1427286521.560736516]: Pylon Camera: ERROR: -520093676  GX status 0xe1000014
        // ROS_WARN("Pylon Camera: GRAB[[]] ERROR: %i  %s", (int)ptrGrabResult->GetErrorCode(), ptrGrabResult->GetErrorDescription().c_str());
        return false;
    }


    // update exposure and binary search parameters:
    if (calibrating_exposure){
        float c_br = getMeanInCenter(orig_msg.image);

        // ROS_INFO("new brightness %f for exposure %f", c_br, calib_exposure);

#ifdef WITH_QT_DB
        if (write_exp_to_db){
            DB_connection con;
            char cmd[200];
            sprintf(cmd,"insert into crane_exposure (exposure, brightness) values (%i,%i)", int(calib_exposure),int(c_br));
            con.execCmd(cmd);
        }
#endif


        exp_feedback.exposure_mu_s = calib_exposure;
        current_exp_handle.publishFeedback(exp_feedback);


        // ROS_INFO("Brightness   current: %.0f, goal: %i", c_br, goal_brightness);
        if (fabs(c_br - goal_brightness) < calib_threshold ){
            //            ROS_INFO("Found new exposure as %f",calib_exposure);
            current_exposure = calib_exposure;
            calibrating_exposure = false;
            // ROS_INFO("Setting exp param to %i", current_exposure);
            nh.setParam("pylon_exposure_mu_s",current_exposure);

            // int new_max = min(50000,int(2*current_exposure));

            // nh.setParam("max_search_exp",int(2*current_exposure));

            ExposureServer::Result res;
            res.success = true;
            res.exposure_mu_s = current_exposure;
            res.no_images = false;
            current_exp_handle.setSucceeded(res);
        }

        /// search has converged and we fail
        if (fabs(right_exp-left_exp) < 20){
            ROS_WARN("Exposure search failed");
            ExposureServer::Result res;
            res.success = false;
            res.exposure_mu_s = current_exposure;
            res.no_images = false;
            current_exp_handle.setSucceeded(res);
            calibrating_exposure = false;
            //            // increase searchrange
            //            left_exp = 100;
            //            max_exposure = max_exposure; //std::min(915000,2*max_exposure);
            //            right_exp = max_exposure;
            //            nh.setParam("max_search_exp",int(right_exp));
            //            // ROS_WARN("Increasing maximal exposure to %i (was %i)",int(right_exp), max_exposure);
            //            calib_exposure = (left_exp+right_exp)/2;
        }


        if (c_br > goal_brightness){
            right_exp = calib_exposure;
        }else{
            left_exp = calib_exposure;
        }


        calib_exposure = (left_exp+right_exp)/2;


        //        ROS_INFO("New exp %f %f   ->   %f",left_exp,right_exp, calib_exposure);
    }



    return true;


}

