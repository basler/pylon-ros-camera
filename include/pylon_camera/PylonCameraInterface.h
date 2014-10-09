#ifndef PYLONCAMERAINTERFACE_H
#define PYLONCAMERAINTERFACE_H

#include <ros/ros.h>
#include <opencv2/core/core.hpp>

#include <pylon/PylonIncludes.h>
#include <pylon/InstantCamera.h>

//#include <pylon/gige/BaslerGigEInstantCamera.h>
//#include <sqlconnection/db_connection.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>

class PylonCameraInterface
{
public:
    PylonCameraInterface();

    ~PylonCameraInterface();

    bool openCamera(const std::string &camera_identifier, const std::string &camera_frame, int camera_id, int exposure_mu_s = -1);

    ros::Publisher pub_img, pub_img_undist, pub_cam_info;

    void close();

    bool sendNextImage();

    ros::NodeHandle *nh;
private:


    cv_bridge::CvImage orig_msg;
    cv_bridge::CvImage undist_msg;


    cv::Mat dist, camm;
    sensor_msgs::CameraInfo cam_info;

    Pylon::CInstantCamera *camera;
    //   Pylon::CBaslerGigEInstantCamera *camera_gige;
   // Pylon::CBaslerUsbInstantCamera *camera_usb;


    Pylon::PylonAutoInitTerm autoInitTerm;
    Pylon::CGrabResultPtr ptrGrabResult;

//    DB_connection *db;
    
};


#endif // PYLONCAMERAINTERFACE_H
