#ifndef PYLONCAMERAINTERFACE_H
#define PYLONCAMERAINTERFACE_H

#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <pylon/gige/BaslerGigEInstantCamera.h>
#include <sqlconnection/db_connection.h>
#include <sensor_msgs/CameraInfo.h>

class PylonCameraInterface
{
public:
    PylonCameraInterface();

    ~PylonCameraInterface();

    bool openCamera(const std::string &camera_identifier, const std::string &camera_frame, int exposure_mu_s = -1);

    ros::Publisher pub_img, pub_img_undist, pub_cam_info;

    void close();

    bool sendNextImage();


private:

    cv::Mat dist, camm;
    sensor_msgs::CameraInfo cam_info;


    Pylon::CBaslerGigEInstantCamera *camera;
    Pylon::PylonAutoInitTerm autoInitTerm;
    Pylon::CGrabResultPtr ptrGrabResult;

    DB_connection *db;

    cv::Mat img;
};


#endif // PYLONCAMERAINTERFACE_H
