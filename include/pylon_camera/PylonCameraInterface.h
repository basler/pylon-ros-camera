#ifndef PYLONCAMERAINTERFACE_H
#define PYLONCAMERAINTERFACE_H

#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <pylon/gige/BaslerGigEInstantCamera.h>
#include <sqlconnection/db_connection.h>


class PylonCameraInterface
{
public:
    PylonCameraInterface();

    ~PylonCameraInterface();

    bool openCamera();

    ros::Publisher pub_img;

    void close();

    bool sendNextImage();


private:

    //    Pylon::CBaslerGigECamera *camera;



    Pylon::CBaslerGigEInstantCamera *camera;
    Pylon::PylonAutoInitTerm autoInitTerm;
    Pylon::CGrabResultPtr ptrGrabResult;

    DB_connection db;

    std::ofstream off;

    cv::Mat img;
};


#endif // PYLONCAMERAINTERFACE_H
