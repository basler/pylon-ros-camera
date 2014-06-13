#ifndef PYLONCAMERAINTERFACE_H
#define PYLONCAMERAINTERFACE_H

#include <ros/ros.h>

#include <opencv2/core/core.hpp>


#include <pylon/PylonIncludes.h>
#ifdef PYLON_WIN_BUILD
#    include <pylon/PylonGUI.h>
#endif

#include <pylon/gige/BaslerGigEInstantCamera.h>
#include <pylon/gige/PylonGigEIncludes.h>


class PylonCameraInterface
{
public:
    PylonCameraInterface();

    ~PylonCameraInterface();

    void startGrabLoop();

    bool openCamera();

    ros::Publisher pub_img;


    void close();

    bool sendNextImage();


private:
    Pylon::CBaslerGigEInstantCamera *camera;
    Pylon::PylonAutoInitTerm autoInitTerm;
    Pylon::CGrabResultPtr ptrGrabResult;

    cv::Mat img;
};


#endif // PYLONCAMERAINTERFACE_H
