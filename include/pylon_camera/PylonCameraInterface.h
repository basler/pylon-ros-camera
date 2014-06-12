#ifndef PYLONCAMERAINTERFACE_H
#define PYLONCAMERAINTERFACE_H

#include <ros/ros.h>
#include "std_msgs/Int32.h"

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

    void testRun();

    bool openCamera();

    ros::Publisher *pub;

private:
    Pylon::CBaslerGigEInstantCamera *camera;
    Pylon::PylonAutoInitTerm autoInitTerm;
};


#endif // PYLONCAMERAINTERFACE_H
