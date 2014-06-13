
#include <iostream>

#include <signal.h>
#include <stdlib.h>
#include <stdio.h>

#include <csignal>

#include "pylon_camera/PylonCameraInterface.h"
#include <ros/ros.h>

using namespace std;


PylonCameraInterface pylon_cam;

int main(int argc, char **argv) {

    ros::init(argc, argv, "pylon_node");
    ros::NodeHandle n;

    if (!pylon_cam.openCamera()){
        return 42;
    }

    ros::Rate r(100);
    while(ros::ok()){
        pylon_cam.sendNextImage();
        r.sleep();
    }

    // pylon interface is closed during Deconstructor
    return 0;
}
