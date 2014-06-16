
#include "pylon_camera/PylonCameraInterface.h"

#include <ros/ros.h>
#include <iostream>

using namespace std;


PylonCameraInterface pylon_cam;

int main(int argc, char **argv) {

    ros::init(argc, argv, "pylon_node");
    ros::NodeHandle n;

    if (!pylon_cam.openCamera()){
        return 42;
    }

    int max_frame_rate = 30;


    ros::Rate r(max_frame_rate);
    while(ros::ok()){
        pylon_cam.sendNextImage();
        r.sleep();
    }

    // pylon interface is closed during deconstructor
    return 0;
}
