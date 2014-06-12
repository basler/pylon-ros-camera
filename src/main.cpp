
#include <iostream>

#include "pylon_camera/PylonCameraInterface.h"
#include <ros/ros.h>

using namespace std;


int main(int argc, char **argv) {


    ros::init(argc, argv, "pylon_node");
    ros::NodeHandle n;

    ros::Publisher pub_img = n.advertise<std_msgs::Int32>("pylon",100);

    PylonCameraInterface pylon_cam;

    if (!pylon_cam.openCamera())
        return 42;

    pylon_cam.pub = &pub_img;






    pylon_cam.testRun();
//    ros::Rate r(10);

    // ros::spin();


    return 0;


}
