/* Copyright 2015 <Magazino GmbH>
 *
 * main.cpp
 *
 *      Author: debout@magazino.eu
 *              grimm@magazino.eu
 *              engelhard@magazino.eu
 */

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <pylon_camera/pylon_camera_node.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pylon_camera_node");

    pylon_camera::PylonCameraNode pylon_camera_node;

    ros::Rate r(pylon_camera_node.frameRate());

    ROS_INFO_STREAM("Start image grabbing if node connects to topic with "
            << " a framerate of : " << pylon_camera_node.frameRate() << " Hz");

    // Main thread and brightness-service thread
    boost::thread th(boost::bind(&ros::spin));

    while (ros::ok())
    {
        pylon_camera_node.spin();
        r.sleep();
    }

    ROS_INFO("Terminate PylonCameraNode");
    return EXIT_SUCCESS;
}
