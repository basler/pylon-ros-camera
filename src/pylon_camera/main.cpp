#include <ros/ros.h>
#include <boost/thread.hpp>

#include <pylon_camera/pylon_camera_node.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pylon_camera_node");

    pylon_camera::PylonCameraNode pylon_camera_node;
    pylon_camera_node.getInitialCameraParameter();

    if (!pylon_camera_node.init())
    {
        ROS_ERROR("Error while initializing the pylon node!");
        ros::shutdown();
    }

    ros::Rate r(pylon_camera_node.desiredFrameRate());

    ROS_INFO("Start image grabbing if node connects to topic with framerate: %.2f Hz",
             pylon_camera_node.desiredFrameRate());

    // Main thread and brightness-service thread
    boost::thread th(boost::bind(&ros::spin));

    while (ros::ok())
    {
    	pylon_camera_node.spin();
        // will now be called from the boost thread
        // ros::spinOnce();
//        r.expectedCycleTime().toSec()
        r.sleep();
    }
    ROS_INFO("Terminate pylon node");
    return 0;
}
