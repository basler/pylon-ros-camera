
#include "pylon_camera/PylonCameraInterface.h"

#include <ros/ros.h>
#include <iostream>
#include <tf/transform_listener.h>


using namespace std;

int main(int argc, char **argv) {



    ros::init(argc, argv, "pylon_node");
    ros::NodeHandle n;
    std::string  camera_identifier, camera_frame;
        
    std::string check_frame;
	  
    n.param<std::string>("camera_identifier", camera_identifier, "*");
    n.param<std::string>("camera_frame", camera_frame, "reference_camera_base");
    n.param<std::string>("check_frame", check_frame, "insertion_y_visual");
    
          
    tf::TransformListener listener(n,ros::Duration(2.0));
	
    PylonCameraInterface pylon_cam;

    if (!pylon_cam.openCamera(camera_identifier, camera_frame)){
        return 42;
    }

    int max_frame_rate = 30;
    ros::Rate r(max_frame_rate);

   while(ros::ok()){
        pylon_cam.sendNextImage();
        ros::spinOnce();

    /// print warning if given camera frame is not in the tf-tree
    if ( ! listener.canTransform(camera_frame,check_frame,ros::Time::now()-ros::Duration(0.5)) ) {
        ROS_INFO("No trafo between %s and %s, is static_publisher running?",camera_frame.c_str(),check_frame.c_str());
    }

    /// does not work since listener does not forget frames...
    //         if ((listener.allFramesAsString().find(camera_frame) == std::string::npos)){

     r.sleep();
      
   }

    // pylon interface is closed in its own deconstructor
    return 0;
}



