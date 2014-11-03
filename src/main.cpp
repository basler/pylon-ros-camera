
#include "pylon_camera/PylonCameraInterface.h"

#include <ros/ros.h>
#include <iostream>
//#include <tf/transform_listener.h>


using namespace std;

int main(int argc, char **argv) {



    ros::init(argc, argv, "pylon_node");
    ros::NodeHandle n("~");
    std::string  camera_identifier, camera_frame;

    std::string check_frame;
    int exposure_mu_s;

    PylonCameraInterface pylon_cam;


    int camID;

    pylon_cam.nh.param<std::string>("camera_identifier", camera_identifier,"Basler acA1300-60gm#00305316D41B#192.168.112.4:3956");//"x""2676:ba02:4:4");
//    pylon_cam.nh->param<std::string>("camera_identifier", camera_identifier, "Basler acA1300-60gm#00305316D41B#192.168.112.4:3956");
//      camera_identifier = "2676:ba02:4:4";
//    camera_identifier = "Basler acA1300-60gm#00305316D41B#192.168.112.4:3956";
//    camera_identifier = "*";
    n.param<std::string>("camera_frame", camera_frame, "reference_camera_base");
    n.param<std::string>("check_frame", check_frame, "insertion_y_visual");
    n.param<int>("pylon_exposure_mu_s", exposure_mu_s, 2000);
    n.param<int>("intrinsic_cam_id", camID, -1); // 22

    n.param<std::string>("intrinsic_param_file_path",pylon_cam.intrinsic_file_path,"/opt/MaruData/calibs/usb_foo.yaml");


    ROS_INFO("Opening camera on frame %s with id %i and exposure %i", camera_frame.c_str(), camID, exposure_mu_s);

    if (!pylon_cam.openCamera(camera_identifier, camera_frame, camID, exposure_mu_s)){
        return 42;
    }

    int max_frame_rate = 30;
    ros::Rate r(max_frame_rate);

    while(ros::ok()){
        pylon_cam.sendNextImage();
        ros::spinOnce();

        // ROS_INFO("SLEEP");
        /// print warning if given camera frame is not in the tf-tree
        //    if ( ! listener.canTransform(camera_frame,check_frame,ros::Time::now()-ros::Duration(0.5)) ) {
        //        ROS_INFO("No trafo between %s and %s, is static_publisher running?",camera_frame.c_str(),check_frame.c_str());
        //    }

        /// does not work since listener does not forget frames...
        //         if ((listener.allFramesAsString().find(camera_frame) == std::string::npos)){

        r.sleep();
    }

    // pylon interface is closed in its own deconstructor
    return 0;
}



