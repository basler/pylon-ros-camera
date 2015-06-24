/*
 * main.cpp
 *
 * main function of the pylon_node
 *
 *  Created on: May 10, 2015
 *      Author: md
 */

#include <ros/ros.h>

#ifdef WITH_OPENCV
    #include <pylon_camera/pylon_camera_opencv_node.h>
#else
    #include <pylon_camera/pylon_camera_node.h>
//    #include <pylon_camera/pylon_camera_parameter.h>
#endif

using namespace pylon_camera;

int main(int argc, char **argv)
{

    ros::init(argc, argv, "pylon_camera_node");

#ifdef WITH_OPENCV
    ROS_INFO("PylonCameraNode running WITH OpenCV");
    PylonCameraOpenCVNode pylon_camera_node;
#else
    ROS_INFO("PylonCameraNode running WITHOUT OpenCV");
    PylonCameraNode pylon_camera_node;
#endif

    //  Create either standard PylonInterface or PylonSequencerInterface (only when OpenCV available
    // TODO: Uli Review: Problem beim kapseln: Funktionen der Abgeleiteten Klasse rufen jeweils auch die Funktionen der Basisklasse auf
    // TODO: Reihenfolge wichtig!
    pylon_camera_node.getInitialCameraParameter();

    // Parameter: Sequencer or not
    pylon_camera_node.createPylonInterface();

    if (pylon_camera_node.init())
    {
        ROS_ERROR("Error while initializing the pylon node!");
    }
    pylon_camera_node.getRuntimeCameraParameter();

    ros::Rate r(pylon_camera_node.params_.desired_frame_rate_);

    if (pylon_camera_node.params_.use_trigger_service_)
    {
        ROS_INFO("Start image grabbing in service trigger mode with framerate: %.2f Hz",
                 pylon_camera_node.params_.desired_frame_rate_);
    }
    else
    {
        ROS_INFO("Start image grabbing if node connects to topic with framerate: %.2f Hz",
                 pylon_camera_node.params_.desired_frame_rate_);
    }

    int params_update_counter = 0;

    while (ros::ok())
    {
        if (pylon_camera_node.getNumSubscribers() > 0)
        {
            // Update all possible runtime parameter (exposure, brightness, etc) every param_update_frequency_ cycles
            params_update_counter++;

            if (params_update_counter % pylon_camera_node.params_.param_update_frequency_ == 0){
                pylon_camera_node.updateAquisitionSettings();
                params_update_counter = 0;
            }
            pylon_camera_node.grabbingCallback();

#ifdef WITH_OPENCV
            if (pylon_camera_node.params_.use_sequencer_)
            {
                if (pylon_camera_node.getNumSubscribersSeq() > 0)
                {
                    pylon_camera_node.img_seq_pub_->publish(pylon_camera_node.cv_img_seq_);
                    pylon_camera_node.exp_times_pub_->publish(pylon_camera_node.exp_times_);
                }
            }
            else
            {
                if (pylon_camera_node.getNumSubscribersRaw() > 0)
                {
                    // Publish via image_transport
                    pylon_camera_node.img_raw_pub_->publish(pylon_camera_node.img_raw_msg_,
                                                            pylon_camera_node.cam_info_msg_);
                }
                if (pylon_camera_node.getNumSubscribersRect() > 0)
                {
                    // Publish via normal publisher
                    pylon_camera_node.img_rect_pub_->publish(pylon_camera_node.cv_img_rect_);
                }
            }
#else
            pylon_camera_node.img_raw_pub_->publish(pylon_camera_node.img_raw_msg_, pylon_camera_node.cam_info_msg_);
#endif
        }
        ros::spinOnce();
        r.sleep();
    }

    pylon_camera_node.terminate();

    return 0;
}
