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

    if (!pylon_camera_node.init())
    {
        ROS_ERROR("Error while initializing the pylon node!");
        ros::shutdown();
    }
    pylon_camera_node.getRuntimeCameraParameter();

    ros::Rate r(pylon_camera_node.params_.desired_frame_rate_);

    ROS_INFO("Start image grabbing if node connects to topic with framerate: %.2f Hz",
                 pylon_camera_node.params_.desired_frame_rate_);

    while (ros::ok())
    {
        if (pylon_camera_node.getNumSubscribers() > 0 && !pylon_camera_node.is_sleeping())
        {

#ifdef WITH_OPENCV
            if (pylon_camera_node.pylon_opencv_interface_.exposure_search_running_)
            {
                if (pylon_camera_node.pylon_opencv_interface_.setExtendedBrightness(pylon_camera_node.params_
                                .brightness_))
                {
                    pylon_camera_node.updateROSBirghtnessParameter();
                }

            }
            else
            {
#endif
                // Update all possible runtime parameter (exposure, brightness, etc) every param_update_frequency_ cycles
                pylon_camera_node.params_update_counter_++;
                if (pylon_camera_node.params_update_counter_ % pylon_camera_node.params_.param_update_frequency_ == 0)
                {
                    pylon_camera_node.updateAquisitionSettings();
                    pylon_camera_node.params_update_counter_ = 0;
                    if (pylon_camera_node.brightness_service_running_)
                    {
                        pylon_camera_node.brightness_service_running_ = false;
                    }
                 }

#ifdef WITH_OPENCV
            }
#endif

#ifdef WITH_OPENCV
            if (pylon_camera_node.params_.use_sequencer_)
            {
                pylon_camera_node.grabSequence();

                if (pylon_camera_node.getNumSubscribersSeq() > 0)
                {
                    pylon_camera_node.img_seq_pub_.publish(pylon_camera_node.cv_img_seq_);
                    pylon_camera_node.exp_times_pub_.publish(pylon_camera_node.exp_times_);
                }
            } else // single image acquisition
            {
                pylon_camera_node.grabImage();

                if (pylon_camera_node.getNumSubscribersRaw() > 0)
                {
                    // Publish via image_transport
                    pylon_camera_node.img_raw_pub_.publish(pylon_camera_node.img_raw_msg_,
                                                           pylon_camera_node.cam_info_msg_);
                }
                if (pylon_camera_node.getNumSubscribersRect() > 0 && pylon_camera_node.have_intrinsic_data())
                {
                    // Publish via normal publisher
                    pylon_camera_node.img_rect_pub_.publish(pylon_camera_node.cv_img_rect_);
                }
            }
#else
            if(pylon_camera_node.grabImage()){
                pylon_camera_node.img_raw_pub_.publish(pylon_camera_node.img_raw_msg_, pylon_camera_node.cam_info_msg_);
            }
#endif

        }
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
