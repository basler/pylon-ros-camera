/*
 * main.cpp
 *
 * main function of the pylon_node
 *
 *  Created on: May 10, 2015
 *      Author: md
 */

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <GenApi/GenApi.h>

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

    if (!pylon_camera_node.init())
    {
        ROS_ERROR("Error while initializing the pylon node!");
        ros::shutdown();
    }

    ros::Rate r(pylon_camera_node.params_.desired_frame_rate_);

    ROS_INFO("Start image grabbing if node connects to topic with framerate: %.2f Hz",
             pylon_camera_node.params_.desired_frame_rate_);

    // Main thread and brightness-service thread
    boost::thread th(boost::bind(&ros::spin));

    while (ros::ok())
    {
        if (pylon_camera_node.getNumSubscribers() > 0 && ! pylon_camera_node.is_sleeping())
        {

#ifdef WITH_OPENCV
            if (pylon_camera_node.pylon_camera_->isOwnBrightnessFunctionRunning())
            {
                pylon_camera_node.pylon_camera_->setExtendedBrightness(pylon_camera_node.target_brightness_);
            }
            else
            {
#endif
                try {
                    pylon_camera_node.checkForPylonAutoFunctionRunning();
                } catch (GenICam::AccessException &e) 
                { }

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

                if(pylon_camera_node.getNumSubscribersHdr() > 0 && pylon_camera_node.params_.output_hdr_){
                    pylon_camera_node.img_hdr_pub_.publish(pylon_camera_node.cv_img_hdr_);
                }

                if (pylon_camera_node.getNumSubscribersRaw() > 0)
                {
                    // Publish via image_transport
                    pylon_camera_node.img_raw_pub_.publish(pylon_camera_node.img_raw_msg_,
                                                           pylon_camera_node.cam_info_msg_);
                }

                if (pylon_camera_node.getNumSubscribersRect() > 0 && pylon_camera_node.have_intrinsic_data())
                {
                    pylon_camera_node.img_rect_pub_.publish(pylon_camera_node.cv_img_rect_);
                }
            } else // single image acquisition
            {
                if(!pylon_camera_node.grabImage()){
                    continue;
                }

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
            if(pylon_camera_node.grabImage())
            {
                pylon_camera_node.img_raw_pub_.publish(pylon_camera_node.img_raw_msg_, pylon_camera_node.cam_info_msg_);
            }
#endif

        }
        // will now be called from the boost thread
        // ros::spinOnce();
//        r.expectedCycleTime().toSec()
        r.sleep();
    }
    ROS_INFO("Terminate pylon node");
    return 0;
}
