#ifndef PYLONCAMERAINTERFACE_H
#define PYLONCAMERAINTERFACE_H

#include <ros/ros.h>
#include <opencv2/core/core.hpp>


#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int32.h>
#include <pylon/PylonIncludes.h>
#include <pylon/InstantCamera.h>
#include <pylon/gige/BaslerGigEInstantCamera.h>
#include <pylon/usb/BaslerUsbInstantCamera.h>
#include <actionlib/server/action_server.h>

// Dirty Hack, Same action in maru_msgs and book_gripper_msgs
#ifdef WITH_QT_DB
	#include <sqlconnection/db_connection.h>
	#include <maru_msgs/calib_exposureAction.h>
	typedef actionlib::ActionServer<maru_msgs::calib_exposureAction> ExposureServer;
#else
	#include <book_gripper_msgs/calib_exposureAction.h>
	typedef actionlib::ActionServer<book_gripper_msgs::calib_exposureAction> ExposureServer;
#endif

class PylonCameraInterface
{
public:
    PylonCameraInterface();

    ~PylonCameraInterface();

    bool openCamera(const std::string &camera_identifier, const std::string &camera_frame, int camera_id, int exposure_mu_s = -1);

    ros::Publisher pub_img, pub_img_undist, pub_cam_info;

    void close();

    bool sendNextImage();

    ros::NodeHandle nh;
    std::string intrinsic_file_path;
    bool calibration_loaded;
    bool write_exp_to_db;

private:

    void handleEndOfNativeExposure();


    void startExposureSearch(const ExposureServer::GoalHandle handle);

    bool has_auto_exposure;

    std::string cam_name;

    ExposureServer exposure_as_;
    void exposure_cb(const ExposureServer::GoalHandle handle);

    void exposure_native_cb(const ExposureServer::GoalHandle handle);


    ExposureServer::GoalHandle current_exp_handle;
    ExposureServer::Feedback exp_feedback;
    int max_exposure;

    cv_bridge::CvImage orig_msg;
    cv_bridge::CvImage undist_msg;

    cv::Mat dist, camm;
    sensor_msgs::CameraInfo cam_info;

    Pylon::CBaslerGigEInstantCamera *camera_gige;
    Pylon::CBaslerUsbInstantCamera *camera_usb;
    Pylon::CInstantCamera *camera(){
        return (is_usb?(Pylon::CInstantCamera*)camera_usb:(Pylon::CInstantCamera*)camera_gige);
    }

    bool is_usb;

    void setupRectifyingMap();
    cv::Mat rect_map_x, rect_map_y;

    // ros::Subscriber sub_exp_calib;
    // void calib_exposure_cb(const std_msgs::Int32ConstPtr &msg);
    void set_exposure(int exposure_mu_s);

    Pylon::PylonAutoInitTerm autoInitTerm;
    Pylon::CGrabResultPtr ptrGrabResult;

#ifdef WITH_QT_DB
    DB_connection *db;
#endif


    int current_exposure;

    bool exposure_once_running;
    ros::Time exp_once_timeout; /// native exposure search has to finish before this timeout for the action to succeed
    int last_exp_once_exposure;

    /// calibration parameters
    bool calibrating_exposure;
    int goal_brightness;
    float calib_exposure;
    float left_exp;
    float right_exp;
    int calib_threshold;

};


#endif // PYLONCAMERAINTERFACE_H
