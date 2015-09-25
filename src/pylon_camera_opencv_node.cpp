/*
 * pylon_camera_opencv_node.cpp
 *
 *  Created on: Jun 10, 2015
 *      Author: md
 */

#include <pylon_camera/pylon_camera_opencv_node.h>

namespace pylon_camera {

PylonCameraOpenCVNode::PylonCameraOpenCVNode() :
                    pylon_opencv_interface_(),
                    cv_img_rect_(),
                    cv_img_seq_(),
                    cv_img_hdr_(),
                    exp_times_(),
                    img_rect_pub_(nh_.advertise<sensor_msgs::Image>("image_rect", 10)),
                    img_seq_pub_(nh_.advertise<sensor_msgs::Image>("image_seq", 10)),
                    img_hdr_pub_(nh_.advertise<sensor_msgs::Image>("image_hdr", 10)),
                    exp_times_pub_(nh_.advertise<pylon_camera_msgs::SequenceExposureTimes>("seq_exp_times", 10)),
                    img_rectifier_(),
                    calib_loader_(),
                    img_raw_()
{
    image_sequence_.resize(3);
}

void PylonCameraOpenCVNode::getInitialCameraParameter()
{
    PylonCameraNode::getInitialCameraParameter();

    nh_.param<bool>("use_sequencer", params_.use_sequencer_, false);
    if (params_.use_sequencer_)
    {
        nh_.getParam("desired_seq_exp_times", params_.desired_seq_exp_times_);
        assert(params_.desired_seq_exp_times_.size() == 3);

#if CV_MAJOR_VERSION > 2   // If you are using OpenCV 3
       nh_.param<bool>("output_hdr_img", params_.output_hdr_, false);
#endif
    }

    nh_.param<std::string>("intrinsic_yaml_string",
                           params_.intrinsic_yaml_file_,
                           "INVALID_YAML_FILE");

    if (params_.intrinsic_yaml_file_ == "INVALID_YAML_FILE")
    {
        ROS_WARN("Yaml file string needed for rectification! Param: 'pylon_camera_node/intrinsic_yaml_string' has entry: %s",
                 params_.intrinsic_yaml_file_.c_str());
        ROS_WARN("Alternative: Get only distorted image by compiling 'WITHOUT_OPENCV'");
        ROS_INFO("Will only provide image_raw!");
        params_.have_intrinsic_data_ = false;
        if(params_.use_sequencer_){
            ROS_WARN("Sequencer-Mode needs intrinsic data! Will disable sequencer!");
            params_.use_sequencer_ =  false;
            params_.output_hdr_ = false;
        }
    }
}

void PylonCameraOpenCVNode::createPylonInterface()
{
    pylon_interface_ = &pylon_opencv_interface_;
    pylon_interface_->is_opencv_interface_ = true;
    ROS_INFO("Created PylonOpenCVInterface");
}

uint32_t PylonCameraOpenCVNode::getNumSubscribers() const
{
    return img_raw_pub_.getNumSubscribers() + img_rect_pub_.getNumSubscribers()
           + img_seq_pub_.getNumSubscribers()
           + img_hdr_pub_.getNumSubscribers();
}

uint32_t PylonCameraOpenCVNode::getNumSubscribersRaw() const
{
    return img_raw_pub_.getNumSubscribers();
}

uint32_t PylonCameraOpenCVNode::getNumSubscribersRect() const
{
    return img_rect_pub_.getNumSubscribers();
}
uint32_t PylonCameraOpenCVNode::getNumSubscribersSeq() const
{
    return img_seq_pub_.getNumSubscribers();
}
uint32_t PylonCameraOpenCVNode::getNumSubscribersHdr() const
{
    return img_hdr_pub_.getNumSubscribers();
}
bool PylonCameraOpenCVNode::init()
{

    if (!PylonCameraNode::initAndRegister())
    {
        return false;
    }

    if (params_.use_sequencer_)
    {
        pylon_opencv_interface_.setupSequencer(params_);
    }

    if (!PylonCameraNode::startGrabbing())
    {
        return false;
    }

    cv_img_rect_.header = img_raw_msg_.header;
    cv_img_seq_.header = img_raw_msg_.header;
    cv_img_hdr_.header = img_raw_msg_.header;
    exp_times_.header = img_raw_msg_.header;
    // Encoding of pixels -- channel meaning, ordering, size
    // taken from the list of strings in include/sensor_msgs/image_encodings.h
    cv_img_rect_.encoding = pylon_interface_->img_encoding();
    cv_img_seq_.encoding = sensor_msgs::image_encodings::BGR8;
    cv_img_hdr_.encoding = pylon_interface_->img_encoding();

    cv_img_hdr_.image = cv::Mat(pylon_interface_->img_rows(), pylon_interface_->img_cols(), CV_8UC1);
    cv_img_seq_.image = cv::Mat(pylon_interface_->img_rows(), pylon_interface_->img_cols(), CV_8UC3);
    cv_img_rect_.image = cv::Mat(pylon_interface_->img_rows(), pylon_interface_->img_cols(), CV_8UC1);


    for (std::size_t i = 0; i < 3; ++i)
        image_sequence_[i] = cv::Mat(pylon_interface_->img_rows(), pylon_interface_->img_cols(), CV_8UC1);

    calib_loader_.init(params_.intrinsic_yaml_file_);

    if (!calib_loader_.loadCalib())
    {
        params_.have_intrinsic_data_ = false;
        cerr << "Error reading intrinsic calibration from yaml file!" << endl;
    }
    if (calib_loader_.img_cols() != (int)img_raw_msg_.width || calib_loader_.img_rows()
                                                          != (int)img_raw_msg_.height)
    {
        cerr << "Error: Image size from yaml file (" << calib_loader_.img_cols() << ", "
             << calib_loader_.img_rows()
             << ") does not match to the size of the connected camera ("
             << img_raw_msg_.width
             << ", " << img_raw_msg_.height << ")! Will publish only raw image!" << endl;
        params_.have_intrinsic_data_ = false;
        if(params_.use_sequencer_){
            ROS_WARN("Sequencer-Mode needs intrinsic data! Will disable sequencer!");
            params_.use_sequencer_ =  false;
            params_.output_hdr_ = false;
        }
    } else
    {
        params_.have_intrinsic_data_ = true;
        setupCameraInfoMsg();
        img_rectifier_.setupRectifyingMap(calib_loader_.K(),
                                          calib_loader_.D(),
                                          pylon_interface_->img_cols(),
                                          pylon_interface_->img_rows());
    }

    if (params_.use_sequencer_)
    {
        exp_times_.exp_times.data.clear();
        exp_times_.exp_times.data.push_back(5000);
        exp_times_.exp_times.data.push_back(10000);
        exp_times_.exp_times.data.push_back(50000);
    }
    return true;
}
void PylonCameraOpenCVNode::setupCameraInfoMsg()
{

    cam_info_msg_.distortion_model = "plumb_bob";
    cam_info_msg_.D.resize(5);

    for (uint i = 0; i < 5; ++i)
    {
        double d = calib_loader_.D().at<double>(0, i);
        cam_info_msg_.D[i] = d;
    }

    int pos = 0;
    for (uint i = 0; i < 3; ++i)
    {
        for (uint j = 0; j < 3; ++j)
        {
            cam_info_msg_.K[pos++] = calib_loader_.K().at<double>(i, j);
        }
    }

    pos = 0;
    for (uint i = 0; i < 3; ++i)
    {
        for (uint j = 0; j < 3; ++j)
        {
            cam_info_msg_.P[pos++] = calib_loader_.K().at<double>(i, j);
        }
        cam_info_msg_.P[pos++] = 0;
    }
    //	cam_info_msg_.P = 0.0;
    //	cam_info_msg_.R = 0.0;
    //	cam_info_msg_.binning_x =
    //	cam_info_msg_.binning_y =
    //	cam_info_msg_.roi =
}

bool PylonCameraOpenCVNode::grabImage()
{
    if (!PylonCameraNode::grabImage())
        return false;

    //img_raw_ = cv::Mat(pylon_interface_->img_rows(), pylon_interface_->img_cols(), CV_8UC1);

    //memcpy(img_raw_.ptr(), img_raw_msg_.data.data(), pylon_interface_->image_size());

    img_raw_ = cv::Mat(pylon_interface_->img_rows(), pylon_interface_->img_cols(), CV_8UC1, img_raw_msg_.data.data());

    if (pylon_opencv_interface_.own_brightness_search_running_)
    {
//        int c = pylon_interface_->img_cols(), r = pylon_interface_->img_rows();
//        pylon_opencv_interface_.exp_search_params_.current_brightness_ = cv::mean(img_raw_.colRange(0.25 * c, 0.75 * c)
//                        .rowRange(0.25 * r,
//                                  0.75 * r))
//                                            .val[0];
        ROS_INFO("Calculating MEAN - maybe unnecessary?");
        pylon_opencv_interface_.exp_search_params_.current_brightness_ = cv::mean(img_raw_).val[0];
//        .colRange(0.25 * c, 0.75 * c)
//                        .rowRange(0.25 * r,
//                                  0.75 * r))
//                                            .val[0];
    }

    if (params_.have_intrinsic_data_)
    {
        cv_img_rect_.header = img_raw_msg_.header;
        img_rectifier_.rectify(img_raw_, cv_img_rect_.image);
    }

    return true;
}
bool PylonCameraOpenCVNode::grabSequence()
{
    for (int i = 0; i < 3; ++i)
    {
        if (!(pylon_opencv_interface_.grab(params_, image_sequence_[i])))
        {
            if (pylon_interface_->is_cam_removed())
            {
                ROS_ERROR("Pylon Camera has been removed!");
                ros::shutdown();
            }
            else
            {
                ROS_ERROR("Pylon Interface returned NULL-Pointer!");
            }
            return false;
        }
        if (i == 1)
        {
            if (getNumSubscribersRaw())
            {
                img_raw_msg_.data = std::vector<uint8_t>(image_sequence_[i].data,
                                                         image_sequence_[i].data + pylon_interface_->image_size());
            }
            if (params_.have_intrinsic_data_)
            {
                if (getNumSubscribersRect())
                {
                    img_rectifier_.rectify(image_sequence_[i], cv_img_rect_.image);
                }
            }
        }
    }

    img_raw_msg_.header.stamp = ros::Time::now();
    cam_info_msg_.header.stamp = img_raw_msg_.header.stamp;
    cv_img_rect_.header = img_raw_msg_.header;
    cv_img_seq_.header.stamp = img_raw_msg_.header.stamp;
    exp_times_.header.stamp = img_raw_msg_.header.stamp;


#if CV_MAJOR_VERSION > 2   // If you are using OpenCV 3
    if (params_.output_hdr_)
    {
        if (getNumSubscribersHdr())
        {
            //ros::Time now = ros::Time::now();
            cv_img_hdr_.header.stamp = img_raw_msg_.header.stamp;
            if (!hdr_generator_.merge(image_sequence_, pylon_opencv_interface_.seq_exp_times_, hdr_img_))
            {
                cerr << "Error while generating HDR image" << endl;
            }
            //ROS_INFO_STREAM("HDR TOOK " << (ros::Time::now() - now).toSec());
            img_rectifier_.rectify(hdr_img_, cv_img_hdr_.image);
        }
    }
    else
    {
#endif

        // I put this into the else clause of hdr as I don't think it makes sense to produce a psoido BGR image out of three grayscale images
        if (getNumSubscribersSeq())
        {
            cv::Mat img_seq_bgr;
            cv::merge(image_sequence_.data(), 3, img_seq_bgr);

            img_rectifier_.rectify(img_seq_bgr, cv_img_seq_.image);
        }

#if CV_MAJOR_VERSION > 2   // If you are using OpenCV 3
    }
#endif

    return true;
}
PylonCameraOpenCVNode::~PylonCameraOpenCVNode()
{
}
} /* namespace pylon_camera */
