/*
 * pylon_camera_opencv_node.cpp
 *
 *  Created on: Jun 10, 2015
 *      Author: md
 */

#include <pylon_camera/pylon_camera_opencv_node.h>

namespace pylon_camera
{

PylonCameraOpenCVNode::PylonCameraOpenCVNode() :
        calib_loader_(NULL),
        img_rectifier_(),
        cv_img_rect_(),
        cv_img_seq_(),
        exp_times_(),
        img_rect_pub_(NULL),
        exp_times_pub_(
        NULL),
        img_seq_pub_(NULL)
{
}
void PylonCameraOpenCVNode::setInitialCameraParameter()
{
  PylonCameraNode::setInitialCameraParameter();

  nh_.param<bool>("use_sequencer", params_.use_sequencer_, false);

  nh_.param<std::string>("intrinsic_yaml_string", params_.intrinsic_yaml_file_, "INVALID_YAML_FILE");

  if (params_.intrinsic_yaml_file_ == "INVALID_YAML_FILE")
  {
    ROS_ERROR("Yaml file string needed for rectification! Param: 'intrinsic_yaml_string' has entry: %s",
              params_.intrinsic_yaml_file_.c_str());
    ROS_ERROR("Alternative: Get only distorted image by compiling 'WITHOUT_OPENCV'");
  }
}
void PylonCameraOpenCVNode::createPylonInterface()
{
  if (params_.use_sequencer_)
  {
    pylon_interface_ = new PylonSequencerInterface();
    ROS_INFO("Created PylonSequencerInterface");
  }
  else
  {
    pylon_interface_ = new PylonInterface();
    ROS_INFO("Created PylonInterface");
  }
}
uint32_t PylonCameraOpenCVNode::getNumSubscribers()
{
  return img_raw_pub_->getNumSubscribers() + img_rect_pub_->getNumSubscribers() + img_seq_pub_->getNumSubscribers();
}
uint32_t PylonCameraOpenCVNode::getNumSubscribersRaw()
{
  return img_raw_pub_->getNumSubscribers();
}
uint32_t PylonCameraOpenCVNode::getNumSubscribersRect()
{
  return img_rect_pub_->getNumSubscribers();
}
uint32_t PylonCameraOpenCVNode::getNumSubscribersSeq()
{
  return img_seq_pub_->getNumSubscribers();
}
int PylonCameraOpenCVNode::init()
{
  if (PylonCameraNode::init())
  {
    return 1;
  }

  img_rect_pub_ = new ros::Publisher(nh_.advertise<sensor_msgs::Image>("image_rect", 10));
  img_seq_pub_ = new ros::Publisher(nh_.advertise<sensor_msgs::Image>("image_seq", 10));
  exp_times_pub_ = new ros::Publisher(nh_.advertise<pylon_camera_msgs::SequenceExposureTimes>("seq_exp_times", 10));

  cv_img_rect_.header = img_raw_msg_.header;
  cv_img_seq_.header = img_raw_msg_.header;
  exp_times_.header = img_raw_msg_.header;
  // Encoding of pixels -- channel meaning, ordering, size
  // taken from the list of strings in include/sensor_msgs/image_encodings.h
  cv_img_rect_.encoding = pylon_interface_->img_encoding();
  cv_img_seq_.encoding = sensor_msgs::image_encodings::BGR8;

  calib_loader_ = new IntrinsicCalibLoader(params_.intrinsic_yaml_file_);

  if (!calib_loader_->loadCalib())
  {
    cerr << "Error reading intrinsic calibration from yaml file!" << endl;
    return 2;
  }
  if (calib_loader_->img_cols_ != img_raw_msg_.width || calib_loader_->img_rows_ != img_raw_msg_.height)
  {
    cerr << "Error: Image size from yaml file (" << calib_loader_->img_cols_ << ", " << calib_loader_->img_rows_
         << ") does not match to the size of the connected camera (" << img_raw_msg_.width << ", "
         << img_raw_msg_.height << ")!" << endl;
    return 3;
  }

  setupCameraInfoMsg();
  img_rectifier_.setupRectifyingMap(calib_loader_->K_, calib_loader_->D_, pylon_interface_->img_cols(),
                                    pylon_interface_->img_rows());

  exp_times_.exp_times.data.clear();
  exp_times_.exp_times.data.push_back(5000);
  exp_times_.exp_times.data.push_back(10000);
  exp_times_.exp_times.data.push_back(50000);

  return 0;
}
void PylonCameraOpenCVNode::setupCameraInfoMsg()
{

  cam_info_msg_.distortion_model = "plumb_bob";
  cam_info_msg_.D.resize(5);

  for (uint i = 0; i < 5; ++i)
  {
    double d = calib_loader_->D_.at<double>(0, i);
    cam_info_msg_.D[i] = d;
  }

  int pos = 0;
  for (uint i = 0; i < 3; ++i)
  {
    for (uint j = 0; j < 3; ++j)
    {
      cam_info_msg_.K[pos++] = calib_loader_->K_.at<double>(i, j);
    }
  }

  pos = 0;
  for (uint i = 0; i < 3; ++i)
  {
    for (uint j = 0; j < 3; ++j)
    {
      cam_info_msg_.P[pos++] = calib_loader_->K_.at<double>(i, j);
    }
    cam_info_msg_.P[pos++] = 0;
  }
//	cam_info_msg_.P = 0.0;
//	cam_info_msg_.R = 0.0;
//	cam_info_msg_.binning_x =
//	cam_info_msg_.binning_y =
//	cam_info_msg_.roi =
}

bool PylonCameraOpenCVNode::grabbingCallback()
{

  if (params_.use_sequencer_)
  {
    std::vector<cv::Mat> img_sequence;
    img_sequence.clear();
    for (int i = 0; i < 3; ++i)
    {
      cv::Mat img(pylon_interface_->img_rows(), pylon_interface_->img_cols(), CV_8UC1);
      const uint8_t* img_ptr(pylon_interface_->grab(params_));
      if (img_ptr == NULL)
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
        return NULL;
      }
      memcpy(img.ptr(), img_ptr, img_size_byte_);
      img_sequence.push_back(img);
      if (i == 1)
      {
        img_raw_msg_.data = std::vector<uint8_t>(img_ptr, img_ptr + img_size_byte_);
      }
    }

    img_raw_msg_.header.stamp = ros::Time::now();
    cam_info_msg_.header.stamp = img_raw_msg_.header.stamp;
    cv_img_seq_.header.stamp = img_raw_msg_.header.stamp;
    exp_times_.header.stamp = img_raw_msg_.header.stamp;

    cv::Mat img_seq_bgr;
    cv::Mat in[] = {img_sequence.at(0), img_sequence.at(1), img_sequence.at(2)};
    cv::merge(in, 3, img_seq_bgr);

    cv_img_seq_.image = cv::Mat(pylon_interface_->img_rows(), pylon_interface_->img_cols(), CV_8UC1);
    img_rectifier_.rectify(img_seq_bgr, cv_img_seq_.image);

    return true;
  }
  else
  {
    const uint8_t* img_raw_ptr = PylonCameraNode::grabbingCallback();

    cv_img_rect_.header = img_raw_msg_.header;

    cv::Mat img_raw = cv::Mat(pylon_interface_->img_rows(), pylon_interface_->img_cols(), CV_8UC1);
    memcpy(img_raw.ptr(), img_raw_ptr, img_size_byte_);

    cv::Mat img_rect = cv::Mat(pylon_interface_->img_rows(), pylon_interface_->img_cols(), CV_8UC1);

    img_rectifier_.rectify(img_raw, img_rect);
    //	img_rectifier_.rectify(cv::Mat(pylon_interface_->img_rows(), pylon_interface_->img_cols(), CV_8UC1, img_raw_ptr), img_rect);

    cv_img_rect_.image = img_rect;

    return true;
  }
  return false;
}
PylonCameraOpenCVNode::~PylonCameraOpenCVNode()
{
  delete img_rect_pub_;
  img_rect_pub_ = NULL;
  if (img_seq_pub_ != NULL)
  {
    delete img_seq_pub_;
    img_seq_pub_ = NULL;
  }
  delete calib_loader_;
  calib_loader_ = NULL;
}
} /* namespace pylon_camera */
