/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (C) 2023, Basler AG. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * No contributors' name may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging_macros.h>

#include <pylon_ros2_camera_component/pylon_ros2_camera_node.hpp>

#include <opencv4/opencv2/highgui.hpp>

#include <sstream>

namespace pylon_ros2_camera
{
  class TestGrabBlazeDataActionClient : public rclcpp::Node
  {
    public:
      using GrabBlazeDataAction              = pylon_ros2_camera_interfaces::action::GrabBlazeData;
      using GrabBlazeDataGoalHandle          = rclcpp_action::ClientGoalHandle<GrabBlazeDataAction>;

      explicit TestGrabBlazeDataActionClient(const rclcpp::NodeOptions & options) : Node("test_grab_blaze_data_action_client", options)
      {
        // to be adapted if needed
        this->client_ptr_ = rclcpp_action::create_client<GrabBlazeDataAction>(this, "/my_camera/pylon_ros2_camera_node/grab_blaze_data");

        this->timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&TestGrabBlazeDataActionClient::send_goal, this));
      }

      void send_goal()
      {
        using namespace std::placeholders;

        this->timer_->cancel();

        if (!this->client_ptr_->wait_for_action_server())
        {
          RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting, shutting down now...");
          rclcpp::shutdown();
        }

        auto goal_msg = GrabBlazeDataAction::Goal();

        // to be adapted if needed
        goal_msg.exposure_given = true;
        std::vector<float> test_exposure_times;
        test_exposure_times.push_back(500);
        test_exposure_times.push_back(800);
        goal_msg.exposure_times = test_exposure_times;

        RCLCPP_INFO(this->get_logger(), "Sending goal...");

        auto send_goal_options = rclcpp_action::Client<GrabBlazeDataAction>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&TestGrabBlazeDataActionClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback = std::bind(&TestGrabBlazeDataActionClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&TestGrabBlazeDataActionClient::result_callback, this, _1);
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);

        RCLCPP_INFO(this->get_logger(), "Goal sent!");
      }

    private:
      rclcpp_action::Client<GrabBlazeDataAction>::SharedPtr client_ptr_;
      rclcpp::TimerBase::SharedPtr timer_;

      void goal_response_callback(const GrabBlazeDataGoalHandle::SharedPtr & goal_handle)
      {
        if (!goal_handle)
        {
          RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        }
        else
        {
          RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
      }

      void feedback_callback(GrabBlazeDataGoalHandle::SharedPtr, const std::shared_ptr<const GrabBlazeDataAction::Feedback> feedback)
      {
        RCLCPP_INFO_STREAM(this->get_logger(), "Number of data set already acquired: " << feedback->curr_nr_data_acquired);
      }

      void result_callback(const GrabBlazeDataGoalHandle::WrappedResult & result)
      {
        switch (result.code)
        {
          case rclcpp_action::ResultCode::SUCCEEDED:
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
          default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
        }

        // only the intensity map is displayed
        // to be adapted if needed

        RCLCPP_INFO_STREAM(this->get_logger(), "Number of acquired data set: " << result.result->intensity_maps.size());
        for (std::size_t i = 0; i < result.result->intensity_maps.size(); i++)
        {
          sensor_msgs::msg::Image& intensity_map = result.result->intensity_maps[i];
          sensor_msgs::msg::Image& depth_map = result.result->depth_maps[i];
          sensor_msgs::msg::Image& depth_color_map = result.result->depth_color_maps[i];
          sensor_msgs::msg::Image& confidence_map = result.result->confidence_maps[i];

          RCLCPP_INFO_STREAM(this->get_logger(), "Blaze data #" << i+1 << "\n\t"
                << "Intensity map" << "\n\t\t"
                    << "Encoding:  " << intensity_map.encoding << "\n\t\t"
                    << "Width:     " << intensity_map.width << "\n\t\t"
                    << "Height:    " << intensity_map.height << "\n\t\t"
                    << "Step:      " << intensity_map.step << "\n\t\t"
                    << "Timestamp: " << intensity_map.header.stamp.sec << "\n\t\t"
                    << "Frame ID:  " << intensity_map.header.frame_id << "\n\t"
                << "Depth map" << "\n\t\t"
                    << "Encoding:  " << depth_map.encoding << "\n\t\t"
                    << "Width:     " << depth_map.width << "\n\t\t"
                    << "Height:    " << depth_map.height << "\n\t\t"
                    << "Step:      " << depth_map.step << "\n\t\t"
                    << "Timestamp: " << depth_map.header.stamp.sec << "\n\t\t"
                    << "Frame ID:  " << depth_map.header.frame_id << "\n\t"
                << "Depth color map" << "\n\t\t"
                    << "Encoding:  " << depth_color_map.encoding << "\n\t\t"
                    << "Width:     " << depth_color_map.width << "\n\t\t"
                    << "Height:    " << depth_color_map.height << "\n\t\t"
                    << "Step:      " << depth_color_map.step << "\n\t\t"
                    << "Timestamp: " << depth_color_map.header.stamp.sec << "\n\t\t"
                    << "Frame ID:  " << depth_color_map.header.frame_id << "\n\t"
                << "Confidence map" << "\n\t\t"
                    << "Encoding:  " << confidence_map.encoding << "\n\t\t"
                    << "Width:     " << confidence_map.width << "\n\t\t"
                    << "Height:    " << confidence_map.height << "\n\t\t"
                    << "Step:      " << confidence_map.step << "\n\t\t"
                    << "Timestamp: " << confidence_map.header.stamp.sec << "\n\t\t"
                    << "Frame ID:  " << confidence_map.header.frame_id);

          cv_bridge::CvImagePtr cv_img = cv_bridge::toCvCopy(result.result->intensity_maps[i], result.result->intensity_maps[i].encoding);

          std::ostringstream ss;
          ss << "Intensity map #" << i+1;

          double ratio = (double)intensity_map.height / (double)intensity_map.width;
          double w = 960;
          double h = w * ratio;
          cv::Mat img_resized;
          cv::resize(cv_img->image, img_resized, cv::Size(int(w), int(h)), cv::INTER_LINEAR);

          cv::namedWindow(ss.str().c_str(), cv::WINDOW_NORMAL);
          cv::imshow(ss.str().c_str(), img_resized);
          cv::waitKey(0);
          cv::destroyWindow(ss.str().c_str());
        }

        rclcpp::shutdown();
      }

  }; // class TestGrabBlazeDataActionClient

} // namespace pylon_ros2_camera

RCLCPP_COMPONENTS_REGISTER_NODE(pylon_ros2_camera::TestGrabBlazeDataActionClient)
