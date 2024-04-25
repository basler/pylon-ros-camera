/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (C) 2022, Basler AG. All rights reserved.
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


namespace pylon_ros2_camera
{
  class TestGrabImagesActionClient : public rclcpp::Node
  {
    public:
      using GrabImagesAction              = pylon_ros2_camera_interfaces::action::GrabImages;
      using GrabImagesGoalHandle          = rclcpp_action::ClientGoalHandle<GrabImagesAction>;

      explicit TestGrabImagesActionClient(const rclcpp::NodeOptions & options) : Node("test_grab_images_action_client", options)
      {
        // to be adapted if needed
        this->client_ptr_ = rclcpp_action::create_client<GrabImagesAction>(this, "/my_camera/pylon_ros2_camera_node/grab_images_raw");

        this->timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&TestGrabImagesActionClient::send_goal, this));
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

        auto goal_msg = GrabImagesAction::Goal();

        // to be adapted if needed
        goal_msg.gain_given = true;
        std::vector<float> test_gain_values;
        test_gain_values.push_back(0.5);
        //test_gain_values.push_back(0.9);
        goal_msg.gain_values = test_gain_values;

        RCLCPP_INFO(this->get_logger(), "Sending goal...");

        auto send_goal_options = rclcpp_action::Client<GrabImagesAction>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&TestGrabImagesActionClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback = std::bind(&TestGrabImagesActionClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&TestGrabImagesActionClient::result_callback, this, _1);
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);

        RCLCPP_INFO(this->get_logger(), "Goal sendt!");
      }

    private:
      rclcpp_action::Client<GrabImagesAction>::SharedPtr client_ptr_;
      rclcpp::TimerBase::SharedPtr timer_;

      void goal_response_callback(const GrabImagesGoalHandle::SharedPtr & goal_handle)
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

      void feedback_callback(GrabImagesGoalHandle::SharedPtr, const std::shared_ptr<const GrabImagesAction::Feedback> feedback)
      {
        RCLCPP_INFO_STREAM(this->get_logger(), "Number of images already acquired: " << feedback->curr_nr_images_taken);
      }

      void result_callback(const GrabImagesGoalHandle::WrappedResult & result)
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

        RCLCPP_INFO_STREAM(this->get_logger(), "Number of acquired images: " << result.result->images.size());
        for (std::size_t i = 0; i < result.result->images.size(); i++)
        {
          sensor_msgs::msg::Image& img = result.result->images[i];

          RCLCPP_INFO_STREAM(this->get_logger(), "Image #" << i+1 << "\n\t"
            << "Encoding:  " << img.encoding << "\n\t"
            << "Width:     " << img.width << "\n\t"
            << "Height:    " << img.height << "\n\t"
            << "Step:      " << img.step << "\n\t"
            << "Timestamp: " << img.header.stamp.sec << "\n\t"
            << "Frame ID:  " << img.header.frame_id);

          cv_bridge::CvImagePtr cv_img = cv_bridge::toCvCopy(result.result->images[i], result.result->images[i].encoding);
          
          std::stringstream ss;
          ss << "Image #" << i+1;

          double ratio = (double)img.height / (double)img.width;
          double w = 960;
          double h = w * ratio;
          cv::Mat img_resized;
          cv::resize(cv_img->image, img_resized, cv::Size(int(w), int(h)), cv::INTER_LINEAR);

          cv::namedWindow(ss.str().c_str(), cv::WINDOW_NORMAL);
          cv::imshow(ss.str().c_str(), img_resized);
          cv::waitKey(0);
          //cv::destroyWindow(ss.str().c_str());
        }

        rclcpp::shutdown();
      }

  }; // class TestGrabImagesActionClient

} // namespace pylon_ros2_camera

RCLCPP_COMPONENTS_REGISTER_NODE(pylon_ros2_camera::TestGrabImagesActionClient)
