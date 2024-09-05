// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Modified by Arlen Feng 4 September 2024
 *
 * Image Subscriber
 *
 * Modifications Made: 
 * 1. Left comments on relevant code
 * 2. Added method to convert from sensor img to Open CV image
 * 3. Added code in main() to create image subscriber node
 */

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "cv_bridge/cv_bridge.h"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Subscribing String: '%s'", msg->data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  try
  {
    cv::imshow("view", cv_bridge::toCvShare(msg, "rgb8")->image);
  }
  catch (cv_bridge::Exception& e)
  {
    auto logger = rclcpp::get_logger("my_subscriber");
    RCLCPP_ERROR(logger, "Could not convert from '%s' to 'rgb8'.", msg->encoding.c_str());
    RCLCPP_ERROR(logger, "%s", e.what());
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  //rclcpp::spin(std::make_shared<MinimalSubscriber>());
  auto node = rclcpp::Node::make_shared("listener");
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(node);
  image_transport::Subscriber sub = it.subscribe("imagetopic", 10, imageCallback);
  rclcpp::spin(node);
  cv::destroyWindow("view");

  rclcpp::shutdown();
  return 0;
}
