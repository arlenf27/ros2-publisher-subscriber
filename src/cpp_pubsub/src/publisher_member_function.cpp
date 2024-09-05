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
 * Image Publisher
 * 
 * Minimal Publisher left by base turorial code still exists but is not used. 
 *
 * Modifications Made: 
 * 1. Left comments on relevant code
 * 2. Created code to publish image
 * 3. Created code to convert image from Open CV format to sensors img
 * 4. MinimalPublisher (that publishes Strings) is not used, but image publisher is used currently
 * 5. Image publisher will continuously publish data until terminated
 */

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/imgcodecs.hpp"

using namespace std::chrono_literals;

/** 
 * This MinimalPublisher creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. It is currently NOT used. 
 */
class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));		// timer_callback() is executed per 500 ms
  }

private:
  /**
   * Creates and publishes message
   */
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing String: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  /*
   * Private instance variables used in class methods and constructor
   */
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  /*
   * Initializes ROS2
   */
  rclcpp::init(argc, argv);
  /*
   * Processes data from the node for MinimalPublisher
   * Enable MinimalPublisher by uncommenting this line, but image publisher will NOT work unless MinimalPublisher is not processing
   */
  // rclcpp::spin(std::make_shared<MinimalPublisher>());

  /*
   * Creates new node for image transport
   * Uses OpenCV to read in an image in RGB from file passed in at argv[1] 
   * Converts CV Mat image to sensor msgs to publish
   * Publishes image data continuously until terminated 
   */
  auto node = rclcpp::Node::make_shared("talker");
  image_transport::ImageTransport it(node);
  image_transport::Publisher pub = it.advertise("imagetopic", 10);
  cv::Mat img = cv::imread(argv[1], cv::IMREAD_COLOR);
  sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", img).toImageMsg();

  rclcpp::Rate loop_rate(5);
  while (rclcpp::ok()) {
    pub.publish(msg);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
