/*
 * Copyright 2021 Clearpath Robotics, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @author Roni Kreinin (rkreinin@clearpathrobotics.com)
 */

#ifndef TURTLEBOT4_IGNITION_TOOLBOX__HMI_NODE_HPP_
#define TURTLEBOT4_IGNITION_TOOLBOX__HMI_NODE_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "turtlebot4_msgs/msg/user_button.hpp"
#include "turtlebot4_msgs/msg/user_display.hpp"

namespace turtlebot4_ignition_toolbox
{

static constexpr auto DISPLAY_CHAR_PER_LINE_HEADER = 21;

class Hmi : public rclcpp::Node
{
public:
  Hmi();

private:
  void display_subscriber_callback(const turtlebot4_msgs::msg::UserDisplay::SharedPtr msg);
  void button_subscriber_callback(const std_msgs::msg::Int32::SharedPtr msg);

  // Publishers
  rclcpp::Publisher<turtlebot4_msgs::msg::UserButton>::SharedPtr button_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr display_raw_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr display_selected_publisher_;

  // Subscribers
  rclcpp::Subscription<turtlebot4_msgs::msg::UserDisplay>::SharedPtr display_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr button_subscriber_;
};

}  // namespace turtlebot4_ignition_toolbox

#endif  // TURTLEBOT4_IGNITION_TOOLBOX__HMI_NODE_HPP_
