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

#include <string>
#include <utility>
#include <memory>

#include "turtlebot4_ignition_toolbox/hmi_node.hpp"

using turtlebot4_ignition_toolbox::Hmi;

Hmi::Hmi()
: rclcpp::Node("hmi_node")
{
  display_subscriber_ = create_subscription<turtlebot4_msgs::msg::UserDisplay>(
    "hmi/display",
    rclcpp::SensorDataQoS(),
    std::bind(&Hmi::display_subscriber_callback, this, std::placeholders::_1));

  button_subscriber_ = create_subscription<std_msgs::msg::Int32>(
    "hmi/buttons/_set",
    rclcpp::QoS(rclcpp::KeepLast(10)),
    std::bind(&Hmi::button_subscriber_callback, this, std::placeholders::_1));

  button_publisher_ = create_publisher<turtlebot4_msgs::msg::UserButton>(
    "hmi/buttons",
    rclcpp::SensorDataQoS());

  display_raw_publisher_ = create_publisher<std_msgs::msg::String>(
    "hmi/display/_raw",
    rclcpp::QoS(rclcpp::KeepLast(10)));

  display_selected_publisher_ = create_publisher<std_msgs::msg::Int32>(
    "hmi/display/_selected",
    rclcpp::QoS(rclcpp::KeepLast(10)));
}

void Hmi::display_subscriber_callback(const turtlebot4_msgs::msg::UserDisplay::SharedPtr msg)
{
  auto raw_msg = std::make_unique<std_msgs::msg::String>();
  auto selected_msg = std::make_unique<std_msgs::msg::Int32>();
  selected_msg->data = msg->selected_entry;

  std::string header = msg->ip + " " + msg->battery + "%";

  if (header.length() < DISPLAY_CHAR_PER_LINE_HEADER) {
    // Pad string
    header.insert(header.length(), DISPLAY_CHAR_PER_LINE_HEADER - header.length(), ' ');
  } else if (header.length() > DISPLAY_CHAR_PER_LINE_HEADER) {
    // Remove excess characters
    header = header.substr(0, DISPLAY_CHAR_PER_LINE_HEADER);
  }

  raw_msg->data = header;

  for (size_t i = 0; i < msg->entries.size(); i++) {
    raw_msg->data += msg->entries[i];
  }

  display_raw_publisher_->publish(std::move(raw_msg));
  display_selected_publisher_->publish(std::move(selected_msg));
}

void Hmi::button_subscriber_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  auto button_msg = std::make_unique<turtlebot4_msgs::msg::UserButton>();

  if (msg->data > 0) {
    button_msg->button[msg->data - 1] = true;
  }

  button_publisher_->publish(std::move(button_msg));
}
