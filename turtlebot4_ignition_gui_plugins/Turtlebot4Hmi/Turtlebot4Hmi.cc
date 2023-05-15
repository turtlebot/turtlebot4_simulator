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

#include "Turtlebot4Hmi.hh"

#include <ignition/msgs/int32.pb.h>

#include <iostream>
#include <vector>

#include <ignition/plugin/Register.hh>
#include <ignition/gui/Application.hh>
#include <ignition/gui/MainWindow.hh>

using ignition::gui::Turtlebot4Hmi;

Turtlebot4Hmi::Turtlebot4Hmi()
: Plugin()
{
  App()->Engine()->rootContext()->setContextProperty("DisplayListView", &this->display_list_);
  CreatePublishers();
  CreateSubscribers();
}

Turtlebot4Hmi::~Turtlebot4Hmi()
{
}

void Turtlebot4Hmi::CreatePublishers()
{
  this->hmi_button_pub_ = ignition::transport::Node::Publisher();
  this->hmi_button_pub_ = this->node_.Advertise < ignition::msgs::Int32 > (this->hmi_button_topic_);
  this->create3_button_pub_ = ignition::transport::Node::Publisher();
  this->create3_button_pub_ = this->node_.Advertise < ignition::msgs::Int32 > (
    this->create3_button_topic_);
}

void Turtlebot4Hmi::CreateSubscribers()
{
  this->node_.Subscribe(this->display_topic_, &Turtlebot4Hmi::OnRawMessage, this);
  this->node_.Subscribe(this->display_selected_topic_, &Turtlebot4Hmi::OnSelectedMessage, this);
  this->node_.Subscribe(this->power_led_topic_, &Turtlebot4Hmi::OnPowerLedMessage, this);
  this->node_.Subscribe(this->motors_led_topic_, &Turtlebot4Hmi::OnMotorsLedMessage, this);
  this->node_.Subscribe(this->comms_led_topic_, &Turtlebot4Hmi::OnCommsLedMessage, this);
  this->node_.Subscribe(this->wifi_led_topic_, &Turtlebot4Hmi::OnWifiLedMessage, this);
  this->node_.Subscribe(this->battery_led_topic_, &Turtlebot4Hmi::OnBatteryLedMessage, this);
  this->node_.Subscribe(this->user1_led_topic_, &Turtlebot4Hmi::OnUser1LedMessage, this);
  this->node_.Subscribe(this->user2_led_topic_, &Turtlebot4Hmi::OnUser2LedMessage, this);
}

void Turtlebot4Hmi::RemovePublishers()
{
  this->node_.UnadvertiseSrv(this->hmi_button_topic_);
  this->node_.UnadvertiseSrv(this->create3_button_topic_);
}

void Turtlebot4Hmi::RemoveSubscribers()
{
  this->node_.Unsubscribe(this->display_topic_);
  this->node_.Unsubscribe(this->display_selected_topic_);
  this->node_.Unsubscribe(this->power_led_topic_);
  this->node_.Unsubscribe(this->motors_led_topic_);
  this->node_.Unsubscribe(this->comms_led_topic_);
  this->node_.Unsubscribe(this->wifi_led_topic_);
  this->node_.Unsubscribe(this->battery_led_topic_);
  this->node_.Unsubscribe(this->user1_led_topic_);
  this->node_.Unsubscribe(this->user2_led_topic_);
}

void Turtlebot4Hmi::LoadConfig(const tinyxml2::XMLElement * _pluginElem)
{
  if (!_pluginElem) {
    return;
  }

  if (this->title.empty()) {
    this->title = "Turtlebot4 HMI";
  }

  auto namespaceElem = _pluginElem->FirstChildElement("namespace");
  if (nullptr != namespaceElem && nullptr != namespaceElem->GetText())
  {
    this->SetNamespace(namespaceElem->GetText());
  }

  this->connect(
    this, SIGNAL(AddMsg(QString)), this, SLOT(OnAddMsg(QString)),
    Qt::QueuedConnection);
}

QString Turtlebot4Hmi::Namespace() const
{
  return QString::fromStdString(this->namespace_);
}

void Turtlebot4Hmi::SetNamespace(const QString &_name)
{
  this->namespace_ = _name.toStdString();

  ignmsg << "A new robot namespace has been entered: '" <<
      this->namespace_ << " ' " <<std::endl;

  // Remove existing pub/subs
  RemovePublishers();
  RemoveSubscribers();

  // Update topic names
  this->create3_button_topic_ = this->namespace_ + "/create3_buttons";
  this->hmi_button_topic_ = this->namespace_ + "/hmi/buttons";
  this->display_topic_ = this->namespace_ + "/hmi/display/raw";
  this->display_selected_topic_ = this->namespace_ + "/hmi/display/selected";
  this->power_led_topic_ = this->namespace_ + "/hmi/led/power";
  this->motors_led_topic_ = this->namespace_ + "/hmi/led/motors";
  this->comms_led_topic_ = this->namespace_ + "/hmi/led/comms";
  this->wifi_led_topic_ = this->namespace_ + "/hmi/led/wifi";
  this->battery_led_topic_ = this->namespace_ + "/hmi/led/battery";
  this->user1_led_topic_ = this->namespace_ + "/hmi/led/user1";
  this->user2_led_topic_ = this->namespace_ + "/hmi/led/user2";

  // Recreate publishers and subscribers with new topics.
  CreatePublishers();
  CreateSubscribers();
  this->NamespaceChanged();
}

void Turtlebot4Hmi::OnHmiButton(const int button)
{
  ignition::msgs::Int32 button_msg;

  button_msg.set_data(button);

  if (!this->hmi_button_pub_.Publish(button_msg)) {
    ignerr << "ignition::msgs::Int32 message couldn't be published at topic: " <<
      this->hmi_button_topic_ << std::endl;
  }
}

void Turtlebot4Hmi::OnCreate3Button(const int button)
{
  ignition::msgs::Int32 button_msg;

  button_msg.set_data(button);

  if (!this->create3_button_pub_.Publish(button_msg)) {
    ignerr << "ignition::msgs::Int32 message couldn't be published at topic: " <<
      this->create3_button_topic_ << std::endl;
  }
}

void Turtlebot4Hmi::OnRawMessage(const ignition::msgs::StringMsg & msg)
{
  std::lock_guard < std::mutex > lock(this->raw_msg_mutex_);
  this->AddMsg(QString::fromStdString(msg.data()));
}

void Turtlebot4Hmi::OnSelectedMessage(const ignition::msgs::Int32 & msg)
{
  std::lock_guard < std::mutex > lock(this->selected_msg_mutex_);
  selected_line_ = msg.data();
}

void Turtlebot4Hmi::OnPowerLedMessage(const ignition::msgs::Int32 & msg)
{
  switch (msg.data()) {
    case 0:
      {
        emit setPowerState(false, "green");
        break;
      }

    case 1:
      {
        emit setPowerState(true, "green");
        break;
      }

    default:
      break;
  }
}

void Turtlebot4Hmi::OnMotorsLedMessage(const ignition::msgs::Int32 & msg)
{
  switch (msg.data()) {
    case 0:
      {
        emit setMotorsState(false, "green");
        break;
      }

    case 1:
      {
        emit setMotorsState(true, "green");
        break;
      }

    default:
      break;
  }
}

void Turtlebot4Hmi::OnCommsLedMessage(const ignition::msgs::Int32 & msg)
{
  switch (msg.data()) {
    case 0:
      {
        emit setCommsState(false, "green");
        break;
      }

    case 1:
      {
        emit setCommsState(true, "green");
        break;
      }

    default:
      break;
  }
}

void Turtlebot4Hmi::OnWifiLedMessage(const ignition::msgs::Int32 & msg)
{
  switch (msg.data()) {
    case 0:
      {
        emit setWifiState(false, "green");
        break;
      }

    case 1:
      {
        emit setWifiState(true, "green");
        break;
      }

    default:
      break;
  }
}

void Turtlebot4Hmi::OnBatteryLedMessage(const ignition::msgs::Int32 & msg)
{
  switch (msg.data()) {
    case 0:
      {
        emit setBatteryState(false, "green");
        break;
      }

    case 1:
      {
        emit setBatteryState(true, "green");
        break;
      }

    case 2:
      {
        emit setBatteryState(true, "red");
        break;
      }

    case 3:
      {
        emit setBatteryState(true, "yellow");
        break;
      }

    default:
      break;
  }
}

void Turtlebot4Hmi::OnUser1LedMessage(const ignition::msgs::Int32 & msg)
{
  switch (msg.data()) {
    case 0:
      {
        emit setUser1State(false, "green");
        break;
      }

    case 1:
      {
        emit setUser1State(true, "green");
        break;
      }

    default:
      break;
  }
}

void Turtlebot4Hmi::OnUser2LedMessage(const ignition::msgs::Int32 & msg)
{
  switch (msg.data()) {
    case 0:
      {
        emit setUser2State(false, "green");
        break;
      }

    case 1:
      {
        emit setUser2State(true, "green");
        break;
      }

    case 2:
      {
        emit setUser2State(true, "red");
        break;
      }

    case 3:
      {
        emit setUser2State(true, "yellow");
        break;
      }

    default:
      break;
  }
}

void Turtlebot4Hmi::OnAddMsg(QString msg)
{
  std::lock_guard < std::mutex > lock(this->raw_msg_mutex_);

  std::string data = msg.toStdString();
  std::vector<std::string> lines(num_lines_);

  // Header
  lines[0] = " " + data.substr(0, char_per_line_header_);
  data = data.substr(char_per_line_header_, data.length());

  for (uint32_t i = 0; i < num_lines_ - 1; i++) {
    if (data.length() < char_per_line_ * i) {
      lines[i + 1] = "";
    } else if (data.length() < char_per_line_ * (i + 1)) {
      lines[i + 1] = data.substr(char_per_line_ * i, data.length() - (char_per_line_ * i));
    } else {
      lines[i + 1] = data.substr(char_per_line_ * i, char_per_line_);
    }
    lines[i + 1].insert(lines[i + 1].begin(), ' ');
  }

  {
    std::lock_guard < std::mutex > lock(this->selected_msg_mutex_);
    if (selected_line_ >= 0) {
      lines[selected_line_ + 1].replace(0, 1, ">");
    }
  }

  for (uint32_t i = 0; i < num_lines_; i++) {
    if (this->display_list_.insertRow(this->display_list_.rowCount())) {
      auto index = this->display_list_.index(this->display_list_.rowCount() - 1, 0);
      this->display_list_.setData(index, QString::fromStdString(lines[i]));
    }
  }

  auto diff = this->display_list_.rowCount() - this->num_lines_;
  this->display_list_.removeRows(0, diff);
}

// Register this plugin
IGNITION_ADD_PLUGIN(
  ignition::gui::Turtlebot4Hmi,
  ignition::gui::Plugin)
