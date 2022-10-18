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

#include <ignition/plugin/Register.hh>
#include <ignition/gui/Application.hh>
#include <ignition/gui/MainWindow.hh>

#include <iostream>

using ignition::gui::Turtlebot4Hmi;

Turtlebot4Hmi::Turtlebot4Hmi()
: Plugin()
{
  App()->Engine()->rootContext()->setContextProperty("DisplayListView", &this->display_list_);
  this->UpdateTopics();
}

Turtlebot4Hmi::~Turtlebot4Hmi()
{
}

QString Turtlebot4Hmi::Namespace() const
{
  return QString::fromStdString(this->namespace_);
}

void Turtlebot4Hmi::UpdateTopics()
{
  // Update subscribers with new topics
  this->node_.Subscribe(this->display_topic_, &Turtlebot4Hmi::OnRawMessage, this);
  this->node_.Subscribe(this->display_selected_topic_, &Turtlebot4Hmi::OnSelectedMessage, this);
  this->node_.Subscribe(this->power_led_topic_, &Turtlebot4Hmi::OnPowerLedMessage, this);
  this->node_.Subscribe(this->motors_led_topic_, &Turtlebot4Hmi::OnMotorsLedMessage, this);
  this->node_.Subscribe(this->comms_led_topic_, &Turtlebot4Hmi::OnCommsLedMessage, this);
  this->node_.Subscribe(this->wifi_led_topic_, &Turtlebot4Hmi::OnWifiLedMessage, this);
  this->node_.Subscribe(this->battery_led_topic_, &Turtlebot4Hmi::OnBatteryLedMessage, this);
  this->node_.Subscribe(this->user1_led_topic_, &Turtlebot4Hmi::OnUser1LedMessage, this);
  this->node_.Subscribe(this->user2_led_topic_, &Turtlebot4Hmi::OnUser2LedMessage, this);

  // Update publisher with new topics
  this->hmi_button_pub_ = ignition::transport::Node::Publisher();
  this->hmi_button_pub_ = this->node_.Advertise < ignition::msgs::Int32 > (this->hmi_button_topic_);
  this->create3_button_pub_ = gz::transport::Node::Publisher();
  this->create3_button_pub_ = this->node_.Advertise< ignition::msgs::Int32 > (this->create3_button_topic_);
  if (!this->create3_button_pub_ || !this->hmi_button_pub_)
  {
    App()->findChild<MainWindow *>()->notifyWithDuration(
      QString::fromStdString("Error when advertising topics"), 4000);
    ignerr << "Error when advertising topics" << std::endl;
  }
  else
  {
    App()->findChild<MainWindow *>()->notifyWithDuration(
      QString::fromStdString("Advertising new topics based on robot name: " + this->namespace_), 4000);
  }
}

void Turtlebot4Hmi::SetNamespace(const QString &_namespace)
{
  this->namespace_ = _namespace.toStdString();
  ignmsg << "A new robot name has been entered: '" <<
      this->namespace_ << " ' " <<std::endl;

  // Unsubscribe from old topics 
  this->node_.Unsubscribe(this->display_topic_);
  this->node_.Unsubscribe(this->display_selected_topic_);
  this->node_.Unsubscribe(this->power_led_topic_);
  this->node_.Unsubscribe(this->motors_led_topic_);
  this->node_.Unsubscribe(this->comms_led_topic_);
  this->node_.Unsubscribe(this->wifi_led_topic_);
  this->node_.Unsubscribe(this->battery_led_topic_);
  this->node_.Unsubscribe(this->user1_led_topic_);
  this->node_.Unsubscribe(this->user2_led_topic_);

  // Change all topics
  if (this->namespace_ != "")
  {
    this->hmi_button_topic_ = "/model/" + this->namespace_ + "/hmi/buttons";
    this->create3_button_topic_ = "/model/" + this->namespace_ + "/buttons";
    this->display_topic_ = "/model/" + this->namespace_ +  "/hmi/display/raw";
    this->display_selected_topic_ = "/model/" + this->namespace_ + "/hmi/display/selected";
    this->power_led_topic_ = "/model/" + this->namespace_ + "/hmi/led/power";
    this->motors_led_topic_ = "/model/" + this->namespace_ + "/hmi/led/motors";
    this->comms_led_topic_ = "/model/" + this->namespace_ + "/hmi/led/comms";
    this->wifi_led_topic_ = "/model/" + this->namespace_ + "/hmi/led/wifi";
    this->battery_led_topic_ = "/model/" + this->namespace_ + "/hmi/led/battery";
    this->user1_led_topic_ = "/model/" + this->namespace_ + "/hmi/led/user1";
    this->user2_led_topic_ = "/model/" + this->namespace_ + "/hmi/led/user2";
  }
  else
  {
    this->hmi_button_topic_ = "/model/turtlebot4/hmi/buttons";
    this->create3_button_topic_ = "model/turtlebot4/buttons";
    this->display_topic_ = "/model/turtlebot4/hmi/display/raw";
    this->display_selected_topic_ = "/model/turtlebot4/hmi/display/selected";
    this->power_led_topic_ = "/model/turtlebot4/hmi/led/power";
    this->motors_led_topic_ = "/model/turtlebot4/hmi/led/motors";
    this->comms_led_topic_ = "/model/turtlebot4/hmi/led/comms";
    this->wifi_led_topic_ = "/model/turtlebot4/hmi/led/wifi";
    this->battery_led_topic_ = "/model/turtlebot4/hmi/led/battery";
    this->user1_led_topic_ = "/model/turtlebot4/hmi/led/user1";
    this->user2_led_topic_ = "/model/turtlebot4/hmi/led/user2";
  }

  this->UpdateTopics();
  this->NamespaceChanged();
}

void Turtlebot4Hmi::LoadConfig(const tinyxml2::XMLElement * _pluginElem)
{
  if (this->title.empty()) {
    this->title = "Turtlebot4 HMI";
  }

  if (_pluginElem)
  {
    auto topicElem = _pluginElem->FirstChildElement("topic");
    if (nullptr != topicElem && nullptr != topicElem->GetText())
      this->SetNamespace(topicElem->GetText());
  }

  this->connect(
    this, SIGNAL(AddMsg(QString)), this, SLOT(OnAddMsg(QString)),
    Qt::QueuedConnection);
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
