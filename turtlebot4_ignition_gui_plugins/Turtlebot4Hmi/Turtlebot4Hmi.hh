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

#ifndef IGNITION_GUI_TURTLEBOT4HMI_HH_
#define IGNITION_GUI_TURTLEBOT4HMI_HH_

#include <string>

#include <ignition/transport/Node.hh>

#include <ignition/gui/qt.h>
#include <ignition/gui/Plugin.hh>

namespace ignition
{
  namespace gui
  {
    class Turtlebot4Hmi : public Plugin
    {
      Q_OBJECT

      /// \brief Constructor
      public: Turtlebot4Hmi();

      /// \brief Destructor
      public: virtual ~Turtlebot4Hmi();

      /// \brief Called by Ignition GUI when plugin is instantiated.
      /// \param[in] _pluginElem XML configuration for this plugin.
      public: virtual void LoadConfig(const tinyxml2::XMLElement *_pluginElem)
          override;

      /// \brief Callback trigged when the button is pressed.
      protected slots: void OnHmiButton(const int button);

      protected slots: void OnCreate3Button(const int button);

      signals: void setPowerState(const bool state, const QString color);

      signals: void setMotorsState(const bool state, const QString color);

      signals: void setCommsState(const bool state, const QString color);

      signals: void setWifiState(const bool state, const QString color);

      signals: void setBatteryState(const bool state, const QString color);

      signals: void setUser1State(const bool state, const QString color);

      signals: void setUser2State(const bool state, const QString color);

      private: void OnRawMessage(const ignition::msgs::StringMsg &msg);

      private: void OnSelectedMessage(const ignition::msgs::Int32 &msg);

      private: void OnPowerLedMessage(const ignition::msgs::Int32 &msg);

      private: void OnMotorsLedMessage(const ignition::msgs::Int32 &msg);

      private: void OnCommsLedMessage(const ignition::msgs::Int32 &msg);

      private: void OnWifiLedMessage(const ignition::msgs::Int32 &msg);

      private: void OnBatteryLedMessage(const ignition::msgs::Int32 &msg);

      private: void OnUser1LedMessage(const ignition::msgs::Int32 &msg);

      private: void OnUser2LedMessage(const ignition::msgs::Int32 &msg);

      signals: void AddMsg(QString msg);

      private slots: void OnAddMsg(QString msg);

      private: ignition::transport::Node node_;

      private: ignition::transport::Node::Publisher hmi_button_pub_;

      private: ignition::transport::Node::Publisher create3_button_pub_;

      private: std::string hmi_button_topic_ = "/model/turtlebot4/hmi/buttons";

      private: std::string create3_button_topic_ = "/create3/buttons";

      private: std::string display_topic_ = "/model/turtlebot4/hmi/display/raw";

      private: std::string display_selected_topic_ = "/model/turtlebot4/hmi/display/selected";

      private: std::string power_led_topic_ = "/model/turtlebot4/hmi/led/power";

      private: std::string motors_led_topic_ = "/model/turtlebot4/hmi/led/motors";

      private: std::string comms_led_topic_ = "/model/turtlebot4/hmi/led/comms";

      private: std::string wifi_led_topic_ = "/model/turtlebot4/hmi/led/wifi";

      private: std::string battery_led_topic_ = "/model/turtlebot4/hmi/led/battery";

      private: std::string user1_led_topic_ = "/model/turtlebot4/hmi/led/user1";

      private: std::string user2_led_topic_ = "/model/turtlebot4/hmi/led/user2";

      private: QStringListModel display_list_;

      private: QObject led1_;

      private: const unsigned int num_lines_{6u};

      private: const unsigned int char_per_line_{18u};

      private: const unsigned int char_per_line_header_{21u};

      private: int selected_line_{0u};

      private: std::mutex raw_msg_mutex_;

      private: std::mutex selected_msg_mutex_;
    };
  }
}

#endif // IGNITION_GUI_TURTLEBOT4HMI_HH_