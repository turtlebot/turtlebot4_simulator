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

#ifndef TURTLEBOT4_IGNITION_GUI_PLUGINS__TURTLEBOT4HMI__TURTLEBOT4HMI_HH_
#define TURTLEBOT4_IGNITION_GUI_PLUGINS__TURTLEBOT4HMI__TURTLEBOT4HMI_HH_

#include <ignition/gui/qt.h>

#include <string>

#include <ignition/transport/Node.hh>
#include <ignition/gui/Plugin.hh>

namespace ignition
{

namespace gui
{

class Turtlebot4Hmi : public Plugin
{
  Q_OBJECT

  // \brief Namespace
  Q_PROPERTY(
    QString name
    READ Namespace
    WRITE SetNamespace
    NOTIFY NamespaceChanged
  )

public:
  /// \brief Constructor
  Turtlebot4Hmi();
  /// \brief Destructor
  virtual ~Turtlebot4Hmi();
  /// \brief Called by Ignition GUI when plugin is instantiated.
  /// \param[in] _pluginElem XML configuration for this plugin.
  void LoadConfig(const tinyxml2::XMLElement * _pluginElem) override;
  // \brief Get the robot namespace as a string, for example
  /// '/robot1'
  /// \return Namespace
  Q_INVOKABLE QString Namespace() const;

public slots:
  /// \brief Callback in Qt thread when the robot namespace changes.
  /// \param[in] _name variable to indicate the robot namespace to
  /// publish the Button commands.
  void SetNamespace(const QString &_name);

protected slots:
  /// \brief Callback trigged when the button is pressed.
  void OnHmiButton(const int button);
  void OnCreate3Button(const int button);

  /// \brief QML signals
signals:
  void setPowerState(const bool state, const QString color);
  void setMotorsState(const bool state, const QString color);
  void setCommsState(const bool state, const QString color);
  void setWifiState(const bool state, const QString color);
  void setBatteryState(const bool state, const QString color);
  void setUser1State(const bool state, const QString color);
  void setUser2State(const bool state, const QString color);
  /// \brief Notify that robot namespace has changed
  void NamespaceChanged();

  /// \brief Subscriber callbacks
private:
  void OnRawMessage(const ignition::msgs::StringMsg & msg);
  void OnSelectedMessage(const ignition::msgs::Int32 & msg);
  void OnPowerLedMessage(const ignition::msgs::Int32 & msg);
  void OnMotorsLedMessage(const ignition::msgs::Int32 & msg);
  void OnCommsLedMessage(const ignition::msgs::Int32 & msg);
  void OnWifiLedMessage(const ignition::msgs::Int32 & msg);
  void OnBatteryLedMessage(const ignition::msgs::Int32 & msg);
  void OnUser1LedMessage(const ignition::msgs::Int32 & msg);
  void OnUser2LedMessage(const ignition::msgs::Int32 & msg);
  void CreatePublishers();
  void CreateSubscribers();
  void RemovePublishers();
  void RemoveSubscribers();

signals:
  void AddMsg(QString msg);

private slots:
  void OnAddMsg(QString msg);

private:
  ignition::transport::Node node_;
  ignition::transport::Node::Publisher hmi_button_pub_;
  ignition::transport::Node::Publisher create3_button_pub_;

  std::string namespace_ = "";
  std::string hmi_button_topic_ = "/hmi/buttons";
  std::string create3_button_topic_ = "/create3_buttons";
  std::string display_topic_ = "/hmi/display/raw";
  std::string display_selected_topic_ = "/hmi/display/selected";
  std::string power_led_topic_ = "/hmi/led/power";
  std::string motors_led_topic_ = "/hmi/led/motors";
  std::string comms_led_topic_ = "/hmi/led/comms";
  std::string wifi_led_topic_ = "/hmi/led/wifi";
  std::string battery_led_topic_ = "/hmi/led/battery";
  std::string user1_led_topic_ = "/hmi/led/user1";
  std::string user2_led_topic_ = "/hmi/led/user2";

  QStringListModel display_list_;

  const unsigned int num_lines_{6u};
  const unsigned int char_per_line_{18u};
  const unsigned int char_per_line_header_{21u};
  int selected_line_{0u};

  std::mutex raw_msg_mutex_;
  std::mutex selected_msg_mutex_;
};

}  // namespace gui

}  // namespace ignition

#endif  // TURTLEBOT4_IGNITION_GUI_PLUGINS__TURTLEBOT4HMI__TURTLEBOT4HMI_HH_
