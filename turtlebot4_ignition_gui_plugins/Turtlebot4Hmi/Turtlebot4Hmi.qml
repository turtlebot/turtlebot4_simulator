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
 
import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.1
import QtQuick.Controls.Styles 1.4
import QtQuick.Layouts 1.3
import QtQuick.Extras 1.4
import "qrc:/qml"

Rectangle 
{
  id: widgetRectangle
  color: "white"
  anchors.fill: parent
  focus: true
  Layout.minimumWidth: 400
  Layout.minimumHeight: 575

  Connections {
      target: Turtlebot4Hmi

      onSetPowerState: {
        ledPower.active = state
        ledPower.color = color
      }

      onSetMotorsState: {
        ledMotors.active = state
        ledMotors.color = color
      }

      onSetCommsState: {
        ledComms.active = state
        ledComms.color = color
      }

      onSetWifiState: {
        ledWifi.active = state
        ledWifi.color = color
      }

      onSetBatteryState: {
        ledBattery.active = state
        ledBattery.color = color
      }

      onSetUser1State: {
        ledUser1.active = state
        ledUser1.color = color
      }

      onSetUser2State: {
        ledUser2.active = state
        ledUser2.color = color
      }
    }

  Rectangle
  {
    id: namespaceRectangle
    border.color: "black"
    border.width: 2
    anchors.top: widgetRectangle.top
    anchors.left: widgetRectangle.left
    focus: true
    height: 75
    width: 400
    // Robot namespace input
    Label {
      id: namespaceLabel
      text: "Namespace:"
      Layout.fillWidth: true
      Layout.margins: 10
      anchors.top: namespaceRectangle.top
      anchors.topMargin: 10
      anchors.left: namespaceRectangle.left
      anchors.leftMargin: 10
    }

    TextField {
      id: nameField
      width: 175
      Layout.fillWidth: true
      Layout.margins: 10
      text: Turtlebot4Hmi.namespace
      placeholderText: qsTr("Robot namespace")
      anchors.top: namespaceLabel.bottom
      anchors.topMargin: 5
      anchors.left: namespaceRectangle.left
      anchors.leftMargin: 10
      onEditingFinished: {
        Turtlebot4Hmi.SetNamespace(text)
      }
    }
  }

  Rectangle
  {
    id: create3ButtonsRectangle
    border.color: "black"
    border.width: 2
    anchors.top: namespaceRectangle.bottom
    anchors.left: namespaceRectangle.left
    focus: true
    height: 125
    width: 400

    // Buttons
    ToolButton {
      id: create3Button1
      anchors.verticalCenter: create3ButtonPower.verticalCenter
      anchors.right: create3ButtonPower.left
      anchors.rightMargin: 15
      checkable: true
      checked: true
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "images/One Dot.png"
        sourceSize.width: 36;
        sourceSize.height: 36;
      }
      onPressed: { Turtlebot4Hmi.OnCreate3Button(1); }
      onReleased: { Turtlebot4Hmi.OnCreate3Button(0); }
    }

    ToolButton {
      id: create3ButtonPower
      anchors.bottom: create3ButtonsRectangle.bottom
      anchors.bottomMargin: 15
      anchors.horizontalCenter: create3ButtonsRectangle.horizontalCenter
      checkable: true
      checked: true
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "images/Power.png"
        sourceSize.width: 72;
        sourceSize.height: 72;
      }
      onPressed: { Turtlebot4Hmi.OnCreate3Button(2); }
      onReleased: { Turtlebot4Hmi.OnCreate3Button(0); }
    }

    ToolButton {
      id: create3Button2
      anchors.verticalCenter: create3ButtonPower.verticalCenter
      anchors.left: create3ButtonPower.right
      anchors.leftMargin: 15
      checkable: true
      checked: true
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "images/Two Dots.png"
        sourceSize.width: 36;
        sourceSize.height: 36;
      }
      onPressed: { Turtlebot4Hmi.OnCreate3Button(3); }
      onReleased: { Turtlebot4Hmi.OnCreate3Button(0); }
    }
  }
  
  // HMI
  Rectangle
  {
    id: hmiRectangle
    height: 325
    width: 400
    color: "transparent"
    border.color: "black"
    border.width: 2
    anchors.top: create3ButtonsRectangle.bottom
    anchors.left: create3ButtonsRectangle.left

    StatusIndicator {
      id: ledPower
      anchors.top: hmiRectangle.top
      anchors.topMargin: 10
      anchors.right: ledMotors.left
      anchors.rightMargin: 35
      color: "green"
      active: false
    }

    Label {
      id: ledPowerLabel
      text: "PWR"
      font.pixelSize: 10
      anchors.top: ledPower.bottom
      anchors.topMargin: 5
      anchors.horizontalCenter: ledPower.horizontalCenter
    }

    StatusIndicator {
      id: ledMotors
      anchors.top: hmiRectangle.top
      anchors.topMargin: 10
      anchors.right: ledComms.left
      anchors.rightMargin: 35
      color: "green"
      active: false
    }

    Label {
      id: ledMotorsLabel
      text: "MOTOR"
      font.pixelSize: 10
      anchors.top: ledMotors.bottom
      anchors.topMargin: 5
      anchors.horizontalCenter: ledMotors.horizontalCenter
    }

    StatusIndicator {
      id: ledComms
      anchors.top: hmiRectangle.top
      anchors.topMargin: 10
      anchors.horizontalCenter: hmiRectangle.horizontalCenter
      color: "green"
      active: false
    }

    Label {
      id: ledCommsLabel
      text: "COMM"
      font.pixelSize: 10
      anchors.top: ledComms.bottom
      anchors.topMargin: 5
      anchors.horizontalCenter: ledComms.horizontalCenter
    }

    StatusIndicator {
      id: ledWifi
      anchors.top: hmiRectangle.top
      anchors.topMargin: 10
      anchors.left: ledComms.right
      anchors.leftMargin: 35
      color: "green"
      active: false
    }

    Label {
      id: ledWifiLabel
      text: "WIFI"
      font.pixelSize: 10
      anchors.top: ledWifi.bottom
      anchors.topMargin: 5
      anchors.horizontalCenter: ledWifi.horizontalCenter
    }

    StatusIndicator {
      id: ledBattery
      anchors.top: hmiRectangle.top
      anchors.topMargin: 10
      anchors.left: ledWifi.right
      anchors.leftMargin: 35
      color: "green"
      active: false
    }

    Label {
      id: ledBatteryLabel
      text: "BAT"
      font.pixelSize: 10
      anchors.top: ledBattery.bottom
      anchors.topMargin: 5
      anchors.horizontalCenter: ledBattery.horizontalCenter
    }

    StatusIndicator {
      id: ledUser1
      anchors.top: ledCommsLabel.bottom
      anchors.topMargin: 10
      anchors.right: ledComms.left
      anchors.rightMargin: 15
      color: "green"
      active: false
    }

    Label {
      id: ledUser1Label
      text: "USER1"
      font.pixelSize: 10
      anchors.top: ledUser1.bottom
      anchors.topMargin: 5
      anchors.horizontalCenter: ledUser1.horizontalCenter
    }

    StatusIndicator {
      id: ledUser2
      anchors.top: ledCommsLabel.bottom
      anchors.topMargin: 10
      anchors.left: ledComms.right
      anchors.leftMargin: 15
      color: "green"
      active: false
    }

    Label {
      id: ledUser2Label
      text: "USER2"
      font.pixelSize: 10
      anchors.top: ledUser2.bottom
      anchors.topMargin: 5
      anchors.horizontalCenter: ledUser2.horizontalCenter
    }
    

    Rectangle
    {
      id: listViewBorder
      height: 150
      width: 250
      border.color: "black"
      border.width: 2
      anchors.horizontalCenter: hmiRectangle.horizontalCenter
      anchors.top: ledUser2Label.bottom
      anchors.topMargin: 20

      ListView {
          id: listView
          clip: true
          anchors.fill: listViewBorder
          anchors.topMargin: 5
          anchors.leftMargin: 5

          focus: true
          currentIndex: -1

          delegate: Text {
            font.pixelSize: 20
            text: display
          }

          model: DisplayListView
      }
    }

    Rectangle
    {
      id: headerBox
      height: 30
      width: 250
      color: "transparent"
      border.color: "black"
      border.width: 2
      anchors.horizontalCenter: hmiRectangle.horizontalCenter
      anchors.top: listViewBorder.top
      anchors.topMargin: 0
    }

    Button {
      id: hmiButton1
      text: qsTr("1")
      highlighted: false
      onPressed: { Turtlebot4Hmi.OnHmiButton(1); }
      onReleased: { Turtlebot4Hmi.OnHmiButton(0); }
      anchors.top: listViewBorder.top
      anchors.topMargin: 10
      anchors.right: listViewBorder.left
      anchors.rightMargin: 15

      background: Rectangle {
        implicitWidth: 40
        implicitHeight: 40
        opacity: 0.3
        border.color: "#000000"
        border.width: control.down ? 2 : 1
        radius: 20
        color: "gray"
      }
    }

    Button {
      id: hmiButton2
      text: qsTr("2")
      highlighted: false
      onPressed: { Turtlebot4Hmi.OnHmiButton(2); }
      onReleased: { Turtlebot4Hmi.OnHmiButton(0); }
      anchors.top: hmiButton1.top
      anchors.topMargin: 80
      anchors.right: listViewBorder.left
      anchors.rightMargin: 15

      background: Rectangle {
        implicitWidth: 40
        implicitHeight: 40
        opacity: 0.3
        border.color: "#000000"
        border.width: control.down ? 2 : 1
        radius: 20
        color: "gray"
      }
    }

    Button {
      id: hmiButton3
      text: qsTr("3")
      highlighted: false
      onPressed: { Turtlebot4Hmi.OnHmiButton(3); }
      onReleased: { Turtlebot4Hmi.OnHmiButton(0); }
      anchors.top: listViewBorder.top
      anchors.topMargin: 10
      anchors.left: listViewBorder.right
      anchors.leftMargin: 15

      background: Rectangle {
        implicitWidth: 40
        implicitHeight: 40
        opacity: 0.3
        border.color: "#000000"
        border.width: control.down ? 2 : 1
        radius: 20
        color: "gray"
      }
    }

    Button {
      id: hmiButton4
      text: qsTr("4")
      highlighted: false
      onPressed: { Turtlebot4Hmi.OnHmiButton(4); }
      onReleased: { Turtlebot4Hmi.OnHmiButton(0); }
      anchors.top: hmiButton3.top
      anchors.topMargin: 80
      anchors.left: listViewBorder.right
      anchors.leftMargin: 15

      background: Rectangle {
        implicitWidth: 40
        implicitHeight: 40
        opacity: 0.3
        border.color: "#000000"
        border.width: control.down ? 2 : 1
        radius: 20
        color: "gray"
      }
    }
  }
}

