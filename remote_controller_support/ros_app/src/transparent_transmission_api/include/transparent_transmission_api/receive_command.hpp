
#pragma once

#include <ros/ros.h>

#include <image_transport/image_transport.h>

#include <dji_sdk/MobileData.h>

#include "std_msgs/String.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt8MultiArray.h"

#include <mavconn/interface.h>
#include <mavconn/serial.h>
#include <mavconn/tcp.h>
#include <mavconn/udp.h>

#include <pluginlib/class_loader.h>

#include "plugin_base.hpp"
#include "send_command.hpp"

#include <mavros/utils.h>

using mavros::utils::enum_value;

class ReceiveCommand {
  // Inherited from TransparentTransmissionAPI
  ros::NodeHandle nodeHandle;
  mavconn::MAVConnInterface::Ptr gcsTcpConnection;
  std::shared_ptr<mavlink::mavlink_status_t> mavlinkStatus;

  std::shared_ptr<SendCommand> sendCommand;

  /// PLUGIN STUFF
  pluginlib::ClassLoader<icg_plugins::PluginBase> plugin_loader;
  std::vector<icg_plugins::PluginBase::Ptr> loaded_plugins;

  std::unordered_map<mavlink::msgid_t, icg_plugins::PluginBase::Subscriptions>
      plugin_subscriptions;

  void handleWithPlugins(const mavlink::mavlink_message_t *mmsg,
                         const mavconn::Framing framing);

  void addPlugin(std::string &pl_name);

  /// DECLARE SUBSCRIBERS
  ros::Subscriber subDataReceivedFromRemoteDevice;

  /// DECLARE CALLBACKS
  void subDataReceivedFromRemoteDeviceCallback(
      const dji_sdk::MobileDataConstPtr &mobileData);

 public:
  // TransparentTransmissionAPI(ros::NodeHandle &nodeHandle);
  ReceiveCommand(const ros::NodeHandle &nh,
                 const mavconn::MAVConnInterface::Ptr &gcsConn,
                 const std::shared_ptr<SendCommand> &sendComm,
                 const std::shared_ptr<mavlink::mavlink_status_t> &status);

  bool sendDataToRemoteDevice(std::vector<unsigned char> &dataToSend);
};
