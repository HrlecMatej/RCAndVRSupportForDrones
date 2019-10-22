
#pragma once

#include <ros/ros.h>

#include <image_transport/image_transport.h>

#include <dji_sdk/MobileData.h>
#include <dji_sdk/SendMobileData.h>
//#include <dji_sdk/dji_sdk.h>

#include "std_msgs/String.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt8MultiArray.h"

#include <mavconn/interface.h>
#include <mavconn/serial.h>
#include <mavconn/tcp.h>
#include <mavconn/udp.h>

#define DJI_PACKET_SIZE 100

class SendCommand {
  // Inherited from TransparentTransmissionAPI
  ros::NodeHandle nodeHandle;
  mavconn::MAVConnInterface::Ptr gcsTcpConnection;
  std::shared_ptr<mavlink::mavlink_status_t> mavlinkStatus;

  // Maximum buffer size with padding for CRC bytes (280 + padding)
  static constexpr size_t MAX_SIZE = MAVLINK_MAX_PACKET_LEN + 16;
  uint8_t packetData[MAX_SIZE];

  /// DECLARE SERVICES
  ros::ServiceClient serviceSendDataToRemoteDevice;

 public:
  SendCommand(const ros::NodeHandle &nh,
              const mavconn::MAVConnInterface::Ptr &gcsConn,
              const std::shared_ptr<mavlink::mavlink_status_t> &status);

  bool sendDataToRemoteDevice(const std::vector<uint8_t> &dataToSend);

  bool sendMavlinkMsgToRemoteDevice(const mavlink::Message &msgToSend);

  void sendHeartbeat();
};
