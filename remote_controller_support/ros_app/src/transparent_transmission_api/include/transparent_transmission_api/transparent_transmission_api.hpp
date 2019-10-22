
#pragma once

#include <ros/ros.h>

#include <image_transport/image_transport.h>

#include "std_msgs/String.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt8MultiArray.h"

#include <mavconn/interface.h>
#include <mavconn/serial.h>
#include <mavconn/tcp.h>
#include <mavconn/udp.h>

#include "receive_command.hpp"
#include "send_command.hpp"

#define DJI_PACKET_SIZE 100

class TransparentTransmissionAPI {
  ros::NodeHandle nodeHandle;
  // image_transport::ImageTransport imageTransport;

  // Connection to the GSC (QGroundControl)
  mavconn::MAVConnInterface::Ptr gcsTcpConnection;

  std::shared_ptr<SendCommand> sendCommand;
  std::shared_ptr<ReceiveCommand> receiveCommand;

  std::shared_ptr<mavlink::mavlink_status_t> mavlinkStatus;

 public:
  // TransparentTransmissionAPI(ros::NodeHandle &nodeHandle);
  TransparentTransmissionAPI(ros::NodeHandle &nh);
};
