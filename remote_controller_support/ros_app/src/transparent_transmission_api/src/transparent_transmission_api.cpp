
#include "transparent_transmission_api/transparent_transmission_api.hpp"

TransparentTransmissionAPI::TransparentTransmissionAPI(ros::NodeHandle &nh)
    : nodeHandle(nh) {
  // We don't need this right now
  // It enables us to send and receive MAVLink messages over a internet link
  gcsTcpConnection = std::make_shared<mavconn::MAVConnTCPServer>(
      1, mavconn::MAV_COMP_ID_UDP_BRIDGE, "localhost", 5777);

  mavlinkStatus = std::make_shared<mavlink::mavlink_status_t>();
  mavlinkStatus->parse_error = 0;

  // Initializes Receive and Send Command Objects
  sendCommand =
      std::make_shared<SendCommand>(nh, gcsTcpConnection, mavlinkStatus);
  receiveCommand = std::make_shared<ReceiveCommand>(nh, gcsTcpConnection,
                                                    sendCommand, mavlinkStatus);

  ROS_INFO("Transparent Transmission API started.");
}
