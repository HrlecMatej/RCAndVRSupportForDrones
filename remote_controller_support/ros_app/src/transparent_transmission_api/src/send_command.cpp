
#include "transparent_transmission_api/send_command.hpp"

/*void SendCommand::sendHeartbeat() {
  mavlink::common::msg::HEARTBEAT hb;
  hb.type = int(mavlink::common::MAV_TYPE::QUADROTOR);
  hb.autopilot = int(mavlink::common::MAV_AUTOPILOT::GENERIC);
  hb.base_mode = int(mavlink::common::MAV_MODE::GUIDED_DISARMED);
  hb.custom_mode = 0;
  hb.system_status = int(mavlink::common::MAV_STATE::STANDBY);
  hb.mavlink_version = mavlink::common::MAVLINK_VERSION;

  gcsTcpConnection->send_message(hb);
}*/

bool SendCommand::sendDataToRemoteDevice(
    const std::vector<uint8_t> &dataToSend) {
  std::vector<std::vector<uint8_t>> dataChunks;
  dataChunks.reserve((dataToSend.size() + DJI_PACKET_SIZE - 1) /
                     DJI_PACKET_SIZE);

  auto start = dataToSend.begin();
  auto end = dataToSend.end();

  while (start != end) {
    auto next = std::distance(start, end) <= DJI_PACKET_SIZE
                    ? end
                    : start + DJI_PACKET_SIZE;

    dataChunks.emplace_back(start, next);
    start = next;
  }

  bool successful = true;

  for (std::vector<uint8_t> data : dataChunks) {
    dji_sdk::SendMobileData msgDataToSend;
    msgDataToSend.request.data = data;

    successful = successful &&
                 // Does not actually tell you, if you're successful...
                 serviceSendDataToRemoteDevice.call(msgDataToSend) &&
                 msgDataToSend.response.result;
  }

  return successful;
}

bool SendCommand::sendMavlinkMsgToRemoteDevice(
    const mavlink::Message &msgToSend) {
  mavlink::mavlink_message_t mavlinkMsg;
  mavlink::MsgMap msgMap(mavlinkMsg);

  auto msgInfo = msgToSend.get_message_info();

  msgToSend.serialize(msgMap);
  mavlink::mavlink_finalize_message_buffer(
      &mavlinkMsg, 1, mavconn::MAV_COMP_ID_UDP_BRIDGE, mavlinkStatus.get(),
      msgInfo.min_length, msgInfo.length, msgInfo.crc_extra);

  uint16_t length =
      mavlink::mavlink_msg_to_send_buffer(packetData, &mavlinkMsg);

  gcsTcpConnection->send_message(&mavlinkMsg);

  std::vector<uint8_t> bytes(packetData, packetData + length);

  /*ROS_INFO("Sending message with msgid: %d, length: %d", mavlinkMsg.msgid,
           length);*/

  return sendDataToRemoteDevice(bytes);
}

// Initializes Receive and Send Command Objects
SendCommand::SendCommand(
    const ros::NodeHandle &nh, const mavconn::MAVConnInterface::Ptr &gcsConn,
    const std::shared_ptr<mavlink::mavlink_status_t> &status)
    : nodeHandle(nh), gcsTcpConnection(gcsConn), mavlinkStatus(status) {
  /// INITIALIZE SERVICES
  serviceSendDataToRemoteDevice =
      nodeHandle.serviceClient<dji_sdk::SendMobileData>("dji_sdk/send_data_to_mobile");

  ROS_INFO("Transparent Transmission API started.");
}
