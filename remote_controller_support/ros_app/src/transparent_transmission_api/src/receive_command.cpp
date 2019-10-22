
#include "transparent_transmission_api/receive_command.hpp"

inline bool is_mavlink_message_t(const size_t rt) {
  static const auto h = typeid(mavlink::mavlink_message_t).hash_code();
  return h == rt;
}

void ReceiveCommand::addPlugin(std::string &pl_name) {
  try {
    auto plugin = plugin_loader.createInstance(pl_name);

    for (auto &info : plugin->get_subscriptions()) {
      auto msgid = std::get<0>(info);
      auto msgname = std::get<1>(info);
      auto type_hash_ = std::get<2>(info);

      if (is_mavlink_message_t(type_hash_))
        ROS_INFO("MSG-ID (%u) <%zu>", msgid, type_hash_);
      else
        ROS_INFO("%s (%u) <%zu>", msgname, msgid, type_hash_);

      auto it = plugin_subscriptions.find(msgid);
      if (it == plugin_subscriptions.end()) {
        // new entry
        ROS_DEBUG_STREAM(msgname << " - new element");
        plugin_subscriptions[msgid] =
            icg_plugins::PluginBase::Subscriptions{{info}};
      } else {
        // existing: check handler message type

        bool append_allowed = is_mavlink_message_t(type_hash_);
        if (!append_allowed) {
          append_allowed = true;
          for (auto &e : it->second) {
            auto t2 = std::get<2>(e);
            if (!is_mavlink_message_t(t2) && t2 != type_hash_) {
              ROS_ERROR_STREAM(msgname
                               << " routed to different message type (hash: "
                               << t2 << ")");
              append_allowed = false;
            }
          }
        }

        if (append_allowed) {
          ROS_DEBUG_STREAM(msgname << " - emplace");
          it->second.emplace_back(info);
        } else
          ROS_ERROR_STREAM(msgname << " handler dropped because this ID "
                                      "are used for another message type");
      }
    }

    plugin->initialize(sendCommand);
    loaded_plugins.push_back(plugin);

    ROS_INFO_STREAM("Plugin " << pl_name << " initialized");
  } catch (pluginlib::PluginlibException &ex) {
    ROS_ERROR_STREAM("Plugin " << pl_name << " load exception: " << ex.what());
  }
}

void ReceiveCommand::handleWithPlugins(const mavlink::mavlink_message_t *mmsg,
                                       const mavconn::Framing framing) {
  auto it = plugin_subscriptions.find(mmsg->msgid);
  if (it == plugin_subscriptions.end()) return;

  for (auto &info : it->second) {
    std::get<3>(info)(mmsg, framing);
  }
}

void ReceiveCommand::subDataReceivedFromRemoteDeviceCallback(
    const dji_sdk::MobileDataConstPtr &mobileData) {
  std::vector<uint8_t> receivedData = mobileData->data;

  // boost::shared_ptr<mavlink::mavlink_message_t> mavlinkMsg;
  mavlink::mavlink_message_t mavlinkMsg;
  mavlinkStatus->parse_error = 0;

  uint8_t channel = mavlink::MAVLINK_COMM_0;
  uint8_t msgReceived = false;

  for (uint8_t byte : receivedData) {
    // printf("%u ", byte);
    // ROS_INFO("Parse state: %d", enum_value(mavlinkStatus->parse_state));
    msgReceived = mavlink::mavlink_parse_char(channel, byte, &mavlinkMsg,
                                              mavlinkStatus.get());
    // The parser silently fails and resets necessary things.
    // It will be prepared to receive new message.
    // I guess we shouldn't worry about the magic behind this.
  }

  if (msgReceived) {
    /*ROS_INFO(
        "Received message with ID %d, sequence: %d from component %d of "
        "system %d",
        mavlinkMsg.msgid, mavlinkMsg.seq, mavlinkMsg.compid, mavlinkMsg.sysid);
*/
    handleWithPlugins(&mavlinkMsg, mavconn::Framing::ok);
  } else {
    /*ROS_WARN(
        "Num of parse errors: %u, Num of received msgs: %u, Parse state: %d",
        mavlinkStatus->parse_error, mavlinkStatus->msg_received,
        enum_value(mavlinkStatus->parse_state));*/
  }
}

ReceiveCommand::ReceiveCommand(
    const ros::NodeHandle &nh, const mavconn::MAVConnInterface::Ptr &gcsConn,
    const std::shared_ptr<SendCommand> &sendComm,
    const std::shared_ptr<mavlink::mavlink_status_t> &status)
    : nodeHandle(nh),
      gcsTcpConnection(gcsConn),
      mavlinkStatus(status),
      sendCommand(sendComm),
      plugin_loader("transparent_transmission_api", "icg_plugins::PluginBase") {
  /// INITIALIZE PLUGIN STUFF
  // Load plugin classes added with PLUGINLIB_EXPORT_CLASS
  for (std::string &name : plugin_loader.getDeclaredClasses()) {
    addPlugin(name);
  }

  /// INITIALIZE SUBSCRIBERS
  subDataReceivedFromRemoteDevice = nodeHandle.subscribe(
      "dji_sdk/from_mobile_data", 1,
      &ReceiveCommand::subDataReceivedFromRemoteDeviceCallback, this);

  // Callback if something comes from GSC
  gcsTcpConnection->message_received_cb = [this](
      const mavlink::mavlink_message_t *msg, const mavconn::Framing framing) {
    handleWithPlugins(msg, framing);
  };
}
