
#include <eigen_conversions/eigen_msg.h>
#include <mavros/frame_tf.h>
#include <mavros/utils.h>
//#include <tf_conversions/tf_eigen.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>

#include "transparent_transmission_api/plugin_base.hpp"

using mavlink::common::MAV_CMD;
using mavlink::common::MAV_RESULT;
using mavros::utils::enum_value;

// Similar to plugins/command.cpp

namespace icg_plugins {

/**
 * @brief Command plugin.
 *
 * Converts COMMAND_INT and COMMAND_LONG messages to actual commands.
 */
class CommandPlugin : public icg_plugins::PluginBase {
 public:
  CommandPlugin() : PluginBase(), nh("~") {}

  void initialize(const std::shared_ptr<SendCommand> &sendComm) {
    PluginBase::initialize(sendComm);
  }

  Subscriptions get_subscriptions() {
    return {make_handler(&CommandPlugin::handleCommandInt),
            make_handler(&CommandPlugin::handleCommandLong)};
  }

 private:
  ros::NodeHandle nh;

  /**
   * @brief handleCommandInt
   * @param msg
   * @param mavCommand http://mavlink.org/messages/common#COMMAND_INT
   */
  void handleCommandInt(const mavlink::mavlink_message_t *msg,
                        mavlink::common::msg::COMMAND_INT &mavCommandInt) {}

  /**
   * @brief handleCommandLong
   * @param msg
   * @param mavCommandLong http://mavlink.org/messages/common#COMMAND_LONG
   */
  void handleCommandLong(const mavlink::mavlink_message_t *msg,
                         mavlink::common::msg::COMMAND_LONG &mavCommandLong) {
    // TODO: check if yaw needs frame conversion

    switch (mavCommandLong.command) {
      case enum_value(MAV_CMD::DO_SET_HOME): {
        if (mavCommandLong.param1) {
          // Use current location as home
          // TODO: Get current position
        } else {
          float latitude = mavCommandLong.param5;
          float longitude = mavCommandLong.param6;
          float altitude = mavCommandLong.param7;
        }
        ROS_INFO("Received MAV_CMD::DO_SET_HOME.");
        // TODO
        break;
      }

      case enum_value(MAV_CMD::NAV_TAKEOFF): {
        float minPitch = mavCommandLong.param1;
        float yaw = mavCommandLong.param4;
        float latitude = mavCommandLong.param5;
        float longitude = mavCommandLong.param6;
        float altitude = mavCommandLong.param7;
        ROS_INFO("Received MAV_CMD::NAV_TAKEOFF.");
        // TODO
        break;
      }
      case enum_value(MAV_CMD::NAV_TAKEOFF_LOCAL): {
        float minPitch = mavCommandLong.param1;
        float ascendRate = mavCommandLong.param3;
        float yaw = mavCommandLong.param4;
        float yAxis = mavCommandLong.param5;
        float xAxis = mavCommandLong.param6;
        float zAxis = mavCommandLong.param7;

        ROS_INFO("Received MAV_CMD::NAV_TAKEOFF_LOCAL.");
        // TODO
        break;
      }
      case enum_value(MAV_CMD::NAV_LAND): {
        ROS_INFO("Received MAV_CMD::NAV_LAND.");
        // TODO
        break;
      }
    }
  }

  void sendCommandAck(uint16_t command, uint8_t result) {
    mavlink::common::msg::COMMAND_ACK ack;
    ack.command = command;
    ack.result = result;
    // It has additional parameters, but they are WIP and I think unnecesary
    // TODO send
    sendCommand->sendMavlinkMsgToRemoteDevice(ack);
  }
};
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(icg_plugins::CommandPlugin, icg_plugins::PluginBase)
