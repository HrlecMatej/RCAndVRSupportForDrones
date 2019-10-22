
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

namespace icg_plugins {

/**
 * @brief Heartbeat plugin.
 *
 * Sends out heartbeats every second.
 */
class HeartbeatPlugin : public icg_plugins::PluginBase {
 public:
  HeartbeatPlugin() : PluginBase(), nh("~") {}

  void initialize(const std::shared_ptr<SendCommand> &sendComm) {
    PluginBase::initialize(sendComm);

    // Create a non-blocking thread
    // std::thread(&HeartbeatPlugin::sendHeartbeat, this).detach();
  }

  Subscriptions get_subscriptions() { return {}; }

 private:
  const size_t HEARTBEAT_MILISECS = 1000;

  ros::NodeHandle nh;

  void sendHeartbeat() {
    while (true) {
      std::this_thread::sleep_for(
          std::chrono::milliseconds(HEARTBEAT_MILISECS));

      mavlink::common::msg::HEARTBEAT hb;
      hb.type = int(mavlink::common::MAV_TYPE::QUADROTOR);
      hb.autopilot = int(mavlink::common::MAV_AUTOPILOT::GENERIC);
      hb.base_mode = int(mavlink::common::MAV_MODE::GUIDED_DISARMED);
      hb.custom_mode = 0;
      hb.system_status = int(mavlink::common::MAV_STATE::STANDBY);
      hb.mavlink_version = mavlink::common::MAVLINK_VERSION;

      sendCommand->sendMavlinkMsgToRemoteDevice(hb);
    }
  }
};
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(icg_plugins::HeartbeatPlugin, icg_plugins::PluginBase)
