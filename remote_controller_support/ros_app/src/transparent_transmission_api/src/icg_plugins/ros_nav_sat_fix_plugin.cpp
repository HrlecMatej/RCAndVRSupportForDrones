
#include <eigen_conversions/eigen_msg.h>
#include <mavros/frame_tf.h>
#include <mavros/utils.h>
//#include <tf_conversions/tf_eigen.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
//#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>

#include "transparent_transmission_api/plugin_base.hpp"

using mavros::utils::enum_value;

namespace icg_plugins {

/**
 * @brief Ros NavSatFix plugin.
 *
 * Sends out NavSatFix positions.
 */
class RosNavSatFixPlugin : public icg_plugins::PluginBase {
 public:
  RosNavSatFixPlugin() : PluginBase(), nh() {}

  void initialize(const std::shared_ptr<SendCommand> &sendComm) {
    PluginBase::initialize(sendComm);

    navSatFixSub = nh.subscribe("dji_sdk/gps_position", 1,
                                &RosNavSatFixPlugin::navSatFixCallback, this);

    // Create a non-blocking thread (for testing purposes)
    // std::thread(&RosNavSatFixPlugin::sendNavSatFix, this).detach();
  }

  Subscriptions get_subscriptions() { return {}; }

 private:
  ros::Subscriber navSatFixSub;

  const size_t NAVSATFIX_RATE_MILISECS = 1000;

  ros::NodeHandle nh;

  // Receive message and send to tablet
  void navSatFixCallback(const sensor_msgs::NavSatFixConstPtr &p_rmsg) {
      static ros::Time t_last_pub = p_rmsg->header.stamp;
      if ((p_rmsg->header.stamp - t_last_pub).toSec()*1000 < NAVSATFIX_RATE_MILISECS)
          return;

      mavlink::dji_icg::msg::ROS_NAV_SAT_FIX msg;
      msg.target_system = GROUND_STATION_ID;
      msg.sec = p_rmsg->header.stamp.sec;
      msg.nsec = p_rmsg->header.stamp.nsec;

      msg.latitude = p_rmsg->latitude;
      msg.longitude = p_rmsg->longitude;
      msg.altitude = p_rmsg->altitude;

      sendCommand->sendMavlinkMsgToRemoteDevice(msg);
      t_last_pub = p_rmsg->header.stamp;
  }

  // Functions created to fake received data for testing purposes
  void sendNavSatFix() {
    while (true) {
      std::this_thread::sleep_for(
          std::chrono::milliseconds(NAVSATFIX_RATE_MILISECS));

      mavlink::dji_icg::msg::ROS_NAV_SAT_FIX msg;
      msg.target_system = GROUND_STATION_ID;

      msg.latitude = 1.0 * rand();
      msg.longitude = 2.0 * rand();
      msg.altitude = 3.0 * rand();

      ros::Time time;
      msg.sec = time.sec;
      msg.nsec = time.nsec;

      sendCommand->sendMavlinkMsgToRemoteDevice(msg);
    }
  }
};
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(icg_plugins::RosNavSatFixPlugin, icg_plugins::PluginBase)
