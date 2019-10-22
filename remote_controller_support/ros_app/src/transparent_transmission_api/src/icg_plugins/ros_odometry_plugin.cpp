
#include <eigen_conversions/eigen_msg.h>
#include <mavros/frame_tf.h>
#include <mavros/utils.h>
//#include <tf_conversions/tf_eigen.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>

#include "transparent_transmission_api/plugin_base.hpp"

using mavlink::common::MAV_CMD;
using mavlink::common::MAV_RESULT;
using mavros::utils::enum_value;

namespace icg_plugins {

/**
 * @brief Ros odometry plugin.
 *
 * Sends out odometry data.
 */
class RosOdometryPlugin : public icg_plugins::PluginBase {
 public:
  RosOdometryPlugin() : PluginBase(), nh() {}

  void initialize(const std::shared_ptr<SendCommand> &sendComm) {
    PluginBase::initialize(sendComm);

    rosOdometrySub = nh.subscribe("dji_sdk/odometry", 1,
                                &RosOdometryPlugin::odometryCallback, this);

    // Create a non-blocking thread (for testing purposes)
    // std::thread(&RosOdometryPlugin::sendRosOdometry, this).detach();
  }

  Subscriptions get_subscriptions() { return {}; }

 private:
  ros::Subscriber rosOdometrySub;

  const size_t ROSODOMETRY_RATE_MILISECS = 250;

  ros::NodeHandle nh;

  // Receive message and send to tablet
  void odometryCallback(const nav_msgs::OdometryConstPtr &p_rmsg) {
      static ros::Time t_last_pub = p_rmsg->header.stamp;
      if ((p_rmsg->header.stamp - t_last_pub).toSec()*1000 < ROSODOMETRY_RATE_MILISECS)
          return;

      mavlink::dji_icg::msg::ROS_ODOMETRY msg;
      msg.target_system = GROUND_STATION_ID;
      msg.sec = p_rmsg->header.stamp.sec;
      msg.nsec = p_rmsg->header.stamp.nsec;

      { const geometry_msgs::Point &pos = p_rmsg->pose.pose.position;
        msg.position = {pos.x, pos.y, pos.z};
      }
      { const geometry_msgs::Quaternion &q = p_rmsg->pose.pose.orientation;
        msg.orientation = {q.x, q.y, q.z, q.w};
      }
      { const geometry_msgs::Vector3 &v = p_rmsg->twist.twist.linear;
        msg.linear = {v.x, v.y, v.z};
      }
      { const geometry_msgs::Vector3 &w = p_rmsg->twist.twist.angular;
        msg.angular = {w.x, w.y, w.z};
      }

      sendCommand->sendMavlinkMsgToRemoteDevice(msg);
      t_last_pub = p_rmsg->header.stamp;
  }

  void sendRosOdometry() {
    // std::this_thread::sleep_for(std::chrono::milliseconds(500));
    while (true) {
      std::this_thread::sleep_for(
          std::chrono::milliseconds(ROSODOMETRY_RATE_MILISECS));

      mavlink::dji_icg::msg::ROS_ODOMETRY msg;
      msg.target_system = GROUND_STATION_ID;

      msg.position = {1.0 * rand(), 2.0 * rand(), 3.0 * rand()};
      msg.orientation = {4.0 * rand(), 5.0 * rand(), 6.0 * rand(),
                         7.0 * rand()};

      msg.linear = {8.0 * rand(), 9.0 * rand(), 10.0 * rand()};
      msg.angular = {11.0 * rand(), 12.0 * rand(), 13.0 * rand()};

      ros::Time time;
      msg.sec = time.sec;
      msg.nsec = time.nsec;

      sendCommand->sendMavlinkMsgToRemoteDevice(msg);

      /*mavlink::common::msg::HEARTBEAT hb;
      hb.type = int(mavlink::common::MAV_TYPE::QUADROTOR);
      hb.autopilot = int(mavlink::common::MAV_AUTOPILOT::GENERIC);
      hb.base_mode = int(mavlink::common::MAV_MODE::GUIDED_DISARMED);
      hb.custom_mode = 0;
      hb.system_status = int(mavlink::common::MAV_STATE::STANDBY);
      hb.mavlink_version = mavlink::common::MAVLINK_VERSION;

      sendCommand->sendMavlinkMsgToRemoteDevice(hb);*/
    }
  }
};
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(icg_plugins::RosOdometryPlugin, icg_plugins::PluginBase)
