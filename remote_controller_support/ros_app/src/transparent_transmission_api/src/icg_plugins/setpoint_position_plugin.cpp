
#include <eigen_conversions/eigen_msg.h>
#include <mavros/frame_tf.h>
#include <mavros/utils.h>

//#include <tf_conversions/tf_eigen.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>

#include "transparent_transmission_api/plugin_base.hpp"

// Similar to plugins/setpoint_raw.cpp -> get setpoint
// and plugins/setpoint_position.cpp -> set setpoint

using mavlink::common::MAV_FRAME;
using mavros::utils::enum_value;

namespace icg_plugins {

/**
 * @brief Setpoint position plugin.
 *
 * Converts SET_POSITION_TARGET_LOCAL_NED to PoseStamped.
 */
class SetpointPositionPlugin : public icg_plugins::PluginBase {
 public:
  SetpointPositionPlugin() : PluginBase(), nh("~") {}

  void initialize(const std::shared_ptr<SendCommand> &sendComm) {
    PluginBase::initialize(sendComm);
  }

  Subscriptions get_subscriptions() {
    return {
        make_handler(&SetpointPositionPlugin::handleSetPositionTargetLocalNed),
        make_handler(
            &SetpointPositionPlugin::handleSetPositionTargetGlobalInt)};
  }

 private:
  ros::NodeHandle nh;

  /**
   * @brief handleSetPositionTargetLocalNed
   * @param msg
   * @param mavLocalSetpoint
   * http://mavlink.org/messages/common#SET_POSITION_TARGET_LOCAL_NED
   */
  void handleSetPositionTargetLocalNed(
      const mavlink::mavlink_message_t *msg,
      mavlink::common::msg::SET_POSITION_TARGET_LOCAL_NED &mavLocalSetpoint) {
    // TODO: We perhaps have to check coordinate frame
    // mavlink::common::MAV_FRAME

    // Conversion reference: https://github.com/mavlink/mavros/issues/49
    // Conversion example:
    // https://github.com/mavlink/mavros/blob/master/mavros/src/plugins/home_position.cpp

    double yaw;
    float yaw_rate;
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d accelerationOfForce;

    switch (mavLocalSetpoint.coordinate_frame) {
      case enum_value(MAV_FRAME::LOCAL_NED): {
        yaw = mavros::ftf::transform_frame_yaw_ned_enu(mavLocalSetpoint.yaw);
        yaw_rate =
            mavros::ftf::transform_frame_yaw_ned_enu(mavLocalSetpoint.yaw_rate);
        position = mavros::ftf::transform_frame_ned_enu(Eigen::Vector3d(
            mavLocalSetpoint.x, mavLocalSetpoint.y, mavLocalSetpoint.z));
        velocity = mavros::ftf::transform_frame_ned_enu(Eigen::Vector3d(
            mavLocalSetpoint.vx, mavLocalSetpoint.vy, mavLocalSetpoint.vz));
        accelerationOfForce = mavros::ftf::transform_frame_ned_enu(
            Eigen::Vector3d(mavLocalSetpoint.afx, mavLocalSetpoint.afy,
                            mavLocalSetpoint.afz));
        break;
      }
    }

    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.frame_id = "something";
    poseStamped.header.stamp = ros::Time::now();
    poseStamped.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

    tf::pointEigenToMsg(position, poseStamped.pose.position);

    ROS_INFO("Received SET_POSITION_TARGET_LOCAL_NED.");
    ROS_INFO("Position: %f %f %f, Yaw: %f", poseStamped.pose.position.x,
             poseStamped.pose.position.y, poseStamped.pose.position.z, yaw);

    // TODO: Do something with the poseStamped
  }

  /**
   * @brief handleSetPositionTargetGlobalInt
   * @param msg
   * @param mavGlobalSetpoint
   * http://mavlink.org/messages/common#SET_POSITION_TARGET_GLOBAL_INT
   */
  void handleSetPositionTargetGlobalInt(
      const mavlink::mavlink_message_t *msg,
      mavlink::common::msg::SET_POSITION_TARGET_GLOBAL_INT &mavGlobalSetpoint) {
    double yaw =
        mavros::ftf::transform_frame_yaw_ned_enu(mavGlobalSetpoint.yaw);
    double latitude = mavGlobalSetpoint.lat_int / 1e7;
    double longitude = mavGlobalSetpoint.lon_int / 1e7;

    // TODO: What is the ROS message for GPS?
  }
};
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(icg_plugins::SetpointPositionPlugin,
                       icg_plugins::PluginBase)
