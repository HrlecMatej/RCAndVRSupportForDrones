
#include <eigen_conversions/eigen_msg.h>
#include <mavros/frame_tf.h>
#include <mavros/utils.h>

//#include <tf_conversions/tf_eigen.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>

#include "transparent_transmission_api/plugin_base.hpp"

#include <nav_msgs/Odometry.h>
#include <icg_dji_common/GSPTargetLocationLocal.h>

// Similar to plugins/setpoint_raw.cpp -> get setpoint
// and plugins/setpoint_position.cpp -> set setpoint

using mavlink::dji_icg::DJI_TOPIC_NAME;
using mavros::utils::enum_value;

namespace icg_plugins {

/**
 * @brief Dji PoseStamped plugin.
 *
 * Receives positions to fly to from the mobile device.
 */
class DjiPoseStampedPlugin : public icg_plugins::PluginBase {
 public:
  DjiPoseStampedPlugin() : PluginBase(), nh(), pnh("~") {}

  void initialize(const std::shared_ptr<SendCommand> &sendComm) {
    PluginBase::initialize(sendComm);

    received_odometry_once = false;
    odom_attitude_yaw_rad = 0.; odom_attitude_pitch_rad = 0.; odom_attitude_roll_rad = 0.;
    pose_estimate_subscriber = nh.subscribe("dji_sdk/odometry", 1, &DjiPoseStampedPlugin::poseEstimateSubCallback  , this);

    target_location_local_publisher = nh.advertise<icg_dji_common::GSPTargetLocationLocal>("planner/gsp_target_location_local", 10);
  }

  Subscriptions get_subscriptions() {
    return {make_handler(&DjiPoseStampedPlugin::handleDjiPoseStamped)};
  }

 private:
  ros::NodeHandle nh, pnh;

  ros::Publisher target_location_local_publisher;
  double target_attitude_yaw_rad, target_attitude_pitch_rad, target_attitude_roll_rad;
  ros::Subscriber pose_estimate_subscriber;
  nav_msgs::Odometry last_odometry_msg; //includes the velocity estimate
  double odom_attitude_yaw_rad, odom_attitude_pitch_rad, odom_attitude_roll_rad;
  bool received_odometry_once;
  void poseEstimateSubCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    last_odometry_msg = *msg;

    // store current attitude
    {
      Eigen::Quaternion<double> att_quaternion( last_odometry_msg.pose.pose.orientation.w,
                                                last_odometry_msg.pose.pose.orientation.x,
                                                last_odometry_msg.pose.pose.orientation.y,
                                                last_odometry_msg.pose.pose.orientation.z);
      if ( att_quaternion.norm() > 0.0 ) {
        att_quaternion.normalize();
        QuaternionXYZWToEulerAngles( att_quaternion.coeffs().data(), &odom_attitude_yaw_rad, &odom_attitude_pitch_rad, &odom_attitude_roll_rad);
      }
    }

    received_odometry_once = true;
  }

  void handleDjiPoseStamped(const mavlink::mavlink_message_t *mavMsg,
                            mavlink::dji_icg::msg::DJI_POSE_STAMPED &msg) {
    // TODO: Do the action
    switch (msg.topic_name) {
      case enum_value(DJI_TOPIC_NAME::SETPOINT): {
        ROS_INFO("Received DJI_TOPIC_NAME::SETPOINT.");

        icg_dji_common::GSPTargetLocationLocal target_waypoint;

        target_waypoint.header.frame_id = "map";
        ros::Time msgTime(msg.sec, msg.nsec);
        // target_waypoint.header.stamp = msgTime;
        target_waypoint.header.stamp = ros::Time::now();

        // store target attitude
        {
          //ROS_INFO("msg.orientation[0 1 2 3]:[%f\t%f\t%f\t%f]", msg.orientation[0], msg.orientation[1], msg.orientation[2], msg.orientation[3]);

          Eigen::Quaternion<double> att_quaternion( msg.orientation[0],
                                                    msg.orientation[1],
                                                    msg.orientation[3],
                                                    msg.orientation[2]);
          att_quaternion.normalize();
          if ( att_quaternion.norm() > 0.0 ) {
            att_quaternion.normalize();
            double relative_yaw;
            QuaternionXYZWToEulerAngles( att_quaternion.coeffs().data(), &relative_yaw, &target_attitude_pitch_rad, &target_attitude_roll_rad);
            target_attitude_yaw_rad = odom_attitude_yaw_rad;
            if (relative_yaw>0) {
              target_attitude_yaw_rad -= 30 * M_PI / 180.;
            } else if (relative_yaw<0) {
              target_attitude_yaw_rad += 30 * M_PI / 180.;
            }
          }
        }


        geometry_msgs::Point &c_p = last_odometry_msg.pose.pose.position;
        double &yaw = odom_attitude_yaw_rad;
        geometry_msgs::Point &p_t = target_waypoint.p_target_m;
        p_t.x = c_p.x + (cos(yaw) * msg.position[0] - sin(yaw) * msg.position[1]) * 30.; //TODO: remove conversion factor when MH is done
        p_t.y = c_p.y + (sin(yaw) * msg.position[0] + cos(yaw) * msg.position[1]) * 30.;
        p_t.z = c_p.z + msg.position[2]                                           * 30.;

        target_waypoint.yaw_target_rad = target_attitude_yaw_rad;

        target_waypoint.max_speed_xy = 1.5;
        target_waypoint.max_speed_z = 1.0;
        target_waypoint.max_accel_xy = 0.05 * 9.81;
        target_waypoint.max_accel_z_up = 0.05 * 9.81;
        target_waypoint.max_accel_z_down = 0.05 * 9.81;
        target_waypoint.waypoint_threshold_m = 0.5;
        target_waypoint.waypoint_threshold_rad = 5. * M_PI / 180.;
        target_waypoint.minimum_trajectory_length_m = 0.5;

        target_location_local_publisher.publish(target_waypoint);

        break;
      }
    }
  }

  template<typename T>
  static void QuaternionXYZWToEulerAngles(const T* q, T* yaw, T* pitch,  T* roll) {
    const T &q_x = q[0];
    const T &q_y = q[1];
    const T &q_z = q[2];
    const T &q_w = q[3];

    if (roll != NULL)
      (*roll) = atan2( (T(2.)*(q_w*q_x + q_y*q_z)), (T(1.) - T(2.) * (q_x*q_x+q_y*q_y) ) );

    if (pitch != NULL) {
      T aux = T(2.)*(q_w*q_y - q_z*q_x);
      if ( aux > T(1.) ) {
        (*pitch) = T(+M_PI/2.);
      } else {
        if ( aux < T(-1.) )
          (*pitch) = T(-M_PI/2.);
        else
          (*pitch) = asin( aux);
      }
    }

    if (yaw != NULL)
      (*yaw) = atan2( (T(2.)*(q_w*q_z + q_x*q_y)), (T(1.) - T(2.) * (q_y*q_y+q_z*q_z) ) );
  }
};
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(icg_plugins::DjiPoseStampedPlugin,
                       icg_plugins::PluginBase)
