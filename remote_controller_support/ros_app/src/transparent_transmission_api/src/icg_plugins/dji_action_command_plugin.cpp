
#include <eigen_conversions/eigen_msg.h>
#include <mavros/frame_tf.h>
#include <mavros/utils.h>

//#include <tf_conversions/tf_eigen.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>

#include "transparent_transmission_api/plugin_base.hpp"

// Task Scheduler Interface (TSI)
#include "icg_dji_common/icg_dji_common.h"
#include "std_msgs/Int32.h"

// Similar to plugins/setpoint_raw.cpp -> get setpoint
// and plugins/setpoint_position.cpp -> set setpoint

using mavlink::dji_icg::DJI_ACTION;
using mavros::utils::enum_value;

namespace icg_plugins {

/**
 * @brief Dji action command plugin.
 *
 * Receives different flight commands from the Android device.
 */
class DjiActionCommandPlugin : public icg_plugins::PluginBase {
 public:
  DjiActionCommandPlugin() : PluginBase(), nh("~") {}

  void initialize(const std::shared_ptr<SendCommand> &sendComm) {
    PluginBase::initialize(sendComm);

    task_scheduler_command_publisher  = nh.advertise<std_msgs::Int32>("in", 10);
    task_scheduler_feedback_publisher = nh.advertise<std_msgs::Int32>("out",10);
  }

  Subscriptions get_subscriptions() {
    return {make_handler(&DjiActionCommandPlugin::handleDjiActionCommand)};
  }

 private:
  ros::NodeHandle nh;
  // Task Scheduler Interface (TSI)
  void task_scheduler_command_callback(const std_msgs::Int32 &msg);
  ros::Publisher task_scheduler_command_publisher;
  ros::Publisher task_scheduler_feedback_publisher;

  // ** Task Scheduler Interface (TSI) **
  //void task_scheduler_command_callback(const std_msgs::Int32::ConstPtr &msg) {
  //  return; /* not yet implemented */
  //}

  void handleDjiActionCommand(const mavlink::mavlink_message_t *mavMsg,
                              mavlink::dji_icg::msg::DJI_ACTION_COMMAND &msg) {
    // TODO: Do the action
    switch (msg.action) {
      case enum_value(DJI_ACTION::TAKE_OFF): {
        ROS_INFO("Received DJI_ACTION::TAKE_OFF.");
        std_msgs::Int32 msg;
        msg.data = IcgDjiCommon::NAVI_TAKE_OFF;
        task_scheduler_command_publisher.publish(msg);
        break;
      }
      case enum_value(DJI_ACTION::LAND): {
        ROS_INFO("Received DJI_ACTION::LAND.");
        std_msgs::Int32 msg;
        msg.data = IcgDjiCommon::NAVI_LAND;
        task_scheduler_command_publisher.publish(msg);
        break;
      }
      case enum_value(DJI_ACTION::FLY_TRAJECTORY): {
        ROS_INFO("Received DJI_ACTION::FLY_TRAJECTORY.");
        std_msgs::Int32 msg;
        msg.data = IcgDjiCommon::ANDROID_START_MISSION_03;
        task_scheduler_feedback_publisher.publish(msg);
        break;
      }
      case enum_value(DJI_ACTION::HOVER): {
        ROS_INFO("Received DJI_ACTION::HOVER.");
        std_msgs::Int32 msg;
        msg.data = IcgDjiCommon::NAVI_HOVER;
        task_scheduler_command_publisher.publish(msg);
        break;
      }
      case enum_value(DJI_ACTION::SYNC_TIME): {
        ROS_INFO("Received DJI_ACTION::SYNC_TIME.");
        break;
      }
    }
  }
};
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(icg_plugins::DjiActionCommandPlugin,
                       icg_plugins::PluginBase)
