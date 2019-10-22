
#include "transparent_transmission_api/transparent_transmission_api.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "talker");

  ros::NodeHandle nodeHandle;
  // TransparentTransmissionAPI ttApi = TransparentTransmissionAPI(nodeHandle);
  TransparentTransmissionAPI ttApi(nodeHandle);
  ros::Rate loop_rate(50);

  // ttApi.sendHeartbeat();

  while (ros::ok()) {
    // ttApi.sendHeartbeat();
    ros::spinOnce();
    loop_rate.sleep();
    // ROS_INFO("I am screaming!");
  }

  /*ros::MultiThreadedSpinner spinner(2);
  spinner.spin();*/

  return 0;
}
