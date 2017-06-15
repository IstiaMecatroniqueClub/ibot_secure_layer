#include <ros/ros.h>

#include "IbotState.h"

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "ibot_secure_layer_node");
  ros::NodeHandle node;


  IbotState state(node);

  ROS_INFO("ibot_secure_layer_node started");
  while(ros::ok())
  {
    state.updateState();
    ros::spinOnce();
    usleep(5000);
  }







}
