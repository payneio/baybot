#include "ros/ros.h"
#include "baybot_base/baybot_base.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "baybot_base");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ROS_INFO("Started baybot hardware interface");
  baybot_base::BaybotBase BaybotBase(nh);
  ros::waitForShutdown();
  return 0;
}
