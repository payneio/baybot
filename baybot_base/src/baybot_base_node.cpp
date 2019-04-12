#include <controller_manager/controller_manager.h>
#include "ros/ros.h"

#include "baybot_base/baybot_hardware.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "baybot_base");
  ros::NodeHandle nh;

  // Try to get hardware update frequency from ROS paramater server
  // Default to 10/s
  double loop_hz;
  nh.param("/baybot/hardware_interface/loop_hz", loop_hz, 0.1);
  ros::Duration update_freq = ros::Duration(1.0 / loop_hz);

  baybot_base::BaybotHardware baybot(nh);
  controller_manager::ControllerManager cm(&baybot, nh);

  ros::AsyncSpinner spinner(1);

  // Call BaybotHardware:update at update_freq in a separate thread
  ros::Timer non_realtime_loop = nh.createTimer(
    update_freq,
    &baybot_base::BaybotHardware::update,
    &baybot
  );

  spinner.start();
  ROS_INFO("Started Baybot base");
  ros::waitForShutdown();
  return 0;
}
