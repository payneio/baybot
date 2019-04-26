#include <controller_manager/controller_manager.h>
#include "ros/ros.h"

#include "baybot_base/hardware.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "baybot_base");
  ros::NodeHandle nh;

  // Try to get hardware update frequency from ROS paramater server
  // Default to 10/s
  double loop_hz;
  nh.param("/baybot/hardware_interface/loop_hz", loop_hz, 0.1);
  ros::Duration update_freq = ros::Duration(1.0 / loop_hz);

  // Baybot uses the MD25 motor controller. Let's initialize it.

  // TODO: set from config, e.g. default address is B0 << 1 which is 0x58
  // as a 7 bit address for the wire lib
  baybot_base::I2C i2c("/dev/i2c-2", 0x58);
  baybot_base::MD25 md25(i2c);

  // Initialize hardware
  baybot_base::BaybotHardware baybot(md25);

  // Link to ros_control controller manager
  controller_manager::ControllerManager cm(&baybot);

  ros::AsyncSpinner spinner(1);

  // Call BaybotHardware:update at update_freq in a separate thread
  ros::Timer non_realtime_loop = nh.createTimer(
      update_freq, &baybot_base::BaybotHardware::Update, &baybot);
  ros::waitForShutdown();
  return 0;
}
