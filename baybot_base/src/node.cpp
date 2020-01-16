#include <controller_manager/controller_manager.h>
#include "ros/ros.h"

#include "baybot_base/hardware.h"
#include "ros/callback_queue.h"

#include <boost/chrono.hpp>

typedef boost::chrono::steady_clock time_source;

/**
* Control loop for Baybot, not realtime safe
*/
void ControlLoop(baybot_base::BaybotHardware &baybot,
                 controller_manager::ControllerManager &cm,
                 time_source::time_point &last_time)
{
  // Calculate monotonic time difference
  time_source::time_point this_time = time_source::now();
  boost::chrono::duration<double> elapsed_duration = this_time - last_time;
  ros::Duration elapsed(elapsed_duration.count());
  last_time = this_time;

  ROS_INFO("Updated");
  baybot.Read();
  cm.update(ros::Time::now(), elapsed);
  baybot.Write(elapsed);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "baybot_base");
  ros::NodeHandle nh;

  std::string i2c_path;
  nh.param("/baybot_base/i2c/path", i2c_path, std::string("/dev/i2c-2"));
  int i2c_channel;
  nh.param("/baybot_base/i2c/channel", i2c_channel, 0x58);
  double control_hz;
  nh.param("/baybot_base/control_hz", control_hz, 10.0);

  // Try to get hardware update frequency from ROS paramater server
  // Default to 10/s
  ros::Duration update_freq = ros::Duration(1.0 / control_hz);
  ROS_INFO("Set update frequency to %fs.", 1.0 / control_hz);

  // Baybot uses the MD25 motor controller. Let's initialize it.

  // TODO: set from config, e.g. default address is B0 << 1 which is 0x58
  // as a 7 bit address for the wire lib
  baybot_base::I2C i2c(i2c_path, i2c_channel);
  baybot_base::MD25 md25(i2c);

  // Initialize hardware
  baybot_base::BaybotHardware baybot(md25);

  // Link to ros_control controller manager
  controller_manager::ControllerManager cm(&baybot, nh);
  ROS_INFO("Controller loaded.");

  // Call Hardware:Update at update_freq in a separate thread
  ros::CallbackQueue queue;
  ros::AsyncSpinner spinner(1, &queue);
  time_source::time_point last_time = time_source::now();
  ros::TimerOptions control_timer(
    ros::Duration(update_freq),
    boost::bind(ControlLoop, boost::ref(baybot), boost::ref(cm), boost::ref(last_time)),
    &queue);
  ros::Timer control_loop = nh.createTimer(control_timer);

  ROS_INFO("Started control timer.");
  
  spinner.start();

  // Process remainder of ROS callbacks separately, mainly ControlManager related
  ros::spin();

  ros::waitForShutdown();
  return 0;
}
