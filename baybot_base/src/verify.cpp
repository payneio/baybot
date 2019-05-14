#include <controller_manager/controller_manager.h>
#include "ros/ros.h"

#include "baybot_base/hardware.h"
#include "ros/callback_queue.h"

#include <boost/chrono.hpp>
#include <thread>
#include <iostream>

typedef boost::chrono::steady_clock time_source;

int main(int argc, char **argv) {
  ros::init(argc, argv, "baybot_base");
  ros::NodeHandle nh;

  std::string i2c_path;
  nh.param("/baybot_base/i2c/path", i2c_path, std::string("/dev/i2c-2"));
  int i2c_channel;
  nh.param("/baybot_base/i2c/channel", i2c_channel, 0x58);

  // Baybot uses the MD25 motor controller. Let's initialize it.

  // TODO: set from config, e.g. default address is B0 << 1 which is 0x58
  // as a 7 bit address for the wire lib
  baybot_base::I2C i2c(i2c_path, i2c_channel);
  baybot_base::MD25 md25(i2c);

  // Initialize hardware
  //baybot_base::BaybotHardware baybot(md25);

  // Do some stuff here... maybe wait for keyboard input?

  char key;
 
  while (std::cin >> key){
    switch (key) {
      case 'x':
        goto exit_loop;
      case 'w':
        std::cout << "A";
        md25.SetMotor1Speed(255);
        md25.SetMotor2Speed(255);
      case 'a':
        std::cout << "A";
        md25.SetMotor1Speed(255);
        md25.SetMotor2Speed(0);
      case 'd':
        std::cout << "D";
        md25.SetMotor1Speed(0);
        md25.SetMotor2Speed(255);
      case 's':
        std::cout << "A";
        md25.SetMotor1Speed(0);
        md25.SetMotor2Speed(0);
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
    md25.StopMotors();
  }
  exit_loop: ;

  //spinner.start();

  // Process remainder of ROS callbacks separately, mainly ControlManager related
  //ros::spin();

  //ros::waitForShutdown();
  return 0;
}
