#ifndef BAYBOT_HAL__BAYBOT_HAL_H
#define BAYBOT_HAL__BAYBOT_HAL_H

#include <boost/scoped_ptr.hpp>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <ros/ros.h>
#include "baybot_base/joint.h"

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;
using joint_limits_interface::SoftJointLimits;

namespace baybot_base {
static const double POSITION_STEP_FACTOR = 10;
static const double VELOCITY_STEP_FACTOR = 10;

class BaybotHardware : public hardware_interface::RobotHW {
public:
  BaybotHardware(ros::NodeHandle &nh);
  ~BaybotHardware();
  void RegisterControlInterfaces();
  void update(const ros::TimerEvent &e);
  void read();
  void write(ros::Duration elapsed_time);

protected:
  ros::NodeHandle nh_;
  ros::Duration control_period_;
  ros::Duration elapsed_time_;
  boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
  double p_error_, v_error_, e_error_;

  // Joints
  Joint lwheel_;
  Joint rwheel_;

  // Interfaces
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;
  joint_limits_interface::VelocityJointSaturationInterface
      velocity_joint_saturation_interface_;
  joint_limits_interface::VelocityJointSoftLimitsInterface
      velocity_joint_limits_interface_;

  // Shared memory
  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  std::vector<double> joint_efforts_;
  std::vector<double> joint_position_commands_;
  std::vector<double> joint_velocity_commands_;
  std::vector<double> joint_effort_commands_;
  std::vector<double> joint_lower_limits_;
  std::vector<double> joint_upper_limits_;
  std::vector<double> joint_effort_limits_;

};

} // namespace baybot_base

#endif