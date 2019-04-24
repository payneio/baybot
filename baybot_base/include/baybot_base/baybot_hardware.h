#pragma once

#include <memory>
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
#include "baybot_base/md25.h"

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;
using joint_limits_interface::SoftJointLimits;

namespace baybot_base {

static const double NULL_ANGLE = 9000;
static const double POSITION_STEP_FACTOR = 10;
static const double VELOCITY_STEP_FACTOR = 10;

class BaybotHardware : public hardware_interface::RobotHW {
public:
  BaybotHardware();
  BaybotHardware(MotorDriver &md);
  void RegisterControlInterfaces();
  void Update(const ros::TimerEvent &e);
  void Read();
  void Write(ros::Duration elapsed_time);

protected:
  ros::Duration control_period_;
  ros::Duration elapsed_time_;
  boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
  double p_error_, v_error_, e_error_;

  // ros_control interfaces
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;
  joint_limits_interface::VelocityJointSaturationInterface
      velocity_joint_saturation_interface_;
  joint_limits_interface::VelocityJointSoftLimitsInterface
      velocity_joint_limits_interface_;

  // ros_control memory buffers
  double cmd_[2];
  double pos_[2];
  double vel_[2];
  double eff_[2];
  
private:
  double SmoothAngle(int joint, const double angle);
  uint8_t VelocityToMD25(double);
  std::string lwheel_name_;
  int lwheel_id_;
  std::string rwheel_name_;
  int rwheel_id_;
  double wheel_buffer_[2][3] = { 
    { NULL_ANGLE,NULL_ANGLE,NULL_ANGLE },
    { NULL_ANGLE,NULL_ANGLE,NULL_ANGLE }
  };
  uint8_t wheel_buffer_pos_[2];

  MotorDriver &md25_;
};

} // namespace baybot_base
