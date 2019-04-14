#include <sstream>

#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>

#include "baybot_base/baybot_hardware.h"

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;
using joint_limits_interface::SoftJointLimits;

namespace baybot_base {
BaybotHardware::BaybotHardware(ros::NodeHandle &nh) : nh_(nh) {

  lwheel_ = std::make_unique<Joint>("JointBaseWheelL", 9);
  rwheel_ = std::make_unique<Joint>("JointBaseWheelR", 10);

  RegisterControlInterfaces();
}

BaybotHardware::~BaybotHardware() {}

// Registers all joint interfaces. For each joint declared in the hardware
// interface, initialize command/state vectors, and initialize and register
// every kind of joint handle and interface.
// Note: this is likely overkill as each type of joint prob needs only one
// type of interface.
void BaybotHardware::RegisterControlInterfaces() {
  // Initialize joint data vector space
  int num_joints = 2;
  joint_positions_.resize(num_joints);
  joint_velocities_.resize(num_joints);
  joint_efforts_.resize(num_joints);
  joint_position_commands_.resize(num_joints);
  joint_velocity_commands_.resize(num_joints);
  joint_effort_commands_.resize(num_joints);

  int i = 0;
  for (const auto joint : {&lwheel_, &rwheel_}) {
    // Create joint state interface
    JointStateHandle jointStateHandle(
      (*joint)->name_,
      &joint_positions_[i],
      &joint_velocities_[i],
      &joint_efforts_[i]);
    joint_state_interface_.registerHandle(jointStateHandle);

    // Create velocity-joint interface
    JointHandle jointVelocityHandle(jointStateHandle,
                                    &joint_velocity_commands_[i]);
    velocity_joint_interface_.registerHandle(jointVelocityHandle);

    // Create velocity-joint soft-limits interface
    // JointLimits limits;
    // SoftJointLimits softLimits;
    // getJointLimits(joint.name, nh_, limits);
    // VelocityJointSoftLimitsHandle jointLimitsHandle(jointVelocityHandle,
    // limits, softLimits);
    // velocity_joint_soft_limits_interface_.registerHandle(jointLimitsHandle);

    i++;
  }

  registerInterface(&joint_state_interface_);
  registerInterface(&velocity_joint_interface_);
  // registerInterface(&velocity_joint_soft_limits_interface_);
}

void BaybotHardware::update(const ros::TimerEvent &e) {
  elapsed_time_ = ros::Duration(e.current_real - e.last_real);
  read();
  controller_manager_->update(ros::Time::now(), elapsed_time_);
  write(elapsed_time_);
}

void BaybotHardware::read() {
  joint_positions_[0] = lwheel_->ReadAngle();
  joint_positions_[1] = rwheel_->ReadAngle();
}

void BaybotHardware::write(ros::Duration elapsed_time) {
  // velocity_joint_soft_limits_interface_.enforceLimits(elapsed_time);
  lwheel_->Actuate(joint_velocity_commands_[0], (uint8_t)elapsed_time.sec);
  rwheel_->Actuate(joint_velocity_commands_[0], (uint8_t)elapsed_time.sec);
}
}  // namespace baybot_base