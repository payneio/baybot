#include <sstream>

#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>

#include "baybot_base/hardware.h"

using namespace hardware_interface;

const double DRIVE_CONTROLLER_SCALE = 128;
const int DRIVE_CONTROLLER_SHIFT = 128;

namespace baybot_base {

BaybotHardware::BaybotHardware(MotorDriver& motorDriver)
    : md25_(motorDriver),
      lwheel_name_("l_wheel_joint"),
      lwheel_id_(9),
      rwheel_name_("r_wheel_joint"),
      rwheel_id_(10) {
  // Give the controller manager (and the controllers inside the
  // controller manager) access to Baybot's joint states (wheels), and to
  // the Baybot base commands. When the controller manager runs, the
  // controllers will read from the pos_, vel_ and eff_ variables, and the
  // controller will write the desired command into the cmd_ variables.

  // Registers all joint interfaces. For each joint declared in the
  // hardware interface, initialize command/state vectors, and initialize
  // and register every kind of joint handle and interface. Note: this is
  // likely overkill as each type of joint prob needs only one type of
  // interface.

  // connect ros_control joint state interface
  JointStateHandle lwheel_state(lwheel_name_, &pos_[0], &vel_[0], &eff_[0]);
  joint_state_interface_.registerHandle(lwheel_state);
  JointStateHandle rwheel_state(rwheel_name_, &pos_[1], &vel_[1], &eff_[1]);
  joint_state_interface_.registerHandle(rwheel_state);
  registerInterface(&joint_state_interface_);

  // connect ros_control velocity_joint_interface
  JointHandle lwheel_vel_handle(lwheel_state, &cmd_[0]);
  velocity_joint_interface_.registerHandle(lwheel_vel_handle);
  JointHandle rwheel_vel_handle(rwheel_state, &cmd_[1]);
  velocity_joint_interface_.registerHandle(rwheel_vel_handle);
  registerInterface(&velocity_joint_interface_);
}

// Make sure the pos_, vel_ and eff_ variables always have the latest
// joint state available, and make sure that whatever is written into the
// cmd_ variable gets executed by the robot.
void BaybotHardware::Update(const ros::TimerEvent& e) {
  ROS_INFO("Updated at %f", e.current_real.toSec());
  elapsed_time_ = ros::Duration(e.current_real - e.last_real);
  Read();
  controller_manager_->update(ros::Time::now(), elapsed_time_);
  Write(elapsed_time_);
}

// Populate joint state variables from Baybot
void BaybotHardware::Read() {
  // if (result == 1) {
  //   double angle = (position / sensor_resolution_ * TAU);
  //   angle = SmoothAngle(angle);
  //   angle += angle_offset_;
  //   if (angle > PI) angle -= TAU;
  //   if (angle < -PI) angle += TAU;
  //   angle *= read_ratio_;
  //   return angle;
  // } else {
  //   ROS_ERROR("I2C Read Error during joint position read. Exiting for
  //   safety.");
  // }

  // pos_[0] = lwheel_->ReadAngle();
  // pos_[1] = rwheel_->ReadAngle();
}

// Write whatever is in the cmd_ variable to Baybot
void BaybotHardware::Write(ros::Duration elapsed_time) {
  // Send commands to wheels
  ROS_INFO("writing");
  md25_.SetMotor1Speed(VelocityToMD25(cmd_[0]));
  md25_.SetMotor2Speed(VelocityToMD25(cmd_[1]));
}

// SmoothAngle returns an average of the last `filter_previous` readings.
double BaybotHardware::SmoothAngle(int joint, const double angle) {
  // put value at next position in buffer
  wheel_buffer_[joint][wheel_buffer_pos_[joint]] = angle;
  wheel_buffer_pos_[joint]++;

  // Find the average
  auto sum = 0.0;
  auto count = 0;
  for (auto i = 0; i < 3; i++) {
    if (wheel_buffer_[joint][i] == NULL_ANGLE) continue;
    sum += wheel_buffer_[joint][i];
    count++;
  }
  auto smooth_result = sum / double(count);

  // ROS_INFO("%f, %f, %f, %i", angle, angle_sum, filterResult,
  // filter_iterations);

  return smooth_result;
}

// The MD25 driver considers 0 = full reverse, 128 = stop, 255 = full forward
// refer: http://www.robot-electronics.co.uk/htm/md25ser.htm
uint8_t VelocityToMD25(double velocity) {
  // clamp velocity range
  if (velocity > 1.0) velocity = 1.0;
  if (velocity < -1.0) velocity = -1.0;
  if (abs(velocity * 100) < 20) velocity = 0.0;  // negligible

  velocity = (velocity * DRIVE_CONTROLLER_SCALE) + DRIVE_CONTROLLER_SHIFT;
  if (velocity > 255) velocity = 255;
  return velocity;
}

}  // namespace baybot_base
