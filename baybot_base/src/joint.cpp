#include "ros/ros.h"

#include "baybot_base/joint.h"

#define PI 3.14159265359
#define TAU 6.28318530718

namespace baybot_base {

Joint::Joint(std::string name, uint8_t motor_id)
    : name_(name), motor_id_(motor_id) {
}

Joint::~Joint(){}

// SmoothAngle returns an average of the last `filter_previous` readings.
double Joint::SmoothAngle(const double angle) {

  // keep track of the number of first few reads so our average will be
  // correct
  if (angle_read_counter_ < smooth_previous_) {
    angle_read_counter_++;
  }

  // put value at front of array
  // TODO: This is wholly unnecessary... since the position doesn't matter
  // for averaging, we could just cycle the index around
  for (int i = smooth_previous_ - 1; i > 0; i--) {
    previous_angles_[i] = previous_angles_[i - 1];
  }
  previous_angles_[0] = angle;

  // Find the average
  double angle_sum = 0.0;
  for (int i = 0; i < angle_read_counter_; i++) {
    angle_sum = angle_sum + previous_angles_[i];
  }
  double smooth_result = angle_sum / double(angle_read_counter_);

  // ROS_INFO("%f, %f, %f, %i", angle, angle_sum, filterResult,
  // filter_iterations);

  return smooth_result;
}

double Joint::ReadAngle() {
  uint16_t position;

  // FIXME: Read from MD25
  return 0.0;

  // if (result == 1) {
  //   double angle = (position / sensor_resolution_ * TAU);
  //   angle = SmoothAngle(angle);
  //   angle += angle_offset_;
  //   if (angle > PI) angle -= TAU;
  //   if (angle < -PI) angle += TAU;
  //   angle *= read_ratio_;
  //   return angle;
  // } else {
  //   ROS_ERROR("I2C Read Error during joint position read. Exiting for safety.");
  // }
}

void Joint::Actuate(double velocity, const uint8_t duration = 15) {
  
  // clamp velocity range
  if (velocity > 1.0) velocity = 1.0;
  if (velocity < -1.0) velocity = -1.0;
  if (abs(velocity * 100) < 20) return; // negligible

  // prepare i2c data
  uint8_t data[4];
  data[0] = motor_id_;
  data[1] = floor(abs(velocity * drive_controller_scale_));
  data[2] = (velocity > 0);  
  data[3] = duration;

  // write data to i2c
  // FIXME: Write to MD25

}

}  // namespace baybot_base
