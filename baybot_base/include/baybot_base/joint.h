#pragma once

#include <sstream>

#include "baybot_base/md25.h"

// i2c address
#define BASE_SLAVE_ADDRESS 0x58

namespace baybot_base {
class Joint {
 public:
  //Joint();
  Joint(std::string, uint8_t);
  ~Joint();
  std::string name_;
  double sensor_resolution_ = 1024;
  double angle_offset_ = 0;
  double read_ratio_ = 1;
  double drive_controller_scale_ = 127;
  double ReadAngle();
  void Actuate(double effort, uint8_t duration);

 private:
  uint8_t motor_id_;
  double previous_velocity_;

  // smoothing
  double SmoothAngle(double angle);
  uint8_t angle_read_counter_ = 0;
  static const uint8_t smooth_previous_ = 3;
  double previous_angles_[smooth_previous_];
  
};
}  // namespace baybot_base
