#ifndef BAYBOT_HAL__JOINT_H
#define BAYBOT_HAL__JOINT_H

#include <sstream>

// We multiple i2c addresses
#define BASE_SLAVE_ADDRESS 0x70
#define ARM_RIGHT_SLAVE_ADDRESS 0x71
#define ARM_LEFT_SLAVE_ADDRESS 0x72
#define HEAD_SLAVE_ADDRESS 0x73

// We can control both motors and servos
#define ACTUATOR_TYPE_NONE -1
#define ACTUATOR_TYPE_MOTOR 0
#define ACTUATOR_TYPE_SERVO 1

namespace baybot_base
{
class Joint
{
private:
  uint8_t motor_id_ = 0;
  uint8_t actuator_type_ = 0;
  uint8_t GetSlaveAddress();
  uint8_t min_servo_value_ = 0;
  uint8_t max_servo_value_ = 75;
  double previous_effort_;
  double FilterAngle(double angle);
  int angle_reads_ = 0;
  static const int filter_previous_ = 3;
  double previous_angles_[filter_previous_];
  void PrepareI2CWrite(uint8_t result[4], double effort);
  void PrepareI2CRead(uint8_t result[4]);

public:
  std::string name;
  Joint();
  Joint(uint8_t motorId);
  ~Joint();
  double sensor_resolution_ = 1024;
  double angle_offset_ = 0;
  double read_ratio_ = 1;
  uint8_t GetMotorId();
  void SetMotorId(uint8_t motor_id);
  void SetActuatorType(uint8_t actuator_type);
  void SetServoLimits(uint8_t min_value, uint8_t max_value);
  int GetActuatorType();
  double GetPreviousEffort();
  void Actuate(double effort, uint8_t duration);
  double ReadAngle();
};
}  // namespace baybot

#endif
