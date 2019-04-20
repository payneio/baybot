#pragma once

#include <cstdint>
#include <cstddef>

#include "baybot_base/i2c.h"

namespace baybot_base {

class MD25 {
 public:
  MD25();
  MD25(uint8_t i2cAddress);
  ~MD25();
  void resetEncoders();
  int getEncoder1();
  int getEncoder2();
  void setMotorsSpeed(uint8_t speed);
  void setMotor1Speed(uint8_t speed);
  void setMotor2Speed(uint8_t speed);
  void stopMotor1();
  void stopMotor2();
  void stopMotors();
  long getSoftwareVersion();
  float getBatteryVolts();
  void changeAddress(uint8_t newAddress);
  uint8_t getAccelerationRate();
  uint8_t getMotor1Current();
  uint8_t getMotor2Current();
  uint8_t getMotor1Speed();
  uint8_t getMotor2Speed();
  uint8_t getMode();
  void enableSpeedRegulation();
  void disableSpeedRegulation();
  void enableTimeout();
  void disableTimeout();
  void setMode(uint8_t mode);
  void setAccelerationRate(uint8_t rate);

 private:
  void setMotorSpeed(uint8_t motor, uint8_t speed);
  uint8_t readRegisterByte(uint8_t reg);
  int readEncoderArray(uint8_t reg);
  void sendWireCommand(uint8_t[], uint8_t);

  I2C i2c_;

  static uint8_t const cmdReg = 0x10;          // command register
  static uint8_t const speed1Reg = 0x00;       // speed to first motor
  static uint8_t const speed2Reg = 0x01;       // speed to second motor
  static uint8_t const encoderOneReg = 0x02;   // motor encoder 1 (first byte)
  static uint8_t const encoderTwoReg = 0x06;   // motor encoder 2 (first byte)
  static uint8_t const voltReg = 0x0A;         // battery volts
  static uint8_t const current1Reg = 0x0B;     // motor 1 current
  static uint8_t const current2Reg = 0x0C;     // motor 2 current
  static uint8_t const softwareVerReg = 0x0D;  // software version
  static uint8_t const accRateReg = 0x0E;      // acceleration rate
  static uint8_t const modeReg = 0x0F;         // mode of operation
  static uint8_t const stopSpeed = 0x88;       // 0 velocity
};

}  // namespace md25