#pragma once

#include <cstdint>
#include <cstddef>

#include "baybot_base/i2c.h"

#define BASE_SLAVE_ADDRESS 0x58

namespace baybot_base {

class MD25 {
 public:
  MD25();
  MD25(uint8_t i2cAddress);

  // Configuration
  void ChangeAddress(uint8_t newAddress);
  void EnableSpeedRegulation();
  void DisableSpeedRegulation();
  void EnableTimeout();
  void DisableTimeout();
  uint8_t GetAccelerationRate();
  void SetAccelerationRate(uint8_t rate);

  uint8_t GetMode();
  void SetMode(uint8_t mode);

  // Encoders
  void ResetEncoders();
  int GetEncoder1();
  int GetEncoder2();

  // Status
  long GetSoftwareVersion();
  float GetBatteryVolts();
  
  // Motors
  uint8_t GetMotor1Current();
  uint8_t GetMotor2Current();
  uint8_t GetMotor1Speed();
  uint8_t GetMotor2Speed();
  void SetMotorsSpeed(uint8_t speed);
  void SetMotor1Speed(uint8_t speed);
  void SetMotor2Speed(uint8_t speed);
  void StopMotor1();
  void StopMotor2();
  void StopMotors();
  
  
 private:
  void SetMotorSpeed(uint8_t motor, uint8_t speed);
  uint8_t ReadRegisterByte(uint8_t reg);
  int ReadEncoderArray(uint8_t reg);
  void SendWireCommand(uint8_t[], uint8_t);

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