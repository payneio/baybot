#pragma once

#include <cstddef>
#include <cstdint>

#include "baybot_base/i2c.h"

#define BASE_SLAVE_ADDRESS 0x58

namespace baybot_base {

// Interface for a motor driver (e.g. MD25)
class MotorDriver {
 public:
  // Configuration
  virtual void ChangeAddress(uint8_t newAddress) = 0;
  virtual void EnableSpeedRegulation() = 0;
  virtual void DisableSpeedRegulation() = 0;
  virtual void EnableTimeout() = 0;
  virtual void DisableTimeout() = 0;
  virtual uint8_t GetAccelerationRate() = 0;
  virtual void SetAccelerationRate(uint8_t rate) = 0;

  virtual uint8_t GetMode() = 0;
  virtual void SetMode(uint8_t mode) = 0;

  // Encoders
  virtual void ResetEncoders() = 0;
  virtual int GetEncoder1() = 0;
  virtual int GetEncoder2() = 0;

  // Status
  virtual long GetSoftwareVersion() = 0;
  virtual float GetBatteryVolts() = 0;

  // Motors
  virtual uint8_t GetMotor1Current() = 0;
  virtual uint8_t GetMotor2Current() = 0;
  virtual uint8_t GetMotor1Speed() = 0;
  virtual uint8_t GetMotor2Speed() = 0;
  virtual void SetMotorsSpeed(uint8_t speed) = 0;
  virtual void SetMotor1Speed(uint8_t speed) = 0;
  virtual void SetMotor2Speed(uint8_t speed) = 0;
  virtual void StopMotor1() = 0;
  virtual void StopMotor2() = 0;
  virtual void StopMotors() = 0;
};

// An MD25 motor driver
class MD25 : public MotorDriver {
 public:
  MD25();
  MD25(SerialProtocol& sp);

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
  void SetMotorsSpeed(const uint8_t speed);
  void SetMotor1Speed(const uint8_t speed);
  void SetMotor2Speed(const uint8_t speed);
  void StopMotor1();
  void StopMotor2();
  void StopMotors();

 private:
  SerialProtocol& serial_;

  void SetMotorSpeed(uint8_t motor, uint8_t speed);
  uint8_t ReadRegisterByte(uint8_t reg);
  int ReadEncoderArray(uint8_t reg);
  void SendWireCommand(uint8_t[], uint8_t);

};

}  // namespace baybot_base