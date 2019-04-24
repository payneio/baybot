#include "baybot_base/md25.h"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

namespace baybot_base {

#define PI 3.14159265359
#define TAU 6.28318530718

const uint8_t STANDARD = 0;
const uint8_t NEGATIVE_SPEEDS = 1;
const uint8_t SPEED_AND_TURN = 2;
const uint8_t NEGATIVE_SPEEDS_AND_TURN = 3;

// MD25 Registers
const uint8_t SPEED_1 = 0;
const uint8_t SPEED_2 = 1;
const uint8_t ENC_1A = 2;
const uint8_t ENC_1B = 3;
const uint8_t ENC_1C = 4;
const uint8_t ENC_1D = 5;
const uint8_t ENC_2A = 6;
const uint8_t ENC_2B = 7;
const uint8_t ENC_2C = 8;
const uint8_t ENC_2D = 9;
const uint8_t BATTERY_VOLTAGE = 10;
const uint8_t MOTOR_1_CURRENT = 11;
const uint8_t MOTOR_2_CURRENT = 12;
const uint8_t SOFTWARE_VERSION = 13;
const uint8_t ACCELERATION = 14;
const uint8_t MODE = 15;
const uint8_t COMMAND = 16;
const uint8_t RESET_ENCODER_REGISTERS = 0x20;
const uint8_t DISABLE_AUTO_SPEED_REGULATION = 0x30;
const uint8_t ENABLE_AUTO_SPEED_REGULATION = 0x31;
const uint8_t DISABLE_2_SECOND_TIMEOUT = 0x32;
const uint8_t ENABLE_2_SECOND_TIMEOUT = 0x33;
const uint8_t CHANGE_I2C_ADDRESS_1 = 0xA0;
const uint8_t CHANGE_I2C_ADDRESS_2 = 0xAA;
const uint8_t CHANGE_I2C_ADDRESS_3 = 0xA5;

/*
 * Constructors
 *
 * default address is B0 << 1 i.e. 0x58 as a 7 bit address for the wire lib
 */
MD25::MD25() { MD25{0xB0 >> 1}; };

MD25::MD25(uint8_t i2cAddress): i2c_{I2C("/dev/i2c-2", i2cAddress)} {

  //this->i2c_ = I2C(boost::format("/dev/i2c-%1%" % i2cAddress);
  //  this->i2c_ { "/dev/i2c-2", 0x58 };

}

// Gets
int MD25::GetEncoder1() { return ReadEncoderArray(encoderOneReg); }

int MD25::GetEncoder2() { return ReadEncoderArray(encoderTwoReg); }

long MD25::GetSoftwareVersion() { return ReadRegisterByte(softwareVerReg); }

float MD25::GetBatteryVolts() { return ReadRegisterByte(voltReg) / 10.0; }

uint8_t MD25::GetAccelerationRate() { return ReadRegisterByte(accRateReg); }

uint8_t MD25::GetMotor1Speed() { return ReadRegisterByte(speed1Reg); }

uint8_t MD25::GetMotor2Speed() { return ReadRegisterByte(speed2Reg); }

uint8_t MD25::GetMotor1Current() { return ReadRegisterByte(current1Reg); }

uint8_t MD25::GetMotor2Current() { return ReadRegisterByte(current2Reg); }

uint8_t MD25::GetMode() { return ReadRegisterByte(modeReg); }

// Sets
void MD25::ResetEncoders() {
  static uint8_t command[] = {cmdReg, 0x20};
  SendWireCommand(command, 2);
}

void MD25::EnableSpeedRegulation() {
  static uint8_t command[] = {cmdReg, 0x31};
  SendWireCommand(command, 2);
}

void MD25::DisableSpeedRegulation() {
  static uint8_t command[] = {cmdReg, 0x30};
  SendWireCommand(command, 2);
}

void MD25::EnableTimeout() {
  static uint8_t command[] = {cmdReg, 0x33};
  SendWireCommand(command, 2);
}

void MD25::DisableTimeout() {
  static uint8_t command[] = {cmdReg, 0x32};
  SendWireCommand(command, 2);
}

void MD25::SetMotorsSpeed(uint8_t speed) {
  SetMotor1Speed(speed);
  SetMotor2Speed(speed);
}

void MD25::SetMotor1Speed(uint8_t speed) { SetMotorSpeed(speed1Reg, speed); }

void MD25::SetMotor2Speed(uint8_t speed) { SetMotorSpeed(speed2Reg, speed); }

void MD25::StopMotor1() { SetMotor1Speed(stopSpeed); }

void MD25::StopMotor2() { SetMotor2Speed(stopSpeed); }

void MD25::StopMotors() {
  StopMotor1();
  StopMotor2();
}

void MD25::SetMode(uint8_t mode) {
  static uint8_t command[] = {modeReg, 0x00};
  command[1] = mode;
  SendWireCommand(command, 2);
}

void MD25::SetAccelerationRate(uint8_t rate) {
  static uint8_t command[] = {accRateReg, 0x05};
  command[1] = rate;
  SendWireCommand(command, 2);
}

void MD25::ChangeAddress(uint8_t newAddress) {
  static uint8_t command[] = {cmdReg, 0x0A};
  command[1] = 0x0A;
  SendWireCommand(command, 2);
  std::this_thread::sleep_for(6ms);
  command[1] = 0xAA;
  SendWireCommand(command, 2);
  std::this_thread::sleep_for(6ms);
  command[1] = 0xA5;
  SendWireCommand(command, 2);
  std::this_thread::sleep_for(6ms);
  command[1] = newAddress;
  SendWireCommand(command, 2);
  std::this_thread::sleep_for(6ms);
}

/*
 * Private Methods
 */

void MD25::SetMotorSpeed(uint8_t motor, uint8_t speed) {
  static uint8_t command[] = {0x00, stopSpeed};
  command[0] = motor;
  command[1] = speed;
  SendWireCommand(command, 2);
}

uint8_t MD25::ReadRegisterByte(uint8_t reg) {
  return i2c_.ReadReg8(reg);
}

int MD25::ReadEncoderArray(uint8_t reg) {

    uint8_t buffer[4];

    // Select the register on the device
    i2c_.Write(&reg, 1);

    // Read the data from the device
    i2c_.Read(buffer, 4);

  int position = 0;
  position = buffer[0] << 8 << 8 << 8;
  position |= buffer[1] << 8 << 8;
  position |= buffer[2] << 8;
  position |= buffer[3];
  return position;
}

void MD25::SendWireCommand(uint8_t bytes[], uint8_t num_bytes) {
    i2c_.Write(bytes, num_bytes);
  //i2c_.write(i2cAddress, bytes[0], bytes + 1, num_bytes);
}

// Private Fields
// enc_1a = 0
// enc_1b = 0
// enc_1c = 0
// enc_1d = 0
// enc_2a = 0
// enc_2b = 0
// enc_2c = 0
// enc_2d = 0
// battery_voltage = 0
// motor_1_current = 0
// motor_2_current = 0
// software_revision = 0

}  // namespace md25
